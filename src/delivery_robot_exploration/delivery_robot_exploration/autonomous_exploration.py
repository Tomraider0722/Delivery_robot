# autonomous_exploration.py

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class AutonomousExploration(Node):

    def __init__(self):
        super().__init__('autonomous_exploration')

        # -----------------------------
        # Subscribers
        # -----------------------------

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # -----------------------------
        # Publisher
        # -----------------------------

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        completion_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.completion_pub = self.create_publisher(
            Bool,
            '/exploration_complete',
            completion_qos
        )

        # -----------------------------
        # TF
        # -----------------------------

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # -----------------------------
        # Data
        # -----------------------------

        self.scan = None
        self.map_msg = None

        self.target_x = None
        self.target_y = None

        self.no_frontier_since = None
        self.exploration_complete = False
        self.last_known_cell_count = None
        self.last_map_progress_time = None
        self.target_best_distance = None
        self.last_target_progress_time = None
        self.has_selected_target = False
        self.failed_targets = []
        self.bootstrap_logged = False

        # -----------------------------
        # Parameters
        # -----------------------------

        self.min_front_distance = 0.65
        self.safe_side_distance = 0.45

        self.linear_speed = 0.20
        self.turn_speed = 0.45

        self.goal_tolerance = 0.35

        # Frontier must be sufficiently
        # far from current robot position.
        self.minimum_frontier_distance = 1.0

        # Ignore tiny frontier regions.
        self.minimum_frontier_size = 20

        # Delay completion to allow a newly updated SLAM map to settle.
        self.completion_no_frontier_seconds = 8.0

        # Stop when SLAM has not revealed enough new map cells for a while.
        # This handles small, unreachable unknown regions around obstacles.
        self.minimum_new_known_cells = 25
        self.completion_no_map_progress_seconds = 60.0

        # Skip a frontier when the robot cannot get closer to it.  This node
        # uses reactive obstacle avoidance, not a global path planner.
        self.target_progress_distance = 0.15
        self.target_stall_seconds = 30.0
        self.failed_target_radius = 0.75

        # While SLAM is creating its first free-space cells there may be no
        # frontier yet.  Move slowly and safely instead of stopping early.
        self.bootstrap_linear_speed = 0.12

        # -----------------------------
        # Control loop
        # -----------------------------

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # -----------------------------
        # Frontier selection timer
        # -----------------------------

        self.frontier_timer = self.create_timer(
            2.0,
            self.update_frontier
        )

        self.get_logger().info(
            'Autonomous exploration started (linear speed: '
            f'{self.linear_speed:.2f} m/s)'
        )

        self.publish_completion_status(False)

    def publish_completion_status(self, complete):
        """Publish the completion state on a latched ROS topic."""
        status = Bool()
        status.data = complete
        self.completion_pub.publish(status)

    def complete_exploration(self, reason):
        """Stop navigation and announce why exploration has ended."""
        if self.exploration_complete:
            return

        self.exploration_complete = True
        self.target_x = None
        self.target_y = None
        self.publish_completion_status(True)
        self.get_logger().info(f'Exploration completed: {reason}')

    # ==========================================================
    # LaserScan
    # ==========================================================

    def scan_callback(self, msg):

        self.scan = msg

    def bootstrap_explore(self, cmd):
        """Move safely until SLAM provides the first usable frontier."""
        front = self.sector_min(math.radians(-25), math.radians(25))
        left = self.sector_min(math.radians(35), math.radians(90))
        right = self.sector_min(math.radians(-90), math.radians(-35))

        if front < self.min_front_distance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = (
                self.turn_speed if left > right else -self.turn_speed
            )
        else:
            cmd.twist.linear.x = self.bootstrap_linear_speed

            if left < self.safe_side_distance:
                cmd.twist.angular.z = -0.25
            elif right < self.safe_side_distance:
                cmd.twist.angular.z = 0.25

    # ==========================================================
    # Map
    # ==========================================================

    def map_callback(self, msg):

        self.map_msg = msg

        known_cells = sum(cell >= 0 for cell in msg.data)
        now = self.get_clock().now()

        if (
            self.last_known_cell_count is None
            or known_cells >= (
                self.last_known_cell_count + self.minimum_new_known_cells
            )
        ):
            self.last_known_cell_count = known_cells
            self.last_map_progress_time = now

    # ==========================================================
    # Get robot pose
    # ==========================================================

    def get_robot_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            q = transform.transform.rotation

            sin_yaw = 2.0 * (
                q.w * q.z +
                q.x * q.y
            )

            cos_yaw = 1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )

            yaw = math.atan2(
                sin_yaw,
                cos_yaw
            )

            return x, y, yaw

        except TransformException:

            return None

    # ==========================================================
    # Laser sector minimum
    # ==========================================================

    def sector_min(self, start_angle, end_angle):

        if self.scan is None:
            return float('inf')

        values = []

        angle = self.scan.angle_min

        for r in self.scan.ranges:

            if start_angle <= angle <= end_angle:

                if (
                    math.isfinite(r)
                    and r >= self.scan.range_min
                    and r <= self.scan.range_max
                ):
                    values.append(r)

            angle += self.scan.angle_increment

        if not values:
            return float('inf')

        return min(values)

    # ==========================================================
    # Find frontier regions
    # ==========================================================

    def find_frontiers(self):

        if self.map_msg is None:
            return []

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        data = self.map_msg.data

        frontier_cells = set()

        # -----------------------------
        # Find frontier cells
        # -----------------------------

        for y in range(1, height - 1):

            for x in range(1, width - 1):

                index = y * width + x

                # Free cell only
                if data[index] != 0:
                    continue

                neighbors = [
                    data[index - 1],
                    data[index + 1],
                    data[index - width],
                    data[index + width]
                ]

                # Unknown cell next to free cell
                if -1 in neighbors:

                    frontier_cells.add(
                        (x, y)
                    )

        regions = []

        # -----------------------------
        # Connected components
        # -----------------------------

        while frontier_cells:

            start = frontier_cells.pop()

            region = [start]
            queue = [start]

            while queue:

                cx, cy = queue.pop()

                for dx in [-1, 0, 1]:

                    for dy in [-1, 0, 1]:

                        if dx == 0 and dy == 0:
                            continue

                        neighbor = (
                            cx + dx,
                            cy + dy
                        )

                        if neighbor in frontier_cells:

                            frontier_cells.remove(
                                neighbor
                            )

                            region.append(
                                neighbor
                            )

                            queue.append(
                                neighbor
                            )

            if len(region) >= self.minimum_frontier_size:

                regions.append(region)

        return regions

    # ==========================================================
    # Select frontier target
    # ==========================================================

    def update_frontier(self):

        if self.exploration_complete:
            return

        if self.map_msg is None:
            return

        now = self.get_clock().now()

        if (
            self.has_selected_target
            and self.last_map_progress_time is not None
            and (
                (now - self.last_map_progress_time).nanoseconds / 1e9
                >= self.completion_no_map_progress_seconds
            )
        ):
            self.complete_exploration(
                'the map has not gained new known cells for '
                f'{self.completion_no_map_progress_seconds:.0f} seconds.'
            )
            return

        pose = self.get_robot_pose()

        if pose is None:
            return

        robot_x, robot_y, robot_yaw = pose

        regions = self.find_frontiers()

        if not regions:

            self.target_x = None
            self.target_y = None

            # The first few SLAM maps often contain only unknown/occupied
            # cells.  Do not treat that initial state as map completion.
            if not self.has_selected_target:
                self.no_frontier_since = None

                if not self.bootstrap_logged:
                    self.get_logger().info(
                        'Waiting for initial SLAM frontiers; using safe '
                        'bootstrap exploration.'
                    )
                    self.bootstrap_logged = True

                return

            if self.no_frontier_since is None:
                self.no_frontier_since = now
                self.get_logger().info(
                    'No frontier found; waiting for SLAM map updates before '
                    'declaring exploration complete.'
                )

            elapsed = (now - self.no_frontier_since).nanoseconds / 1e9

            if elapsed >= self.completion_no_frontier_seconds:
                self.complete_exploration(
                    'no reachable frontier regions remain.'
                )

            return

        self.no_frontier_since = None
        self.bootstrap_logged = False

        resolution = self.map_msg.info.resolution

        origin_x = (
            self.map_msg.info.origin.position.x
        )

        origin_y = (
            self.map_msg.info.origin.position.y
        )

        candidates = []

        # ======================================================
        # Generate candidate points
        # ======================================================

        for region in regions:

            # Sample region cells instead of
            # using only the mathematical center.

            step = max(
                1,
                len(region) // 30
            )

            sampled = region[::step]

            for cell_x, cell_y in sampled:

                map_x = (
                    origin_x +
                    (cell_x + 0.5) *
                    resolution
                )

                map_y = (
                    origin_y +
                    (cell_y + 0.5) *
                    resolution
                )

                dx = map_x - robot_x
                dy = map_y - robot_y

                distance = math.sqrt(
                    dx * dx +
                    dy * dy
                )

                if (
                    distance >=
                    self.minimum_frontier_distance
                    and not any(
                        math.hypot(
                            map_x - failed_x,
                            map_y - failed_y
                        ) < self.failed_target_radius
                        for failed_x, failed_y in self.failed_targets
                    )
                ):

                    candidates.append(
                        (
                            map_x,
                            map_y,
                            distance
                        )
                    )

        if not candidates:

            self.target_x = None
            self.target_y = None

            self.get_logger().info(
                'Frontiers exist, but no '
                'suitable distant target found'
            )

            return

        # ======================================================
        # Score candidates
        # ======================================================

        # Prefer reasonably distant frontier points,
        # but avoid extremely distant targets.

        def score(candidate):

            x, y, distance = candidate

            dx = x - robot_x
            dy = y - robot_y

            angle = math.atan2(
                dy,
                dx
            )

            angle_error = abs(
                math.atan2(
                    math.sin(angle - robot_yaw),
                    math.cos(angle - robot_yaw)
                )
            )

            # Prefer larger distance,
            # but penalize excessive distance
            # and sharp direction changes.

            return (
                distance
                - 1.5 * angle_error
            )

        best = max(
            candidates,
            key=score
        )

        new_x = best[0]
        new_y = best[1]

        # ======================================================
        # Prevent unnecessary target switching
        # ======================================================

        if (
            self.target_x is not None
            and self.target_y is not None
        ):

            target_change = math.sqrt(
                (new_x - self.target_x) ** 2 +
                (new_y - self.target_y) ** 2
            )

            if target_change < 0.5:

                return

        self.target_x = new_x
        self.target_y = new_y
        self.target_best_distance = best[2]
        self.last_target_progress_time = now
        self.has_selected_target = True

        self.get_logger().info(
            f'New exploration target: '
            f'x={new_x:.2f}, '
            f'y={new_y:.2f}, '
            f'distance={best[2]:.2f} m'
        )

    # ==========================================================
    # Navigation
    # ==========================================================

    def control_loop(self):

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        if self.exploration_complete:
            self.cmd_pub.publish(cmd)
            return

        # No LiDAR
        if self.scan is None:

            self.cmd_pub.publish(cmd)

            return

        # No target
        if (
            self.target_x is None
            or self.target_y is None
        ):

            if not self.has_selected_target:
                self.bootstrap_explore(cmd)

            self.cmd_pub.publish(cmd)

            return

        pose = self.get_robot_pose()

        if pose is None:

            self.cmd_pub.publish(cmd)

            return

        robot_x, robot_y, robot_yaw = pose

        # ======================================================
        # Distance and direction
        # ======================================================

        dx = self.target_x - robot_x
        dy = self.target_y - robot_y

        distance = math.sqrt(
            dx * dx +
            dy * dy
        )

        now = self.get_clock().now()

        if (
            self.target_best_distance is None
            or distance <= (
                self.target_best_distance - self.target_progress_distance
            )
        ):
            self.target_best_distance = distance
            self.last_target_progress_time = now

        elif (
            self.last_target_progress_time is not None
            and (
                (now - self.last_target_progress_time).nanoseconds / 1e9
                >= self.target_stall_seconds
            )
        ):
            self.failed_targets.append((self.target_x, self.target_y))
            self.target_x = None
            self.target_y = None
            self.target_best_distance = None
            self.last_target_progress_time = None

            self.get_logger().warn(
                'Target made no progress for '
                f'{self.target_stall_seconds:.0f} seconds; selecting another '
                'frontier.'
            )

            self.cmd_pub.publish(cmd)
            return

        target_angle = math.atan2(
            dy,
            dx
        )

        angle_error = math.atan2(
            math.sin(
                target_angle - robot_yaw
            ),
            math.cos(
                target_angle - robot_yaw
            )
        )

        # ======================================================
        # Goal reached
        # ======================================================

        if distance < self.goal_tolerance:

            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                'Exploration target reached'
            )

            # Force selection of another frontier
            self.target_x = None
            self.target_y = None
            self.target_best_distance = None
            self.last_target_progress_time = None

            return

        # ======================================================
        # LiDAR safety
        # ======================================================

        front = self.sector_min(
            math.radians(-25),
            math.radians(25)
        )

        left = self.sector_min(
            math.radians(35),
            math.radians(90)
        )

        right = self.sector_min(
            math.radians(-90),
            math.radians(-35)
        )

        # ======================================================
        # Emergency obstacle
        # ======================================================

        if front < self.min_front_distance:

            cmd.twist.linear.x = 0.0

            # Turn toward the side with more space

            if left > right:

                cmd.twist.angular.z = (
                    self.turn_speed
                )

            else:

                cmd.twist.angular.z = (
                    -self.turn_speed
                )

            self.cmd_pub.publish(cmd)

            return

        # ======================================================
        # Large heading error
        # ======================================================

        if abs(angle_error) > 0.35:

            cmd.twist.linear.x = 0.0

            if angle_error > 0:

                cmd.twist.angular.z = (
                    self.turn_speed
                )

            else:

                cmd.twist.angular.z = (
                    -self.turn_speed
                )

        # ======================================================
        # Move toward target
        # ======================================================

        else:

            cmd.twist.linear.x = (
                self.linear_speed
            )

            # Small heading correction

            cmd.twist.angular.z = (
                0.8 * angle_error
            )

        # ======================================================
        # Additional side safety
        # ======================================================

        if left < self.safe_side_distance:

            if cmd.twist.angular.z > 0:

                cmd.twist.angular.z = 0.0

        if right < self.safe_side_distance:

            if cmd.twist.angular.z < 0:

                cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)


# ==============================================================
# Main
# ==============================================================

def main(args=None):

    rclpy.init(args=args)

    node = AutonomousExploration()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        stop = TwistStamped()

        stop.header.stamp = (
            node.get_clock().now().to_msg()
        )

        stop.twist.linear.x = 0.0
        stop.twist.angular.z = 0.0

        node.cmd_pub.publish(stop)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
