"""Frontier exploration that drives with Nav2.

Finds frontiers (free cells next to unknown cells) in the SLAM /map, sends
the best one to Nav2 as a NavigateToPose goal, and repeats until no
frontier is left. Nav2 plans through narrow gaps such as the room doorways,
which the reactive autonomous_exploration node cannot do.
"""

import math
import subprocess

import numpy as np

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2Exploration(Node):
    """Send frontier goals to Nav2 until the map is fully explored."""

    def __init__(self):
        super().__init__('nav2_exploration')

        # -----------------------------
        # Parameters
        # -----------------------------

        self.declare_parameter('min_frontier_size', 8)
        self.declare_parameter('min_goal_distance', 0.6)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('blacklist_radius', 0.6)
        self.declare_parameter('no_frontier_seconds', 10.0)
        self.declare_parameter('max_failures', 30)
        # Optional: save the finished map to this path (no extension).
        self.declare_parameter('map_save_path', '')

        self.min_frontier_size = self.get_parameter(
            'min_frontier_size').value
        self.min_goal_distance = self.get_parameter(
            'min_goal_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.blacklist_radius = self.get_parameter(
            'blacklist_radius').value
        self.no_frontier_seconds = self.get_parameter(
            'no_frontier_seconds').value
        self.max_failures = self.get_parameter('max_failures').value
        self.map_save_path = self.get_parameter('map_save_path').value

        # -----------------------------
        # Interfaces
        # -----------------------------

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        completion_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.completion_pub = self.create_publisher(
            Bool, '/exploration_complete', completion_qos)

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # State
        # -----------------------------

        self.map_msg = None
        self.blacklist = []
        self.failures = 0
        self.complete = False

        self.goal_handle = None
        self.goal_xy = None
        self.goal_start = None
        self.goal_count = 0
        self.sending_goal = False
        self.no_frontier_since = None

        self.publish_completion(False)
        self.timer = self.create_timer(2.0, self.step)
        self.get_logger().info('Nav2 frontier exploration started')

    # ==========================================================
    # Helpers
    # ==========================================================

    def publish_completion(self, done):
        """Publish the latched exploration status."""
        msg = Bool()
        msg.data = done
        self.completion_pub.publish(msg)

    def map_callback(self, msg):
        """Store the latest SLAM map."""
        self.map_msg = msg

    def robot_pose(self):
        """Return (x, y) of base_link in the map frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def now_seconds(self):
        """Return the node clock (sim time) in seconds."""
        return self.get_clock().now().nanoseconds / 1e9

    def is_blacklisted(self, x, y):
        """Return True if (x, y) is near a failed or visited goal."""
        return any(
            math.hypot(x - bx, y - by) < self.blacklist_radius
            for bx, by in self.blacklist
        )

    # ==========================================================
    # Frontier detection
    # ==========================================================

    def frontier_mask(self):
        """Return a boolean grid of free cells that touch unknown cells."""
        info = self.map_msg.info
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(
            info.height, info.width)

        free = grid == 0
        unknown = grid < 0

        near_unknown = np.zeros_like(unknown)
        near_unknown[1:, :] |= unknown[:-1, :]
        near_unknown[:-1, :] |= unknown[1:, :]
        near_unknown[:, 1:] |= unknown[:, :-1]
        near_unknown[:, :-1] |= unknown[:, 1:]

        return free & near_unknown

    def frontier_regions(self, mask):
        """Group frontier cells into 8-connected regions (lists of cells)."""
        cells = {(int(r), int(c)) for r, c in np.argwhere(mask)}
        regions = []

        while cells:
            seed = cells.pop()
            region = [seed]
            stack = [seed]
            while stack:
                r, c = stack.pop()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nb = (r + dr, c + dc)
                        if nb in cells:
                            cells.remove(nb)
                            region.append(nb)
                            stack.append(nb)
            if len(region) >= self.min_frontier_size:
                regions.append(region)

        return regions

    def cell_to_world(self, row, col):
        """Convert a grid cell to map-frame coordinates (cell centre)."""
        info = self.map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def candidate_goals(self, robot_xy):
        """Return [(distance, x, y, size)] for usable frontier regions."""
        candidates = []

        for region in self.frontier_regions(self.frontier_mask()):
            rows = [r for r, _ in region]
            cols = [c for _, c in region]
            mean_r = sum(rows) / len(rows)
            mean_c = sum(cols) / len(cols)

            # Use the real frontier cell closest to the region centre, so
            # the goal is always a known free cell.
            row, col = min(
                region,
                key=lambda rc: (rc[0] - mean_r) ** 2 + (rc[1] - mean_c) ** 2)
            x, y = self.cell_to_world(row, col)

            distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
            if distance < self.min_goal_distance:
                continue
            if self.is_blacklisted(x, y):
                continue

            candidates.append((distance, x, y, len(region)))

        return candidates

    def goal_still_frontier(self):
        """Return True if the current goal still borders unknown space."""
        if self.goal_xy is None or self.map_msg is None:
            return False

        info = self.map_msg.info
        col = int((self.goal_xy[0] - info.origin.position.x)
                  / info.resolution)
        row = int((self.goal_xy[1] - info.origin.position.y)
                  / info.resolution)

        mask = self.frontier_mask()
        r0, r1 = max(row - 3, 0), min(row + 4, info.height)
        c0, c1 = max(col - 3, 0), min(col + 4, info.width)
        return bool(mask[r0:r1, c0:c1].any())

    # ==========================================================
    # Goal handling
    # ==========================================================

    def send_goal(self, x, y, robot_xy):
        """Send a NavigateToPose goal facing away from the robot."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        yaw = math.atan2(y - robot_xy[1], x - robot_xy[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_xy = (x, y)
        self.goal_start = self.now_seconds()
        self.goal_count += 1
        self.sending_goal = True

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

        self.get_logger().info(
            f'Goal {self.goal_count}: x={x:.2f}, y={y:.2f}')

    def goal_response(self, future):
        """Handle Nav2 accepting or rejecting the goal."""
        self.sending_goal = False
        handle = future.result()

        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.finish_goal(success=False)
            return

        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        """Handle Nav2 finishing a goal."""
        status = future.result().status
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.finish_goal(success=True)
        elif status == 5:
            self.finish_goal(success=None)
        else:
            self.get_logger().warn(f'Goal failed (status {status})')
            self.finish_goal(success=False)

    def finish_goal(self, success):
        """Clear the active goal; blacklist it unless it was cancelled."""
        if self.goal_xy is not None and success is not None:
            self.blacklist.append(self.goal_xy)
        if success is False:
            self.failures += 1

        self.goal_handle = None
        self.goal_xy = None
        self.goal_start = None

    def cancel_goal(self, reason, blacklist=False):
        """Cancel the active goal."""
        if self.goal_handle is None:
            return
        self.get_logger().info(f'Cancelling goal: {reason}')
        if blacklist and self.goal_xy is not None:
            self.blacklist.append(self.goal_xy)
        self.goal_handle.cancel_goal_async()

    # ==========================================================
    # Main loop
    # ==========================================================

    def step(self):
        """Pick a new goal when idle; cancel stale or stuck goals."""
        if self.complete or self.map_msg is None:
            return

        robot_xy = self.robot_pose()
        if robot_xy is None:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().info(
                'Waiting for Nav2 navigate_to_pose action server...',
                throttle_duration_sec=10.0)
            return

        # ---- A goal is active ----------------------------------
        if self.goal_xy is not None:
            if self.sending_goal or self.goal_handle is None:
                return

            elapsed = self.now_seconds() - self.goal_start
            if elapsed > self.goal_timeout:
                self.cancel_goal('timed out', blacklist=True)
                self.failures += 1
            elif not self.goal_still_frontier():
                self.cancel_goal('frontier already explored')
            return

        # ---- Idle: choose the next frontier --------------------
        if self.failures >= self.max_failures:
            self.finish_exploration(
                f'too many failed goals ({self.failures}).')
            return

        candidates = self.candidate_goals(robot_xy)

        if not candidates:
            now = self.now_seconds()
            if self.no_frontier_since is None:
                self.no_frontier_since = now
                self.get_logger().info(
                    'No frontier found; waiting for map updates.')
            elif now - self.no_frontier_since >= self.no_frontier_seconds:
                if self.goal_count == 0:
                    # Nothing explored yet: SLAM may still be starting.
                    self.no_frontier_since = None
                else:
                    self.finish_exploration('no frontiers remain.')
            return

        self.no_frontier_since = None

        # Nearest frontier first; slightly prefer larger regions.
        distance, x, y, size = min(
            candidates, key=lambda c: c[0] - 0.01 * min(c[3], 50))
        self.send_goal(x, y, robot_xy)

    def finish_exploration(self, reason):
        """Announce completion and optionally save the map."""
        self.complete = True
        self.publish_completion(True)
        self.get_logger().info(
            f'Exploration completed: {reason} '
            f'({self.goal_count} goals, {self.failures} failed)')

        if self.map_save_path:
            self.get_logger().info(
                f'Saving map to {self.map_save_path}.pgm/.yaml')
            subprocess.Popen([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', self.map_save_path,
                '--ros-args', '-p', 'use_sim_time:=true',
            ])


def main(args=None):
    """Run the exploration node."""
    rclpy.init(args=args)
    node = Nav2Exploration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_goal('shutting down')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
