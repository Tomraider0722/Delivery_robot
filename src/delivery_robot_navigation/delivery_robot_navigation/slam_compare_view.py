"""Live side-by-side view of LiDAR SLAM and visual SLAM.

Three panels: the camera image, the LiDAR SLAM map (/map) and the visual
SLAM map (/vslam/map). Each map panel shows the robot and the path it
estimated for itself, so you can see where the two methods differ: the
LiDAR map is complete and steady, the visual map only fills in where the
camera has looked, and its path can jump when tracking is lost.

Start the simulation with mode:=slam and visual_slam.launch.py first.

    ros2 run delivery_robot_navigation slam_compare_view
    ros2 run delivery_robot_navigation slam_compare_view --snapshot /tmp/view.png --no-window

The window is only a viewer; it publishes nothing.
"""

import argparse
import math
import sys

import cv2
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

# World window shown in the map panels (metres) and its scale.
X_MIN, X_MAX = -6.6, 6.6
Y_MIN, Y_MAX = -5.6, 5.6
PX_PER_M = 50
PANEL_W = int((X_MAX - X_MIN) * PX_PER_M)
PANEL_H = int((Y_MAX - Y_MIN) * PX_PER_M)
HEADER = 34
LOST_COVARIANCE = 1000.0


def yaw_of(q):
    """Yaw angle of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_px(x, y):
    """World (x, y) in metres to a pixel in a map panel."""
    return (int((x - X_MIN) * PX_PER_M), int((Y_MAX - y) * PX_PER_M))


def render_map(grid):
    """Draw an OccupancyGrid into a panel image (grey = unknown)."""
    panel = np.full((PANEL_H, PANEL_W, 3), 205, np.uint8)
    if grid is None:
        return panel
    w, h = grid.info.width, grid.info.height
    data = np.array(grid.data, np.int8).reshape(h, w)
    img = np.full((h, w), 205, np.uint8)
    img[data == 0] = 255
    img[data > 50] = 0
    img = np.flipud(img)                       # row 0 = top of the map
    scale = grid.info.resolution * PX_PER_M
    new_w, new_h = max(1, int(w * scale)), max(1, int(h * scale))
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    # pixel of the map's top-left corner
    x0, y0 = to_px(grid.info.origin.position.x,
                   grid.info.origin.position.y + h * grid.info.resolution)
    sx, sy = max(0, -x0), max(0, -y0)
    ex, ey = min(new_w, PANEL_W - x0), min(new_h, PANEL_H - y0)
    if ex > sx and ey > sy:
        panel[y0 + sy:y0 + ey, x0 + sx:x0 + ex] = img[sy:ey, sx:ex, None]
    return panel


def draw_robot(panel, path, color):
    """Draw the estimated path and the robot (circle with heading)."""
    if len(path) > 1:
        pts = np.array([to_px(x, y) for x, y, _ in path], np.int32)
        cv2.polylines(panel, [pts], False, color, 2, cv2.LINE_AA)
    if path:
        x, y, yaw = path[-1]
        cx, cy = to_px(x, y)
        cv2.circle(panel, (cx, cy), 8, color, -1, cv2.LINE_AA)
        cv2.line(panel, (cx, cy),
                 (int(cx + 18 * math.cos(yaw)), int(cy - 18 * math.sin(yaw))),
                 (0, 0, 0), 2, cv2.LINE_AA)


def header(width, text, color):
    """Title bar above a panel."""
    bar = np.full((HEADER, width, 3), 40, np.uint8)
    cv2.putText(bar, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1,
                cv2.LINE_AA)
    return bar


class CompareView(Node):
    """Collect maps, poses and the camera image, and compose the view."""

    def __init__(self):
        super().__init__(
            'slam_compare_view',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lidar_map = None
        self.visual_map = None
        self.image = None
        self.lidar_path = []
        self.visual_path = []
        self.lost = True
        self.tracking_lost_frames = 0
        self.frames = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.on_lidar_map,
                                 map_qos)
        self.create_subscription(OccupancyGrid, '/vslam/map',
                                 self.on_visual_map, map_qos)
        self.create_subscription(Image, '/camera/image', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vo', self.on_vo, 10)
        self.create_timer(0.2, self.on_lidar_pose)

    def on_lidar_map(self, msg):
        """Keep the latest LiDAR map."""
        self.lidar_map = msg

    def on_visual_map(self, msg):
        """Keep the latest visual map."""
        self.visual_map = msg

    def on_image(self, msg):
        """Keep the latest camera image (RGB8)."""
        arr = np.frombuffer(bytes(msg.data), np.uint8)
        self.image = arr.reshape(msg.height, msg.width, 3)

    def on_lidar_pose(self):
        """Robot pose from the LiDAR SLAM TF (map to base_link)."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link',
                                                rclpy.time.Time())
        except Exception:  # transform not available yet
            return
        tr = t.transform
        self.lidar_path.append(
            (tr.translation.x, tr.translation.y, yaw_of(tr.rotation)))

    def on_vo(self, msg):
        """Camera-only pose: map to vo_odom (from RTAB-Map) times /vo."""
        self.frames += 1
        self.lost = msg.pose.covariance[0] > LOST_COVARIANCE
        if self.lost:
            self.tracking_lost_frames += 1
            return
        try:
            t = self.tf_buffer.lookup_transform('map', msg.header.frame_id,
                                                rclpy.time.Time())
        except Exception:  # RTAB-Map has not published map->vo_odom yet
            return
        a = t.transform
        ay = yaw_of(a.rotation)
        p = msg.pose.pose
        x = a.translation.x + math.cos(ay) * p.position.x \
            - math.sin(ay) * p.position.y
        y = a.translation.y + math.sin(ay) * p.position.x \
            + math.cos(ay) * p.position.y
        self.visual_path.append((x, y, ay + yaw_of(p.orientation)))

    def compose(self):
        """Build the side-by-side image (BGR)."""
        if self.image is not None:
            cam = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
        else:
            cam = np.full((480, 640, 3), 60, np.uint8)
        cam = cv2.resize(cam, (int(PANEL_H * 640 / 480), PANEL_H))
        cam_panel = np.vstack([header(cam.shape[1], 'Robot camera',
                                      (255, 255, 255)), cam])

        lidar = render_map(self.lidar_map)
        draw_robot(lidar, self.lidar_path, (60, 160, 40))
        lidar_panel = np.vstack([header(
            PANEL_W, 'LiDAR SLAM (laser)', (120, 255, 120)),
            lidar])

        visual = render_map(self.visual_map)
        draw_robot(visual, self.visual_path, (30, 120, 255))
        state = 'TRACKING LOST' if self.lost else 'tracking'
        lost_pct = (100.0 * self.tracking_lost_frames / self.frames
                    if self.frames else 0.0)
        label = f'Visual SLAM (camera only)  {state}  lost {lost_pct:.0f}%'
        visual_panel = np.vstack([header(
            PANEL_W, label, (80, 80, 255) if self.lost else (80, 200, 255)),
            visual])

        return np.hstack([cam_panel, lidar_panel, visual_panel])


def main(args=None):
    """Show the view in a window and/or save snapshots."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--snapshot', help='also save the view to this PNG '
                        'every 2 s')
    parser.add_argument('--no-window', action='store_true',
                        help='do not open a window (snapshots only)')
    opts = parser.parse_args(remove_ros_args(args if args else sys.argv)[1:])

    rclpy.init(args=args)
    node = CompareView()
    scale = 0.75
    try:
        last_draw = last_snap = -1e9
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = node.get_clock().now().nanoseconds / 1e9
            if now - last_draw < 0.25:
                continue
            last_draw = now
            view = node.compose()
            if opts.snapshot and now - last_snap >= 2.0:
                last_snap = now
                cv2.imwrite(opts.snapshot, view)
            if not opts.no_window:
                small = cv2.resize(view, None, fx=scale, fy=scale,
                                   interpolation=cv2.INTER_AREA)
                cv2.imshow('LiDAR SLAM vs visual SLAM', small)
                if cv2.waitKey(1) in (27, ord('q')):
                    break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
