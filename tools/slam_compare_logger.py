"""Log wheel odometry, LiDAR SLAM and visual SLAM poses next to Gazebo ground truth.

Run while the simulation, slam.launch.py (or bringup mode:=slam) and
visual_slam.launch.py are up. Every --period seconds it stores one sample;
stop it with Ctrl-C or by creating the file given with --stop-file.
Output: a JSON list of samples (x, y, yaw per source, in the map frame).
"""

import argparse
import json
import math
import os
import re
import subprocess

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

WORLD_POSE_TOPIC = '/world/delivery_warehouse/dynamic_pose/info'


def yaw_of(z, w):
    """Yaw from the z and w quaternion components (planar motion)."""
    return 2.0 * math.atan2(z, w)


def ground_truth():
    """Return (x, y, yaw) of the robot from Gazebo, or None."""
    try:
        text = subprocess.run(
            ['gz', 'topic', '-e', '-n', '1', '-t', WORLD_POSE_TOPIC],
            capture_output=True, text=True, timeout=10).stdout
    except subprocess.TimeoutExpired:
        return None
    i = text.find('name: "delivery_robot"')
    if i < 0:
        return None
    block = text[i:i + 600]
    num = r'(-?\d+\.?\d*(?:[eE][-+]?\d+)?)'
    x = re.search(r'x: ' + num, block)
    y = re.search(r'y: ' + num, block)
    oz = re.search(r'orientation \{[^}]*?z: ' + num, block, re.S)
    ow = re.search(r'orientation \{[^}]*?w: ' + num, block, re.S)
    if not (x and y):
        return None
    z = float(oz.group(1)) if oz else 0.0
    w = float(ow.group(1)) if ow else 1.0
    return float(x.group(1)), float(y.group(1)), yaw_of(z, w)


class Logger(Node):
    """Collect poses from every source."""

    def __init__(self):
        super().__init__('slam_compare_logger')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.wheel = None
        self.vo = None
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom',
            lambda m: setattr(self, 'wheel', m), 10)
        self.create_subscription(
            Odometry, '/vo', lambda m: setattr(self, 'vo', m), 10)
        self.image = None
        self.create_subscription(
            Image, '/camera/image', lambda m: setattr(self, 'image', m), 1)

    def save_image(self, path):
        """Write the latest camera image as a PNG (for debugging)."""
        if self.image is None:
            return
        import cv2
        m = self.image
        a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(
            m.height, m.width, 3)
        cv2.imwrite(path, a[:, :, ::-1])

    def pose_of(self, msg):
        """x, y, yaw of an Odometry message."""
        p = msg.pose.pose
        return (p.position.x, p.position.y,
                yaw_of(p.orientation.z, p.orientation.w))

    def tf_pose(self, parent, child):
        """x, y, yaw of child in parent, or None."""
        try:
            t = self.tf.lookup_transform(parent, child, Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, yaw_of(q.z, q.w)

    def visual_pose_in_map(self):
        """Compose map->vo_odom (rtabmap) with the /vo pose."""
        if self.vo is None:
            return None
        m = self.tf_pose('map', 'vo_odom')
        if m is None:
            return None
        x, y, yaw = self.pose_of(self.vo)
        c, s = math.cos(m[2]), math.sin(m[2])
        return (m[0] + c * x - s * y, m[1] + s * x + c * y, m[2] + yaw)

    def sample(self):
        """One row with every source (None if unavailable)."""
        return {
            't': self.get_clock().now().nanoseconds / 1e9,
            'truth': ground_truth(),
            'wheel': self.pose_of(self.wheel) if self.wheel else None,
            'lidar_slam': self.tf_pose('map', 'base_link'),
            'visual_slam': self.visual_pose_in_map(),
        }


def main():
    """Log until the stop file appears."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--period', type=float, default=2.0)
    parser.add_argument('--stop-file', default='')
    parser.add_argument('--image-dir', default='',
                        help='also save a camera image per sample')
    opts = parser.parse_args()

    rclpy.init()
    node = Logger()
    rows = []
    try:
        while rclpy.ok():
            if opts.stop_file and os.path.exists(opts.stop_file):
                break
            end = node.get_clock().now().nanoseconds / 1e9 + opts.period
            while node.get_clock().now().nanoseconds / 1e9 < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            rows.append(node.sample())
            if opts.image_dir:
                os.makedirs(opts.image_dir, exist_ok=True)
                node.save_image(os.path.join(
                    opts.image_dir, f'{len(rows) - 1:03d}.png'))
    except KeyboardInterrupt:
        pass
    finally:
        with open(opts.out, 'w') as f:
            json.dump(rows, f)
        print(f'logged {len(rows)} samples to {opts.out}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
