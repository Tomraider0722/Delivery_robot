"""Deliver to a named location and return to the dock.

Usage:
    ros2 run delivery_robot_controller deliver room2
    ros2 run delivery_robot_controller deliver --list
    ros2 run delivery_robot_controller deliver room1 --wait 5 --no-return

Needs the simulation, localization and Nav2 running (see CLAUDE.md).
Locations come from config/locations.yaml (or --locations FILE).
"""

import argparse
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml

# action_msgs/GoalStatus values
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5


def load_locations(path):
    """Read the named locations from a yaml file."""
    with open(path) as f:
        return yaml.safe_load(f)['locations']


class Deliver(Node):
    """Send NavigateToPose goals for a delivery run."""

    def __init__(self):
        super().__init__('deliver')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.last_log = 0.0

    def go_to(self, name, loc):
        """Drive to a location; return True when Nav2 reports success."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc['x'])
        goal.pose.pose.position.y = float(loc['y'])
        yaw = float(loc.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Going to {name} ({loc["x"]:.1f}, {loc["y"]:.1f})')

        send = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error(f'Nav2 rejected the goal for {name}')
            return False

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        status = result.result().status

        if status == STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrived at {name}')
            return True

        self.get_logger().error(
            f'Could not reach {name} (status {status}: '
            f'{"canceled" if status == STATUS_CANCELED else "aborted"})')
        return False

    def feedback(self, msg):
        """Log the distance left every few seconds."""
        remaining = msg.feedback.distance_remaining
        now = self.get_clock().now().nanoseconds / 1e9
        # distance_remaining is 0 until Nav2 has planned a path.
        if remaining > 0.0 and now - self.last_log >= 5.0:
            self.last_log = now
            self.get_logger().info(f'  {remaining:.1f} m to go')

    def wait(self, seconds):
        """Wait (e.g. while the cargo is unloaded), still spinning ROS."""
        end = self.get_clock().now().nanoseconds / 1e9 + seconds
        while self.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(self, timeout_sec=0.2)


def main(args=None):
    """Run a delivery from the command line."""
    default_file = os.path.join(
        get_package_share_directory('delivery_robot_controller'),
        'config', 'locations.yaml')

    parser = argparse.ArgumentParser(description='Deliver to a named location')
    parser.add_argument('destination', nargs='?', help='location name')
    parser.add_argument('--list', action='store_true',
                        help='list known locations and exit')
    parser.add_argument('--wait', type=float, default=3.0,
                        help='seconds to wait at the destination (default 3)')
    parser.add_argument('--no-return', action='store_true',
                        help='stay at the destination instead of returning')
    parser.add_argument('--home', default='dock',
                        help='location to return to (default dock)')
    parser.add_argument('--locations', default=default_file,
                        help='locations yaml file')
    opts = parser.parse_args(remove_ros_args(args if args else sys.argv)[1:])

    locations = load_locations(opts.locations)

    if opts.list:
        for name, loc in locations.items():
            print(f'{name}: x={loc["x"]}, y={loc["y"]}, yaw={loc.get("yaw", 0.0)}')
        return 0

    if opts.destination not in locations:
        known = ', '.join(locations)
        print(f'Unknown destination "{opts.destination}". Known: {known}')
        return 2
    if not opts.no_return and opts.home not in locations:
        print(f'Unknown home location "{opts.home}"')
        return 2

    rclpy.init(args=args)
    node = Deliver()
    code = 1

    try:
        node.get_logger().info('Waiting for Nav2...')
        if not node.client.wait_for_server(timeout_sec=30.0):
            node.get_logger().error(
                'navigate_to_pose is not available. Start the simulation, '
                'localization and navigation first.')
        elif node.go_to(opts.destination, locations[opts.destination]):
            node.get_logger().info(
                f'Delivered to {opts.destination}; waiting {opts.wait:.0f} s')
            node.wait(opts.wait)
            if opts.no_return:
                code = 0
            elif node.go_to(opts.home, locations[opts.home]):
                node.get_logger().info('Delivery complete')
                code = 0
            else:
                node.get_logger().error('Delivered, but could not return')
        else:
            node.get_logger().error('Delivery failed')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
