"""Deliver to one or more named locations and return to the dock.

Usage:
    ros2 run delivery_robot_controller deliver room2
    ros2 run delivery_robot_controller deliver room1 room2 --confirm
    ros2 run delivery_robot_controller deliver --list
    ros2 run delivery_robot_controller deliver room1 --wait 5 --no-return

Several destinations are visited in the order given. With --confirm the
robot waits at each stop until someone presses Enter in this terminal or
calls the service /delivery/confirm (std_srvs/Trigger), then moves on; it
gives up waiting after --confirm-timeout seconds. A stop that cannot be
reached is skipped and reported at the end.

Needs the simulation, localization and Nav2 running (see CLAUDE.md).
Locations come from config/locations.yaml (or --locations FILE).
"""

import argparse
import math
import os
import select
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger
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

    def __init__(self, **kwargs):
        super().__init__('deliver', **kwargs)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.last_log = 0.0
        self.confirmed = False
        self.cancel_requested = False
        self.on_status = None
        self.create_service(Trigger, '/delivery/confirm', self.on_confirm)

    def say(self, text):
        """Log a status line and pass it to the panel, if there is one."""
        self.get_logger().info(text)
        if self.on_status:
            self.on_status(text)

    def cancel(self):
        """Stop the mission in progress (called from another thread)."""
        self.cancel_requested = True

    def on_confirm(self, request, response):
        """Service callback: the cargo has been taken."""
        self.confirmed = True
        response.success = True
        response.message = 'Delivery confirmed'
        return response

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

        self.say(f'Going to {name}')

        send = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error(f'Nav2 rejected the goal for {name}')
            return False

        result = handle.get_result_async()
        while not result.done():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.cancel_requested:
                handle.cancel_goal_async()
        status = result.result().status

        if status == STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrived at {name}')
            return True

        if self.cancel_requested:
            self.get_logger().info('Cancelled')
            return False

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
        while (self.get_clock().now().nanoseconds / 1e9 < end
               and not self.cancel_requested):
            rclpy.spin_once(self, timeout_sec=0.2)

    def wait_for_confirmation(self, timeout):
        """Wait for Enter or /delivery/confirm; return True if confirmed."""
        self.confirmed = False
        self.say('Waiting for confirmation (Enter, /delivery/confirm or '
                 'the Delivered button)')
        end = self.get_clock().now().nanoseconds / 1e9 + timeout
        while not self.confirmed:
            if (self.cancel_requested
                    or self.get_clock().now().nanoseconds / 1e9 >= end):
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
            if sys.stdin.isatty() and select.select(
                    [sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                self.confirmed = True
        return True

    def run_mission(self, names, locations, home, no_return, confirm,
                    wait, confirm_timeout):
        """Visit the stops in order; return (delivered, failed, returned)."""
        self.cancel_requested = False
        delivered, failed = [], []
        for name in names:
            if self.cancel_requested:
                break
            if not self.go_to(name, locations[name]):
                if not self.cancel_requested:
                    failed.append(name)
                continue
            delivered.append(name)
            if confirm:
                if self.wait_for_confirmation(confirm_timeout):
                    self.say(f'Confirmed at {name}')
                else:
                    self.get_logger().warn(
                        f'No confirmation at {name}; moving on')
            else:
                self.say(f'Arrived at {name}; waiting {wait:.0f} s')
                self.wait(wait)

        returned = True
        if not no_return and not self.cancel_requested:
            returned = self.go_to(home, locations[home])
            if not returned and not self.cancel_requested:
                self.get_logger().error('Could not return to the dock')
        return delivered, failed, returned


def main(args=None):
    """Run a delivery from the command line."""
    default_file = os.path.join(
        get_package_share_directory('delivery_robot_controller'),
        'config', 'locations.yaml')

    parser = argparse.ArgumentParser(
        description='Deliver to one or more named locations')
    parser.add_argument('destinations', nargs='*', help='location names')
    parser.add_argument('--list', action='store_true',
                        help='list known locations and exit')
    parser.add_argument('--wait', type=float, default=3.0,
                        help='seconds to wait at each stop (default 3)')
    parser.add_argument('--confirm', action='store_true',
                        help='wait for Enter or /delivery/confirm at each '
                             'stop instead of a fixed time')
    parser.add_argument('--confirm-timeout', type=float, default=120.0,
                        help='seconds to wait for confirmation (default 120)')
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
            print(f'{name}: x={loc["x"]}, y={loc["y"]}, '
                  f'yaw={loc.get("yaw", 0.0)}')
        return 0

    if not opts.destinations:
        print('Give at least one destination, or use --list')
        return 2
    unknown = [d for d in opts.destinations if d not in locations]
    if unknown:
        known = ', '.join(locations)
        print(f'Unknown destination "{unknown[0]}". Known: {known}')
        return 2
    if not opts.no_return and opts.home not in locations:
        print(f'Unknown home location "{opts.home}"')
        return 2

    rclpy.init(args=args)
    node = Deliver()
    log = node.get_logger()
    code = 1

    try:
        log.info('Waiting for Nav2...')
        if not node.client.wait_for_server(timeout_sec=30.0):
            log.error(
                'navigate_to_pose is not available. Start the simulation, '
                'localization and navigation first.')
            return code

        delivered, failed, returned = node.run_mission(
            opts.destinations, locations, opts.home, opts.no_return,
            opts.confirm, opts.wait, opts.confirm_timeout)
        log.info(f'Summary: delivered {delivered or "nothing"}, '
                 f'failed {failed or "none"}')
        code = 0 if delivered and not failed and returned else 1
    except KeyboardInterrupt:
        log.info('Interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
