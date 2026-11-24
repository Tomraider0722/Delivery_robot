"""Carry objects between a workspace and a storage room.

Usage:
    ros2 run delivery_robot_controller courier table1 room1
    ros2 run delivery_robot_controller courier table1 room1 --bring-back
    ros2 run delivery_robot_controller courier table1 room1 --bring-back --sim-people

The first place is the workspace (table1 or table2), the second the storage
room (room1 or room2, with the racks). The robot works like this:

  1. It goes to the workspace and waits until someone puts a finished object
     on its deck (its load sensor sees it).
  2. It drives to the storage room and waits until the object is taken off.
  3. With --bring-back it then waits until someone puts an object from the
     racks on its deck, drives back to the workspace, and waits until that
     object is taken off.
  4. It returns to the dock.

--sim-people makes a helper play the people, using the simulation (an object
appears on the deck after a few seconds at each pick-up place and disappears
after a few seconds at each drop-off place). Without it, use
`ros2 run delivery_robot_perception cargo_tool load|take` or the Delivery
window's Load and Take buttons, or put the object there yourself.

Needs the simulation, localization, Nav2 and the load monitor running
(bringup.launch.py starts them all).
"""

import argparse
import os
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from delivery_robot_controller.deliver import Deliver, load_locations
import rclpy
from rclpy.utilities import remove_ros_args


class Courier(Deliver):
    """A delivery node that also waits for people to load and unload it."""

    def __init__(self, **kwargs):
        super().__init__(name='courier', **kwargs)
        self.sim_people = False
        self.helper_delay = 6.0

    def settle(self, seconds=3.0):
        """Spin for a moment so the clock, TF and the load state arrive."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for(self, wanted, timeout):
        """Wait until the load state equals `wanted`; False on timeout."""
        end = time.monotonic() + timeout
        while self.cargo_present != wanted:
            if self.cancel_requested or time.monotonic() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        return True

    def person(self, action):
        """Play the person after a few seconds (simulation only)."""
        if not self.sim_people:
            return

        def work():
            from delivery_robot_perception import cargo_tool
            time.sleep(self.helper_delay)
            if action == 'load':
                self.say('(simulation) someone puts an object on the robot')
                cargo_tool.load_cargo()
            else:
                self.say('(simulation) someone takes the object off')
                cargo_tool.take_cargo()
        threading.Thread(target=work, daemon=True).start()

    def pick_up(self, place, what, timeout):
        """Wait at a place until an object has been put on the robot."""
        if self.cargo_present:
            self.say('There is already an object on the robot')
            return True
        self.say(f'At {place}. Waiting for {what} to be put on the robot')
        self.person('load')
        if self.wait_for(True, timeout):
            self.say('Object loaded')
            return True
        self.say(f'Nothing was put on the robot at {place}')
        return False

    def drop_off(self, place, timeout):
        """Wait at a place until the object has been taken off the robot."""
        self.say(f'At {place}. Please take the object off the robot')
        self.person('take')
        if self.wait_for(False, timeout):
            self.say('Object taken')
            return True
        self.say(f'The object was not taken off at {place}')
        return False

    def run_courier(self, workspace, storage, bring_back, no_return,
                    locations, home, timeout):
        """Do the whole job; return True when everything was delivered."""
        self.settle()
        if self.cargo_present is None:
            self.get_logger().error(
                'No load sensor reading (/cargo_present). Is the load '
                'monitor running?')
            return False
        ok = False
        try:
            self.say(f'Task: {workspace} to {storage}'
                     + (' and back' if bring_back else ''))
            if not self.go_to(workspace, locations[workspace]):
                return False
            if not self.pick_up(workspace, 'the finished object', timeout):
                return False
            self.say(f'Taking it to {storage}')
            if not self.go_to(storage, locations[storage]):
                return False
            if not self.drop_off(storage, timeout):
                return False
            if bring_back:
                if not self.pick_up(storage, 'an object from the racks',
                                    timeout):
                    return False
                self.say(f'Taking it to {workspace}')
                if not self.go_to(workspace, locations[workspace]):
                    return False
                if not self.drop_off(workspace, timeout):
                    return False
            ok = True
        finally:
            if not no_return and not self.cancel_requested:
                self.go_to(home, locations[home])
        self.say('Task done' if ok else 'Task failed')
        return ok


def main(args=None):
    """Run a courier job from the command line."""
    share = get_package_share_directory('delivery_robot_controller')
    parser = argparse.ArgumentParser(
        description='Carry objects between a workspace and a storage room')
    parser.add_argument('workspace', help='table1 or table2')
    parser.add_argument('storage', help='room1 or room2')
    parser.add_argument('--bring-back', action='store_true',
                        help='also carry an object from the racks back')
    parser.add_argument('--sim-people', action='store_true',
                        help='simulation: a helper puts objects on and takes '
                             'them off')
    parser.add_argument('--timeout', type=float, default=600.0,
                        help='seconds to wait for a person (default 600)')
    parser.add_argument('--no-return', action='store_true',
                        help='stay at the last place instead of going home')
    parser.add_argument('--home', default='dock')
    parser.add_argument('--locations', default=os.path.join(
        share, 'config', 'locations.yaml'))
    opts = parser.parse_args(remove_ros_args(args if args else sys.argv)[1:])

    locations = load_locations(opts.locations)
    for name in (opts.workspace, opts.storage, opts.home):
        if name not in locations:
            print(f'Unknown place "{name}". Known: {", ".join(locations)}')
            return 2

    rclpy.init(args=args)
    node = Courier()
    node.sim_people = opts.sim_people
    code = 1
    try:
        if not node.client.wait_for_server(timeout_sec=30.0):
            node.get_logger().error(
                'navigate_to_pose is not available. Start the simulation, '
                'localization and navigation first.')
            return code
        ok = node.run_courier(opts.workspace, opts.storage, opts.bring_back,
                              opts.no_return, locations, opts.home,
                              opts.timeout)
        code = 0 if ok else 1
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
