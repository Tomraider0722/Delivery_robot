"""Carry one block from one place to another.

Usage:
    ros2 run delivery_robot_controller transport red_cube table1 room1
    ros2 run delivery_robot_controller transport magenta_bar room1 table2
    ros2 run delivery_robot_controller transport blue_cylinder room2 table1 --shelf lower

The block is named by colour and shape (red_cube, blue_cylinder, yellow_bar;
add the size to be exact, e.g. blue_cube_small). The places are table1,
table2, room1 and room2. For a rack, --shelf lower or middle picks the shelf
the block is put on (default: the first surface of the place).

Steps: drive to the source and look, check that the block is there, pick it,
check that it left the source, drive to the destination, place it on a free
spot, check with the camera that it is there, and return to the dock.

THE PICK AND THE PLACE ARE A SIMULATION SHORTCUT (delivery_robot_perception
sim_arm moves the block model in Gazebo); there is no arm yet. Everything
else, including finding and checking the block, uses the perception camera.

Needs the simulation, localization, Nav2 and the block detector running
(bringup.launch.py starts them all).
"""

import argparse
import json
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from delivery_robot_controller.deliver import Deliver, load_locations
from delivery_robot_perception import sim_arm
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

CAMERA_Z = 1.50                 # world height of the perception camera
CAMERA_AHEAD = 0.15             # how far in front of base_link it is
CAMERA_HALF_FOV = 0.65          # rad, keeps a margin inside the 0.75 rad view
FREE_SPOT_CLEARANCE = 0.12      # m between a placed block and its neighbours
EDGE_MARGIN = 0.07              # m from the edge of a surface


def parse_spec(spec):
    """Split red_cube or red_cube_small into (colour, shape, size)."""
    parts = spec.lower().split('_')
    if len(parts) not in (2, 3):
        raise ValueError(
            f'"{spec}" is not a block name like red_cube or blue_cube_small')
    return parts[0], parts[1], parts[2] if len(parts) == 3 else None


class Transport(Deliver):
    """A delivery node that can also look at blocks and move one."""

    def __init__(self, **kwargs):
        super().__init__(name='transport', **kwargs)
        self.blocks = None
        self.blocks_seq = 0
        self.blocks_refresh = 0
        self.create_subscription(
            String, '/blocks', self.on_blocks,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.refresh_client = self.create_client(Trigger, '/blocks/refresh')

    def on_blocks(self, msg):
        """Keep the latest block list from the detector."""
        data = json.loads(msg.data)
        self.blocks = data['blocks']
        self.blocks_refresh = data.get('refresh', 0)
        self.blocks_seq += 1

    def observe(self, timeout=20.0):
        """Ask the detector for a fresh list and wait for it; None on timeout."""
        if not self.refresh_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'The block detector is not running (/blocks/refresh)')
            return None
        # Wall-clock deadline: with simulated time the node clock reads zero
        # until the first /clock message arrives.
        end = time.monotonic() + timeout
        future = self.refresh_client.call_async(Trigger.Request())
        while not future.done():
            if time.monotonic() > end:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        wanted = int(future.result().message)
        # a list stamped with this refresh number was made from fresh frames
        while self.blocks_refresh < wanted:
            if self.cancel_requested or time.monotonic() > end:
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.blocks

    @staticmethod
    def find(blocks, spec, place, surface=None):
        """Blocks of the list that match the spec at the place."""
        colour, shape, size = spec
        return [b for b in blocks or []
                if b['colour'] == colour and b['shape'] == shape
                and (size is None or b['size'] == size)
                and b['place'] == place
                and (surface is None or surface in b['surface'])]

    def free_spot(self, place, surface_key, others, pose, surfaces):
        """Pick a free spot on a surface of the place, in view of the camera.

        Returns (x, y, surface_z, surface_name) in the world frame or None.
        """
        options = [(name, s) for name, s in surfaces[place].items()
                   if surface_key is None or surface_key in name]
        if not options:
            return None
        name, s = options[0]
        taken = [sim_arm.relative_to_world(b['relative'], pose)[:2]
                 for b in others if 'relative' in b]
        best, best_score = None, -1.0
        nx = int((s['x'][1] - s['x'][0] - 2 * EDGE_MARGIN) / 0.04) + 1
        ny = int((s['y'][1] - s['y'][0] - 2 * EDGE_MARGIN) / 0.04) + 1
        for i in range(max(nx, 1)):
            for j in range(max(ny, 1)):
                x = s['x'][0] + EDGE_MARGIN + i * 0.04
                y = s['y'][0] + EDGE_MARGIN + j * 0.04
                fwd, left = sim_arm.world_to_relative((x, y), pose)
                ahead = fwd - CAMERA_AHEAD            # from the camera
                if ahead < 0.9 or ahead > 2.6 or abs(left) > 0.8 * ahead:
                    continue            # outside the camera's view
                if (CAMERA_Z - s['z']) / ahead > math.tan(CAMERA_HALF_FOV):
                    continue            # too steep to see, e.g. a low shelf
                gap = min([math.hypot(x - tx, y - ty) for tx, ty in taken]
                          or [1.0])
                if gap < FREE_SPOT_CLEARANCE:
                    continue
                score = gap - 0.1 * abs(left)       # roomy, near the middle
                if score > best_score:
                    best, best_score = (x, y), score
        if best is None:
            return None
        return best[0], best[1], s['z'], name

    def go(self, name, locations):
        """Drive to a named place (turning to face it when it asks)."""
        return self.go_to(name, locations[name])

    def settle(self, seconds=3.0):
        """Spin for a moment so the clock and the TF buffer have data."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_transport(self, spec, source, target, shelf, locations,
                      places, home, no_return):
        """Carry the block; return True when it ended up at the target."""
        say = self.say
        self.settle()
        say(f'Task: carry {"_".join(p for p in spec if p)} '
            f'from {source} to {target}')
        ok = False
        carrying = False
        try:
            if not self.go(source, locations):
                say(f'Could not reach {source}')
                return False
            say(f'At {source}, looking for the block')
            blocks = self.observe()
            found = self.find(blocks, spec, source)
            if not found:
                seen = ', '.join(b['id'] for b in blocks or []
                                 if b['place'] == source) or 'nothing'
                say(f'Block not found at {source} (I see: {seen})')
                return False
            if len(found) > 1:
                say('More than one matching block; give the size too, e.g. '
                    + found[0]['id'])
                return False
            block = found[0]
            say(f'Found {block["id"]} at {source}, '
                f'{block["relative"]["x"]:.2f} m ahead')

            pose = sim_arm.robot_pose()
            model = sim_arm.find_block_model(block['relative'], pose)
            if model is None:
                say('SIMULATED ARM: no block model where the camera saw it')
                return False
            height = block['dimensions_m'][2]
            say(f'SIMULATED ARM: picking {model}')
            if not sim_arm.pick(model, height, pose):
                say('SIMULATED ARM: pick failed')
                return False
            carrying = True
            self.wait(1.5)
            after = self.observe()
            if after is None or self.find(after, spec, source):
                say(f'The block is still at {source}, the pick did not work')
                return False
            say('Checked: the block is no longer at ' + source)

            if not self.go(target, locations):
                say(f'Could not reach {target}, still carrying the block')
                return False
            say(f'At {target}, looking for a free spot')
            blocks = self.observe()
            if blocks is None:
                say('No camera result at the target, not placing')
                return False
            pose = sim_arm.robot_pose()
            others = [b for b in blocks if b['place'] == target]
            spot = self.free_spot(target, shelf, others, pose, places)
            if spot is None:
                say(f'No free spot in view at {target}, not placing')
                return False
            x, y, sz, surface = spot
            say(f'SIMULATED ARM: placing it on the {surface} at '
                f'({x:.2f}, {y:.2f})')
            if not sim_arm.place(model, x, y, sz, height):
                say('SIMULATED ARM: place failed')
                return False
            carrying = False
            self.wait(1.5)
            result = self.observe()
            there = self.find(result, spec, target, shelf)
            if not there:
                say(f'The camera does not see the block at {target}')
                return False
            rel = there[0]['relative']
            want = sim_arm.world_to_relative((x, y), pose)
            err = math.hypot(rel['x'] - want[0], rel['y'] - want[1])
            say(f'Checked: {there[0]["id"]} is on the {there[0]["surface"]} '
                f'at {target} ({err * 100:.1f} cm from where it was put)')
            ok = True
        finally:
            if carrying:
                say('The block is still on the deck')
            if not no_return and not self.cancel_requested:
                self.go(home, locations)
        say('Task done' if ok else 'Task failed')
        return ok


def main(args=None):
    """Run a transport task from the command line."""
    share = get_package_share_directory('delivery_robot_controller')
    parser = argparse.ArgumentParser(
        description='Carry one block from one place to another')
    parser.add_argument('block', help='e.g. red_cube, blue_cylinder_medium')
    parser.add_argument('source', help='table1, table2, room1 or room2')
    parser.add_argument('target', help='table1, table2, room1 or room2')
    parser.add_argument('--shelf', help='lower or middle (racks)')
    parser.add_argument('--no-return', action='store_true',
                        help='stay at the target instead of going to the dock')
    parser.add_argument('--home', default='dock')
    parser.add_argument('--locations', default=os.path.join(
        share, 'config', 'locations.yaml'))
    opts = parser.parse_args(remove_ros_args(args if args else sys.argv)[1:])

    locations = load_locations(opts.locations)
    with open(os.path.join(
            get_package_share_directory('delivery_robot_perception'),
            'config', 'places.yaml')) as f:
        places = yaml.safe_load(f)['places']
    try:
        spec = parse_spec(opts.block)
    except ValueError as err:
        print(err)
        return 2
    for name in (opts.source, opts.target):
        if name not in places or name not in locations:
            print(f'Unknown place "{name}". Known: {", ".join(places)}')
            return 2

    rclpy.init(args=args)
    node = Transport()
    code = 1
    try:
        if not node.client.wait_for_server(timeout_sec=30.0):
            node.get_logger().error(
                'navigate_to_pose is not available. Start the simulation, '
                'localization and navigation first.')
            return code
        ok = node.run_transport(spec, opts.source, opts.target, opts.shelf,
                                locations, places, opts.home, opts.no_return)
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
