"""SIMULATION SHORTCUT standing in for the robot arm (picking and placing).

There is no arm yet. Until there is one, picking and placing a block is done
by moving the block's model in Gazebo with the world's set_pose service:

    pick   moves the block onto the robot's deck
    place  puts the block down on a given spot

It works like a real arm would: it is given a position relative to the robot
(from the block detector's `relative` field) and the robot's true pose, finds
the block model that is there, and moves it. When an arm is added, the
`pick` and `place` functions are what it replaces.
"""

import math
import re
import subprocess

WORLD = 'delivery_warehouse'
POSE_TOPIC = f'/world/{WORLD}/dynamic_pose/info'
BASE_HEIGHT = 0.05      # base_link rests this far above the floor
DECK_X = -0.10          # where a carried block sits on the deck (base_link)
DECK_TOP = 0.37         # world height of the deck surface
FIND_RADIUS = 0.08      # a block model within this of the expected spot

_NUM = r'(-?\d+\.?\d*(?:[eE][-+]?\d+)?)'


def gz_poses():
    """Poses of all moving models: {name: (x, y, z, yaw)} (world frame)."""
    try:
        text = subprocess.run(
            ['gz', 'topic', '-e', '-n', '1', '-t', POSE_TOPIC],
            capture_output=True, text=True, timeout=10).stdout
    except subprocess.TimeoutExpired:
        return {}
    poses = {}
    for chunk in text.split('pose {')[1:]:
        name = re.search(r'name: "([^"]+)"', chunk)
        pos = re.search(r'position \{([^}]*)\}', chunk)
        ori = re.search(r'orientation \{([^}]*)\}', chunk)
        if not (name and pos):
            continue

        def get(block, key, default=0.0):
            m = re.search(key + ': ' + _NUM, block or '')
            return float(m.group(1)) if m else default

        yaw = 2.0 * math.atan2(get(ori.group(1) if ori else '', 'z'),
                               get(ori.group(1) if ori else '', 'w', 1.0))
        poses[name.group(1)] = (get(pos.group(1), 'x'), get(pos.group(1), 'y'),
                                get(pos.group(1), 'z'), yaw)
    return poses


def robot_pose():
    """Return the true pose (x, y, yaw) of the robot in the world, or None."""
    pose = gz_poses().get('delivery_robot')
    return None if pose is None else (pose[0], pose[1], pose[3])


def relative_to_world(rel, pose):
    """World (x, y, z) of a point given relative to the robot's base_link."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * rel['x'] - s * rel['y'],
            y + s * rel['x'] + c * rel['y'],
            BASE_HEIGHT + rel['z'])


def world_to_relative(point, pose):
    """Return a world point relative to the robot (x forward, y left)."""
    x, y, yaw = pose
    dx, dy = point[0] - x, point[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def find_block_model(rel, pose):
    """Name of the block model at the relative position, or None."""
    wx, wy, _ = relative_to_world(rel, pose)
    best, best_d = None, FIND_RADIUS
    for name, (x, y, _, _) in gz_poses().items():
        if name.startswith('block_'):
            d = math.hypot(x - wx, y - wy)
            if d < best_d:
                best, best_d = name, d
    return best


def set_model_pose(name, x, y, z, yaw=0.0):
    """Move a model to a pose in the world; True on success."""
    req = (f'name: "{name}" position {{x: {x} y: {y} z: {z}}} '
           f'orientation {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')
    out = subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', req],
        capture_output=True, text=True, timeout=20)
    return 'data: true' in out.stdout


def pick(model, height, pose):
    """Pick a block (simulated) by putting it on the robot's deck."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return set_model_pose(
        model, x + DECK_X * c, y + DECK_X * s,
        DECK_TOP + height / 2.0 + 0.005, yaw)


def place(model, x, y, surface_z, height):
    """Place a block (simulated) by putting it down at (x, y) on a surface."""
    return set_model_pose(model, x, y, surface_z + height / 2.0 + 0.003)
