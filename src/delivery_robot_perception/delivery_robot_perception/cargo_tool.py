"""Put the cargo box on the robot's deck, or take it away (simulation).

Usage:
    ros2 run delivery_robot_perception cargo_tool load
    ros2 run delivery_robot_perception cargo_tool take

`load` creates a red box on the deck of the robot, wherever the robot is
(the person loading it). `take` removes it from the world (the person taking
it off). The box is invisible to the lidar, so it does not appear in the scan
or the maps. The robot notices the change through its load sensor (the deck
contact sensor read by the load monitor).
"""

import math
import re
import subprocess
import sys

WORLD = 'delivery_warehouse'
POSE_TOPIC = f'/world/{WORLD}/dynamic_pose/info'
MODEL = 'cargo_box'

# Cargo size and where it sits on the deck, relative to base_link (metres).
SIZE = 0.14
DECK_X = -0.06
DECK_TOP_Z = 0.37    # 0.32 above base_link, which rests 0.05 above the floor

# visibility_flags 2: the lidar mask (see the URDF) leaves this bit out.
CARGO_SDF = (
    '<sdf version="1.9"><model name="' + MODEL + '"><link name="box">'
    '<inertial><mass>0.3</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy>'
    '<izz>0.001</izz></inertia></inertial>'
    f'<collision name="c"><geometry><box><size>{SIZE} {SIZE} {SIZE}</size>'
    '</box></geometry><surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2>'
    '</ode></friction></surface></collision>'
    f'<visual name="v"><geometry><box><size>{SIZE} {SIZE} {SIZE}</size></box>'
    '</geometry><material><ambient>0.9 0.05 0.05 1</ambient>'
    '<diffuse>0.9 0.05 0.05 1</diffuse></material>'
    '<visibility_flags>2</visibility_flags></visual>'
    '</link></model></sdf>')


def gz_service(service, reqtype, req):
    """Call a Gazebo service; return True when it answered true."""
    out = subprocess.run(
        ['gz', 'service', '-s', service, '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req],
        capture_output=True, text=True, timeout=20)
    return 'data: true' in out.stdout


def robot_pose():
    """Return (x, y, yaw) of the robot in the world, or None."""
    try:
        text = subprocess.run(
            ['gz', 'topic', '-e', '-n', '1', '-t', POSE_TOPIC],
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
    return float(x.group(1)), float(y.group(1)), 2.0 * math.atan2(z, w)


def take_cargo():
    """Remove the cargo box from the world; True if it was there."""
    return gz_service(f'/world/{WORLD}/remove', 'gz.msgs.Entity',
                      f'name: "{MODEL}" type: MODEL')


def load_cargo():
    """Create the cargo box on the robot's deck; True on success."""
    pose = robot_pose()
    if pose is None:
        return False
    x, y, yaw = pose
    take_cargo()                      # never two boxes
    cx = x + DECK_X * math.cos(yaw)
    cy = y + DECK_X * math.sin(yaw)
    cz = DECK_TOP_Z + SIZE / 2.0 + 0.01
    sdf = CARGO_SDF.replace('"', '\\"')       # escape for the text request
    req = (f'sdf: "{sdf}" name: "{MODEL}" '
           f'pose: {{position: {{x: {cx}, y: {cy}, z: {cz}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}')
    return gz_service(f'/world/{WORLD}/create', 'gz.msgs.EntityFactory', req)


def main(args=None):
    """Command line: load or take."""
    argv = sys.argv[1:] if args is None else args
    if len(argv) != 1 or argv[0] not in ('load', 'take'):
        print('Usage: cargo_tool load|take')
        return 2
    ok = load_cargo() if argv[0] == 'load' else take_cargo()
    print(('Cargo loaded' if argv[0] == 'load' else 'Cargo taken') if ok
          else 'Failed (is the simulation running?)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
