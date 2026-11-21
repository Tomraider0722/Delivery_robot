"""Compare the detector's /blocks list with the true block layout.

Waits for a /blocks message that contains blocks of the given places, then
matches every true block of those places with the nearest detected block
(within 30 cm) and prints colour, shape, size and position error.

    python3 tools/blocks_eval.py table1 table2      # places to check
    python3 tools/blocks_eval.py room1 --timeout 20

Exit code 0 when every true block was found with the right colour, shape and
size and no extra block was reported.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
import yaml

TRUTH = 'src/delivery_robot_perception/config/blocks_truth.yaml'


def place_of(truth_block):
    """Table or room name of a true block (room1_lower -> room1)."""
    return truth_block['place'].split('_')[0]


class Listener(Node):
    """Keep the latest /blocks message."""

    def __init__(self):
        super().__init__('blocks_eval')
        self.latest = None
        self.stamp = 0.0
        self.create_subscription(
            String, '/blocks', self.on_blocks,
            QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

    def on_blocks(self, msg):
        """Remember the list and when it arrived."""
        self.latest = json.loads(msg.data)['blocks']
        self.stamp = time.time()


def evaluate(places, detected, truth):
    """Print the comparison; return True when everything matches."""
    ok = True
    used = set()
    shifts = []                 # (dx, dy) of every matched block
    for t in truth:
        if place_of(t) not in places:
            continue
        best, dist = None, 0.30
        for i, d in enumerate(detected):
            e = math.hypot(d['x'] - t['x'], d['y'] - t['y'])
            if i not in used and e < dist:
                best, dist = i, e
        want = f"{t['colour']} {t['shape']} {t['size']}"
        if best is None:
            print(f'  MISSED   {want:24s} at {t["place"]}')
            ok = False
            continue
        used.add(best)
        d = detected[best]
        shifts.append((d['x'] - t['x'], d['y'] - t['y']))
        got = f"{d['colour']} {d['shape']} {d['size']}"
        match = got == want
        ok &= match
        print(f"  {'ok     ' if match else 'WRONG  '}  {want:24s} -> "
              f'{got:24s} {dist * 100:4.1f} cm  {d["features"]}')
    if shifts:
        mx = sum(s[0] for s in shifts) / len(shifts)
        my = sum(s[1] for s in shifts) / len(shifts)
        resid = max(math.hypot(s[0] - mx, s[1] - my) for s in shifts)
        print(f'  common shift of the blocks ({mx * 100:+.1f}, '
              f'{my * 100:+.1f}) cm = robot localization error; '
              f'what is left after removing it: {resid * 100:.1f} cm')
    for i, d in enumerate(detected):
        if i not in used and d['place'] in places:
            print(f"  EXTRA    {d['colour']} {d['shape']} {d['size']} at "
                  f"({d['x']:.2f}, {d['y']:.2f}) {d['place']}")
            ok = False
    return ok


def main():
    """Wait for a detection and compare it with the truth."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('places', nargs='+', help='table1 table2 room1 room2')
    parser.add_argument('--timeout', type=float, default=30.0)
    opts = parser.parse_args()
    with open(TRUTH) as f:
        truth = yaml.safe_load(f)['blocks']

    rclpy.init()
    node = Listener()
    end = time.time() + opts.timeout
    start = time.time()
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        # a list published after we started, with blocks of those places
        if node.latest and node.stamp > start and any(
                b['place'] in opts.places for b in node.latest):
            time.sleep(1.0)
            rclpy.spin_once(node, timeout_sec=0.2)
            break
    detected = node.latest or []
    print(f'places {opts.places}: {len(detected)} blocks reported')
    ok = evaluate(opts.places, detected, truth)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
