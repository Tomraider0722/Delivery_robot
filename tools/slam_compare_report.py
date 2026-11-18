"""Score a SLAM comparison run and draw the comparison figure.

Input: the poses.json written by slam_compare_logger.py and, optionally,
the two saved maps (lidar_map.yaml, visual_map.yaml) from the same folder.
Output: a metrics table (printed and saved as metrics.md) and comparison.png.

Usage:  python3 tools/slam_compare_report.py RUN_DIR OUT_DIR
"""

import json
import math
import os
import sys

import cv2
import matplotlib
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SOURCES = [
    ('wheel', 'Wheel odometry only', '#c77700'),
    ('lidar_slam', 'LiDAR SLAM (slam_toolbox)', '#0b3d6b'),
    ('visual_slam', 'Visual SLAM (RTAB-Map, camera only)', '#2a9d3f'),
]


def position_errors(rows, key):
    """Distance to the Gazebo ground truth for every sample."""
    out = []
    for row in rows:
        a, b = row[key], row['truth']
        if a is not None and b is not None:
            out.append(math.hypot(a[0] - b[0], a[1] - b[1]))
    return out


def path_length(points):
    """Length of a polyline."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def load_map(folder, name):
    """Return (image, resolution, origin) of a saved map, or None."""
    path = os.path.join(folder, name + '.yaml')
    if not os.path.exists(path):
        return None
    with open(path) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(os.path.join(folder, meta['image']), cv2.IMREAD_GRAYSCALE)
    return img, meta['resolution'], meta['origin']


def draw_map(ax, loaded, title):
    """Show a saved map in metres."""
    ax.set_title(title, fontsize=10)
    ax.set_aspect('equal')
    if loaded is None:
        ax.text(0.5, 0.5, 'map not saved', ha='center', transform=ax.transAxes)
        return
    img, res, origin = loaded
    h, w = img.shape
    ax.imshow(img, cmap='gray', origin='upper', vmin=0, vmax=255,
              extent=[origin[0], origin[0] + w * res,
                      origin[1], origin[1] + h * res])
    ax.set_xlabel('x (m)')


def main(run_dir, out_dir):
    """Write metrics.md and comparison.png."""
    with open(os.path.join(run_dir, 'poses.json')) as f:
        rows = json.load(f)
    os.makedirs(out_dir, exist_ok=True)

    truth = [r['truth'] for r in rows if r['truth']]
    lines = ['| Source | Mean error (m) | RMSE (m) | Max error (m) | Final error (m) |',
             '|---|---|---|---|---|']
    for key, label, _ in SOURCES:
        e = position_errors(rows, key)
        if not e:
            continue
        rmse = math.sqrt(sum(v * v for v in e) / len(e))
        lines.append(f'| {label} | {sum(e) / len(e):.2f} | {rmse:.2f} '
                     f'| {max(e):.2f} | {e[-1]:.2f} |')
    lines.append('')
    lines.append(f'Route length (ground truth): {path_length([t[:2] for t in truth]):.1f} m, '
                 f'{len(rows)} samples.')
    table = '\n'.join(lines)
    print(table)
    with open(os.path.join(out_dir, 'metrics.md'), 'w') as f:
        f.write(table + '\n')

    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(2, 3, (1, 2))
    ax.plot([t[0] for t in truth], [t[1] for t in truth], 'k--', lw=2,
            label='Ground truth (Gazebo)')
    for key, label, colour in SOURCES:
        pts = [r[key] for r in rows if r[key]]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colour, lw=1.6, label=label)
    ax.set_aspect('equal')
    ax.set_title('Robot path in the map frame (dock - room 1 - room 2 - dock)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(2, 3, 3)
    t0 = rows[0]['t']
    for key, label, colour in SOURCES:
        xs = [r['t'] - t0 for r in rows if r[key] and r['truth']]
        ys = [math.hypot(r[key][0] - r['truth'][0], r[key][1] - r['truth'][1])
              for r in rows if r[key] and r['truth']]
        ax.plot(xs, ys, color=colour, label=label)
    ax.set_title('Position error over time')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('error (m)')
    ax.grid(alpha=0.3)

    draw_map(fig.add_subplot(2, 3, 4), load_map(run_dir, 'lidar_map'),
             'Map from LiDAR SLAM')
    draw_map(fig.add_subplot(2, 3, 5), load_map(run_dir, 'visual_map'),
             'Map from visual SLAM (depth camera)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'comparison.png'), dpi=110)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
