"""Generate surface textures for the warehouse and add them to the world file.

The plain-coloured walls and floor give a camera nothing to track, so visual
SLAM cannot work. This writes one random texture per surface (different on
every wall, so walls cannot be mistaken for each other) into
src/delivery_robot_gazebo/worlds/textures and points the world file at them.
It only changes how the world looks, not its geometry. Safe to run again.

Usage (from the workspace root):  python3 tools/make_world_textures.py
"""

import os
import re

import cv2
import numpy as np

WORLD = 'src/delivery_robot_gazebo/worlds/delivery_warehouse.world'
OUT = 'src/delivery_robot_gazebo/worlds/textures'
PX_PER_M = 200

# model name -> base colour (RGB 0..1)
SURFACES = {
    'warehouse_floor': (0.48, 0.48, 0.48),
    'outer_wall_left': (0.62, 0.28, 0.12),
    'outer_wall_right': (0.62, 0.28, 0.12),
    'outer_wall_back': (0.62, 0.28, 0.12),
    'outer_wall_front': (0.62, 0.28, 0.12),
    'room1_right_wall': (0.62, 0.28, 0.12),
    'room1_front_wall': (0.62, 0.28, 0.12),
    'room2_left_wall': (0.62, 0.28, 0.12),
    'room2_top_wall': (0.62, 0.28, 0.12),
}


def noise(rng, h, w, octaves=6):
    """Smooth multi-scale noise in 0..1."""
    out = np.zeros((h, w), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        cells = 2 ** (o + 2)
        small = rng.random((max(2, cells * h // max(h, w)),
                            max(2, cells * w // max(h, w)))).astype(np.float32)
        out += amp * cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC)
        total += amp
        amp *= 0.55
    out /= total
    return (out - out.min()) / (out.max() - out.min() + 1e-6)


def decorate(rng, img, count, size_m, stains):
    """Add random posters, stripes and marks (unique per surface)."""
    h, w = img.shape[:2]
    for _ in range(count):
        pw = int(rng.uniform(0.3, size_m) * PX_PER_M)
        ph = int(rng.uniform(0.3, size_m) * PX_PER_M)
        x = int(rng.integers(0, max(1, w - pw)))
        y = int(rng.integers(0, max(1, h - ph)))
        base = rng.uniform(40, 255, 3)
        cv2.rectangle(img, (x, y), (x + pw, y + ph), base.tolist(), -1)
        for _ in range(int(rng.integers(3, 8))):
            c = rng.uniform(0, 255, 3).tolist()
            kind = rng.integers(0, 3)
            px = int(x + rng.uniform(0.1, 0.9) * pw)
            py = int(y + rng.uniform(0.1, 0.9) * ph)
            if kind == 0:
                cv2.circle(img, (px, py), int(rng.uniform(0.05, 0.25) * min(pw, ph)), c, -1)
            elif kind == 1:
                cv2.line(img, (x, py), (x + pw, py), c, int(rng.integers(2, 9)))
            else:
                cv2.rectangle(img, (px, py),
                              (min(x + pw, px + int(pw * 0.3)), min(y + ph, py + int(ph * 0.3))), c, -1)
        cv2.rectangle(img, (x, y), (x + pw, y + ph), (20, 20, 20), 3)
    mean = img.reshape(-1, 3).mean(axis=0)
    for _ in range(stains):
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
        c = np.clip(mean * rng.uniform(0.55, 1.45) + rng.uniform(-25, 25, 3), 0, 255).tolist()
        cv2.ellipse(img, (x, y), (int(rng.integers(6, 50)), int(rng.integers(6, 50))),
                    int(rng.integers(0, 180)), 0, 360, c, -1)


def make_texture(name, base, width_m, height_m, seed):
    """Write one texture and return its file name."""
    rng = np.random.default_rng(seed)
    w, h = int(width_m * PX_PER_M), int(height_m * PX_PER_M)
    n1, n2, n3 = noise(rng, h, w), noise(rng, h, w, 8), noise(rng, h, w, 3)
    b = np.array(base[::-1]) * 255          # OpenCV is BGR
    tint = rng.uniform(-40, 40, 3)
    img = (b + tint + (n1[..., None] - 0.5) * 120 + (n2[..., None] - 0.5) * 80
           + (n3[..., None] - 0.5) * 90).astype(np.float32)
    img = np.clip(img, 0, 255).astype(np.uint8)
    floor = name == 'warehouse_floor'
    posters = 10 if floor else int(width_m * height_m / 3) + 4
    decorate(rng, img, posters, 0.8 if floor else 1.0,
             int(width_m * height_m * (6 if floor else 10)))
    speckle = rng.normal(0, 10, img.shape).astype(np.float32)
    img = np.clip(img.astype(np.float32) + speckle, 0, 255).astype(np.uint8)
    os.makedirs(OUT, exist_ok=True)
    fname = f'{name}.jpg'
    cv2.imwrite(os.path.join(OUT, fname), img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return fname


def model_block(text, name):
    """(start, end) of the <model name=...> block."""
    start = text.index(f'<model name="{name}">')
    return start, text.index('</model>', start)


def box_size(block):
    """Visual box size (x, y, z) of a model block."""
    m = re.search(r'<visual.*?<size>([\d.\s-]+)</size>', block, re.S)
    return [float(v) for v in m.group(1).split()]


def main():
    """Create the textures and patch the world."""
    with open(WORLD) as f:
        text = f.read()
    for i, (name, base) in enumerate(SURFACES.items()):
        start, end = model_block(text, name)
        block = text[start:end]
        sx, sy, sz = box_size(block)
        dims = sorted([sx, sy, sz], reverse=True)
        width_m, height_m = dims[0], dims[1]
        if name == 'warehouse_floor':
            width_m, height_m = sx, sy
        fname = make_texture(name, base, width_m, height_m, 1000 + i)
        material = (
            '<material>\n'
            '            <ambient>1 1 1 1</ambient>\n'
            '            <diffuse>1 1 1 1</diffuse>\n'
            '            <pbr>\n'
            '              <metal>\n'
            f'                <albedo_map>textures/{fname}</albedo_map>\n'
            '                <roughness>1.0</roughness>\n'
            '                <metalness>0.0</metalness>\n'
            '              </metal>\n'
            '            </pbr>\n'
            '          </material>')
        new_block = re.sub(r'<material>.*?</material>', material, block, count=1, flags=re.S)
        text = text[:start] + new_block + text[end:]
        print(f'{name}: {fname} ({int(width_m * PX_PER_M)}x{int(height_m * PX_PER_M)})')
    with open(WORLD, 'w') as f:
        f.write(text)


if __name__ == '__main__':
    main()
