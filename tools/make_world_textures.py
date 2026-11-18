"""Generate surface textures for the warehouse and add them to the world file.

Plain-coloured walls and floor give a camera nothing to track, so visual
SLAM cannot work. This writes a realistic texture per surface (brick outer
walls, painted plaster room walls, a concrete floor; different on every wall) into
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

# model name -> (kind, texture pixels per metre, seed)
SURFACES = {
    'warehouse_floor': ('concrete', 200, 1000),
    'outer_wall_left': ('brick', 300, 1001),
    'outer_wall_right': ('brick', 300, 1002),
    'outer_wall_back': ('brick', 300, 1003),
    'outer_wall_front': ('brick', 300, 1004),
    'room1_right_wall': ('plaster', 300, 1005),
    'room1_front_wall': ('plaster', 300, 1006),
    'room2_left_wall': ('plaster', 300, 1007),
    'room2_top_wall': ('plaster', 300, 1008),
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


def stains(rng, img, count, strength=0.25):
    """Darken the image with soft, irregular stains and streaks."""
    h, w = img.shape[:2]
    layer = np.zeros((h, w), np.float32)
    for _ in range(count):
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
        ax, ay = int(rng.integers(8, 70)), int(rng.integers(8, 110))
        cv2.ellipse(layer, (x, y), (ax, ay), int(rng.integers(0, 180)),
                    0, 360, float(rng.uniform(0.3, 1.0)), -1)
    layer = cv2.GaussianBlur(layer, (0, 0), 25) * noise(rng, h, w, 5)
    return img * (1.0 - strength * 0.5 * np.clip(layer * 2.0, 0, 1))[..., None]


def brick_texture(rng, w, h, ppm):
    """Running-bond brick wall with varied brick colours and mortar."""
    bw, bh, mortar = int(0.23 * ppm), int(0.075 * ppm), max(2, int(0.01 * ppm))
    img = np.empty((h, w, 3), np.float32)
    img[:] = (170, 175, 180)                      # mortar (BGR)
    palette = rng.choice([0, 1, 2])
    base = [(55, 70, 150), (45, 85, 160), (70, 100, 165)][palette]
    for row, y in enumerate(range(0, h, bh)):
        shift = 0 if row % 2 == 0 else bw // 2
        for x in range(-shift, w, bw):
            col = np.array(base) * rng.uniform(0.93, 1.07, 3) * rng.uniform(0.8, 1.12)
            x0, x1 = max(0, x + mortar), min(w, x + bw)
            y0, y1 = y + mortar, min(h, y + bh)
            if x1 > x0 and y1 > y0:
                img[y0:y1, x0:x1] = col
    img *= (0.8 + 0.4 * noise(rng, h, w, 7))[..., None]
    img = stains(rng, img, int(w * h / 60000), 0.3)
    return img


def plaster_texture(rng, w, h, ppm):
    """Painted plaster: pale wall, grey lower band, scuffs, a sign and a pipe."""
    tone = np.array(rng.choice([[205, 215, 220], [195, 205, 210], [210, 210, 215]]),
                    np.float32)
    img = np.empty((h, w, 3), np.float32)
    img[:] = tone
    img *= (0.72 + 0.4 * noise(rng, h, w, 8))[..., None]
    # Lower band (0.9 m): darker grey paint, as in many industrial rooms.
    band = int(0.9 * ppm)
    img[h - band:] = img[h - band:] * np.array([0.62, 0.62, 0.6])
    cv2.line(img, (0, h - band), (w, h - band), (90, 90, 90), 3)
    # yellow safety strip above the floor
    img[h - int(0.12 * ppm):h - int(0.08 * ppm)] = (30, 190, 235)
    # a pipe along the wall, a few signs and switch boxes
    py = int(h * rng.uniform(0.12, 0.25))
    cv2.line(img, (0, py), (w, py), (110, 115, 120), int(0.05 * ppm))
    cv2.line(img, (0, py - 4), (w, py - 4), (170, 175, 180), 4)
    for _ in range(int(w / ppm / 0.55)):
        sx = int(rng.integers(0, max(1, w - int(0.4 * ppm))))
        sy = int(h * rng.uniform(0.22, 0.55))
        sw, sh = int(rng.uniform(0.25, 0.5) * ppm), int(rng.uniform(0.2, 0.35) * ppm)
        col = np.array(rng.choice([[40, 60, 190], [60, 150, 60], [200, 200, 200],
                                    [30, 190, 235]]), np.float32)
        cv2.rectangle(img, (sx, sy), (sx + sw, sy + sh), col.tolist(), -1)
        cv2.rectangle(img, (sx, sy), (sx + sw, sy + sh), (40, 40, 40), 2)
        for k in range(1, 4):
            ly = sy + sh * k // 4
            cv2.line(img, (sx + sw // 8, ly), (sx + sw * 7 // 8, ly),
                     (255, 255, 255) if col.mean() < 150 else (40, 40, 40), 2)
    for _ in range(int(w / ppm / 0.9)):            # cracks and chipped paint
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h - band))
        for _ in range(int(rng.integers(8, 20))):
            nx, ny = x + int(rng.integers(-14, 15)), y + int(rng.integers(0, 16))
            cv2.line(img, (x, y), (nx, ny), (70, 75, 80), 2)
            x, y = nx, ny
    for _ in range(int(w / ppm / 0.7)):            # scuffs on the lower band
        x = int(rng.integers(0, w))
        y = int(h - rng.uniform(0.15, 0.85) * ppm)
        cv2.ellipse(img, (x, y), (int(rng.integers(6, 30)), int(rng.integers(3, 9))),
                    int(rng.integers(0, 180)), 0, 360,
                    (rng.uniform(40, 150),) * 3, -1)
    img = stains(rng, img, int(w * h / 30000), 0.45)
    return img


def concrete_texture(rng, w, h, ppm):
    """Concrete slab floor with joints, stains and faded lane markings."""
    img = np.empty((h, w, 3), np.float32)
    img[:] = (140, 145, 148)
    img *= (0.8 + 0.4 * noise(rng, h, w, 8))[..., None]
    slab = int(2.5 * ppm)
    for x in range(0, w, slab):
        cv2.line(img, (x, 0), (x, h), (70, 72, 75), 2)
    for y in range(0, h, slab):
        cv2.line(img, (0, y), (w, y), (70, 72, 75), 2)
    for _ in range(6):                                 # tyre and scuff marks
        x0, y0 = int(rng.integers(0, w)), int(rng.integers(0, h))
        x1, y1 = x0 + int(rng.integers(-500, 500)), y0 + int(rng.integers(-500, 500))
        cv2.line(img, (x0, y0), (x1, y1), (60, 62, 65), int(rng.integers(3, 9)))
    for _ in range(4):                                 # faded yellow lane lines
        x0, y0 = int(rng.integers(0, w)), int(rng.integers(0, h))
        horiz = rng.random() < 0.5
        x1, y1 = (x0 + int(rng.integers(300, 900)), y0) if horiz else (x0, y0 + int(rng.integers(300, 900)))
        cv2.line(img, (x0, y0), (x1, y1), (60, 170, 205), 7)
    for _ in range(int(w * h / 90000)):            # cracks
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
        for _ in range(int(rng.integers(10, 40))):
            nx, ny = x + int(rng.integers(-18, 19)), y + int(rng.integers(-18, 19))
            cv2.line(img, (x, y), (nx, ny), (55, 58, 60), 2)
            x, y = nx, ny
    for _ in range(int(w * h / 150000)):           # repair patches and oil spots
        x, y = int(rng.integers(0, w)), int(rng.integers(0, h))
        c = float(rng.uniform(70, 175))
        cv2.rectangle(img, (x, y), (x + int(rng.integers(20, 90)),
                      y + int(rng.integers(20, 90))), (c, c, c), -1)
    img = stains(rng, img, int(w * h / 25000), 0.45)
    return img


def make_texture(name, kind, ppm, width_m, height_m, seed):
    """Write one texture and return its file name."""
    rng = np.random.default_rng(seed)
    w, h = int(width_m * ppm), int(height_m * ppm)
    img = {'brick': brick_texture, 'plaster': plaster_texture,
           'concrete': concrete_texture}[kind](rng, w, h, ppm)
    img = img + rng.normal(0, 5, img.shape)            # fine grain
    img = np.clip(img, 0, 255).astype(np.uint8)
    os.makedirs(OUT, exist_ok=True)
    fname = f'{name}.jpg'
    cv2.imwrite(os.path.join(OUT, fname), img, [cv2.IMWRITE_JPEG_QUALITY, 82])
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
    for name, (kind, ppm, seed) in SURFACES.items():
        start, end = model_block(text, name)
        block = text[start:end]
        sx, sy, sz = box_size(block)
        dims = sorted([sx, sy, sz], reverse=True)
        width_m, height_m = dims[0], dims[1]
        if name == 'warehouse_floor':
            width_m, height_m = sx, sy
        fname = make_texture(name, kind, ppm, width_m, height_m, seed)
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
        print(f'{name}: {fname} ({kind})')
    with open(WORLD, 'w') as f:
        f.write(text)


if __name__ == '__main__':
    main()
