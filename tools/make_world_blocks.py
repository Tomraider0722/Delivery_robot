"""Put blocks of different shapes, sizes and colours on the tables and racks.

Adds a marked section to the warehouse world files (safe to run again). The
blocks are scenery now (the objects that are on the tables and racks).

The blocks are hidden from the lidar (visibility flag 2, see the URDF), so a
block carried on the deck does not show up in the scan.

Surface heights: table tops at 0.80 m; each rack has shelves with tops at
0.30 m (lower) and 0.95 m (middle). A block's pose is its centre.

Usage (from the workspace root):  python3 tools/make_world_blocks.py
"""


WORLDS = [
    'src/delivery_robot_gazebo/worlds/delivery_warehouse.world',
    'src/delivery_robot_gazebo/worlds/delivery_warehouse_vslam.world',
]
BEGIN, END = '<!-- BLOCKS BEGIN -->', '<!-- BLOCKS END -->'

COLOURS = {
    'red': (0.90, 0.05, 0.05), 'orange': (1.00, 0.50, 0.00),
    'yellow': (1.00, 0.90, 0.05), 'green': (0.10, 0.70, 0.10),
    'cyan': (0.00, 0.80, 0.80), 'blue': (0.10, 0.20, 0.90),
    'purple': (0.50, 0.10, 0.70), 'magenta': (0.90, 0.10, 0.60),
}
TABLE_TOP, SHELF_LOWER, SHELF_MIDDLE = 0.80, 0.30, 0.95

# place, colour, shape, size in metres (cube: edge; bar: x y z; cylinder:
# radius height), x, y, surface height
BLOCKS = [
    ('table1', 'red', 'cube', (0.06,), 2.65, 2.00, TABLE_TOP),
    ('table1', 'blue', 'cube', (0.04,), 3.05, 2.25, TABLE_TOP),
    ('table1', 'green', 'cylinder', (0.03, 0.08), 3.40, 2.00, TABLE_TOP),
    ('table2', 'yellow', 'bar', (0.12, 0.04, 0.04), -3.35, -2.00, TABLE_TOP),
    ('table2', 'orange', 'cube', (0.08,), -2.90, -2.25, TABLE_TOP),
    ('table2', 'purple', 'cylinder', (0.022, 0.045), -2.60, -2.00,
     TABLE_TOP),
    ('room1_lower', 'cyan', 'cube', (0.06,), -4.75, 4.50, SHELF_LOWER),
    ('room1_middle', 'magenta', 'bar', (0.12, 0.04, 0.04), -4.45, 4.50,
     SHELF_MIDDLE),
    ('room2_lower', 'blue', 'cylinder', (0.035, 0.07), 4.35, -4.50,
     SHELF_LOWER),
    ('room2_middle', 'orange', 'bar', (0.11, 0.04, 0.04), 4.75, -4.50,
     SHELF_MIDDLE),
]


def dims(shape, size):
    """Bounding box (x, y, z) of a block."""
    if shape == 'cube':
        return size[0], size[0], size[0]
    if shape == 'bar':
        return size
    return 2 * size[0], 2 * size[0], size[1]


def geometry(shape, size):
    """SDF geometry element."""
    if shape == 'cube':
        return f'<box><size>{size[0]} {size[0]} {size[0]}</size></box>'
    if shape == 'bar':
        return f'<box><size>{size[0]} {size[1]} {size[2]}</size></box>'
    return (f'<cylinder><radius>{size[0]}</radius>'
            f'<length>{size[1]}</length></cylinder>')


def size_class(biggest):
    """small / medium / large from the longest edge (metres)."""
    return 'small' if biggest <= 0.055 else 'medium' if biggest <= 0.10 \
        else 'large'


def model(name, colour, shape, size, x, y, z):
    """One dynamic block model."""
    r, g, b = COLOURS[colour]
    geom = geometry(shape, size)
    return f'''    <model name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="block">
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry>{geom}</geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <visibility_flags>2</visibility_flags>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''


def main():
    """Write the blocks into the worlds."""
    models = []
    for place, colour, shape, size, x, y, surface in BLOCKS:
        name = f'block_{colour}_{shape}_{place}'
        height = dims(shape, size)[2]
        z = round(surface + height / 2.0 + 0.002, 4)
        models.append(model(name, colour, shape, size, x, y, z))

    section = (f'    {BEGIN}\n    <!-- Blocks to carry (made by '
               'tools/make_world_blocks.py) -->\n' + ''.join(models)
               + f'    {END}\n')

    for path in WORLDS:
        with open(path) as f:
            text = f.read()
        if BEGIN in text:
            a = text.index(BEGIN)
            b = text.index(END) + len(END)
            text = text[:a].rstrip(' ') + section.strip(' ') + text[b:].lstrip('\n')
        else:
            i = text.rindex('</world>')
            text = text[:i] + section + '\n  ' + text[i:]
        with open(path, 'w') as f:
            f.write(text)
        print('updated', path)


if __name__ == '__main__':
    main()
