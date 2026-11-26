"""Put simple standing people in the warehouse (scenery).

Adds a marked section to both world files (safe to run again): one person at
each workspace (table1, table2) and one in each storage room (room1, room2),
standing beside the robot's stop and facing it, out of its way.

The people are static and built from simple shapes (shoes, legs, hips,
torso, arms, hands, neck, head, hair), about 1.7 m tall. They have collision
shapes, so the robot cannot drive through them, and their legs are at the
height of the lidar, so the lidar sees them as obstacles. They do not move
or hand anything over: putting objects on the robot and taking them off is
still simulated with cargo_tool (or courier --sim-people).

Usage (from the workspace root):  python3 tools/make_world_people.py
"""

import math

WORLDS = [
    'src/delivery_robot_gazebo/worlds/delivery_warehouse.world',
    'src/delivery_robot_gazebo/worlds/delivery_warehouse_vslam.world',
]
BEGIN, END = '<!-- PEOPLE BEGIN -->', '<!-- PEOPLE END -->'

SKIN = (0.80, 0.62, 0.50)
HAIR = (0.12, 0.08, 0.05)
SHOES = (0.10, 0.10, 0.10)

# name, x, y, shirt colour, trousers colour, and the stop it faces
PEOPLE = [
    ('person_table1', 1.25, 1.60, (0.15, 0.35, 0.70), (0.18, 0.20, 0.28),
     (1.7, 0.9)),
    ('person_table2', -1.25, -1.60, (0.20, 0.55, 0.30), (0.25, 0.22, 0.20),
     (-1.7, -0.9)),
    ('person_room1', -3.70, 4.00, (0.85, 0.55, 0.15), (0.15, 0.17, 0.25),
     (-4.6, 2.7)),
    ('person_room2', 3.70, -4.00, (0.55, 0.55, 0.58), (0.20, 0.20, 0.22),
     (4.6, -2.7)),
]


def rgb(c):
    """SDF colour string."""
    return f'{c[0]} {c[1]} {c[2]} 1'


def part(name, geom, pose, colour, collide=False):
    """One visual (and optionally a collision) of a person."""
    mat = (f'<material><ambient>{rgb(colour)}</ambient>'
           f'<diffuse>{rgb(colour)}</diffuse></material>')
    out = (f'        <visual name="{name}"><pose>{pose}</pose>'
           f'<geometry>{geom}</geometry>{mat}</visual>\n')
    if collide:
        out += (f'        <collision name="{name}_collision"><pose>{pose}'
                f'</pose><geometry>{geom}</geometry></collision>\n')
    return out


def box(x, y, z):
    """Box geometry."""
    return f'<box><size>{x} {y} {z}</size></box>'


def cyl(r, length):
    """Cylinder geometry."""
    return f'<cylinder><radius>{r}</radius><length>{length}</length></cylinder>'


def sphere(r):
    """Sphere geometry."""
    return f'<sphere><radius>{r}</radius></sphere>'


def person(name, x, y, shirt, trousers, face):
    """A static person at (x, y), turned to face the point `face`."""
    yaw = math.atan2(face[1] - y, face[0] - x)
    parts = ''.join([
        part('shoe_left', box(0.26, 0.10, 0.06), '0.04 0.09 0.03 0 0 0',
             SHOES),
        part('shoe_right', box(0.26, 0.10, 0.06), '0.04 -0.09 0.03 0 0 0',
             SHOES),
        part('leg_left', cyl(0.065, 0.80), '0 0.09 0.46 0 0 0', trousers,
             True),
        part('leg_right', cyl(0.065, 0.80), '0 -0.09 0.46 0 0 0', trousers,
             True),
        part('hips', box(0.22, 0.36, 0.12), '0 0 0.90 0 0 0', trousers,
             True),
        part('torso', box(0.24, 0.42, 0.55), '0 0 1.22 0 0 0', shirt, True),
        part('arm_left', cyl(0.05, 0.62), '0 0.26 1.17 0 0 0', shirt),
        part('arm_right', cyl(0.05, 0.62), '0 -0.26 1.17 0 0 0', shirt),
        part('hand_left', sphere(0.05), '0 0.26 0.84 0 0 0', SKIN),
        part('hand_right', sphere(0.05), '0 -0.26 0.84 0 0 0', SKIN),
        part('neck', cyl(0.05, 0.08), '0 0 1.53 0 0 0', SKIN),
        part('head', sphere(0.11), '0 0 1.66 0 0 0', SKIN, True),
        part('hair', sphere(0.112), '-0.02 0 1.69 0 0 0', HAIR),
    ])
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {yaw:.4f}</pose>
      <link name="body">
{parts}      </link>
    </model>
'''


def main():
    """Write the people into both world files."""
    section = (f'    {BEGIN}\n    <!-- People (made by tools/make_world_people.py) -->\n'
               + ''.join(person(*p) for p in PEOPLE) + f'    {END}\n')
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
