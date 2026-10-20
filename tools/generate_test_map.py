#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate a static navigation map from the AIGC test world.

Reads catkin_ws/src/aigc_sim/worlds/test.world, extracts the axis-aligned box
collision geometry of the static walls and the test object, and writes a
map_server-compatible PGM + YAML pair into aigc_navigation/maps/.

Python 3 standard library only (no ROS, no numpy/opencv), so it also runs on
Windows. It only supports the current scene's static, axis-aligned boxes; any
rotated model or non-box geometry raises an error instead of silently
producing a wrong map.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

# Map settings (must match the YAML written below).
MAP_RESOLUTION = 0.02     # m / pixel
MAP_WIDTH = 200           # pixels
MAP_HEIGHT = 200          # pixels
MAP_ORIGIN_X = -2.0       # world x of the left edge
MAP_ORIGIN_Y = -2.0       # world y of the bottom edge
MAP_MAX_X = MAP_ORIGIN_X + MAP_WIDTH * MAP_RESOLUTION   # 2.0
MAP_MAX_Y = MAP_ORIGIN_Y + MAP_HEIGHT * MAP_RESOLUTION  # 2.0

FREE = 254      # light gray -> free (map_server convention)
OCCUPIED = 0    # black -> occupied


def parse_pose(text):
    """Parse an SDF <pose> (x y z roll pitch yaw) into 6 floats."""
    parts = text.split()
    if len(parts) != 6:
        raise ValueError('pose must have exactly 6 values, got: %r' % text)
    return [float(p) for p in parts]


def parse_size(text):
    """Parse an SDF box <size> (x y z) into 3 floats."""
    parts = text.split()
    if len(parts) != 3:
        raise ValueError('box <size> must have exactly 3 values, got: %r' % text)
    return [float(p) for p in parts]


def world_to_grid(x, y):
    """Map a world point to (col, row) in the image grid."""
    col = int(math.floor((x - MAP_ORIGIN_X) / MAP_RESOLUTION))
    row = int(math.floor((MAP_MAX_Y - y) / MAP_RESOLUTION))
    return col, row


def rasterize(grid, xmin, xmax, ymin, ymax):
    """Mark every pixel overlapping [xmin,xmax]x[ymin,ymax] as occupied.

    floor()/ceil() are used so the rasterized rectangle is never smaller than
    the real collision box.
    """
    c0 = int(math.floor((xmin - MAP_ORIGIN_X) / MAP_RESOLUTION))
    c1 = int(math.ceil((xmax - MAP_ORIGIN_X) / MAP_RESOLUTION)) - 1
    r0 = int(math.floor((MAP_MAX_Y - ymax) / MAP_RESOLUTION))
    r1 = int(math.ceil((MAP_MAX_Y - ymin) / MAP_RESOLUTION)) - 1

    c0 = max(0, min(MAP_WIDTH - 1, c0))
    c1 = max(0, min(MAP_WIDTH - 1, c1))
    r0 = max(0, min(MAP_HEIGHT - 1, r0))
    r1 = max(0, min(MAP_HEIGHT - 1, r1))

    for r in range(r0, r1 + 1):
        row = grid[r]
        for c in range(c0, c1 + 1):
            row[c] = OCCUPIED


def write_pgm(path, grid):
    """Write a P5 (binary) PGM: header + one byte per pixel, row-major."""
    header = 'P5\n%d %d\n255\n' % (MAP_WIDTH, MAP_HEIGHT)
    data = bytearray()
    for row in grid:
        data.extend(row)
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(bytes(data))


def write_yaml(path):
    """Write the map_server metadata file."""
    lines = [
        'image: test.pgm',
        'resolution: %.2f' % MAP_RESOLUTION,
        'origin: [%.1f, %.1f, 0.0]' % (MAP_ORIGIN_X, MAP_ORIGIN_Y),
        'negate: 0',
        'occupied_thresh: 0.65',
        'free_thresh: 0.196',
    ]
    # newline='\n' keeps LF even on Windows (the project stores LF everywhere).
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(
        repo_root, 'catkin_ws', 'src', 'aigc_sim', 'worlds', 'test.world')
    if not os.path.isfile(world_path):
        print('ERROR: world file not found: %s' % world_path, file=sys.stderr)
        return 1

    tree = ET.parse(world_path)
    root = tree.getroot()

    # grid[r][c]: r = image row (0 = top = max y), c = column (0 = left = min x)
    grid = [[FREE] * MAP_WIDTH for _ in range(MAP_HEIGHT)]
    obstacles = []

    for model in root.iter('model'):
        name = model.get('name', '')

        static_el = model.find('static')
        is_static = (static_el is not None and
                     static_el.text is not None and
                     static_el.text.strip().lower() == 'true')
        if not is_static:
            continue

        cx = cy = cz = 0.0
        pose_el = model.find('pose')
        if pose_el is not None and pose_el.text and pose_el.text.strip():
            cx, cy, cz, roll, pitch, yaw = parse_pose(pose_el.text.strip())
            if abs(roll) > 1e-9 or abs(pitch) > 1e-9 or abs(yaw) > 1e-9:
                raise ValueError(
                    'Unsupported rotated model %r (roll/pitch/yaw must be 0)'
                    % name)

        for link in model.findall('link'):
            # Reject link-level poses (not used in the current scene) rather
            # than silently mis-placing the geometry.
            link_pose_el = link.find('pose')
            if link_pose_el is not None and link_pose_el.text and \
                    link_pose_el.text.strip():
                raise ValueError(
                    'Unsupported link-level pose in model %r' % name)

            for collision in link.findall('collision'):
                geom = collision.find('geometry')
                if geom is None:
                    continue
                box = geom.find('box')
                if box is None:
                    raise ValueError(
                        'Unsupported geometry in model %r (only axis-aligned '
                        'box is supported)' % name)
                size_el = box.find('size')
                if size_el is None or not (size_el.text and size_el.text.strip()):
                    raise ValueError('box without <size> in model %r' % name)
                sx, sy, sz = parse_size(size_el.text.strip())

                xmin = cx - sx / 2.0
                xmax = cx + sx / 2.0
                ymin = cy - sy / 2.0
                ymax = cy + sy / 2.0
                zmax = cz + sz / 2.0

                # Ground: skip. It is either named as ground or does not
                # extend above z = 0 (the ground plane top).
                if 'ground' in name.lower() or zmax <= 0.0:
                    continue

                obstacles.append((name, xmin, xmax, ymin, ymax))
                rasterize(grid, xmin, xmax, ymin, ymax)

    print('Obstacles extracted from collision geometry:')
    for name, xmin, xmax, ymin, ymax in obstacles:
        print('  %-14s x=[%.3f, %.3f] y=[%.3f, %.3f]'
              % (name, xmin, xmax, ymin, ymax))

    out_dir = os.path.join(
        repo_root, 'catkin_ws', 'src', 'aigc_navigation', 'maps')
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, 'test.pgm')
    yaml_path = os.path.join(out_dir, 'test.yaml')

    write_pgm(pgm_path, grid)
    write_yaml(yaml_path)
    print('wrote %s' % pgm_path)
    print('wrote %s' % yaml_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
