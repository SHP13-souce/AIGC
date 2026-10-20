#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate the AR test-target marker textures (grayscale PNG) used by aigc_sim.

Each marker is a 9x9 grid (0 = black, 1 = white) taken from the first row of
the official ar_track_alvar Markers_0_to_8.png. Every cell is scaled to
``CELL`` pixels, producing a 180x180 lossless 8-bit grayscale PNG (black=0,
white=255). No smoothing, no mirroring, no text is applied.

Python 3 standard library only (struct + zlib), so it also runs on Windows.
The generated PNGs are committed to the repository; Ubuntu does not need to
run this generator before use.

Source pattern reference:
  https://raw.githubusercontent.com/ros-perception/ar_track_alvar/38c599eecd0bf0f4da02164c8401308d12246d82/ar_track_alvar/markers/Markers_0_to_8.png
"""
import os
import struct
import sys
import zlib

CELL = 20          # pixels per grid cell (9 * 20 = 180)
GRID = 9

# 9x9 patterns, first row = top row of the marker. 0 = black, 1 = white.
PATTERNS = {
    0: [
        '000000000',
        '000000000',
        '001101100',
        '001101100',
        '001010100',
        '001111100',
        '001111100',
        '000000000',
        '000000000',
    ],
    1: [
        '000000000',
        '000000000',
        '001101100',
        '001101100',
        '001010100',
        '000011000',
        '001110100',
        '000000000',
        '000000000',
    ],
    2: [
        '000000000',
        '000000000',
        '001101100',
        '001101100',
        '001010100',
        '001011000',
        '001011000',
        '000000000',
        '000000000',
    ],
}


def _chunk(typ, data):
    """Build one PNG chunk (length + type + data + CRC)."""
    out = struct.pack('>I', len(data)) + typ + data
    out += struct.pack('>I', zlib.crc32(typ + data) & 0xffffffff)
    return out


def write_grayscale_png(path, width, height, rows):
    """Write an 8-bit grayscale PNG (color type 0, filter type 0)."""
    signature = b'\x89PNG\r\n\x1a\n'
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    raw = b''.join(b'\x00' + bytes(row) for row in rows)
    idat = zlib.compress(raw, 9)
    with open(path, 'wb') as f:
        f.write(signature)
        f.write(_chunk(b'IHDR', ihdr))
        f.write(_chunk(b'IDAT', idat))
        f.write(_chunk(b'IEND', b''))


def marker_rows(pattern):
    """Scale a 9x9 pattern into a 180x180 list of pixel rows (0..255)."""
    size = GRID * CELL
    rows = []
    for row_str in pattern:
        for _ in range(CELL):
            rows.append([255 if row_str[c] == '1' else 0
                         for c in range(GRID) for _ in range(CELL)])
    assert len(rows) == size and all(len(r) == size for r in rows)
    return rows


def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = os.path.join(
        repo_root, 'catkin_ws', 'src', 'aigc_sim', 'materials', 'textures')
    os.makedirs(out_dir, exist_ok=True)

    for marker_id, pattern in PATTERNS.items():
        path = os.path.join(out_dir, 'marker_%d.png' % marker_id)
        write_grayscale_png(path, GRID * CELL, GRID * CELL,
                            marker_rows(pattern))
        print('wrote %s (%dx%d, marker %d)'
              % (path, GRID * CELL, GRID * CELL, marker_id))
    return 0


if __name__ == '__main__':
    sys.exit(main())
