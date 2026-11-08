#!/usr/bin/env python3
"""Generate a visual semantic-region PGM from the Small House occupancy map."""

from pathlib import Path
import sys

import yaml


def read_pgm(path):
    raw = path.read_bytes()
    index = 0

    def token():
        nonlocal index
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        while raw[index:index + 1] == b'#':
            while index < len(raw) and raw[index:index + 1] not in (b'\n', b'\r'):
                index += 1
            while index < len(raw) and raw[index:index + 1].isspace():
                index += 1
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        return raw[start:index]

    magic = token()
    width, height, maximum = int(token()), int(token()), int(token())
    if magic == b'P5':
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        pixels = list(raw[index:index + width * height])
    else:
        pixels = [int(token()) for _ in range(width * height)]
    return width, height, maximum, pixels


def inside(point, polygon):
    x, y = point
    inside_value = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside_value = not inside_value
        previous = current
    return inside_value


def main():
    root = Path(__file__).resolve().parents[1]
    map_yaml = root / 'maps' / 'small_house_slam.yaml'
    zones_yaml = root / 'config' / 'zones.yaml'
    output_pgm = root / 'maps' / 'small_house_regions.pgm'
    output_yaml = root / 'maps' / 'small_house_regions.yaml'
    metadata = yaml.safe_load(map_yaml.read_text())
    width, height, maximum, pixels = read_pgm(root / 'maps' / metadata['image'])
    zones = yaml.safe_load(zones_yaml.read_text())['zones']
    resolution = float(metadata['resolution'])
    origin_x, origin_y = metadata['origin'][:2]
    values = list(pixels)
    for row in range(height):
        for column in range(width):
            index = row * width + column
            if pixels[index] < 250:
                continue
            x = origin_x + (column + 0.5) * resolution
            y = origin_y + (height - row - 0.5) * resolution
            for zone in zones.values():
                polygon = zone['polygon']
                if inside((x, y), polygon):
                    values[index] = min(245, 40 + int(zone['id']) * 45)
                    break
    with output_pgm.open('wb') as stream:
        stream.write(f'P5\n{width} {height}\n{maximum}\n'.encode())
        stream.write(bytes(values))
    output_yaml.write_text(
        'image: small_house_regions.pgm\n'
        f'resolution: {resolution}\n'
        f'origin: [{origin_x}, {origin_y}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    print(f'Wrote {output_pgm} and {output_yaml}')


if __name__ == '__main__':
    main()
