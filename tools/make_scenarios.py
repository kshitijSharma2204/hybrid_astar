#!/usr/bin/env python3
"""Write the benchmark scenarios to scenarios/ as PGM + text files."""

import argparse
import os

import numpy as np

FREE, OBSTACLE = 255, 0


def write_pgm(path, img):
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.astype(np.uint8).tobytes())


def write_scenario(out_dir, name, img, res, start, goal, comment):
    """img row 0 is the top of the map. start/goal are (x, y, heading_deg)."""
    write_pgm(os.path.join(out_dir, name + ".pgm"), img)
    with open(os.path.join(out_dir, name + ".txt"), "w") as f:
        f.write("# %s\n" % comment)
        f.write("map: %s.pgm\n" % name)
        f.write("resolution: %g\n" % res)
        f.write("origin: 0.0 0.0\n")
        f.write("start: %g %g %g\n" % start)
        f.write("goal: %g %g %g\n" % goal)


def make_s1():
    # Same geometry as the old scripts/env_publisher.py, before its flipud.
    size = 800
    img = np.full((size, size), FREE, dtype=np.uint8)
    x0 = size // 2
    for dx in range(-4, 4):
        img[100:size - 100, x0 + dx] = OBSTACLE
    y0 = size // 2
    for offset in range(-300, 301, 100):
        yy = y0 + offset
        for dy in range(-4, 4):
            img[yy + dy, 300:size - 300] = OBSTACLE
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "scenarios"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    write_scenario(args.out, "S1", make_s1(), 0.1, (35, 55, 0), (50, 17, 180),
                   "S1: demo map from the old env_publisher.py")


if __name__ == "__main__":
    main()
