#!/usr/bin/env python3
"""Write the benchmark scenarios to scenarios/ as PGM + text files.

S1        demo map from the old env_publisher.py
S2        parallel parking, slot = car length + 1.5 m
S2_tight  the same with car length + 1.0 m, which this car cannot do
          (S2_tight_exit is the same map driven out of the slot)
S3        dead end, the car has to back out
S4        maze with narrow corridors
S5        random obstacle fields, seeds 0..49, in scenarios/S5/

Poses in the files are the rear axle, like in the planner. Here I place
cars by their footprint centre and convert with to_rear_axle().
"""

import argparse
import math
import os

import numpy as np

FREE, OBSTACLE = 255, 0
RES = 0.1
CAR_L, CAR_W = 4.5, 2.0
REAR_AXLE_TO_CENTER = 1.25


def to_rear_axle(x, y, heading_deg):
    """Footprint centre pose -> rear axle pose (what the planner uses)."""
    th = math.radians(heading_deg)
    return (round(x - REAR_AXLE_TO_CENTER * math.cos(th), 3),
            round(y - REAR_AXLE_TO_CENTER * math.sin(th), 3), heading_deg)


def write_pgm(path, img):
    h, w = img.shape
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.astype(np.uint8).tobytes())


def write_scenario(out_dir, name, img, start, goal, comment):
    """img row 0 is the top of the map. start/goal are (x, y, heading_deg)."""
    write_pgm(os.path.join(out_dir, name + ".pgm"), img)
    with open(os.path.join(out_dir, name + ".txt"), "w") as f:
        f.write("# %s\n" % comment)
        f.write("map: %s.pgm\n" % name)
        f.write("resolution: %g\n" % RES)
        f.write("origin: 0.0 0.0\n")
        f.write("start: %g %g %g\n" % start)
        f.write("goal: %g %g %g\n" % goal)


class Canvas:
    """Draw in world meters (y up) on an image whose row 0 is the top."""

    def __init__(self, width_m, height_m):
        self.w = int(round(width_m / RES))
        self.h = int(round(height_m / RES))
        self.img = np.full((self.h, self.w), FREE, dtype=np.uint8)

    def box(self, x0, y0, x1, y1):
        c0, c1 = int(round(x0 / RES)), int(round(x1 / RES))
        r0, r1 = int(round(y0 / RES)), int(round(y1 / RES))
        c0, c1 = max(c0, 0), min(c1, self.w)
        r0, r1 = max(r0, 0), min(r1, self.h)
        self.img[self.h - r1:self.h - r0, c0:c1] = OBSTACLE

    def border(self, t=0.5):
        W, H = self.w * RES, self.h * RES
        self.box(0, 0, W, t)
        self.box(0, H - t, W, H)
        self.box(0, 0, t, H)
        self.box(W - t, 0, W, H)

    def rotated_box(self, cx, cy, length, width, angle):
        # Fill every cell whose centre is inside the rotated rectangle.
        ys, xs = np.mgrid[0:self.h, 0:self.w]
        wx = (xs + 0.5) * RES
        wy = (self.h - 1 - ys + 0.5) * RES
        c, s = math.cos(angle), math.sin(angle)
        u = (wx - cx) * c + (wy - cy) * s
        v = -(wx - cx) * s + (wy - cy) * c
        self.img[(np.abs(u) <= length / 2) & (np.abs(v) <= width / 2)] = OBSTACLE

    def disc(self, cx, cy, r):
        ys, xs = np.mgrid[0:self.h, 0:self.w]
        wx = (xs + 0.5) * RES
        wy = (self.h - 1 - ys + 0.5) * RES
        self.img[(wx - cx) ** 2 + (wy - cy) ** 2 <= r * r] = OBSTACLE

    def occupied(self):
        """Occupancy with row 0 at the bottom, like the planner's GridMap."""
        return np.flipud(self.img == OBSTACLE)


def footprint_clear(occ, x, y, theta, margin):
    """True if the car rectangle grown by margin touches no occupied cell.
    (x, y) is the footprint centre here."""
    h, w = occ.shape
    half = math.hypot(CAR_L / 2 + margin, CAR_W / 2 + margin)
    c0, c1 = int((x - half) / RES), int((x + half) / RES) + 1
    r0, r1 = int((y - half) / RES), int((y + half) / RES) + 1
    if c0 < 0 or r0 < 0 or c1 > w or r1 > h:
        return False
    rows, cols = np.mgrid[r0:r1, c0:c1]
    cx, cy = (cols + 0.5) * RES, (rows + 0.5) * RES
    c, s = math.cos(theta), math.sin(theta)
    u = (cx - x) * c + (cy - y) * s
    v = -(cx - x) * s + (cy - y) * c
    inside = (np.abs(u) <= CAR_L / 2 + margin) & (np.abs(v) <= CAR_W / 2 + margin)
    return not occ[r0:r1, c0:c1][inside].any()


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
    return img, (35, 55, 0), (50, 17, 180)


def make_s2(slack=1.5):
    # A street with a curb along the bottom and two parked cars. The free
    # slot between them is CAR_L + slack long.
    cv = Canvas(30, 14)
    cv.box(0, 0, 30, 0.5)                  # curb
    slot_x0 = 11.0
    slot_x1 = slot_x0 + CAR_L + slack
    lane_y = 0.75 + CAR_W / 2              # parked cars sit 0.25 m off the curb
    cv.box(slot_x0 - CAR_L, lane_y - CAR_W / 2, slot_x0, lane_y + CAR_W / 2)
    cv.box(slot_x1, lane_y - CAR_W / 2, slot_x1 + CAR_L, lane_y + CAR_W / 2)
    goal = to_rear_axle((slot_x0 + slot_x1) / 2, lane_y, 0)
    start = to_rear_axle(4.0, 5.5, 0)
    return cv.img, start, goal


def make_s3():
    # A 6 m wide dead-end corridor. The car starts facing the end wall and
    # the goal is outside, facing the other way.
    cv = Canvas(40, 30)
    cv.border()
    t = 0.5
    y0, y1 = 12.0, 18.0
    cv.box(20, y0 - t, 39.5, y0)           # lower wall
    cv.box(20, y1, 39.5, y1 + t)           # upper wall
    return cv.img, to_rear_axle(34, 15, 0), to_rear_axle(8, 15, 180)


def make_s4():
    # A 5x5 maze of 8 m cells cut by a seeded depth-first search, so the
    # layout is fixed. Corridors are 7.6 m wide between walls.
    n, cell, t = 5, 8.0, 0.4
    size = n * cell + t
    cv = Canvas(size, size)
    rng = np.random.default_rng(4)
    # walls[r][c] = [east, north] still standing for cell (r, c)
    east = np.ones((n, n), dtype=bool)
    north = np.ones((n, n), dtype=bool)
    seen = np.zeros((n, n), dtype=bool)
    stack = [(0, 0)]
    seen[0, 0] = True
    while stack:
        r, c = stack[-1]
        nbrs = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= r + dr < n and 0 <= c + dc < n and not seen[r + dr, c + dc]]
        if not nbrs:
            stack.pop()
            continue
        nr, nc = nbrs[rng.integers(len(nbrs))]
        if nr == r + 1:
            north[r, c] = False
        elif nr == r - 1:
            north[nr, nc] = False
        elif nc == c + 1:
            east[r, c] = False
        else:
            east[nr, nc] = False
        seen[nr, nc] = True
        stack.append((nr, nc))
    cv.border(t)
    for r in range(n):
        for c in range(n):
            x0, y0 = c * cell, r * cell
            if east[r, c] and c < n - 1:
                cv.box(x0 + cell, y0, x0 + cell + t, y0 + cell + t)
            if north[r, c] and r < n - 1:
                cv.box(x0, y0 + cell, x0 + cell + t, y0 + cell + t)
    mid = cell / 2 + t / 2
    start = to_rear_axle(mid, mid, 0)
    goal = to_rear_axle((n - 1) * cell + mid, (n - 1) * cell + mid, 90)
    return cv.img, start, goal


def reachable(occ, start, goal):
    """2D flood fill on the map with obstacles grown by half the car width."""
    from collections import deque
    grow = int(math.ceil((CAR_W / 2) / RES))
    h, w = occ.shape
    grown = occ.copy()
    ys, xs = np.nonzero(occ)
    for dy in range(-grow, grow + 1):
        for dx in range(-grow, grow + 1):
            if dx * dx + dy * dy > grow * grow:
                continue
            yy = np.clip(ys + dy, 0, h - 1)
            xx = np.clip(xs + dx, 0, w - 1)
            grown[yy, xx] = True
    sx, sy = int(start[0] / RES), int(start[1] / RES)
    gx, gy = int(goal[0] / RES), int(goal[1] / RES)
    if grown[sy, sx] or grown[gy, gx]:
        return False
    seen = np.zeros_like(grown)
    q = deque([(sy, sx)])
    seen[sy, sx] = True
    while q:
        y, x = q.popleft()
        if (y, x) == (gy, gx):
            return True
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= ny < h and 0 <= nx < w and not seen[ny, nx] and not grown[ny, nx]:
                seen[ny, nx] = True
                q.append((ny, nx))
    return False


def make_s5(seed):
    # 40 x 40 m with a border, 14 random boxes and 6 random discs. Start and
    # goal are random poses with 0.5 m of free space around the car, at least
    # 20 m apart, and connected in a 2D flood fill.
    rng = np.random.default_rng(seed)
    size = 40.0
    for _ in range(200):
        cv = Canvas(size, size)
        cv.border()
        for _ in range(14):
            cv.rotated_box(rng.uniform(3, size - 3), rng.uniform(3, size - 3),
                           rng.uniform(1.0, 6.0), rng.uniform(1.0, 3.0),
                           rng.uniform(0, math.pi))
        for _ in range(6):
            cv.disc(rng.uniform(3, size - 3), rng.uniform(3, size - 3),
                    rng.uniform(0.5, 1.5))
        occ = cv.occupied()

        def sample_pose():
            for _ in range(500):
                x, y = rng.uniform(4, size - 4, 2)
                th = rng.uniform(-math.pi, math.pi)
                if footprint_clear(occ, x, y, th, 0.5):
                    return x, y, th
            return None

        start, goal = sample_pose(), sample_pose()
        if start is None or goal is None:
            continue
        if math.hypot(start[0] - goal[0], start[1] - goal[1]) < 20.0:
            continue
        if not reachable(occ, start, goal):
            continue
        def deg(p):
            return to_rear_axle(round(p[0], 2), round(p[1], 2), round(math.degrees(p[2]), 1))
        return cv.img, deg(start), deg(goal)
    raise RuntimeError("seed %d: no valid start/goal after 200 maps" % seed)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "scenarios"))
    args = ap.parse_args()
    os.makedirs(os.path.join(args.out, "S5"), exist_ok=True)

    makers = [
        ("S1", make_s1, "S1: demo map from the old env_publisher.py"),
        ("S2", make_s2, "S2: parallel parking, slot is car length + 1.5 m"),
        ("S2_tight", lambda: make_s2(1.0),
         "S2_tight: slot is car length + 1.0 m, too short for this car (see docs/RESULTS.md)"),
        ("S3", make_s3, "S3: 6 m wide dead end, the car has to back out"),
        ("S4", make_s4, "S4: 5x5 maze, 7.6 m corridors"),
    ]
    for name, fn, comment in makers:
        img, start, goal = fn()
        write_scenario(args.out, name, img, start, goal, comment)
        print(name, img.shape, start, goal)

    # S2_tight backwards: start parked in the slot, goal out on the street.
    # If the planner cannot get out, it cannot get in either.
    _, _, parked = make_s2(1.0)
    with open(os.path.join(args.out, "S2_tight_exit.txt"), "w") as f:
        f.write("# S2_tight driven out of the slot, used to check that it is infeasible\n")
        f.write("map: S2_tight.pgm\nresolution: %g\norigin: 0.0 0.0\n" % RES)
        f.write("start: %g %g %g\n" % parked)
        f.write("goal: %g %g %g\n" % to_rear_axle(24, 5.5, 0))

    for seed in range(50):
        img, start, goal = make_s5(seed)
        write_scenario(os.path.join(args.out, "S5"), "seed_%02d" % seed, img, start, goal,
                       "S5: random obstacles, seed %d" % seed)
    print("S5: 50 seeds")


if __name__ == "__main__":
    main()
