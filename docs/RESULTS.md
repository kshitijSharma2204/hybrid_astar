# Results

All numbers here come from runs of committed code. Unless a section says
otherwise they were measured at commit `fb06823` on one machine:

- CPU: Intel Core i7-10870H (8 cores, the planner uses one)
- Ubuntu 24.04, kernel 6.8, g++ 13.3.0, `CMAKE_BUILD_TYPE=Release`
- OMPL 1.5.2, pybind11 2.13.6

The raw CSVs and summary tables are in `results/v2/data/`. To repeat them:

```bash
git checkout fb06823
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DPython_EXECUTABLE=$PWD/.venv/bin/python
cmake --build build
tools/benchmark.py --repeat 20 --stats benchmark.csv
for w in 1 1.5 2 5 50; do tools/benchmark.py --stats weight_sweep.csv --label "w=$w" --set heuristic_weight=$w; done
for rp in 1.5 10; do for dc in 5 150; do
  tools/benchmark.py --stats cost_sweep.csv --label "rp=$rp dc=$dc" \
    --set reverse_penalty=$rp --set direction_change_cost=$dc; done; done
for h in 0 1; do tools/benchmark.py --stats rs_heuristic.csv --label "rs_heuristic=$h" --set rs_heuristic=$h; done
for m in 0 1 2; do tools/benchmark.py --stats collision_modes.csv --label "collision_mode=$m" --set collision_mode=$m; done
tools/benchmark.py --stats smoothing_off.csv --label off --set smooth_enabled=0
build/collision_bench 2000
tools/compare_v1.py --v1-cli <v1 build>/hybrid_astar_cli --out v1_vs_v2.csv
tools/make_media.py
```

`tools/benchmark.py --summary-only FILE.csv` prints the tables below from a
CSV. For the v1 build I check out `7719e2a` in a separate worktree and build
`hybrid_astar_cli` there.

## Scenarios

| | map | what it tests |
|---|---|---|
| S1 | 80 x 80 m, 0.1 m cells | the demo map from v1's `env_publisher.py` |
| S2 | 30 x 14 m | parallel parking, slot = car length + 1.5 m |
| S2_tight | same, slot = car length + 1.0 m | too short for this car, see below |
| S3 | 40 x 30 m | 6 m wide dead end, the car has to back out |
| S4 | 40 x 40 m | 5x5 maze, 7.6 m corridors |
| S5 | 40 x 40 m, 50 seeds | 14 random boxes and 6 discs, random start and goal at least 20 m apart |

`tools/make_scenarios.py` writes all of them. Poses are the rear axle.

## Benchmark (default parameters)

S1-S4 planned 20 times each, S5 once per seed. "search ms" is the search
only. Before it, the distance map is built once per map: 12 ms for S1, about
3 ms for the 40 x 40 m maps. "smooth ms" is the smoother. Curvatures are in
1/m; full steer is 1/R_min = 0.1932, and 1.05 / R_min = 0.2029. Clearance is
from the car footprint to the nearest occupied cell centre, over the whole
path.

| scenario | runs | success | search ms median | search ms p95 | smooth ms median | expansions median | length m | direction switches | max curvature raw | max curvature smoothed | min clearance m | stretches put back |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | 20 | 20/20 | 306.2 | 329.3 | 9.5 | 7482 | 104.2 | 3 | 0.1932 | 0.1973 | 0.07 | 0 |
| S2 | 20 | 20/20 | 119.5 | 123.3 | 2.4 | 2578 | 21.8 | 5 | 0.1932 | 0.2001 | 0.06 | 0 |
| S3 | 20 | 20/20 | 14.3 | 15.3 | 2.0 | 10 | 29.4 | 1 | 0.1932 | 0.1972 | 1.15 | 0 |
| S4 | 20 | 20/20 | 204.0 | 216.7 | 8.4 | 6220 | 93.3 | 4 | 0.1932 | 0.2011 | 0.09 | 0 |
| S5 | 50 | 50/50 | 27.1 | 66.7 | 3.7 | 196 | 35.5 | 1 | 0.1932 | 0.2021 | 0.06 | 3 |

S2_tight: no path after 200000 expansions (8.4 s).

The search is deterministic, so repeated runs give the same path. "Stretches
put back" counts the pieces between cusps where the smoothed version broke a
limit and the raw one was kept.

On every successful run the smoothed path stays under 1.05 / R_min (largest
value 0.2021, which is 1.046 / R_min), every pose is free in the exact
collision check, and the start, goal and cusp poses do not move. The unit
tests check all three on S1-S4 and one S5 seed.

The path ends on the goal pose: the largest error over all runs is
1.3e-14 m and 1.3e-15 rad.

## v1 vs v2

`tools/compare_v1.py` at `fb06823`, with v1 built from `7719e2a` (the v1
planner with ROS taken out and nothing else changed). Each scenario is
planned once. v1 has no expansion limit, so it gets 30 s and 4 GB per run.
v1 puts the centre of the car on the pose, so it gets the poses moved 1.25 m
forward: the same physical start and goal.

Times are the whole call. For v1 that includes building its unused k-d
tree; for v2 the distance map, the search and the smoother.

| scenario | version | success | time ms (median) | goal position error m (median) | goal heading error rad (median) |
|---|---|---|---|---|---|
| S1 | v1 | 1/1 | 299.7 | 0.376 | 0.143 |
| S1 | v2 | 1/1 | 323.7 | 0 | 0 |
| S2 | v1 | 0/1 | 11823.8 | - | - |
| S2 | v2 | 1/1 | 119.8 | 7e-16 | 0 |
| S3 | v1 | 1/1 | 1855.3 | 0.467 | 0.243 |
| S3 | v2 | 1/1 | 19.7 | 7e-15 | 0 |
| S4 | v1 | 1/1 | 483.3 | 0.345 | 0.218 |
| S4 | v2 | 1/1 | 212.7 | 0 | 0 |
| S5 | v1 | 39/50 | 518.0 | 0.381 | 0.179 |
| S5 | v2 | 50/50 | 33.9 | 3e-15 | 6e-17 |

v1 cannot reach the exact goal: it stops anywhere within 0.5 m and 0.3 rad.
On S2 it searches the whole space and finds nothing (open set empty after
11.8 s). All 11 S5 failures hit the 30 s limit. On S1 both take about the
same time, but v1's path ends 0.38 m off the goal.

`results/v2/baseline_v1_S1.csv` is the S1 path from `7719e2a` with the
original v1 poses, before any of the changes.

## Heuristic weight

`heuristic_weight` multiplies the heuristic, so w = 1 is plain A* and w > 1
expands fewer nodes but accepts longer paths. One run per scenario (50 for
S5); "fail" means 200000 expansions without a path.

| w | S1 | S2 | S3 | S4 | S5 success | S5 expansions median | S5 length m median | S5 switches median | S5 search ms median / p95 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | fail | fail | 20 exp, 29.4 m | fail | 50/50 | 2106 | 33.5 | 1 | 95.6 / 1097.9 |
| 1.5 | fail | fail | 10 exp, 29.4 m | 145320 exp, 88.5 m | 50/50 | 764 | 33.4 | 1 | 47.6 / 612.5 |
| 2 | fail | 185030 exp, 23.8 m | 10 exp, 29.4 m | 47140 exp, 89.7 m | 50/50 | 285 | 34.0 | 1 | 28.9 / 163.7 |
| 5 | 7482 exp, 104.2 m | 2578 exp, 21.8 m | 10 exp, 29.4 m | 6220 exp, 93.3 m | 50/50 | 196 | 35.5 | 1 | 26.3 / 64.3 |
| 50 | 10230 exp, 119.8 m | 1091 exp, 28.0 m | 10 exp, 29.4 m | 440 exp, 108.7 m | 50/50 | 88 | 35.5 | 6 | 23.3 / 125.3 |

I picked w = 5. It is the smallest weight that solves S1, S2 and S4 within
the expansion limit. Going to 50 saves expansions but gives many more
direction switches (S1: 25 instead of 3, S5 median: 6 instead of 1). On S5,
where every weight succeeds, the median w = 5 path is 2 m (6%) longer than
the median w = 1 path.

S1 shows the cost of the weight most clearly: the w = 5 path backs up most
of its 104 m instead of turning around first.

`results/v2/weights_w1_vs_w50.png` and the two `S5_seed_36_w*_search.gif`
show one S5 map searched with w = 1 (9005 expansions, 33.1 m, 2 switches)
and w = 50 (230 expansions, 36.8 m, 6 switches).

## Reverse penalty and direction change cost

v1's code used `reverse_penalty` 10 while the old params.yaml said 1.5, and
it charged 150 per direction change. One run per scenario:

| reverse_penalty | direction_change_cost | S1 | S2 | S4 | S5 success | S5 search ms median / p95 |
|---|---|---|---|---|---|---|
| 1.5 | 5 | 7482 exp | 2578 exp | 6220 exp | 50/50 | 26.3 / 63.1 |
| 1.5 | 150 | 6966 exp | fail | 8040 exp | 50/50 | 27.8 / 197.9 |
| 10 | 5 | 21070 exp | 63369 exp | 8330 exp | 50/50 | 30.0 / 309.7 |
| 10 | 150 | fail | fail | 8040 exp | 50/50 | 30.4 / 1264.9 |

I use 1.5 and 5. Parking needs several short back-and-forth moves, and at
150 per switch the search cannot afford them, so S2 fails. With
reverse_penalty 10, S2 takes 25 times the expansions and S1 almost 3 times.

## Reeds-Shepp heuristic

h = max(2D distance with obstacles, obstacle-free Reeds-Shepp length),
switched by `rs_heuristic`:

| rs_heuristic | S1 | S2 | S4 | S5 search ms median / p95 | S5 length m median |
|---|---|---|---|---|---|
| 0 (2D only) | 4840 exp, 150.6 ms | fail | 6650 exp, 118.9 ms | 23.7 / 64.6 | 37.0 |
| 1 | 7482 exp, 311.7 ms | 2578 exp, 124.6 ms | 6220 exp, 200.2 ms | 27.2 / 64.1 | 35.5 |

Without it S2 fails: next to the slot the 2D distance is close to zero at
any heading, so the search cannot tell good parking poses from bad ones. It
costs time on the larger maps (S1 takes twice as long), since every new node
computes a Reeds-Shepp length.

## Collision checking

Three checks, all of the 4.5 x 2.0 m car rectangle on the grid:

- exact: every cell the rectangle touches, row by row;
- circles: 4 discs along the centre line that cover the rectangle. A pose
  collides when a disc centre is within r + one cell diagonal of an
  occupied cell centre, read from a distance map;
- mode 2 (default): circles first, and the exact check only for poses the
  circles reject.

`collision_bench 2000`: 100000 random poses on the 50 S5 maps, of which
55964 (56%) collide in the exact check.

| check | free where exact says collision | collision where exact says free | ns per pose | build ms per map |
|---|---|---|---|---|
| exact | - | - | 1132 | - |
| circles x3 | 0 | 5194 (11.8% of free poses) | 56 | 2.92 |
| circles x4 | 0 | 5091 (11.6%) | 59 | 2.73 |
| circles x4, then exact (mode 2) | 0 | 0 | 422 | 2.73 |

At commit `650b397` the same benchmark also had a nanoflann k-d tree behind
the circles, and v1's check, which looks at the corners and at points every
0.2 m along the outline:

| check | free where exact says collision | collision where exact says free | ns per pose |
|---|---|---|---|
| v1 outline | 490 | 0 | 371 |
| circles x4, k-d tree | 0 | 3979 (9.0%) | 1025 |
| circles x4, distance map | 0 | 5091 (11.6%) | 68 |

The distance map is 15 times faster than the k-d tree and a bit more
cautious, because it only knows cell centres. I kept the distance map and
deleted the k-d tree. The outline check missed 1 collision in 114, and on S5
it let paths through with an obstacle cell centre inside the footprint. That
is why the reference is now the exact cell check.

Effect on planning, one run per scenario:

| collision_mode | S1 | S2 | S4 | S5 success | S5 search ms median |
|---|---|---|---|---|---|
| 0 exact | 680.9 ms | 290.2 ms | 523.6 ms | 50/50 | 34.6 |
| 1 circles | 345.9 ms, other path | fail | 142.1 ms, other path | 49/50 | 23.8 |
| 2 circles, then exact | 311.8 ms | 118.9 ms | 206.2 ms | 50/50 | 27.5 |

Modes 0 and 2 give exactly the same paths and expansions, and mode 2 takes
about half the time. Circles alone are too cautious past the bumpers (the
end discs reach about 0.6 m beyond the car), which rules out S2 and one S5
map.

## Smoothing

| | S1 | S2 | S3 | S4 | S5 (worst) |
|---|---|---|---|---|---|
| max curvature, raw | 0.1932 | 0.1932 | 0.1932 | 0.1932 | 0.1932 |
| max curvature, smoothed | 0.1973 | 0.2001 | 0.1972 | 0.2011 | 0.2021 |
| min clearance m, smoothing off | 0.08 | 0.06 | 1.14 | 0.06 | 0.06 |
| min clearance m, smoothing on | 0.07 | 0.06 | 1.15 | 0.09 | 0.06 |

The raw path is made of full-steer arcs, half-steer arcs and straight
pieces, so its curvature jumps between those values. The smoothed path's
curvature changes continuously and can go slightly over 1/R_min where it
blends two pieces. Clearance barely changes: the tightest spots are usually
the goal pose and the cusps, which the smoother does not move.

## S2: how long a parking slot this car needs

The plan asked for a slot of car length + 1.0 m. The search cannot get the
car into it. To check that this is the geometry and not the search, I ran
the search the other way round, from the parked pose out to the street, with
much finer settings (0.2 m steps, 2 degree heading bins, 5 cm cells, 5
steering angles, w = 1.5). If the car cannot get out, it cannot get in. Maps
from `make_s2(slack)` in `tools/make_scenarios.py`, commit `b4b8af0`:

| slot length | result |
|---|---|
| car + 1.0 m | open set empty after 139 expansions |
| car + 1.1 m | open set empty after 172 expansions |
| car + 1.2 m | out after 325 expansions |
| car + 1.5 m | out after 449 expansions |

At `fb06823` the 1.0 m case (`scenarios/S2_tight_exit.txt`, command in
`results/v2/data/s2_tight.txt`) gives the same answer: empty after 149
expansions. The exact cell check finds a few more free poses than the old
outline check. S2 uses car + 1.5 m.

## Own code and libraries

- Written here: the search, motion primitives, both heuristics, the closed
  set, analytic expansion, the collision checks and the distance transform,
  the smoother, path metrics, the config and scenario parsers, the CLI and
  the plotting.
- OMPL: only `ompl::base::ReedsSheppStateSpace`, for the length of the
  shortest Reeds-Shepp curve and for points along it
  (`src/reeds_shepp.cpp`).
- pybind11: only the Python bindings (`python/bindings.cpp`).
- GoogleTest and pytest for the tests, numpy and matplotlib for the plots.
