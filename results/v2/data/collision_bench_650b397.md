100000 poses on 50 maps, 55964 (56.0%) collide in the exact check

| check | misses a collision | over-cautious (% of free poses) | ns per pose | build ms per map |
|---|---|---|---|---|
| exact (every touched cell) | - | - | 1178 | 0 |
| outline (v1) | 490 | 0 (0.00%) | 371 | 0.00 |
| circles x3, k-d tree | 0 | 4128 (9.37%) | 838 | 2.77 |
| circles x3, distance map | 0 | 5194 (11.79%) | 63 | 3.07 |
| circles x4, k-d tree | 0 | 3979 (9.04%) | 1025 | 2.77 |
| circles x4, distance map | 0 | 5091 (11.56%) | 68 | 3.10 |
| circles x4 + exact (mode 2) | 0 | 0 (0.00%) | 444 | 3.10 |
