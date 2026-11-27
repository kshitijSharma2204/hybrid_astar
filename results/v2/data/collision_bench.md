100000 poses on 50 maps, 55964 (56.0%) collide in the exact check

| check | misses a collision | over-cautious (% of free poses) | ns per pose | build ms per map |
|---|---|---|---|---|
| exact (every touched cell) | - | - | 1132 | - |
| circles x3 | 0 | 5194 (11.79%) | 56 | 2.92 |
| circles x4 | 0 | 5091 (11.56%) | 59 | 2.73 |
| circles x4 + exact (mode 2) | 0 | 0 (0.00%) | 422 | 2.73 |
