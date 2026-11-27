import csv
import math
import os
import subprocess

import numpy as np
import pytest

import hybrid_astar as ha

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CLI = os.environ.get("HYBRID_ASTAR_CLI", os.path.join(ROOT, "build", "hybrid_astar_cli"))
PARAMS = os.path.join(ROOT, "config", "params.yaml")


def scenario(name):
    return os.path.join(ROOT, "scenarios", name + ".txt")


def test_config_matches_yaml():
    import yaml
    cfg = ha.load_config(PARAMS)
    with open(PARAMS) as f:
        values = yaml.safe_load(f)
    assert set(values) == set(ha.PlannerConfig.keys())
    for key, value in values.items():
        assert getattr(cfg, key) == pytest.approx(value), key


def test_bad_config_value_raises():
    cfg = ha.PlannerConfig()
    with pytest.raises(Exception):
        cfg.set("no_such_key", 1.0)
    with pytest.raises(Exception):
        cfg.wheel_base = -1.0


def test_grid_round_trip():
    sc = ha.load_scenario(scenario("S1"))
    data = sc.grid.data
    assert data.shape == (800, 800) and data.dtype == np.int8
    g = ha.GridMap(data, sc.grid.resolution, sc.grid.origin)
    assert np.array_equal(g.data, data)


def test_python_and_cli_give_same_s1_path(tmp_path):
    if not os.path.exists(CLI):
        pytest.skip("build the CLI first (cmake --build build)")
    out = tmp_path / "s1.csv"
    subprocess.run([CLI, "--scenario", scenario("S1"), "--config", PARAMS, "--out", str(out)],
                   check=True, capture_output=True)
    with open(out) as f:
        rows = [r for r in csv.DictReader(f)]

    sc = ha.load_scenario(scenario("S1"))
    r = ha.plan(sc.grid, sc.start, sc.goal, ha.load_config(PARAMS))
    assert r.success
    assert len(r.x) == len(rows)
    for i, row in enumerate(rows):
        assert r.x[i] == pytest.approx(float(row["x"]), abs=1e-5)
        assert r.y[i] == pytest.approx(float(row["y"]), abs=1e-5)
        assert r.theta[i] == pytest.approx(float(row["theta"]), abs=1e-5)
        assert r.direction[i] == int(row["direction"])


def test_plan_from_numpy_array():
    grid = np.zeros((200, 300), dtype=np.int8)
    grid[:, 150] = 100
    grid[60:140, 150] = 0          # a 8 m gap in a wall at x = 15 m
    r = ha.plan(grid, (5.0, 10.0, 0.0), (25.0, 10.0, 0.0), resolution=0.1)
    assert r.success, r.message
    assert math.hypot(r.x[-1] - 25.0, r.y[-1] - 10.0) < 0.5
    with pytest.raises(ValueError):
        ha.plan(grid, (5.0, 10.0, 0.0), (25.0, 10.0, 0.0))   # no resolution


def test_trace_is_consistent():
    sc = ha.load_scenario(scenario("S1"))
    cfg = ha.load_config(PARAMS)
    r = ha.plan(sc.grid, sc.start, sc.goal, cfg, record_trace=True)
    nodes = r.trace.nodes
    assert len(nodes) == r.stats.expansions
    assert nodes[0, 3] == -1
    parents = nodes[1:, 3].astype(int)
    assert (parents >= 0).all() and (parents < np.arange(1, len(nodes))).all()
    assert ha.plan(sc.grid, sc.start, sc.goal, cfg).trace is None


def test_progress_callback_sees_every_node():
    sc = ha.load_scenario(scenario("S1"))
    chunks = []
    r = ha.plan(sc.grid, sc.start, sc.goal, ha.load_config(PARAMS),
                on_progress=lambda n: chunks.append(n.copy()), every=500)
    seen = sum(len(c) for c in chunks)
    assert seen == (r.stats.expansions // 500) * 500
