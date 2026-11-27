import matplotlib
matplotlib.use("Agg")

import numpy as np
import pytest

import hybrid_astar as ha
from hybrid_astar_viz import animate_search, car_polygon, plot_result


def small_plan(record_trace):
    grid = np.zeros((150, 250), dtype=np.int8)
    grid[:100, 120:124] = 100
    g = ha.GridMap(grid, 0.1)
    r = ha.plan(g, (5.0, 4.0, 0.0), (20.0, 4.0, 0.0), ha.PlannerConfig(),
                record_trace=record_trace)
    assert r.success, r.message
    return g, r


def test_car_polygon_size():
    p = car_polygon(1.0, 2.0, np.pi / 2, 4.5, 2.0)
    # turned 90 degrees, so 2.0 m wide in x and 4.5 m long in y
    assert np.ptp(p[:, 0]) == pytest.approx(2.0)
    assert np.ptp(p[:, 1]) == pytest.approx(4.5)


def test_plot_and_animate(tmp_path):
    g, r = small_plan(record_trace=True)
    cfg = ha.PlannerConfig()
    png = tmp_path / "p.png"
    plot_result(g, r, cfg, start=(5.0, 4.0, 0.0), goal=(20.0, 4.0, 0.0), out=str(png))
    assert png.stat().st_size > 1000
    gif = tmp_path / "a.gif"
    animate_search(g, r, cfg, str(gif), width_px=200, search_frames=5, drive_frames=3)
    assert gif.stat().st_size > 1000
