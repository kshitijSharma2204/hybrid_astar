#!/usr/bin/env python3
"""Write the pictures and animations in results/v2/.

    tools/make_media.py            # everything
    tools/make_media.py --no-gif   # PNGs only

PNGs for S1-S4 and S2_tight, one overview PNG of all S5 seeds, search
animations for S1 and S2, a comparison of heuristic weight 1 and 50 on
S5 seed 36 (as a PNG and two GIFs), and S1 with the search settings before
and after commit 78ae297.
"""

import argparse
import glob
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "python"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import hybrid_astar as ha
from hybrid_astar_viz import animate_search, draw_map, plot_result
from hybrid_astar_viz.animate import TREE, _tree_segments
from hybrid_astar_viz.plot import draw_car, draw_path
from matplotlib.collections import LineCollection

OUT = os.path.join(ROOT, "results", "v2")
PARAMS = os.path.join(ROOT, "config", "params.yaml")


def scenario(name):
    return ha.load_scenario(os.path.join(ROOT, "scenarios", name + ".txt"))


def config(**overrides):
    cfg = ha.load_config(PARAMS)
    for k, v in overrides.items():
        cfg.set(k, v)
    return cfg


def summary(name, r):
    length = float(np.sum(np.hypot(np.diff(r.x), np.diff(r.y)))) if r.success else 0.0
    return "%s: %s, %d expansions, %.0f ms search, %.1f m" % (
        name, "ok" if r.success else "no path", r.stats.expansions, r.stats.search_ms, length)


def single_pngs():
    for name in ("S1", "S2", "S2_tight", "S3", "S4"):
        sc, cfg = scenario(name), config()
        r = ha.plan(sc.grid, sc.start, sc.goal, cfg)
        title = summary(name, r)
        plot_result(sc.grid, r, cfg, start=tuple(sc.start), goal=tuple(sc.goal),
                    out=os.path.join(OUT, name + ".png"), title=title)
        print(title)


def s5_overview():
    files = sorted(glob.glob(os.path.join(ROOT, "scenarios", "S5", "seed_*.txt")))
    cfg = config()
    fig, axes = plt.subplots(5, 10, figsize=(25, 13))
    ok = 0
    for ax, f in zip(axes.flat, files):
        sc = ha.load_scenario(f)
        r = ha.plan(sc.grid, sc.start, sc.goal, cfg)
        ok += r.success
        draw_map(ax, sc.grid)
        draw_car(ax, *sc.start, cfg, color="#2a9d3c", lw=1.0)
        draw_car(ax, *sc.goal, cfg, color="#c2185b", lw=1.0)
        if r.success:
            draw_path(ax, r.x, r.y, r.direction, lw=1.2)
        ax.set_title("%s  %d exp" % (sc.name, r.stats.expansions), fontsize=8)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlabel("")
        ax.set_ylabel("")
    fig.suptitle("S5: %d/%d seeds solved with the default config (blue forward, red reverse)"
                 % (ok, len(files)))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "S5_overview.png"), dpi=60)
    plt.close(fig)
    print("S5 overview: %d/%d" % (ok, len(files)))


def weight_comparison(gif):
    sc = scenario("S5/seed_36")
    fig, axes = plt.subplots(1, 2, figsize=(14, 7.4))
    for ax, w in zip(axes, (1, 50)):
        cfg = config(heuristic_weight=w)
        r = ha.plan(sc.grid, sc.start, sc.goal, cfg, record_trace=True)
        draw_map(ax, sc.grid)
        ax.add_collection(LineCollection(_tree_segments(r.trace.nodes), colors=TREE,
                                         linewidths=0.5, zorder=2))
        draw_car(ax, *sc.start, cfg, color="#2a9d3c", lw=1.5)
        draw_car(ax, *sc.goal, cfg, color="#c2185b", lw=1.5)
        draw_path(ax, r.x, r.y, r.direction, lw=2.0)
        length = float(np.sum(np.hypot(np.diff(r.x), np.diff(r.y))))
        switches = int(np.sum(np.diff(r.direction) != 0))
        ax.set_title("w = %g: %d expansions, %.1f m, %d direction switches"
                     % (w, r.stats.expansions, length, switches))
        print("seed_36 w=%g: %d expansions, %.1f m, %d switches" % (w, r.stats.expansions,
                                                                     length, switches))
        if gif:
            out = os.path.join(OUT, "S5_seed_36_w%g_search.gif" % w)
            animate_search(sc.grid, r, cfg, out, start=tuple(sc.start), goal=tuple(sc.goal),
                           title="S5 seed 36, heuristic weight %g" % w, width_px=600)
            print("  %s %.1f MB" % (os.path.basename(out), os.path.getsize(out) / 1e6))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "weights_w1_vs_w50.png"), dpi=80)
    plt.close(fig)


# The search settings up to fb06823. 78ae297 changed these four and added
# the shortcut pass.
OLD_SEARCH = dict(heuristic_weight=5, xy_resolution=0.2, steer_cost=1, steer_change_cost=5,
                  shortcut_enabled=0)


def s1_before_after():
    sc = scenario("S1")
    fig, axes = plt.subplots(1, 2, figsize=(14, 8))
    panels = (("before: w = 5, 0.2 m cells, steering costs 1 and 5", OLD_SEARCH),
              ("now: w = 2, 0.5 m cells, no steering costs, shortcut", {}))
    for ax, (label, over) in zip(axes, panels):
        cfg = config(**over)
        r = ha.plan(sc.grid, sc.start, sc.goal, cfg, record_trace=True)
        draw_map(ax, sc.grid)
        ax.add_collection(LineCollection(_tree_segments(r.trace.nodes), colors=TREE,
                                         linewidths=0.5, zorder=2))
        draw_car(ax, *sc.start, cfg, color="#2a9d3c", lw=1.5)
        draw_car(ax, *sc.goal, cfg, color="#c2185b", lw=1.5)
        draw_path(ax, r.x, r.y, r.direction, lw=2.0)
        length = float(np.sum(np.hypot(np.diff(r.x), np.diff(r.y))))
        ax.set_title("%s\n%d expansions, %.1f m" % (label, r.stats.expansions, length),
                     fontsize=10)
        ax.set_xlim(10, 70)
        ax.set_ylim(0, 75)
        print("S1 %s: %d expansions, %.1f m" % (label.split(":")[0], r.stats.expansions, length))
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "S1_before_after.png"), dpi=80)
    plt.close(fig)


def search_gifs():
    for name in ("S1", "S2"):
        sc, cfg = scenario(name), config()
        r = ha.plan(sc.grid, sc.start, sc.goal, cfg, record_trace=True)
        out = os.path.join(OUT, name + "_search.gif")
        animate_search(sc.grid, r, cfg, out, start=tuple(sc.start), goal=tuple(sc.goal),
                       title=summary(name, r), width_px=800)
        print("%s %.1f MB" % (os.path.basename(out), os.path.getsize(out) / 1e6))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    single_pngs()
    s5_overview()
    weight_comparison(gif=not args.no_gif)
    s1_before_after()
    if not args.no_gif:
        search_gifs()


if __name__ == "__main__":
    main()
