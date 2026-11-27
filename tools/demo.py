#!/usr/bin/env python3
"""Plan one scenario and write a PNG and a search animation to results/v2/.

    tools/demo.py S1
    tools/demo.py S5/seed_03 --set heuristic_weight=1 --suffix _w1
"""

import argparse
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "python"))

import matplotlib
matplotlib.use("Agg")

import hybrid_astar as ha
from hybrid_astar_viz import animate_search, plot_result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", help="name under scenarios/ (S1, S5/seed_03) or a path")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "params.yaml"))
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "v2"))
    ap.add_argument("--suffix", default="", help="added to the output file names")
    ap.add_argument("--anim", default="gif", choices=["gif", "mp4", "none"])
    ap.add_argument("--width", type=int, default=800, help="animation width in pixels")
    args = ap.parse_args()

    path = args.scenario
    if not os.path.exists(path):
        path = os.path.join(ROOT, "scenarios", args.scenario + ".txt")
    sc = ha.load_scenario(path)
    cfg = ha.load_config(args.config)
    for kv in args.set:
        k, v = kv.split("=", 1)
        cfg.set(k, float(v))

    r = ha.plan(sc.grid, sc.start, sc.goal, cfg, record_trace=args.anim != "none")
    name = args.scenario.replace("/", "_").replace(".txt", "") if path != args.scenario \
        else sc.name
    name += args.suffix
    length = sum(math.hypot(r.x[i + 1] - r.x[i], r.y[i + 1] - r.y[i]) for i in range(len(r.x) - 1))
    print("%s: success=%s expansions=%d search_ms=%.1f smooth_ms=%.1f length=%.1f m" % (
        name, r.success, r.stats.expansions, r.stats.search_ms, r.stats.smooth_ms, length))
    if not r.success:
        print("  " + r.message)

    os.makedirs(args.out, exist_ok=True)
    title = "%s  (w=%g)" % (name, cfg.heuristic_weight)
    png = os.path.join(args.out, name + ".png")
    plot_result(sc.grid, r, cfg, start=tuple(sc.start), goal=tuple(sc.goal), out=png, title=title)
    print("  wrote " + os.path.relpath(png, ROOT))
    if args.anim != "none":
        anim = os.path.join(args.out, "%s_search.%s" % (name, args.anim))
        animate_search(sc.grid, r, cfg, anim, start=tuple(sc.start), goal=tuple(sc.goal),
                       title=title, width_px=args.width)
        print("  wrote %s (%.1f MB)" % (os.path.relpath(anim, ROOT), os.path.getsize(anim) / 1e6))


if __name__ == "__main__":
    main()
