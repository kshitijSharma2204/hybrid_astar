#!/usr/bin/env python3
"""Run scenarios through hybrid_astar_cli and summarise the results.

    tools/benchmark.py --stats out.csv                      # S1-S5, defaults
    tools/benchmark.py --scenarios S1 S2 --repeat 20 --stats out.csv
    tools/benchmark.py --label w1 --set heuristic_weight=1 --stats sweep.csv
    tools/benchmark.py --summary-only sweep.csv             # table from a CSV

S1-S4 run --repeat times; S5 runs each of its 50 seeds once. Every run is
one row in the --stats CSV (see hybrid_astar_cli --help). The summary has
one line per (label, scenario), with S5 seeds pooled.
"""

import argparse
import csv
import glob
import os
import statistics
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CLI = os.path.join(ROOT, "build", "hybrid_astar_cli")


def scenario_files(names):
    out = []
    for n in names:
        if n == "S5":
            out += sorted(glob.glob(os.path.join(ROOT, "scenarios", "S5", "seed_*.txt")))
        else:
            out.append(os.path.join(ROOT, "scenarios", n + ".txt"))
    return out


def group_of(name):
    return "S5" if name.startswith("seed_") else name


def pct(values, q):
    v = sorted(values)
    if not v:
        return float("nan")
    k = (len(v) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def summarise(rows):
    groups = {}
    for r in rows:
        groups.setdefault((r["label"], group_of(r["scenario"])), []).append(r)
    head = ("| label | scenario | runs | success | search ms median | p95 | smooth ms median "
            "| expansions median | length m | switches | max curv. raw | max curv. "
            "| min clearance m | runs put back |")
    lines = [head, "|" + "---|" * (head.count("|") - 1)]
    for (label, sc), rs in sorted(groups.items()):
        ok = [r for r in rs if r["success"] == "1"]

        def col(key, src=ok):
            return [float(r[key]) for r in src if r.get(key) not in (None, "")]

        def fmt(v, f):
            return f % v if ok else "-"

        t = col("search_ms", rs)
        lines.append("| %s | %s | %d | %d/%d | %.1f | %.1f | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            label, sc, len(rs), len(ok), len(rs), statistics.median(t), pct(t, 0.95),
            fmt(statistics.median(col("smooth_ms")) if ok else 0, "%.1f"),
            fmt(statistics.median(col("expansions")) if ok else 0, "%.0f"),
            fmt(statistics.median(col("length")) if ok else 0, "%.1f"),
            fmt(statistics.median(col("switches")) if ok else 0, "%.0f"),
            fmt(max(col("max_curvature_raw")) if ok else 0, "%.4f"),
            fmt(max(col("max_curvature")) if ok else 0, "%.4f"),
            fmt(min(col("min_clearance")) if ok else 0, "%.2f"),
            fmt(sum(col("smooth_fallbacks")) if ok else 0, "%.0f")))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", nargs="+", default=["S1", "S2", "S3", "S4", "S5"])
    ap.add_argument("--repeat", type=int, default=1, help="runs per scenario for S1-S4")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--config", default=os.path.join(ROOT, "config", "params.yaml"))
    ap.add_argument("--label", default="default")
    ap.add_argument("--stats", required=False, help="CSV to append runs to")
    ap.add_argument("--summary", help="also write the summary table to this file")
    ap.add_argument("--summary-only", metavar="CSV", help="just summarise an existing CSV")
    ap.add_argument("--cli", default=CLI)
    args = ap.parse_args()

    if args.summary_only:
        with open(args.summary_only) as f:
            print(summarise(list(csv.DictReader(f))))
        return
    if not args.stats:
        ap.error("--stats is required")

    for path in scenario_files(args.scenarios):
        repeat = 1 if os.path.basename(os.path.dirname(path)) == "S5" else args.repeat
        cmd = [args.cli, "--scenario", path, "--config", args.config,
               "--repeat", str(repeat), "--stats", args.stats, "--label", args.label]
        for kv in args.set:
            cmd += ["--set", kv]
        p = subprocess.run(cmd, capture_output=True, text=True)
        if p.returncode == 2:
            sys.exit(p.stderr)
        print(p.stdout.strip().splitlines()[-1], flush=True)

    with open(args.stats) as f:
        rows = [r for r in csv.DictReader(f) if r["label"] == args.label]
    table = summarise(rows)
    print(table)
    if args.summary:
        with open(args.summary, "w") as f:
            f.write(table + "\n")


if __name__ == "__main__":
    main()
