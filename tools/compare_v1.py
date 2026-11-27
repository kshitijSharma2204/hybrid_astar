#!/usr/bin/env python3
"""Run the v1 baseline CLI and the current CLI on S1-S5 and compare.

    tools/compare_v1.py --v1-cli /path/to/v1/hybrid_astar_cli --out v1_vs_v2.csv

The v1 CLI is the one from the v1 baseline commit (7719e2a): it takes
"<scenario.txt> [path.csv]", plans with the old ROS defaults and has no
expansion limit, so every v1 run gets --timeout seconds and a memory cap.

v1 puts the centre of the car on the pose; v2 puts the rear axle there.
So v1 gets scenario files with the poses moved rear_axle_to_center
forward, which is the same physical start and goal.
"""

import argparse
import csv
import glob
import math
import os
import re
import resource
import statistics
import subprocess
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REAR_AXLE_TO_CENTER = 1.25


def scenario_files():
    files = [os.path.join(ROOT, "scenarios", s + ".txt") for s in ("S1", "S2", "S3", "S4")]
    return files + sorted(glob.glob(os.path.join(ROOT, "scenarios", "S5", "seed_*.txt")))


def read_pose(text, key):
    m = re.search(r"^%s:\s*(\S+)\s+(\S+)\s+(\S+)" % key, text, re.M)
    return tuple(float(v) for v in m.groups())


def centre_version(path, tmpdir):
    """Scenario file with poses at the footprint centre, for v1."""
    text = open(path).read()
    out = []
    for line in text.splitlines():
        key = line.split(":")[0].strip()
        if key in ("start", "goal"):
            x, y, deg = read_pose(text, key)
            th = math.radians(deg)
            line = "%s: %.4f %.4f %g" % (key, x + REAR_AXLE_TO_CENTER * math.cos(th),
                                          y + REAR_AXLE_TO_CENTER * math.sin(th), deg)
        elif key == "map":
            # v1's loader only takes map paths relative to the text file
            pgm = line.split(":", 1)[1].strip()
            link = os.path.join(tmpdir, pgm)
            if not os.path.exists(link):
                os.symlink(os.path.join(os.path.dirname(os.path.abspath(path)), pgm), link)
        out.append(line)
    name = os.path.join(tmpdir, os.path.basename(path))
    with open(name, "w") as f:
        f.write("\n".join(out) + "\n")
    return name, read_pose("\n".join(out), "goal")


def limit_memory():
    resource.setrlimit(resource.RLIMIT_AS, (4 << 30, 4 << 30))


def run_v1(cli, scenario, goal, timeout, tmpdir):
    out_csv = os.path.join(tmpdir, "v1_path.csv")
    if os.path.exists(out_csv):
        os.remove(out_csv)
    try:
        p = subprocess.run([cli, scenario, out_csv], capture_output=True, text=True,
                           timeout=timeout, preexec_fn=limit_memory)
    except subprocess.TimeoutExpired:
        return {"success": 0, "ms": timeout * 1000.0, "note": "timeout"}
    m = re.search(r"success=(\d) expansions=(\d+) time_ms=([\d.]+)", p.stdout)
    if not m:
        return {"success": 0, "ms": float("nan"), "note": "crashed (%d)" % p.returncode}
    row = {"success": int(m.group(1)), "expansions": int(m.group(2)), "ms": float(m.group(3)),
           "note": ""}
    if row["success"]:
        rows = [r for r in csv.reader(open(out_csv)) if r and r[0][0] not in "#x"]
        pts = [(float(r[0]), float(r[1])) for r in rows]
        row["length"] = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        x, y, th = (float(v) for v in rows[-1])
        row["goal_pos_error"] = math.hypot(x - goal[0], y - goal[1])
        d = th - math.radians(goal[2])
        row["goal_heading_error"] = abs(math.atan2(math.sin(d), math.cos(d)))
    return row


def run_v2(cli, scenario, tmpdir):
    stats = os.path.join(tmpdir, "v2.csv")
    if os.path.exists(stats):
        os.remove(stats)
    subprocess.run([cli, "--scenario", scenario, "--config",
                    os.path.join(ROOT, "config", "params.yaml"), "--stats", stats],
                   capture_output=True, text=True)
    r = next(csv.DictReader(open(stats)))
    row = {"success": int(r["success"]), "expansions": int(r["expansions"]),
           "ms": float(r["setup_ms"]) + float(r["search_ms"]) + float(r["shortcut_ms"])
                 + float(r["smooth_ms"]), "note": ""}
    if row["success"]:
        row["length"] = float(r["length"])
        row["goal_pos_error"] = float(r["goal_pos_error"])
        row["goal_heading_error"] = float(r["goal_heading_error"])
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v1-cli", required=True)
    ap.add_argument("--v2-cli", default=os.path.join(ROOT, "build", "hybrid_astar_cli"))
    ap.add_argument("--timeout", type=float, default=30.0, help="seconds per v1 run")
    ap.add_argument("--out", required=True, help="CSV with one row per scenario and version")
    args = ap.parse_args()

    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for path in scenario_files():
            name = os.path.splitext(os.path.basename(path))[0]
            v1_file, v1_goal = centre_version(path, tmp)
            for version, row in (("v1", run_v1(args.v1_cli, v1_file, v1_goal, args.timeout, tmp)),
                                 ("v2", run_v2(args.v2_cli, path, tmp))):
                row.update(scenario=name, version=version)
                rows.append(row)
                print("%-8s %s success=%d ms=%.1f %s" % (name, version, row["success"],
                                                         row["ms"], row["note"]), flush=True)

    keys = ["scenario", "version", "success", "expansions", "ms", "length", "goal_pos_error",
            "goal_heading_error", "note"]
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})

    # S5 lengths only over the seeds both versions solve, so they compare
    both = {r["scenario"] for r in rows if r["version"] == "v1" and r["success"]} & \
           {r["scenario"] for r in rows if r["version"] == "v2" and r["success"]}
    print("\n| scenario | version | success | time ms median | length m median "
          "| goal position error median | goal heading error median |\n"
          "|---|---|---|---|---|---|---|")
    for group in ("S1", "S2", "S3", "S4", "S5"):
        for version in ("v1", "v2"):
            rs = [r for r in rows if r["version"] == version and
                  (r["scenario"] == group or (group == "S5" and r["scenario"].startswith("seed")))]
            ok = [r for r in rs if r["success"]]
            med = lambda k: "%.3g" % statistics.median(r[k] for r in ok) if ok else "-"
            lens = [r["length"] for r in ok if r["scenario"] in both]
            print("| %s | %s | %d/%d | %.1f | %s | %s | %s |" % (
                group, version, len(ok), len(rs), statistics.median(r["ms"] for r in rs),
                "%.1f" % statistics.median(lens) if lens else "-",
                med("goal_pos_error"), med("goal_heading_error")))


if __name__ == "__main__":
    main()
