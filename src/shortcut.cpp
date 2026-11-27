#include "hybrid_astar/shortcut.hpp"

#include <cmath>

namespace hybrid_astar {

namespace {

// Checks every stride-th sample first and then the rest, so a curve through
// an obstacle is usually rejected after a few checks.
bool curveIsFree(const std::vector<Pose2D>& poses, const CollisionChecker& cc) {
  const size_t stride = 8;
  for (size_t start = 0; start < stride; ++start) {
    for (size_t i = start; i < poses.size(); i += stride) {
      if (cc.inCollision(poses[i])) return false;
    }
  }
  return true;
}

// Shortcuts poses[a..b], one run with a single direction dir, and appends the
// result after poses[a] (which the caller has already written).
int shortcutRun(const std::vector<Pose2D>& poses, int a, int b, int dir,
                const ReedsShepp& rs, const CollisionChecker& cc,
                const PlannerConfig& cfg, ShortcutResult& out) {
  // arc length along the run, and anchors about shortcut_spacing apart
  std::vector<double> s(b - a + 1, 0.0);
  for (int k = a + 1; k <= b; ++k) {
    s[k - a] = s[k - a - 1] + std::hypot(poses[k].x - poses[k-1].x,
                                         poses[k].y - poses[k-1].y);
  }
  std::vector<int> anchor = {a};
  for (int k = a + 1; k < b; ++k) {
    if (s[k - a] - s[anchor.back() - a] >= cfg.shortcut_spacing) anchor.push_back(k);
  }
  anchor.push_back(b);

  auto keep = [&](int from, int to) {   // original poses (from, to]
    for (int k = from + 1; k <= to; ++k) {
      out.poses.push_back(poses[k]);
      out.direction.push_back(dir);
    }
  };

  int count = 0;
  std::vector<Pose2D> curve;
  std::vector<int> curve_dir;
  size_t i = 0;
  while (i + 1 < anchor.size()) {
    const Pose2D& from = poses[anchor[i]];
    size_t next = i + 1;
    for (size_t j = anchor.size() - 1; j >= i + 2; --j) {
      const Pose2D& to = poses[anchor[j]];
      double along = s[anchor[j] - a] - s[anchor[i] - a];
      if (rs.length(from, to) >= along - 1e-6) continue;
      curve.clear();
      curve_dir.clear();
      rs.sample(from, to, cfg.rs_sample_step, curve, curve_dir);
      bool one_way = true;
      for (int d : curve_dir) one_way = one_way && d == dir;
      if (!one_way || !curveIsFree(curve, cc)) continue;
      curve.back() = to;   // the sampled end is within rounding of it
      out.poses.insert(out.poses.end(), curve.begin(), curve.end());
      out.direction.insert(out.direction.end(), curve.size(), dir);
      ++count;
      next = j;
      break;
    }
    if (next == i + 1) keep(anchor[i], anchor[i + 1]);
    i = next;
  }
  return count;
}

} // namespace

ShortcutResult shortcutPath(const std::vector<Pose2D>& poses,
                            const std::vector<int>& direction,
                            const ReedsShepp& rs,
                            const CollisionChecker& checker,
                            const PlannerConfig& cfg) {
  ShortcutResult out;
  if (poses.size() < 3) {
    out.poses = poses;
    out.direction = direction;
    return out;
  }
  out.poses.push_back(poses[0]);
  out.direction.push_back(direction[0]);

  // direction[k] is the motion arriving at pose k, so a run ends at the last
  // pose before the direction changes; that pose is the cusp.
  const int n = int(poses.size());
  int a = 0;
  while (a < n - 1) {
    int dir = direction[a + 1];
    int b = a + 1;
    while (b + 1 < n && direction[b + 1] == dir) ++b;
    out.shortcuts += shortcutRun(poses, a, b, dir, rs, checker, cfg, out);
    a = b;
  }
  return out;
}

} // namespace hybrid_astar
