#pragma once

#include "hybrid_astar/collision.hpp"
#include "hybrid_astar/reeds_shepp.hpp"
#include "hybrid_astar/types.hpp"

#include <vector>

namespace hybrid_astar {

struct ShortcutResult {
  std::vector<Pose2D> poses;
  std::vector<int> direction;
  int shortcuts = 0;   // Reeds-Shepp curves put in
};

// Shortens each run between cusps on its own.
//
// The search drives fixed 0.5 m primitives, so on a long run it zig-zags
// between steering angles that cost the same. Here I take anchor poses about
// shortcut_spacing apart along the run and, from the first one, connect to
// the farthest later anchor whose shortest Reeds-Shepp curve
//   - drives only in the run's direction (no new cusps),
//   - is shorter than the stretch of path it replaces, and
//   - is collision free (the planner's check every rs_sample_step).
// Then I continue from that anchor. If no anchor can be reached, the original
// stretch to the next anchor is kept.
//
// The first and last pose of every run are unchanged, so start, goal and cusp
// poses stay where the search put them. The new path is never longer.
ShortcutResult shortcutPath(const std::vector<Pose2D>& poses,
                            const std::vector<int>& direction,
                            const ReedsShepp& rs,
                            const CollisionChecker& checker,
                            const PlannerConfig& cfg);

} // namespace hybrid_astar
