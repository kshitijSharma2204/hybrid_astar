#pragma once

#include "hybrid_astar/collision.hpp"
#include "hybrid_astar/types.hpp"

#include <vector>

namespace hybrid_astar {

struct SmoothResult {
  std::vector<Pose2D> poses;
  std::vector<int> direction;
  int segments = 0;            // runs between cusps
  int collision_fallbacks = 0; // runs put back because a pose collided
  int curvature_fallbacks = 0; // runs put back for turning too tight
};

// Smooths each run between cusps on its own.
//
// 1. Pick control points about smooth_spacing apart along the run.
// 2. Gradient descent on the control points' positions:
//      w_smooth    * sum |c[i+1] - 2 c[i] + c[i-1]|^2
//    + w_obstacle  * sum (d_max - d)^2 for footprint discs closer than d_max
//    + w_curvature * sum (|c[i+1] - 2 c[i] + c[i-1]| - k_max s^2)^2 where
//      that is positive; for points s apart the second difference of a
//      curve with curvature k is about k s^2.
//    The first two and last two control points do not move.
// 3. Rebuild the run as a uniform cubic B-spline over the control points,
//    sampled about collision_step apart. The two control points at each end
//    are placed so the spline starts and ends at the original poses with the
//    original curvature, so start, goal and cusp poses are unchanged.
// 4. If any new pose collides (exact check) or the run turns tighter than
//    smooth_curvature_limit / R_min (1.05), the original run is used instead.
SmoothResult smoothPath(const std::vector<Pose2D>& poses,
                        const std::vector<int>& direction,
                        const CollisionChecker& checker,
                        const PlannerConfig& cfg);

} // namespace hybrid_astar
