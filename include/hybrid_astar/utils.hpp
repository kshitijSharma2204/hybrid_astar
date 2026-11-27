#pragma once

#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/types.hpp"

#include <cmath>
#include <memory>
#include <vector>

namespace hybrid_astar {

struct State {
  double x    = 0.0;       // position
  double y    = 0.0;       // position
  double theta= 0.0;       // orientation
  double g    = 0.0;       // cost-to-come
  double h    = 0.0;       // heuristic cost-to-go

  std::shared_ptr<State> parent = nullptr;  
                          // for backtracking

  bool   is_reverse     = false;  // did we arrive here in reverse?
  double steering_angle = 0.0;    // what steer angle got us here
  int    trace_id       = -1;     // index in SearchTrace::nodes once expanded
};

struct MotionPrimitive {
  double steering_angle; // radians
  bool   is_reverse = false; // true if reverse motion
};

inline std::vector<MotionPrimitive>
generateMotionPrimitives(const PlannerConfig& cfg) {
  std::vector<MotionPrimitive> primitives;
  // steer_samples angles evenly spaced from full right to full left,
  // each driven forward and in reverse
  const int n = cfg.steer_samples;
  for (bool rev : {false,true}) {
    for (int i = 0; i < n; ++i) {
      double a = -cfg.max_steer + 2.0 * cfg.max_steer * i / (n - 1);
      primitives.push_back({a, rev});
    }
  }
  return primitives;
}

// Pose after driving a signed distance l (negative = reverse) at a fixed
// steering angle. Uses the v1 update: move l along the mean heading. The
// heading comes back wrapped to (-pi, pi].
inline Pose2D primitivePose(const Pose2D& from, double steer, double l,
                            double wheel_base) {
  double beta = (l / wheel_base) * std::tan(steer);
  return {from.x + l * std::cos(from.theta + beta * 0.5),
          from.y + l * std::sin(from.theta + beta * 0.5),
          wrapAngle(from.theta + beta)};
}

} // namespace hybrid_astar
