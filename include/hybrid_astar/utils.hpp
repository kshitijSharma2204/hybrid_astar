#pragma once

#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/types.hpp"

#include <cmath>
#include <vector>

namespace hybrid_astar {

// One search node. Nodes live in a vector and point to their parent by index.
struct Node {
  double x     = 0.0;
  double y     = 0.0;
  double theta = 0.0;
  double g     = 0.0;        // cost-to-come
  double h     = 0.0;        // weighted cost-to-go estimate
  int    parent    = -1;     // index of the parent node, -1 for the start
  int    primitive = -1;     // motion primitive that led here
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
// steering angle, along an exact arc of curvature tan(steer) / wheel_base.
// The heading comes back wrapped to (-pi, pi].
inline Pose2D primitivePose(const Pose2D& from, double steer, double l,
                            double wheel_base) {
  double kappa = std::tan(steer) / wheel_base;
  double th1 = from.theta + l * kappa;
  if (std::fabs(kappa) < 1e-12) {
    return {from.x + l * std::cos(from.theta),
            from.y + l * std::sin(from.theta),
            wrapAngle(from.theta)};
  }
  return {from.x + (std::sin(th1) - std::sin(from.theta)) / kappa,
          from.y - (std::cos(th1) - std::cos(from.theta)) / kappa,
          wrapAngle(th1)};
}

} // namespace hybrid_astar
