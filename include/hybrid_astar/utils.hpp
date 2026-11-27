#pragma once

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
};

struct MotionPrimitive {
  double steering_angle; // radians
  bool   is_reverse = false; // true if reverse motion
};

inline std::vector<MotionPrimitive>
generateMotionPrimitives(const PlannerConfig& cfg) {
  std::vector<MotionPrimitive> primitives;
  // full left, straight, full right; forward and reverse
  std::vector<double> steers = {
    -cfg.max_steer,
     0.0,
     cfg.max_steer
  };
  for (bool rev : {false,true}) {
    for (double a : steers) {
      primitives.push_back({a, rev});
    }
  }
  return primitives;
}

} // namespace hybrid_astar
