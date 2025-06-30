#pragma once

#include <cmath>
#include <memory>
#include <vector>

struct VehicleModel {
  double wheel_base = 2.5;                // meters
  double max_steering_angle = 0.45;       // radians
  double step_size = 1.0;                 // meters
  double length = 4.5;                    // meters
  double width = 2.0;                     // meters
  double min_turning_radius = wheel_base
    / std::tan(max_steering_angle);       // meters
};

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
generateMotionPrimitives(const VehicleModel& model) {
  std::vector<MotionPrimitive> primitives;
  // example: straight/no-steer/left-steer, forward and reverse
  std::vector<double> steers = {
    -model.max_steering_angle,
     0.0,
     model.max_steering_angle
  };
  for (bool rev : {false,true}) {
    for (double a : steers) {
      primitives.push_back({a, rev});
    }
  }
  return primitives;
}
