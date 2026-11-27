#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace hybrid_astar {

struct Pose2D {
  double x     = 0.0;   // meters
  double y     = 0.0;   // meters
  double theta = 0.0;   // radians
};

// Same layout as nav_msgs/OccupancyGrid: row 0 is the bottom of the map
// (smallest y), values are 0..100 and 100 means occupied.
struct GridMap {
  int width  = 0;
  int height = 0;
  double resolution = 0.1;   // meters per cell
  double origin_x = 0.0;     // world position of cell (0,0)'s corner
  double origin_y = 0.0;
  std::vector<int8_t> data;
};

struct PlannerConfig {
  // vehicle
  double wheel_base     = 2.5;    // m
  double max_steer      = 0.45;   // rad
  double vehicle_length = 4.5;    // m
  double vehicle_width  = 2.0;    // m

  // search
  double step_size      = 1.0;    // m per motion primitive
  double collision_step = 0.2;    // m between collision checks on a primitive
  int    heading_bins   = 24;
  double goal_pos_tol     = 0.5;  // m
  double goal_heading_tol = 0.3;  // rad

  // cost weights
  double reverse_penalty       = 10.0;   // multiplier on reverse distance
  double direction_change_cost = 150.0;
  double steer_cost            = 1.0;    // per rad of steering
  double steer_change_cost     = 5.0;    // per rad of steering change
  double heuristic_weight      = 50.0;

  double minTurningRadius() const { return wheel_base / std::tan(max_steer); }
};

struct PlanStats {
  bool   success    = false;
  int    expansions = 0;
  double search_ms  = 0.0;
  double smooth_ms  = 0.0;
};

struct PlanResult {
  std::vector<Pose2D> poses;
  std::vector<int> direction;   // per pose: +1 forward, -1 reverse
  PlanStats stats;
  std::string message;
};

} // namespace hybrid_astar
