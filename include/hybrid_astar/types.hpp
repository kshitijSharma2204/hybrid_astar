#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
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
  int    steer_samples  = 3;      // steering angles from -max_steer to +max_steer
  double xy_resolution  = 0.1;    // m, cell size of the closed set
  int    heading_bins   = 24;
  int    max_expansions = 1000000;
  double goal_pos_tol     = 0.5;  // m, only used when analytic expansion is off
  double goal_heading_tol = 0.3;  // rad, same

  // analytic expansion: try a Reeds-Shepp curve to the goal
  int    analytic_expansion_interval = 10;    // every N expansions, 0 = off
  double analytic_expansion_distance = 10.0;  // m, try on every expansion when closer
  double rs_sample_step = 0.1;                // m between collision checks on the curve

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

// What the search did, in order. Only filled when PlanOptions asks for it.
struct SearchTrace {
  struct Node {
    double x, y, theta;
    int parent;       // index into nodes, -1 for the start
  };
  struct RsAttempt {
    int node;         // index into nodes the attempt started from
    bool collided;
    std::vector<Pose2D> samples;
  };
  std::vector<Node> nodes;
  std::vector<RsAttempt> rs_attempts;
};

struct PlanOptions {
  bool record_trace = false;
  // Called every progress_every expansions with the trace so far. Needs
  // record_trace.
  std::function<void(const SearchTrace&)> on_progress;
  int progress_every = 1000;
};

struct PlanResult {
  std::vector<Pose2D> poses;
  std::vector<int> direction;   // per pose: +1 forward, -1 reverse
  PlanStats stats;
  std::string message;
  std::optional<SearchTrace> trace;
};

} // namespace hybrid_astar
