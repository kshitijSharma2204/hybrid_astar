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
  // Poses are the rear axle centre, which is what the bicycle model turns
  // about. The footprint centre is this far ahead of it. 1.25 m gives 1.0 m
  // overhangs front and back for a 4.5 m car with a 2.5 m wheelbase.
  double rear_axle_to_center = 1.25;   // m

  // search
  double step_size      = 0.5;    // m per motion primitive
  double collision_step = 0.2;    // m between collision checks on a primitive
  int    steer_samples  = 5;      // steering angles from -max_steer to +max_steer
  double xy_resolution  = 0.5;    // m, cell size of the closed set
  int    heading_bins   = 72;
  int    max_expansions = 200000;
  int    collision_mode = 2;      // 0 exact, 1 circles, 2 circles then exact
  int    collision_circles = 4;   // discs covering the footprint
  double goal_pos_tol     = 0.5;  // m, only used when analytic expansion is off
  double goal_heading_tol = 0.3;  // rad, same

  // analytic expansion: try a Reeds-Shepp curve to the goal
  int    analytic_expansion_interval = 10;    // every N expansions, 0 = off
  double analytic_expansion_distance = 10.0;  // m, try on every expansion when closer
  double rs_sample_step = 0.1;                // m between collision checks on the curve

  // shortcutting, see shortcut.hpp
  int    shortcut_enabled = 1;
  double shortcut_spacing = 0.5;        // m between the poses that can be joined

  // smoothing, see smoother.hpp
  int    smooth_enabled       = 1;
  int    smooth_iterations    = 300;
  double smooth_spacing       = 0.5;    // m between control points
  double smooth_rate          = 0.5;    // gradient step, as a fraction of the stable maximum
  double smooth_w_smooth      = 1.0;
  double smooth_w_obstacle    = 0.05;
  double smooth_w_curvature   = 200.0;
  double smooth_w_length      = 1.0;    // keeps control points evenly spaced
  double smooth_obstacle_dmax = 1.0;    // m, footprint discs closer than this are pushed out
  double smooth_curvature_limit = 1.05; // keep a smoothed run only if max curvature <= this / R_min

  // cost weights
  double reverse_penalty       = 1.5;   // multiplier on reverse distance
  double direction_change_cost = 5.0;
  double steer_cost            = 0.0;    // per rad of steering, per primitive
  double steer_change_cost     = 0.0;    // per rad of steering change, per primitive
  double heuristic_weight      = 2.0;
  int    rs_heuristic          = 1;      // 1: h = max(2D distance, Reeds-Shepp length)

  double minTurningRadius() const { return wheel_base / std::tan(max_steer); }
};

struct PlanStats {
  bool   success    = false;
  int    expansions = 0;
  double search_ms  = 0.0;
  double smooth_ms  = 0.0;
  double shortcut_ms = 0.0;
  int    shortcuts   = 0;      // Reeds-Shepp curves put in by the shortcut pass
  int    smooth_segments = 0;
  int    smooth_fallbacks = 0;   // segments left unsmoothed (collision or curvature)
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
  std::vector<Pose2D> poses;    // final path, smoothed if smoothing is on
  std::vector<int> direction;   // per pose: +1 forward, -1 reverse
  std::vector<Pose2D> raw_poses;  // the search path, before shortcutting and smoothing
  std::vector<int> raw_direction;
  PlanStats stats;
  std::string message;
  std::optional<SearchTrace> trace;
};

} // namespace hybrid_astar
