#pragma once

#include "hybrid_astar/types.hpp"

#include <vector>

namespace hybrid_astar {

struct PathMetrics {
  double length = 0.0;            // m, sum of distances between poses
  double reverse_length = 0.0;    // m driven in reverse
  int    direction_switches = 0;
  double max_curvature = 0.0;     // 1/m, see maxCurvature()
  double min_clearance = 0.0;     // m, footprint to nearest occupied cell centre
  double goal_pos_error = 0.0;    // m, last pose to goal
  double goal_heading_error = 0.0;  // rad
  // forward m + reverse_penalty * reverse m + direction_change_cost * switches
  double cost = 0.0;
};

// Largest curvature along the path, from the circle through each three
// consecutive poses. Triples that span a cusp are skipped, since the path
// is allowed a corner there.
double maxCurvature(const std::vector<Pose2D>& poses, const std::vector<int>& direction);

// Smallest distance from the car footprint at any pose to the centre of an
// occupied cell. Searches up to max_dist meters out and returns max_dist if
// nothing is closer.
double minClearance(const std::vector<Pose2D>& poses, const GridMap& grid,
                    const PlannerConfig& cfg, double max_dist = 3.0);

PathMetrics computeMetrics(const std::vector<Pose2D>& poses,
                           const std::vector<int>& direction,
                           const GridMap& grid, const PlannerConfig& cfg,
                           const Pose2D& goal);

} // namespace hybrid_astar
