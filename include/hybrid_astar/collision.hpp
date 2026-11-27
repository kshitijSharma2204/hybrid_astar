#pragma once

#include "hybrid_astar/types.hpp"

namespace hybrid_astar {

// Checks the car rectangle against a grid. The pose is the rear axle; the
// rectangle's centre is rear_axle_to_center ahead of it.
class CollisionChecker {
public:
  CollisionChecker(const GridMap& grid, const PlannerConfig& cfg);

  // Tests the 4 corners and points every 0.2 m along the outline. Anything
  // outside the map counts as a collision.
  bool inCollision(const Pose2D& p) const;

  const GridMap& grid() const { return grid_; }

private:
  const GridMap& grid_;
  double length_;
  double width_;
  double offset_;

  bool isPointInObstacle(double x, double y) const;
};

} // namespace hybrid_astar
