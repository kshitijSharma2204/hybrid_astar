#pragma once

#include "hybrid_astar/types.hpp"

namespace hybrid_astar {

// Checks the car rectangle, centred on the pose, against a grid.
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

  bool isPointInObstacle(double x, double y) const;
};

} // namespace hybrid_astar
