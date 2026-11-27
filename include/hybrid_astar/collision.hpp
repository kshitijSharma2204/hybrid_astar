#pragma once

#include "hybrid_astar/types.hpp"

#include <vector>

namespace hybrid_astar {

// Checks the car rectangle against a grid. The pose is the rear axle; the
// rectangle's centre is rear_axle_to_center ahead of it.
//
//  - exact: every grid cell the rectangle touches. This is the reference.
//  - circles: collision_circles discs along the centre line whose union
//    covers the rectangle. A pose collides when a disc centre is within
//    r + res*sqrt(2)/2 of an occupied cell centre, which also catches cells
//    that only touch a disc. The distance comes from a distance map built
//    once in the constructor. Never says free when the exact check says
//    collision, but is over-cautious, mostly past the bumpers.
//
// collision_mode picks exact (0), circles (1), or circles first and the
// exact check only for poses the circles reject (2).
class CollisionChecker {
public:
  CollisionChecker(const GridMap& grid, const PlannerConfig& cfg);

  bool inCollision(const Pose2D& p) const;
  bool exactCollision(const Pose2D& p) const;
  bool circleCollision(const Pose2D& p) const;

  // Distance from a point to the nearest occupied cell centre, read from
  // the distance map with bilinear interpolation. Not a strict bound; the
  // smoother uses it.
  double obstacleDistance(double x, double y) const;

  const GridMap& grid() const { return grid_; }
  double circleRadius() const { return radius_; }
  const std::vector<double>& circleOffsets() const { return circle_offsets_; }

private:
  GridMap grid_;
  double length_;
  double width_;
  double offset_;
  int mode_;
  std::vector<double> circle_offsets_;   // along the heading, from the rear axle
  double radius_ = 0.0;
  double threshold_ = 0.0;               // circle radius + cell half-diagonal
  std::vector<float> dist_;              // per cell: distance to nearest occupied cell centre
};

// Euclidean distance from each cell centre to the nearest occupied cell
// centre, in meters (two-pass squared distance transform).
std::vector<float> distanceTransform(const GridMap& grid);

} // namespace hybrid_astar
