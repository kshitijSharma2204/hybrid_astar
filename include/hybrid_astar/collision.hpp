#pragma once

#include "hybrid_astar/types.hpp"

#include <memory>
#include <vector>

namespace hybrid_astar {

// Checks the car rectangle against a grid. The pose is the rear axle; the
// rectangle's centre is rear_axle_to_center ahead of it.
//
//  - exact: every grid cell the rectangle touches. This is the reference.
//  - outline: v1's check, the corners and points every 0.2 m along the
//    outline. Kept for comparison only; it can miss a cell that sits
//    between two samples.
//  - circles: collision_circles discs along the centre line whose union
//    covers the rectangle. A pose collides when an occupied cell centre is
//    within r + res*sqrt(2)/2 of a disc centre, which also catches cells
//    that only touch a disc. Never says free when the exact check says
//    collision, but is over-cautious, mostly past the bumpers.
//
// collision_mode picks exact (0), circles (1), or circles first and the
// exact check only for poses the circles reject (2). The obstacle lookup
// behind the circles is built once, in the constructor.
class CollisionChecker {
public:
  enum class Lookup { KdTree, DistanceMap };

  CollisionChecker(const GridMap& grid, const PlannerConfig& cfg,
                   Lookup lookup = Lookup::DistanceMap);
  ~CollisionChecker();

  bool inCollision(const Pose2D& p) const;
  bool exactCollision(const Pose2D& p) const;
  bool outlineCollision(const Pose2D& p) const;
  bool circleCollision(const Pose2D& p) const;

  const GridMap& grid() const { return grid_; }
  double circleRadius() const { return radius_; }

private:
  struct KdIndex;

  GridMap grid_;
  double length_;
  double width_;
  double offset_;
  int mode_;
  Lookup lookup_;
  std::vector<double> circle_offsets_;   // along the heading, from the rear axle
  double radius_ = 0.0;
  double threshold_ = 0.0;               // circle radius + cell half-diagonal
  std::vector<float> dist_;              // per cell: distance to nearest occupied cell centre
  std::unique_ptr<KdIndex> kd_;

  bool isPointInObstacle(double x, double y) const;
  double nearestObstacle(double x, double y) const;
};

// Euclidean distance from each cell centre to the nearest occupied cell
// centre, in meters (two-pass squared distance transform).
std::vector<float> distanceTransform(const GridMap& grid);

} // namespace hybrid_astar
