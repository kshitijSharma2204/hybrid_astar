#include "hybrid_astar/collision.hpp"

#include "hybrid_astar/geometry.hpp"

#include <cmath>
#include <utility>

namespace hybrid_astar {

CollisionChecker::CollisionChecker(const GridMap& grid, const PlannerConfig& cfg)
  : grid_{grid},
    length_{cfg.vehicle_length},
    width_{cfg.vehicle_width}
{
}

bool CollisionChecker::isPointInObstacle(double x, double y) const {
  int ix = worldToCellX(grid_, x);
  int iy = worldToCellY(grid_, y);
  if (!inBounds(grid_, ix, iy)) return true;
  return isOccupied(grid_, ix, iy);
}

bool CollisionChecker::inCollision(const Pose2D& s) const {
  double L = length_, W = width_;
  double c = std::cos(s.theta), s_ = std::sin(s.theta);
  const std::pair<double,double> corners[4] = {
    { L/2,  W/2}, { L/2, -W/2},
    {-L/2, -W/2}, {-L/2,  W/2}
  };
  for (auto &p : corners) {
    if (isPointInObstacle(s.x + p.first*c - p.second*s_,
                          s.y + p.first*s_ + p.second*c)) return true;
  }
  for (double v=-W/2; v<=W/2; v+=0.2) {
    if (isPointInObstacle(
          s.x + (L/2)*c - v*s_,
          s.y + (L/2)*s_ + v*c)) return true;
    if (isPointInObstacle(
          s.x +(-L/2)*c - v*s_,
          s.y +(-L/2)*s_ + v*c)) return true;
  }
  for (double u=-L/2; u<=L/2; u+=0.2) {
    if (isPointInObstacle(
          s.x + u*c - (W/2)*s_,
          s.y + u*s_ + (W/2)*c)) return true;
    if (isPointInObstacle(
          s.x + u*c -(-W/2)*s_,
          s.y + u*s_ +(-W/2)*c)) return true;
  }
  return false;
}

} // namespace hybrid_astar
