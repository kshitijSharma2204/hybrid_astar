#include "hybrid_astar/collision.hpp"

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
  double res = grid_.resolution;
  int ix = int(std::floor((x - grid_.origin_x)/res));
  int iy = int(std::floor((y - grid_.origin_y)/res));
  if(ix<0||iy<0||ix>=grid_.width||iy>=grid_.height) return true;
  return grid_.data[iy*grid_.width + ix] >= 100;
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
