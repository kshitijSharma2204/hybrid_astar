#include "hybrid_astar/metrics.hpp"

#include "hybrid_astar/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace hybrid_astar {

double maxCurvature(const std::vector<Pose2D>& p, const std::vector<int>& dir) {
  double kmax = 0.0;
  for (size_t i = 1; i + 1 < p.size(); ++i) {
    // dir[i] is the motion into pose i; a change at i+1 means i is a cusp
    if (dir[i] != dir[i+1]) continue;
    double ax = p[i].x - p[i-1].x, ay = p[i].y - p[i-1].y;
    double bx = p[i+1].x - p[i].x, by = p[i+1].y - p[i].y;
    double cx = p[i+1].x - p[i-1].x, cy = p[i+1].y - p[i-1].y;
    double a = std::hypot(ax, ay), b = std::hypot(bx, by), c = std::hypot(cx, cy);
    if (a < 1e-9 || b < 1e-9 || c < 1e-9) continue;
    // Menger curvature: 4 * triangle area / product of the side lengths
    double cross = std::fabs(ax * by - ay * bx);
    kmax = std::max(kmax, 2.0 * cross / (a * b * c));
  }
  return kmax;
}

double minClearance(const std::vector<Pose2D>& poses, const GridMap& g,
                    const PlannerConfig& cfg, double max_dist) {
  const double hl = cfg.vehicle_length / 2, hw = cfg.vehicle_width / 2;
  const double reach = std::hypot(hl, hw) + max_dist;
  const int r = int(std::ceil(reach / g.resolution));
  double best = max_dist;
  for (const auto& p : poses) {
    double c = std::cos(p.theta), s = std::sin(p.theta);
    double fx = p.x + cfg.rear_axle_to_center * c;
    double fy = p.y + cfg.rear_axle_to_center * s;
    int cx = worldToCellX(g, fx), cy = worldToCellY(g, fy);
    for (int iy = std::max(0, cy - r); iy <= std::min(g.height - 1, cy + r); ++iy) {
      for (int ix = std::max(0, cx - r); ix <= std::min(g.width - 1, cx + r); ++ix) {
        if (!isOccupied(g, ix, iy)) continue;
        double dx = cellCenterX(g, ix) - fx, dy = cellCenterY(g, iy) - fy;
        // into the car frame, then distance to the rectangle
        double u = std::fabs(dx * c + dy * s) - hl;
        double v = std::fabs(-dx * s + dy * c) - hw;
        double d = std::hypot(std::max(u, 0.0), std::max(v, 0.0));
        best = std::min(best, d);
      }
    }
  }
  return best;
}

PathMetrics computeMetrics(const std::vector<Pose2D>& poses,
                           const std::vector<int>& dir,
                           const GridMap& grid, const PlannerConfig& cfg,
                           const Pose2D& goal) {
  PathMetrics m;
  if (poses.empty()) return m;
  for (size_t i = 1; i < poses.size(); ++i) {
    double d = std::hypot(poses[i].x - poses[i-1].x, poses[i].y - poses[i-1].y);
    m.length += d;
    if (dir[i] < 0) m.reverse_length += d;
    if (dir[i] != dir[i-1]) ++m.direction_switches;
  }
  m.max_curvature = maxCurvature(poses, dir);
  m.min_clearance = minClearance(poses, grid, cfg);
  m.goal_pos_error = std::hypot(poses.back().x - goal.x, poses.back().y - goal.y);
  m.goal_heading_error = std::fabs(angleDiff(poses.back().theta, goal.theta));
  m.cost = (m.length - m.reverse_length) + cfg.reverse_penalty * m.reverse_length +
           cfg.direction_change_cost * m.direction_switches;
  return m;
}

} // namespace hybrid_astar
