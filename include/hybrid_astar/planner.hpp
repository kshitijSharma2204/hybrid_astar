#pragma once

#include "hybrid_astar/types.hpp"
#include "hybrid_astar/utils.hpp"

#include <functional>
#include <memory>
#include <nanoflann.hpp>
#include <string>
#include <vector>

namespace hybrid_astar {

using Point = std::array<double,2>;

struct PointCloud {
  std::vector<Point> points;
  inline size_t kdtree_get_point_count() const { return points.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return points[idx][dim];
  }
  template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    2,
    size_t
>;

class HybridAStarPlanner {
public:
  explicit HybridAStarPlanner(const PlannerConfig& config);

  PlanResult plan(const GridMap& map,
                  const Pose2D& start_pose,
                  const Pose2D& goal_pose);

  void setLogger(std::function<void(const std::string&)> logger) {
    logger_ = std::move(logger);
  }
  const PlannerConfig& config() const { return config_; }

private:
  PlannerConfig config_;
  std::vector<MotionPrimitive> motion_primitives_;
  GridMap map_;
  PointCloud obstacle_cloud_;
  std::unique_ptr<KDTree> obstacle_tree_;
  std::vector<double> heuristic_;
  double yaw_resolution_;
  std::function<void(const std::string&)> logger_;

  void buildObstacleKDTree();
  double computeHeuristic(const State &s) const;
  void buildHeuristic(const State &goal);
  
  bool isInCollision(const State& s) const;
  bool isVehicleInCollision(const State& s) const;
  bool isPointInObstacle(double x, double y) const;
  std::vector<std::pair<double,double>> getVehicleCorners(const State& s) const;
  void log(const std::string& msg) const;
};

} // namespace hybrid_astar
