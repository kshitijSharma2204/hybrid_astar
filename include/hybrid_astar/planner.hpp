#pragma once

#include "utils.hpp"
#include <functional>
#include <memory>
#include <nanoflann.hpp>
#include <optional>
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
  explicit HybridAStarPlanner(const VehicleModel& model,
                              double yaw_resolution,
                              int yaw_bins);

  std::optional<std::vector<Pose2D>> plan(
    const Pose2D& start_pose,
    const Pose2D& goal_pose,
    const GridMap& map);

  bool tryReedsSheppConnect(
    const State& current,
    const State& goal,
    std::vector<State>& path_states) const;

  void setLogger(std::function<void(const std::string&)> logger) {
    logger_ = std::move(logger);
  }
  int lastExpansions() const { return expansions_; }

private:
  // Cost weights to match Python version
  static constexpr double kReverseCost      = 10.0;
  static constexpr double kDirectionChange  = 150.0;
  static constexpr double kSteerAngleCost   = 1.0;
  static constexpr double kSteerAngleChange = 5.0;
  static constexpr double kHybridWeight     = 50.0;

  VehicleModel vehicle_model_;
  std::vector<MotionPrimitive> motion_primitives_;
  GridMap map_;
  PointCloud obstacle_cloud_;
  std::unique_ptr<KDTree> obstacle_tree_;
  std::vector<double> heuristic_;
  double yaw_resolution_;
  int yaw_bins_;
  std::function<void(const std::string&)> logger_;
  int expansions_ = 0;

  void buildObstacleKDTree();
  std::vector<State> generateNextStates(const State& current);
  double computeHeuristic(const State &s) const;
  void buildHeuristic(const State &goal);
  
  bool isInCollision(const State& s) const;
  bool isVehicleInCollision(const State& s) const;
  bool isPointInObstacle(double x, double y) const;
  std::vector<std::pair<double,double>> getVehicleCorners(const State& s) const;
  void log(const std::string& msg) const;
};

} // namespace hybrid_astar
