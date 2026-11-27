#pragma once

#include "hybrid_astar/collision.hpp"
#include "hybrid_astar/reeds_shepp.hpp"
#include "hybrid_astar/types.hpp"
#include "hybrid_astar/utils.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hybrid_astar {

class HybridAStarPlanner {
public:
  explicit HybridAStarPlanner(const PlannerConfig& config);

  // Copies the map and builds the collision lookups for it. Call once per
  // map; plan() can then run many times on it.
  void setMap(const GridMap& map);
  PlanResult plan(const Pose2D& start_pose,
                  const Pose2D& goal_pose,
                  const PlanOptions& options = {});

  // setMap() then plan(), for one-off calls.
  PlanResult plan(const GridMap& map,
                  const Pose2D& start_pose,
                  const Pose2D& goal_pose,
                  const PlanOptions& options = {});

  double setupMs() const { return setup_ms_; }

  void setLogger(std::function<void(const std::string&)> logger) {
    logger_ = std::move(logger);
  }
  const PlannerConfig& config() const { return config_; }

private:
  PlannerConfig config_;
  std::vector<MotionPrimitive> motion_primitives_;
  GridMap map_;
  std::unique_ptr<CollisionChecker> checker_;
  std::vector<Node> nodes_;
  Pose2D goal_;
  int n_substeps_ = 1;
  double setup_ms_ = 0.0;
  ReedsShepp rs_;
  std::vector<double> heuristic_;
  double yaw_resolution_;
  std::function<void(const std::string&)> logger_;

  double heuristic2d(const Node &s) const;
  double computeHeuristic(const Node &s) const;
  void buildHeuristic(const Node &goal);
  void buildPath(int goal_node, PlanResult& result) const;
  bool tryReedsSheppConnect(int node, const Node& goal, PlanResult& result,
                            SearchTrace* trace) const;

  bool isInCollision(const Node& s) const;
  void log(const std::string& msg) const;
};

} // namespace hybrid_astar
