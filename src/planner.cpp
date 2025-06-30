#include "hybrid_astar/planner.hpp"

#include "rclcpp/rclcpp.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_set>

namespace hybrid_astar {

HybridAStarPlanner::HybridAStarPlanner(
    const VehicleModel& model,
    double yaw_resolution,
    int yaw_bins)
  : vehicle_model_{model},
    yaw_resolution_{yaw_resolution},
    yaw_bins_{yaw_bins}
{
  motion_primitives_ = generateMotionPrimitives(vehicle_model_);
}

std::optional<nav_msgs::msg::Path> HybridAStarPlanner::plan(
    const geometry_msgs::msg::PoseStamped& start_msg,
    const geometry_msgs::msg::PoseStamped& goal_msg,
    const nav_msgs::msg::OccupancyGrid& map)
{
  auto logger = rclcpp::get_logger("HybridAStarPlanner");
  map_ = map;
  buildObstacleKDTree();

  // Build start state
  State start;
  start.x = start_msg.pose.position.x;
  start.y = start_msg.pose.position.y;
  {
    auto &q = start_msg.pose.orientation;
    start.theta = std::atan2(
      2.0*(q.w*q.z + q.x*q.y),
      1.0 - 2.0*(q.y*q.y + q.z*q.z)
    );
  }
  start.g = 0.0; start.h = 0.0;
  start.parent = nullptr;
  start.is_reverse = false;
  start.steering_angle = 0.0;

  // Build goal state
  State goal;
  goal.x = goal_msg.pose.position.x;
  goal.y = goal_msg.pose.position.y;
  {
    auto &q = goal_msg.pose.orientation;
    goal.theta = std::atan2(
      2.0*(q.w*q.z + q.x*q.y),
      1.0 - 2.0*(q.y*q.y + q.z*q.z)
    );
  }

  // Build grid‐based Dijkstra heuristic
  buildHeuristic(goal);

  // Sanity checks
  if (isInCollision(start)) {
    RCLCPP_ERROR(logger,"Start in obstacle — abort");
    return std::nullopt;
  }
  if (isInCollision(goal)) {
    RCLCPP_ERROR(logger,"Goal in obstacle — abort");
    return std::nullopt;
  }

  RCLCPP_INFO(logger,
    "plan(): from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",
    start.x, start.y, start.theta,
    goal.x,  goal.y,  goal.theta);

  // Open set: min‐heap by f = g + h
  auto cmp = [](auto &a, auto &b){
    return (a->g + a->h) > (b->g + b->h);
  };
  std::priority_queue<
    std::shared_ptr<State>,
    std::vector<std::shared_ptr<State>>,
    decltype(cmp)
  > open_set(cmp);

  // Discretizer
  auto packKey = [&](int ix,int iy,int iyaw){
    return (uint64_t(ix)<<40) | (uint64_t(iy)<<20) | uint64_t(iyaw);
  };
  auto discretize = [&](const State &s){
    double res = map_.info.resolution;
    double ox  = map_.info.origin.position.x;
    double oy  = map_.info.origin.position.y;
    int W      = map_.info.width;

    int ix = int(std::round((s.x - ox)/res));
    int iy = int(std::round((s.y - oy)/res));
    double th = std::fmod(s.theta, 2*M_PI);
    if (th < 0) th += 2*M_PI;
    int iyaw = int(std::floor(th/yaw_resolution_)) % yaw_bins_;

    ix = std::clamp(ix, 0, W-1);
    iy = std::clamp(iy, 0, int(map_.info.height)-1);
    return packKey(ix, iy, iyaw);
  };

  // push start
  open_set.push(std::make_shared<State>(start));
  std::unordered_set<uint64_t> closed_set;

  // fallback tolerances
  constexpr double pos_tol   = 0.5;
  constexpr double theta_tol = 0.3;

  // main A* loop
  while (!open_set.empty()) {
    auto current = open_set.top(); open_set.pop();

    uint64_t key = discretize(*current);
    if (closed_set.count(key)) continue;
    closed_set.insert(key);

    // 2) fallback: goal‐proximity
    double dx    = current->x - goal.x;
    double dy    = current->y - goal.y;
    double dpos  = std::hypot(dx, dy);
    double dtheta= std::fabs(current->theta - goal.theta);
    if (dpos < pos_tol && dtheta < theta_tol) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = map_.header.frame_id;
      path_msg.header.stamp    = map_.header.stamp;

      // prepend start
      {
        auto p = start_msg;
        p.header = path_msg.header;
        path_msg.poses.push_back(p);
      }
      // backtrack
      std::vector<geometry_msgs::msg::PoseStamped> tail;
      for (auto s = current; s!=nullptr; s=s->parent) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = s->x;
        ps.pose.position.y = s->y;
        ps.pose.orientation.w = std::cos(s->theta/2.0);
        ps.pose.orientation.z = std::sin(s->theta/2.0);
        tail.push_back(ps);
      }
      std::reverse(tail.begin(), tail.end());
      path_msg.poses.insert(
        path_msg.poses.end(),
        tail.begin(), tail.end()
      );
      RCLCPP_INFO(logger,
        "plan(): success → %zu poses",
        path_msg.poses.size());
      return path_msg;
    }

    // 3) expand non‐holonomic primitives (with full cost + sub‐step collision)
    for (auto &prim : motion_primitives_) {
      // simulate in 5 sub‐steps
      const int n_steps = 5;
      bool   bad       = false;
      State  last = *current;

      for (int i=1; i<=n_steps; ++i) {
        double frac = double(i)/double(n_steps);
        double dir  = prim.is_reverse ? -1.0 : 1.0;
        double l    = vehicle_model_.step_size * dir;
        double beta = (l / vehicle_model_.wheel_base)
                    * std::tan(prim.steering_angle);
        double dxp = frac*l * std::cos(current->theta + beta*frac*0.5);
        double dyp = frac*l * std::sin(current->theta + beta*frac*0.5);
        double th  = current->theta + beta*frac;

        State temp;
        temp.x             = current->x + dxp;
        temp.y             = current->y + dyp;
        temp.theta         = th;
        temp.is_reverse    = prim.is_reverse;
        temp.steering_angle= prim.steering_angle;

        if (isInCollision(temp)) {
          bad = true;
          break;
        }
        last = temp;
      }
      if (bad) continue;

      // compute full cost
      double move = std::hypot(last.x-current->x,
                               last.y-current->y);
      double g = current->g
               + move * (prim.is_reverse
                   ? kReverseCost : 1.0)
               + (prim.is_reverse != current->is_reverse
                   ? kDirectionChange : 0.0)
               + std::abs(prim.steering_angle
                   - current->steering_angle)
                   * kSteerAngleChange
               + std::abs(prim.steering_angle)
                   * kSteerAngleCost;

      // push successor
      auto succ = std::make_shared<State>(last);
      succ->g              = g;
      succ->h              = computeHeuristic(*succ);
      succ->parent         = current;
      open_set.push(succ);
    }
  }

  RCLCPP_WARN(logger, "plan(): failed");
  return std::nullopt;
}

void HybridAStarPlanner::buildHeuristic(const State &goal) {
  auto &info = map_.info;
  int W = info.width, H = info.height;
  double res = info.resolution;
  heuristic_.assign(W*H, std::numeric_limits<double>::infinity());

  auto idx=[&](int x,int y){ return y*W + x; };
  int gx = std::clamp(int((goal.x-info.origin.position.x)/res),
                      0, W-1);
  int gy = std::clamp(int((goal.y-info.origin.position.y)/res),
                      0, H-1);

  using P = std::pair<double,int>;
  std::priority_queue<P,std::vector<P>,std::greater<P>> pq;
  heuristic_[idx(gx,gy)] = 0.0;
  pq.push({0.0, idx(gx,gy)});

  static const int dx[8]={-1,-1,0,1,1,1,0,-1},
                   dy[8]={ 0, 1,1,1,0,-1,-1,-1};

  while(!pq.empty()){
    auto [c,u] = pq.top(); pq.pop();
    if (c>heuristic_[u]) continue;
    int ux=u%W, uy=u/W;
    for(int k=0;k<8;++k){
      int vx=ux+dx[k], vy=uy+dy[k];
      if(vx<0||vy<0||vx>=W||vy>=H) continue;
      int v=idx(vx,vy);
      if(map_.data[v]>=100) continue;
      double nc = c + std::hypot(dx[k],dy[k]) * res;
      if (nc < heuristic_[v]) {
        heuristic_[v] = nc;
        pq.push({nc,v});
      }
    }
  }
}

double HybridAStarPlanner::computeHeuristic(const State &s) const {
  auto &info = map_.info;
  double res = info.resolution;
  int gx = std::clamp(int((s.x-info.origin.position.x)/res),
                      0, int(info.width)-1);
  int gy = std::clamp(int((s.y-info.origin.position.y)/res),
                      0, int(info.height)-1);
  return kHybridWeight * heuristic_[gy * info.width + gx];
}

bool HybridAStarPlanner::isPointInObstacle(double x, double y) const {
  auto &info = map_.info;
  double res = info.resolution;
  int ix = int(std::floor((x - info.origin.position.x)/res));
  int iy = int(std::floor((y - info.origin.position.y)/res));
  if(ix<0||iy<0||ix>=int(info.width)||iy>=int(info.height)) return true;
  return map_.data[iy*info.width + ix] >= 100;
}

std::vector<std::pair<double,double>>
HybridAStarPlanner::getVehicleCorners(const State &s) const {
  double L = vehicle_model_.length, W = vehicle_model_.width;
  double c = std::cos(s.theta), s_ = std::sin(s.theta);
  std::vector<std::pair<double,double>> lc = {
    { L/2,  W/2}, { L/2, -W/2},
    {-L/2, -W/2}, {-L/2,  W/2}
  }, out;
  for (auto &p : lc) {
    out.push_back({
      s.x + p.first*c - p.second*s_,
      s.y + p.first*s_ + p.second*c
    });
  }
  return out;
}

bool HybridAStarPlanner::isVehicleInCollision(const State &s) const {
  for (auto &p : getVehicleCorners(s)) {
    if (isPointInObstacle(p.first, p.second)) return true;
  }
  double L = vehicle_model_.length, W = vehicle_model_.width;
  double c = std::cos(s.theta), s_ = std::sin(s.theta);
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

bool HybridAStarPlanner::isInCollision(const State &s) const {
  return isVehicleInCollision(s);
}

void HybridAStarPlanner::buildObstacleKDTree() {
  auto &info = map_.info;
  double res = info.resolution;
  obstacle_cloud_.points.clear();  
  for(int y=0;y<int(info.height);++y){
    for(int x=0;x<int(info.width);++x){
      int idx = y*info.width + x;
      if(map_.data[idx]>=100){
        obstacle_cloud_.points.push_back({
          info.origin.position.x + x*res,
          info.origin.position.y + y*res
        });
      }
    }
  }
  obstacle_tree_.reset(new KDTree(
    2, obstacle_cloud_,
    nanoflann::KDTreeSingleIndexAdaptorParams(10)
  ));
  obstacle_tree_->buildIndex();
}

} // namespace hybrid_astar
