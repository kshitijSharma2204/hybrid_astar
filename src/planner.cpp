#include "hybrid_astar/planner.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <queue>
#include <unordered_set>

namespace hybrid_astar {

HybridAStarPlanner::HybridAStarPlanner(const PlannerConfig& config)
  : config_{config},
    yaw_resolution_{2*M_PI / config.heading_bins}
{
  motion_primitives_ = generateMotionPrimitives(config_);
}

PlanResult HybridAStarPlanner::plan(
    const GridMap& map,
    const Pose2D& start_pose,
    const Pose2D& goal_pose,
    const PlanOptions& options)
{
  auto t_start = std::chrono::steady_clock::now();
  auto elapsedMs = [&]() {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
  };
  PlanResult result;
  if (options.record_trace) result.trace.emplace();
  SearchTrace* trace = result.trace ? &*result.trace : nullptr;
  map_ = map;
  checker_ = std::make_unique<CollisionChecker>(map_, config_);
  buildObstacleKDTree();

  // Build start state
  State start;
  start.x = start_pose.x;
  start.y = start_pose.y;
  start.theta = start_pose.theta;
  start.g = 0.0; start.h = 0.0;
  start.parent = nullptr;
  start.is_reverse = false;
  start.steering_angle = 0.0;

  // Build goal state
  State goal;
  goal.x = goal_pose.x;
  goal.y = goal_pose.y;
  goal.theta = goal_pose.theta;

  // Build grid‐based Dijkstra heuristic
  buildHeuristic(goal);

  // Sanity checks
  if (isInCollision(start)) {
    result.message = "start pose is in collision";
    log(result.message);
    return result;
  }
  if (isInCollision(goal)) {
    result.message = "goal pose is in collision";
    log(result.message);
    return result;
  }

  {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "plan(): from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",
      start.x, start.y, start.theta,
      goal.x,  goal.y,  goal.theta);
    log(buf);
  }

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
    double res = map_.resolution;
    double ox  = map_.origin_x;
    double oy  = map_.origin_y;
    int W      = map_.width;

    int ix = int(std::round((s.x - ox)/res));
    int iy = int(std::round((s.y - oy)/res));
    double th = std::fmod(s.theta, 2*M_PI);
    if (th < 0) th += 2*M_PI;
    int iyaw = int(std::floor(th/yaw_resolution_)) % config_.heading_bins;

    ix = std::clamp(ix, 0, W-1);
    iy = std::clamp(iy, 0, map_.height-1);
    return packKey(ix, iy, iyaw);
  };

  // push start
  open_set.push(std::make_shared<State>(start));
  std::unordered_set<uint64_t> closed_set;

  const double pos_tol   = config_.goal_pos_tol;
  const double theta_tol = config_.goal_heading_tol;
  const int n_steps = std::max(1, int(std::ceil(
    config_.step_size / config_.collision_step - 1e-9)));

  // main A* loop
  while (!open_set.empty()) {
    auto current = open_set.top(); open_set.pop();

    uint64_t key = discretize(*current);
    if (closed_set.count(key)) continue;
    closed_set.insert(key);
    ++result.stats.expansions;
    if (trace) {
      current->trace_id = int(trace->nodes.size());
      trace->nodes.push_back({current->x, current->y, current->theta,
                              current->parent ? current->parent->trace_id : -1});
      if (options.on_progress &&
          result.stats.expansions % std::max(1, options.progress_every) == 0) {
        options.on_progress(*trace);
      }
    }

    // 2) fallback: goal‐proximity
    double dx    = current->x - goal.x;
    double dy    = current->y - goal.y;
    double dpos  = std::hypot(dx, dy);
    double dtheta= std::fabs(current->theta - goal.theta);
    if (dpos < pos_tol && dtheta < theta_tol) {
      // prepend start
      result.poses.push_back(start_pose);
      result.direction.push_back(1);
      // backtrack
      std::vector<std::shared_ptr<State>> tail;
      for (auto s = current; s!=nullptr; s=s->parent) tail.push_back(s);
      std::reverse(tail.begin(), tail.end());
      for (size_t i = 0; i < tail.size(); ++i) {
        result.poses.push_back({tail[i]->x, tail[i]->y, tail[i]->theta});
        // the start node has no motion; give it the direction of the next one
        const State& m = (i == 0 && tail.size() > 1) ? *tail[1] : *tail[i];
        result.direction.push_back(m.is_reverse ? -1 : 1);
      }
      result.direction[0] = result.direction[1];
      result.stats.success = true;
      result.stats.search_ms = elapsedMs();
      result.message = "found a path with " +
                       std::to_string(result.poses.size()) + " poses";
      log("plan(): " + result.message);
      return result;
    }

    // 3) expand non‐holonomic primitives (with full cost + sub‐step collision)
    for (auto &prim : motion_primitives_) {
      bool   bad       = false;
      State  last = *current;

      for (int i=1; i<=n_steps; ++i) {
        double frac = double(i)/double(n_steps);
        double l    = config_.step_size * (prim.is_reverse ? -1.0 : 1.0);
        Pose2D p = primitivePose({current->x, current->y, current->theta},
                                 prim.steering_angle, l * frac,
                                 config_.wheel_base);

        State temp;
        temp.x             = p.x;
        temp.y             = p.y;
        temp.theta         = p.theta;
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
                   ? config_.reverse_penalty : 1.0)
               + (prim.is_reverse != current->is_reverse
                   ? config_.direction_change_cost : 0.0)
               + std::abs(prim.steering_angle
                   - current->steering_angle)
                   * config_.steer_change_cost
               + std::abs(prim.steering_angle)
                   * config_.steer_cost;

      // push successor
      auto succ = std::make_shared<State>(last);
      succ->g              = g;
      succ->h              = computeHeuristic(*succ);
      succ->parent         = current;
      open_set.push(succ);
    }
  }

  result.stats.search_ms = elapsedMs();
  result.message = "open set exhausted, no path";
  log("plan(): " + result.message);
  return result;
}

void HybridAStarPlanner::buildHeuristic(const State &goal) {
  int W = map_.width, H = map_.height;
  double res = map_.resolution;
  heuristic_.assign(W*H, std::numeric_limits<double>::infinity());

  auto idx=[&](int x,int y){ return y*W + x; };
  int gx = std::clamp(int((goal.x-map_.origin_x)/res),
                      0, W-1);
  int gy = std::clamp(int((goal.y-map_.origin_y)/res),
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
  double res = map_.resolution;
  int gx = std::clamp(int((s.x-map_.origin_x)/res),
                      0, map_.width-1);
  int gy = std::clamp(int((s.y-map_.origin_y)/res),
                      0, map_.height-1);
  return config_.heuristic_weight * heuristic_[gy * map_.width + gx];
}

bool HybridAStarPlanner::isInCollision(const State &s) const {
  return checker_->inCollision({s.x, s.y, s.theta});
}

void HybridAStarPlanner::buildObstacleKDTree() {
  double res = map_.resolution;
  obstacle_cloud_.points.clear();  
  for(int y=0;y<map_.height;++y){
    for(int x=0;x<map_.width;++x){
      int idx = y*map_.width + x;
      if(map_.data[idx]>=100){
        obstacle_cloud_.points.push_back({
          map_.origin_x + x*res,
          map_.origin_y + y*res
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

void HybridAStarPlanner::log(const std::string& msg) const {
  if (logger_) logger_(msg);
}

} // namespace hybrid_astar
