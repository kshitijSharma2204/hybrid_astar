#include "hybrid_astar/planner.hpp"

#include "hybrid_astar/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <queue>

namespace hybrid_astar {

HybridAStarPlanner::HybridAStarPlanner(const PlannerConfig& config)
  : config_{config},
    rs_{config.minTurningRadius()},
    yaw_resolution_{2*M_PI / config.heading_bins}
{
  motion_primitives_ = generateMotionPrimitives(config_);
  n_substeps_ = std::max(1, int(std::ceil(
    config_.step_size / config_.collision_step - 1e-9)));
}

void HybridAStarPlanner::setMap(const GridMap& map) {
  auto t0 = std::chrono::steady_clock::now();
  map_ = map;
  checker_ = std::make_unique<CollisionChecker>(map_, config_);
  setup_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
}

PlanResult HybridAStarPlanner::plan(
    const GridMap& map,
    const Pose2D& start_pose,
    const Pose2D& goal_pose,
    const PlanOptions& options)
{
  setMap(map);
  return plan(start_pose, goal_pose, options);
}

PlanResult HybridAStarPlanner::plan(
    const Pose2D& start_pose,
    const Pose2D& goal_pose,
    const PlanOptions& options)
{
  PlanResult result;
  if (!checker_) {
    result.message = "no map, call setMap() first";
    return result;
  }
  auto t_start = std::chrono::steady_clock::now();
  auto elapsedMs = [&]() {
    return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
  };
  if (options.record_trace) result.trace.emplace();
  SearchTrace* trace = result.trace ? &*result.trace : nullptr;
  nodes_.clear();

  Node start;
  start.x = start_pose.x;
  start.y = start_pose.y;
  start.theta = wrapAngle(start_pose.theta);

  Node goal;
  goal.x = goal_pose.x;
  goal.y = goal_pose.y;
  goal.theta = wrapAngle(goal_pose.theta);

  goal_ = {goal.x, goal.y, goal.theta};
  buildHeuristic(goal);

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

  // The closed set is a flat array over (x cell, y cell, heading bin). It
  // has its own cell size, so it gets its own grid header and goes through
  // the same world -> cell conversion as everything else.
  GridMap closed_grid;
  closed_grid.resolution = config_.xy_resolution;
  closed_grid.origin_x = map_.origin_x;
  closed_grid.origin_y = map_.origin_y;
  closed_grid.width = int(std::ceil(map_.width * map_.resolution / config_.xy_resolution));
  closed_grid.height = int(std::ceil(map_.height * map_.resolution / config_.xy_resolution));
  std::vector<uint8_t> closed(
    size_t(closed_grid.width) * closed_grid.height * config_.heading_bins, 0);
  auto closedIndex = [&](const Node &s) {
    int ix = std::clamp(worldToCellX(closed_grid, s.x), 0, closed_grid.width-1);
    int iy = std::clamp(worldToCellY(closed_grid, s.y), 0, closed_grid.height-1);
    double th = s.theta;   // already in (-pi, pi]
    if (th < 0) th += 2*M_PI;
    int iyaw = int(std::floor(th/yaw_resolution_)) % config_.heading_bins;
    return (size_t(iy) * closed_grid.width + ix) * config_.heading_bins + iyaw;
  };

  // Open set: min-heap on f = g + h. Ties go to the older node so runs are
  // repeatable.
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open_set;
  start.h = computeHeuristic(start);
  nodes_.push_back(start);
  open_set.push({start.g + start.h, 0});

  while (!open_set.empty()) {
    if (result.stats.expansions >= config_.max_expansions) {
      result.stats.search_ms = elapsedMs();
      result.message = "gave up after max_expansions";
      log("plan(): " + result.message);
      return result;
    }
    int cur = open_set.top().second;
    open_set.pop();

    size_t key = closedIndex(nodes_[cur]);
    if (closed[key]) continue;
    closed[key] = 1;
    ++result.stats.expansions;
    if (trace) {
      Node& n = nodes_[cur];
      n.trace_id = int(trace->nodes.size());
      trace->nodes.push_back({n.x, n.y, n.theta,
                              n.parent >= 0 ? nodes_[n.parent].trace_id : -1});
      if (options.on_progress &&
          result.stats.expansions % std::max(1, options.progress_every) == 0) {
        options.on_progress(*trace);
      }
    }

    const Node current = nodes_[cur];
    bool done = false;
    if (config_.analytic_expansion_interval > 0) {
      // Shoot a Reeds-Shepp curve at the goal every N expansions, and on
      // every expansion once we are close (by the 2D distance-to-goal map).
      bool close = heuristic2d(current) < config_.analytic_expansion_distance;
      if (close ||
          result.stats.expansions % config_.analytic_expansion_interval == 0) {
        done = tryReedsSheppConnect(cur, goal, result, trace);
      }
    } else {
      double dpos   = std::hypot(current.x - goal.x, current.y - goal.y);
      double dtheta = std::fabs(angleDiff(current.theta, goal.theta));
      if (dpos < config_.goal_pos_tol && dtheta < config_.goal_heading_tol) {
        buildPath(cur, result);
        done = true;
      }
    }
    if (done) {
      result.stats.success = true;
      result.stats.search_ms = elapsedMs();
      result.message = "found a path with " +
                       std::to_string(result.poses.size()) + " poses";
      log("plan(): " + result.message);
      return result;
    }

    for (int k = 0; k < int(motion_primitives_.size()); ++k) {
      const auto& prim = motion_primitives_[k];
      double l = config_.step_size * (prim.is_reverse ? -1.0 : 1.0);
      bool bad = false;
      Pose2D last;
      for (int i = 1; i <= n_substeps_; ++i) {
        last = primitivePose({current.x, current.y, current.theta},
                             prim.steering_angle, l * i / n_substeps_,
                             config_.wheel_base);
        if (checker_->inCollision(last)) {
          bad = true;
          break;
        }
      }
      if (bad) continue;

      Node succ;
      succ.x = last.x;
      succ.y = last.y;
      succ.theta = last.theta;
      succ.is_reverse = prim.is_reverse;
      succ.steering_angle = prim.steering_angle;
      succ.parent = cur;
      succ.primitive = k;
      succ.g = current.g
             + config_.step_size * (prim.is_reverse
                 ? config_.reverse_penalty : 1.0)
             // the start has no direction yet, so its first move is free
             + (cur != 0 && prim.is_reverse != current.is_reverse
                 ? config_.direction_change_cost : 0.0)
             + std::abs(prim.steering_angle - current.steering_angle)
                 * config_.steer_change_cost
             + std::abs(prim.steering_angle) * config_.steer_cost;
      if (closed[closedIndex(succ)]) continue;
      succ.h = computeHeuristic(succ);
      nodes_.push_back(succ);
      open_set.push({succ.g + succ.h, int(nodes_.size()) - 1});
    }
  }

  result.stats.search_ms = elapsedMs();
  result.message = "open set exhausted, no path";
  log("plan(): " + result.message);
  return result;
}

// Walks back from the goal node and re-runs each primitive, so the path
// has a pose every collision_step meters, each tagged with its direction.
void HybridAStarPlanner::buildPath(int goal_node, PlanResult& result) const {
  std::vector<int> chain;
  for (int i = goal_node; i >= 0; i = nodes_[i].parent) chain.push_back(i);
  std::reverse(chain.begin(), chain.end());

  const Node& s = nodes_[chain[0]];
  result.poses.push_back({s.x, s.y, s.theta});
  result.direction.push_back(1);
  for (size_t c = 1; c < chain.size(); ++c) {
    const Node& from = nodes_[chain[c-1]];
    const Node& to = nodes_[chain[c]];
    const auto& prim = motion_primitives_[to.primitive];
    double l = config_.step_size * (prim.is_reverse ? -1.0 : 1.0);
    int dir = prim.is_reverse ? -1 : 1;
    for (int i = 1; i <= n_substeps_; ++i) {
      result.poses.push_back(primitivePose({from.x, from.y, from.theta},
                                           prim.steering_angle,
                                           l * i / n_substeps_,
                                           config_.wheel_base));
      result.direction.push_back(dir);
    }
  }
  // The start pose drives off in the direction of the first motion.
  if (result.direction.size() > 1) result.direction[0] = result.direction[1];
}

void HybridAStarPlanner::buildHeuristic(const Node &goal) {
  int W = map_.width, H = map_.height;
  double res = map_.resolution;
  heuristic_.assign(W*H, std::numeric_limits<double>::infinity());

  auto idx=[&](int x,int y){ return y*W + x; };
  int gx = std::clamp(worldToCellX(map_, goal.x), 0, W-1);
  int gy = std::clamp(worldToCellY(map_, goal.y), 0, H-1);

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

double HybridAStarPlanner::heuristic2d(const Node &s) const {
  int gx = std::clamp(worldToCellX(map_, s.x), 0, map_.width-1);
  int gy = std::clamp(worldToCellY(map_, s.y), 0, map_.height-1);
  return heuristic_[gy * map_.width + gx];
}

// Two lower bounds on the remaining path length; I take the larger. The
// 2D one knows the obstacles but not the turning radius, the Reeds-Shepp
// one knows the turning radius but not the obstacles.
double HybridAStarPlanner::computeHeuristic(const Node &s) const {
  double h = heuristic2d(s);
  if (config_.rs_heuristic) {
    h = std::max(h, rs_.length({s.x, s.y, s.theta}, goal_));
  }
  return config_.heuristic_weight * h;
}

// Samples the shortest Reeds-Shepp curve from the node to the goal and
// checks every sample with the full footprint. On success the curve is
// appended to the search path and the result is complete.
bool HybridAStarPlanner::tryReedsSheppConnect(int node, const Node& goal,
                                              PlanResult& result,
                                              SearchTrace* trace) const {
  const Node& n = nodes_[node];
  std::vector<Pose2D> poses;
  std::vector<int> dirs;
  rs_.sample({n.x, n.y, n.theta}, {goal.x, goal.y, goal.theta},
             config_.rs_sample_step, poses, dirs);

  bool collided = false;
  for (const auto& p : poses) {
    if (checker_->inCollision(p)) {
      collided = true;
      break;
    }
  }
  if (trace) trace->rs_attempts.push_back({n.trace_id, collided, poses});
  if (collided) return false;

  buildPath(node, result);
  result.poses.insert(result.poses.end(), poses.begin(), poses.end());
  result.direction.insert(result.direction.end(), dirs.begin(), dirs.end());
  if (result.direction.size() > 1 && result.poses.size() == poses.size() + 1) {
    result.direction[0] = result.direction[1];   // path is only the curve
  }
  return true;
}

bool HybridAStarPlanner::isInCollision(const Node &s) const {
  return checker_->inCollision({s.x, s.y, s.theta});
}

void HybridAStarPlanner::log(const std::string& msg) const {
  if (logger_) logger_(msg);
}

} // namespace hybrid_astar
