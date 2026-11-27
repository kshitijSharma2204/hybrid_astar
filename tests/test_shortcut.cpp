#include "hybrid_astar/config.hpp"
#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/metrics.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"
#include "hybrid_astar/shortcut.hpp"
#include "hybrid_astar/utils.hpp"

#include <gtest/gtest.h>

using namespace hybrid_astar;

namespace {

double pathLength(const std::vector<Pose2D>& p) {
  double l = 0.0;
  for (size_t i = 1; i < p.size(); ++i) l += std::hypot(p[i].x - p[i-1].x, p[i].y - p[i-1].y);
  return l;
}

// sum of |heading change| between consecutive poses
double totalTurning(const std::vector<Pose2D>& p) {
  double t = 0.0;
  for (size_t i = 1; i < p.size(); ++i) t += std::fabs(angleDiff(p[i].theta, p[i-1].theta));
  return t;
}

std::vector<size_t> cusps(const std::vector<int>& dir) {
  std::vector<size_t> out;
  for (size_t i = 1; i + 1 < dir.size(); ++i)
    if (dir[i] != dir[i+1]) out.push_back(i);
  return out;
}

void expectSamePose(const Pose2D& a, const Pose2D& b, const std::string& what) {
  EXPECT_NEAR(a.x, b.x, 1e-9) << what;
  EXPECT_NEAR(a.y, b.y, 1e-9) << what;
  EXPECT_NEAR(angleDiff(a.theta, b.theta), 0.0, 1e-9) << what;
}

} // namespace

// Full left, full right, ... in free space: the shortcut should replace the
// zig-zag with curves that turn much less, are shorter and drive the same
// way. The zig-zag swings only about 11 degrees, so it is just 3 cm longer
// than the straightest path; the turning is what shows the difference.
TEST(Shortcut, StraightensAZigZag) {
  GridMap g;
  g.width = g.height = 400;
  g.resolution = 0.1;
  g.data.assign(400 * 400, 0);
  PlannerConfig cfg;
  CollisionChecker cc(g, cfg);
  ReedsShepp rs(cfg.minTurningRadius());

  std::vector<Pose2D> p = {{10.0, 20.0, 0.0}};
  std::vector<int> d = {1};
  for (int k = 0; k < 40; ++k) {
    double steer = (k / 2) % 2 ? -cfg.max_steer : cfg.max_steer;
    for (int i = 1; i <= 5; ++i) {
      p.push_back(primitivePose(p.back(), steer, 0.1, cfg.wheel_base));
      d.push_back(1);
    }
  }
  ShortcutResult s = shortcutPath(p, d, rs, cc, cfg);
  EXPECT_GE(s.shortcuts, 1);
  EXPECT_LT(pathLength(s.poses), pathLength(p));
  EXPECT_LT(totalTurning(s.poses), 0.25 * totalTurning(p));
  expectSamePose(s.poses.front(), p.front(), "start");
  expectSamePose(s.poses.back(), p.back(), "end");
  for (int dir : s.direction) EXPECT_EQ(dir, 1);
}

class ShortcutOnScenario : public ::testing::TestWithParam<const char*> {};

// Shortcut only (no smoothing): never longer, same ends and cusps, no
// collisions, and no tighter than the car can turn.
TEST_P(ShortcutOnScenario, ShorterAndValid) {
  const std::string root = HA_SOURCE_DIR;
  Scenario sc = loadScenario(root + "/scenarios/" + GetParam() + ".txt");
  PlannerConfig cfg = loadConfig(root + "/config/params.yaml");
  cfg.smooth_enabled = 0;
  HybridAStarPlanner planner(cfg);
  PlanResult r = planner.plan(sc.grid, sc.start, sc.goal);
  ASSERT_TRUE(r.stats.success) << r.message;

  EXPECT_LE(pathLength(r.poses), pathLength(r.raw_poses) + 1e-9);
  expectSamePose(r.poses.front(), r.raw_poses.front(), "start");
  expectSamePose(r.poses.back(), r.raw_poses.back(), "goal");
  auto c_raw = cusps(r.raw_direction), c_new = cusps(r.direction);
  ASSERT_EQ(c_raw.size(), c_new.size());
  for (size_t i = 0; i < c_raw.size(); ++i) {
    expectSamePose(r.poses[c_new[i]], r.raw_poses[c_raw[i]], "cusp " + std::to_string(i));
  }
  CollisionChecker cc(sc.grid, cfg);
  for (size_t i = 0; i < r.poses.size(); ++i) {
    EXPECT_FALSE(cc.exactCollision(r.poses[i])) << "pose " << i;
  }
  EXPECT_LE(maxCurvature(r.poses, r.direction), 1.0 / cfg.minTurningRadius() + 1e-6);
}

INSTANTIATE_TEST_SUITE_P(Scenarios, ShortcutOnScenario,
                         ::testing::Values("S1", "S2", "S3", "S4", "S5/seed_00"));

// v1 found an 88.0 m path on S1 (results/v2/baseline_v1_S1.csv); the
// defaults must not do worse.
TEST(Shortcut, S1NoLongerThanV1) {
  const std::string root = HA_SOURCE_DIR;
  Scenario sc = loadScenario(root + "/scenarios/S1.txt");
  PlannerConfig cfg = loadConfig(root + "/config/params.yaml");
  HybridAStarPlanner planner(cfg);
  PlanResult r = planner.plan(sc.grid, sc.start, sc.goal);
  ASSERT_TRUE(r.stats.success) << r.message;
  EXPECT_LT(pathLength(r.poses), 88.0);
}
