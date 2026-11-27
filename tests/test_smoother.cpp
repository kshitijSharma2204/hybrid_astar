#include "hybrid_astar/config.hpp"
#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/metrics.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"
#include "hybrid_astar/smoother.hpp"

#include <gtest/gtest.h>

using namespace hybrid_astar;

namespace {

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

class SmootherOnScenario : public ::testing::TestWithParam<const char*> {};

TEST_P(SmootherOnScenario, KeepsEndsAndCuspsAndLimits) {
  const std::string root = HA_SOURCE_DIR;
  Scenario sc = loadScenario(root + "/scenarios/" + GetParam() + ".txt");
  PlannerConfig cfg = loadConfig(root + "/config/params.yaml");
  HybridAStarPlanner planner(cfg);
  PlanResult r = planner.plan(sc.grid, sc.start, sc.goal);
  ASSERT_TRUE(r.stats.success) << r.message;

  expectSamePose(r.poses.front(), r.raw_poses.front(), "start");
  expectSamePose(r.poses.back(), r.raw_poses.back(), "goal");

  // the same cusps, at the same poses, in the same order
  auto c_raw = cusps(r.raw_direction), c_new = cusps(r.direction);
  ASSERT_EQ(c_raw.size(), c_new.size());
  for (size_t i = 0; i < c_raw.size(); ++i) {
    expectSamePose(r.poses[c_new[i]], r.raw_poses[c_raw[i]], "cusp " + std::to_string(i));
  }

  CollisionChecker cc(sc.grid, cfg);
  for (size_t i = 0; i < r.poses.size(); ++i) {
    EXPECT_FALSE(cc.exactCollision(r.poses[i])) << "pose " << i;
  }
  double kmax = 1.0 / cfg.minTurningRadius();
  EXPECT_LE(maxCurvature(r.poses, r.direction), cfg.smooth_curvature_limit * kmax);
  EXPECT_EQ(r.stats.smooth_fallbacks, 0);
}

INSTANTIATE_TEST_SUITE_P(Scenarios, SmootherOnScenario,
                         ::testing::Values("S1", "S2", "S3", "S4", "S5/seed_00"));

TEST(Smoother, StraightLineStaysStraight) {
  GridMap g;
  g.width = g.height = 300;
  g.resolution = 0.1;
  g.data.assign(300 * 300, 0);
  PlannerConfig cfg;
  CollisionChecker cc(g, cfg);
  std::vector<Pose2D> p;
  std::vector<int> d;
  for (int i = 0; i <= 50; ++i) {
    p.push_back({5.0 + 0.2 * i, 15.0, 0.0});
    d.push_back(1);
  }
  SmoothResult s = smoothPath(p, d, cc, cfg);
  for (const auto& q : s.poses) {
    EXPECT_NEAR(q.y, 15.0, 1e-9);
    EXPECT_NEAR(q.theta, 0.0, 1e-9);
  }
  EXPECT_NEAR(s.poses.back().x, 15.0, 1e-12);
}
