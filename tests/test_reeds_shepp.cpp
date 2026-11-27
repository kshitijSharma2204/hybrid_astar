#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/reeds_shepp.hpp"

#include <gtest/gtest.h>

using namespace hybrid_astar;

namespace {

GridMap emptyGrid(double size_m, double res = 0.1) {
  GridMap g;
  g.width = g.height = int(size_m / res);
  g.resolution = res;
  g.data.assign(size_t(g.width) * g.height, 0);
  return g;
}

} // namespace

TEST(ReedsShepp, SamplesAreDenseAndEndAtGoal) {
  ReedsShepp rs(PlannerConfig{}.minTurningRadius());
  Pose2D a{2.0, 3.0, 0.3}, b{4.0, 1.0, -2.5};
  std::vector<Pose2D> poses;
  std::vector<int> dirs;
  rs.sample(a, b, 0.1, poses, dirs);
  ASSERT_FALSE(poses.empty());
  ASSERT_EQ(poses.size(), dirs.size());
  double len = 0.0;
  Pose2D prev = a;
  for (const auto& p : poses) {
    double d = std::hypot(p.x - prev.x, p.y - prev.y);
    EXPECT_LE(d, 0.1 + 1e-9);
    len += d;
    prev = p;
  }
  EXPECT_NEAR(poses.back().x, b.x, 1e-6);
  EXPECT_NEAR(poses.back().y, b.y, 1e-6);
  EXPECT_NEAR(angleDiff(poses.back().theta, b.theta), 0.0, 1e-6);
  // chords are a little shorter than the arcs
  EXPECT_NEAR(len, rs.length(a, b), 0.01);
}

TEST(ReedsShepp, PureReverseIsMarkedReverse) {
  ReedsShepp rs(5.0);
  std::vector<Pose2D> poses;
  std::vector<int> dirs;
  rs.sample({10.0, 0.0, 0.0}, {7.0, 0.0, 0.0}, 0.1, poses, dirs);
  ASSERT_EQ(poses.size(), 30u);
  for (int d : dirs) EXPECT_EQ(d, -1);
  EXPECT_NEAR(rs.length({10.0, 0.0, 0.0}, {7.0, 0.0, 0.0}), 3.0, 1e-9);
}

TEST(Planner, EmptyMapEndsExactlyAtGoal) {
  GridMap g = emptyGrid(40.0);
  PlannerConfig cfg;
  HybridAStarPlanner planner(cfg);
  for (Pose2D goal : {Pose2D{30.0, 25.0, M_PI / 2}, Pose2D{12.0, 8.0, M_PI},
                      Pose2D{25.0, 10.0, -2.0}}) {
    PlanResult r = planner.plan(g, {10.0, 10.0, 0.0}, goal);
    ASSERT_TRUE(r.stats.success) << r.message;
    const Pose2D& end = r.poses.back();
    EXPECT_LT(std::hypot(end.x - goal.x, end.y - goal.y), 1e-6);
    EXPECT_LT(std::fabs(angleDiff(end.theta, goal.theta)), 1e-6);
  }
}

TEST(Planner, ReedsSheppAttemptsAreTraced) {
  GridMap g = emptyGrid(40.0);
  // a wall between start and goal so the first shots hit it
  for (int iy = 0; iy < 300; ++iy)
    for (int ix = 195; ix < 205; ++ix) g.data[size_t(iy) * g.width + ix] = 100;
  HybridAStarPlanner planner(PlannerConfig{});
  PlanOptions opts;
  opts.record_trace = true;
  PlanResult r = planner.plan(g, {10.0, 10.0, 0.0}, {30.0, 10.0, 0.0}, opts);
  ASSERT_TRUE(r.stats.success) << r.message;
  const auto& attempts = r.trace->rs_attempts;
  ASSERT_GE(attempts.size(), 2u);
  EXPECT_TRUE(attempts.front().collided);
  EXPECT_FALSE(attempts.back().collided);
  for (const auto& a : attempts) {
    EXPECT_GE(a.node, 0);
    EXPECT_LT(a.node, int(r.trace->nodes.size()));
  }
}
