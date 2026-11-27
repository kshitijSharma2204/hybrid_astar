#include "hybrid_astar/config.hpp"
#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"

#include <gtest/gtest.h>


using namespace hybrid_astar;


namespace {

// Checks a path pose by pose: consecutive poses at most collision_step
// apart, no collisions, headings wrapped, direction set on every pose.
void expectValidPath(const PlanResult& r, const GridMap& grid,
                     const PlannerConfig& cfg) {
  ASSERT_EQ(r.poses.size(), r.direction.size());
  CollisionChecker cc(grid, cfg);
  for (size_t i = 0; i < r.poses.size(); ++i) {
    const auto& p = r.poses[i];
    EXPECT_FALSE(cc.inCollision(p)) << "pose " << i;
    EXPECT_GT(p.theta, -M_PI);
    EXPECT_LE(p.theta, M_PI);
    EXPECT_TRUE(r.direction[i] == 1 || r.direction[i] == -1);
    if (i > 0) {
      double d = std::hypot(p.x - r.poses[i-1].x, p.y - r.poses[i-1].y);
      EXPECT_LE(d, cfg.collision_step + 1e-9) << "pose " << i;
      EXPECT_GT(d, 1e-6) << "pose " << i << " repeats the one before";
    }
  }
}

} // namespace

TEST(Planner, S1PathIsValid) {
  const std::string root = HA_SOURCE_DIR;
  Scenario sc = loadScenario(root + "/scenarios/S1.txt");
  PlannerConfig cfg = loadConfig(root + "/config/params.yaml");
  HybridAStarPlanner planner(cfg);
  PlanResult r = planner.plan(sc.grid, sc.start, sc.goal);
  ASSERT_TRUE(r.stats.success) << r.message;
  expectValidPath(r, sc.grid, cfg);
  EXPECT_DOUBLE_EQ(r.poses.front().x, sc.start.x);
  EXPECT_DOUBLE_EQ(r.poses.front().y, sc.start.y);
  EXPECT_LT(std::hypot(r.poses.back().x - sc.goal.x, r.poses.back().y - sc.goal.y),
            cfg.goal_pos_tol);
  EXPECT_LT(std::fabs(angleDiff(r.poses.back().theta, sc.goal.theta)), cfg.goal_heading_tol);
}

TEST(Planner, GoalCheckWrapsHeading) {
  // Start and goal at the same spot, headings 3.1 and -3.1 rad: 0.083 rad
  // apart, inside the 0.3 rad tolerance. v1 compared them as 6.2 rad apart.
  GridMap g;
  g.width = g.height = 200;
  g.resolution = 0.1;
  g.data.assign(200 * 200, 0);
  HybridAStarPlanner planner(PlannerConfig{});
  PlanResult r = planner.plan(g, {10.0, 10.0, 3.1}, {10.0, 10.0, -3.1});
  ASSERT_TRUE(r.stats.success) << r.message;
  EXPECT_EQ(r.stats.expansions, 1);
}

TEST(Planner, ScenarioLoaderReadsS1) {
  Scenario sc = loadScenario(std::string(HA_SOURCE_DIR) + "/scenarios/S1.txt");
  EXPECT_EQ(sc.grid.width, 800);
  EXPECT_EQ(sc.grid.height, 800);
  EXPECT_DOUBLE_EQ(sc.start.x, 35.0);
  EXPECT_DOUBLE_EQ(sc.goal.theta, M_PI);
  // The divider in env_publisher.py: image columns 396..403, rows 100..699,
  // which is grid rows 100..699 after the flip.
  auto occ = [&](int ix, int iy) { return sc.grid.data[size_t(iy) * 800 + ix] >= 100; };
  EXPECT_TRUE(occ(396, 100));
  EXPECT_TRUE(occ(403, 699));
  EXPECT_FALSE(occ(404, 450));   // just right of the divider, between bars
  EXPECT_FALSE(occ(396, 50));    // below the lowest bar
  // the bar at image row 400 covers columns 300..499
  EXPECT_TRUE(occ(300, 400));
  EXPECT_FALSE(occ(299, 400));
}
