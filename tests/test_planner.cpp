#include "hybrid_astar/config.hpp"
#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

using namespace hybrid_astar;

namespace {

std::vector<Pose2D> readCsv(const std::string& file) {
  std::ifstream in(file);
  std::vector<Pose2D> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#' || line[0] == 'x') continue;
    std::stringstream ss(line);
    Pose2D p;
    char comma;
    ss >> p.x >> comma >> p.y >> comma >> p.theta;
    out.push_back(p);
  }
  return out;
}

} // namespace

TEST(Planner, S1MatchesV1Baseline) {
  const std::string root = HA_SOURCE_DIR;
  Scenario sc = loadScenario(root + "/scenarios/S1.txt");
  HybridAStarPlanner planner(loadConfig(root + "/config/params.yaml"));
  PlanResult r = planner.plan(sc.grid, sc.start, sc.goal);
  ASSERT_TRUE(r.stats.success) << r.message;

  auto baseline = readCsv(root + "/results/v2/baseline_v1_S1.csv");
  ASSERT_EQ(r.poses.size(), baseline.size());
  for (size_t i = 0; i < baseline.size(); ++i) {
    EXPECT_NEAR(r.poses[i].x, baseline[i].x, 1e-5) << i;
    EXPECT_NEAR(r.poses[i].y, baseline[i].y, 1e-5) << i;
    // v1 never wrapped headings, so compare them modulo 2 pi
    EXPECT_NEAR(angleDiff(r.poses[i].theta, baseline[i].theta), 0.0, 1e-5) << i;
  }
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
