#include "hybrid_astar/metrics.hpp"

#include <gtest/gtest.h>

using namespace hybrid_astar;

TEST(Metrics, CurvatureOfACircle) {
  std::vector<Pose2D> p;
  std::vector<int> d;
  for (int i = 0; i < 50; ++i) {
    double a = 0.02 * i;
    p.push_back({4.0 * std::cos(a), 4.0 * std::sin(a), a + M_PI / 2});
    d.push_back(1);
  }
  EXPECT_NEAR(maxCurvature(p, d), 0.25, 1e-9);
}

TEST(Metrics, CuspIsNotACorner) {
  // forward along x, then straight back: a 180 degree corner at the cusp
  std::vector<Pose2D> p = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {1.5, 0.01, 0}, {1, 0.02, 0}};
  std::vector<int> d = {1, 1, 1, -1, -1};
  EXPECT_LT(maxCurvature(p, d), 0.1);
}

TEST(Metrics, LengthSwitchesClearance) {
  GridMap g;
  g.width = g.height = 200;
  g.resolution = 0.1;
  g.data.assign(200 * 200, 0);
  g.data[size_t(100) * 200 + 150] = 100;    // one cell, centre (15.05, 10.05)
  PlannerConfig cfg;
  cfg.rear_axle_to_center = 0.0;
  std::vector<Pose2D> p = {{10, 10, 0}, {11, 10, 0}, {10.5, 10, 0}};
  std::vector<int> d = {1, 1, -1};
  PathMetrics m = computeMetrics(p, d, g, cfg, {10.5, 10.0, 0.0});
  EXPECT_DOUBLE_EQ(m.length, 1.5);
  EXPECT_DOUBLE_EQ(m.reverse_length, 0.5);
  EXPECT_EQ(m.direction_switches, 1);
  // front bumper reaches 11 + 2.25 = 13.25, obstacle centre at 15.05
  // the cell centre is level with the car (0.05 m off its axis), so the gap
  // is straight ahead
  EXPECT_NEAR(m.min_clearance, 15.05 - 13.25, 1e-9);
  EXPECT_NEAR(m.goal_pos_error, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(m.cost, 1.0 + cfg.reverse_penalty * 0.5 + cfg.direction_change_cost);
}
