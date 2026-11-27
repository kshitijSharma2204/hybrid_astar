#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/utils.hpp"

#include <gtest/gtest.h>

using namespace hybrid_astar;

TEST(WrapAngle, StaysInHalfOpenRange) {
  EXPECT_DOUBLE_EQ(wrapAngle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(wrapAngle(M_PI), M_PI);
  EXPECT_DOUBLE_EQ(wrapAngle(-M_PI), M_PI);
  EXPECT_NEAR(wrapAngle(3 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(2 * M_PI + 0.1), 0.1, 1e-12);
  EXPECT_NEAR(wrapAngle(-2 * M_PI - 0.1), -0.1, 1e-12);
  for (double a = -20.0; a < 20.0; a += 0.37) {
    double w = wrapAngle(a);
    EXPECT_GT(w, -M_PI);
    EXPECT_LE(w, M_PI);
    EXPECT_NEAR(std::cos(w), std::cos(a), 1e-9);
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-9);
  }
}

TEST(WrapAngle, DifferenceAcrossPi) {
  // 3.1 and -3.1 rad are 0.083 rad apart, not 6.2.
  EXPECT_NEAR(angleDiff(3.1, -3.1), 3.1 + 3.1 - 2 * M_PI, 1e-12);
  EXPECT_NEAR(std::fabs(angleDiff(-3.1, 3.1)), 2 * M_PI - 6.2, 1e-12);
}

TEST(WorldToCell, RoundTripThroughCellCentre) {
  GridMap g;
  g.width = 50;
  g.height = 40;
  g.resolution = 0.1;
  g.origin_x = -1.3;
  g.origin_y = 2.05;
  for (int ix = 0; ix < g.width; ++ix) {
    EXPECT_EQ(worldToCellX(g, cellCenterX(g, ix)), ix);
  }
  for (int iy = 0; iy < g.height; ++iy) {
    EXPECT_EQ(worldToCellY(g, cellCenterY(g, iy)), iy);
  }
  // Points anywhere inside a cell map to that cell, including negatives.
  EXPECT_EQ(worldToCellX(g, g.origin_x + 0.0001), 0);
  EXPECT_EQ(worldToCellX(g, g.origin_x + 0.0999), 0);
  EXPECT_EQ(worldToCellX(g, g.origin_x - 0.0001), -1);
  EXPECT_EQ(worldToCellY(g, g.origin_y - 0.05), -1);
}

TEST(Primitive, FullSteerFollowsMinTurningCircle) {
  PlannerConfig cfg;
  double R = cfg.minTurningRadius();
  EXPECT_NEAR(R, 2.5 / std::tan(0.45), 1e-12);
  for (double steer : {cfg.max_steer, -cfg.max_steer}) {
    for (double dir : {1.0, -1.0}) {
      Pose2D start{3.0, -2.0, 0.7};
      double side = steer > 0 ? 1.0 : -1.0;
      // Centre of the turning circle is R to the left (or right) of the car.
      double cx = start.x - side * R * std::sin(start.theta);
      double cy = start.y + side * R * std::cos(start.theta);
      // 30 chained 1 m primitives, checked at the 0.2 m sub-steps the
      // planner uses. That is more than a full circle.
      Pose2D from = start;
      for (int k = 0; k < 30; ++k) {
        for (int i = 1; i <= 5; ++i) {
          Pose2D p = primitivePose(from, steer, dir * 0.2 * i, cfg.wheel_base);
          EXPECT_NEAR(std::hypot(p.x - cx, p.y - cy), R, 1e-9) << k << " " << i;
          EXPECT_GT(p.theta, -M_PI);
          EXPECT_LE(p.theta, M_PI);
        }
        from = primitivePose(from, steer, dir * 1.0, cfg.wheel_base);
      }
      double turned = side * dir * 30.0 / R;
      EXPECT_NEAR(angleDiff(from.theta, start.theta + turned), 0.0, 1e-9);
    }
  }
}

TEST(Primitive, StraightMovesAlongHeading) {
  Pose2D p = primitivePose({1.0, 1.0, M_PI / 2}, 0.0, -2.0, 2.5);
  EXPECT_NEAR(p.x, 1.0, 1e-12);
  EXPECT_NEAR(p.y, -1.0, 1e-12);
  EXPECT_NEAR(p.theta, M_PI / 2, 1e-12);
}
