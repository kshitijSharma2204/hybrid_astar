#include "hybrid_astar/collision.hpp"

#include <gtest/gtest.h>

using namespace hybrid_astar;

namespace {

GridMap emptyGrid(int w, int h, double res) {
  GridMap g;
  g.width = w;
  g.height = h;
  g.resolution = res;
  g.data.assign(size_t(w) * h, 0);
  return g;
}

// Fills every cell whose x range starts at or after x_face.
void wallFrom(GridMap& g, double x_face) {
  for (int iy = 0; iy < g.height; ++iy) {
    for (int ix = 0; ix < g.width; ++ix) {
      if (g.origin_x + ix * g.resolution >= x_face - 1e-9) {
        g.data[size_t(iy) * g.width + ix] = 100;
      }
    }
  }
}

} // namespace

// Most of these put the pose at the footprint centre (rear_axle_to_center
// = 0) to keep the numbers simple. The car is 4.5 m long, so facing +x at
// x = 10 its bumper is at 12.25.

namespace {
PlannerConfig centred() {
  PlannerConfig cfg;
  cfg.rear_axle_to_center = 0.0;
  return cfg;
}
} // namespace

TEST(Collision, BumperOnWallFaceCollides) {
  // 0.125 m cells keep the wall face exactly representable.
  GridMap g = emptyGrid(160, 80, 0.125);
  wallFrom(g, 12.25);
  PlannerConfig cfg = centred();
  CollisionChecker cc(g, cfg);
  EXPECT_TRUE(cc.inCollision({10.0, 5.0, 0.0}));
}

TEST(Collision, BumperOneCmIntoWallCollides) {
  GridMap g = emptyGrid(200, 100, 0.1);
  wallFrom(g, 12.3);
  PlannerConfig cfg = centred();
  CollisionChecker cc(g, cfg);
  EXPECT_TRUE(cc.inCollision({10.06, 5.0, 0.0}));   // bumper at 12.31
}

TEST(Collision, TenCmGapIsFree) {
  GridMap g = emptyGrid(200, 100, 0.1);
  wallFrom(g, 12.4);
  PlannerConfig cfg = centred();
  CollisionChecker cc(g, cfg);
  EXPECT_FALSE(cc.inCollision({10.05, 5.0, 0.0}));  // bumper at 12.30
  EXPECT_TRUE(cc.inCollision({10.16, 5.0, 0.0}));   // bumper at 12.41
}

TEST(Collision, SideGapAndRotation) {
  GridMap g = emptyGrid(200, 200, 0.1);
  // a wall along y >= 6.0
  for (int iy = 60; iy < 200; ++iy)
    for (int ix = 0; ix < 200; ++ix) g.data[size_t(iy) * 200 + ix] = 100;
  PlannerConfig cfg = centred();
  CollisionChecker cc(g, cfg);
  EXPECT_FALSE(cc.inCollision({10.0, 4.9, 0.0}));          // side at 5.9
  EXPECT_TRUE(cc.inCollision({10.0, 5.05, 0.0}));          // side at 6.05
  // Turned 90 degrees the car is 4.5 m tall, so the same spot collides.
  EXPECT_TRUE(cc.inCollision({10.0, 4.9, M_PI / 2}));
  EXPECT_FALSE(cc.inCollision({10.0, 3.65, M_PI / 2}));    // front at 5.9
}

TEST(Collision, OutsideMapCollides) {
  GridMap g = emptyGrid(100, 100, 0.1);
  PlannerConfig cfg = centred();
  CollisionChecker cc(g, cfg);
  EXPECT_FALSE(cc.inCollision({5.0, 5.0, 0.0}));
  EXPECT_TRUE(cc.inCollision({1.0, 5.0, 0.0}));            // rear at -1.25
}

TEST(Collision, PoseIsTheRearAxle) {
  GridMap g = emptyGrid(200, 100, 0.1);
  wallFrom(g, 12.4);
  PlannerConfig cfg;   // rear_axle_to_center = 1.25, so 1.0 m rear overhang
  CollisionChecker cc(g, cfg);
  // bumper = rear axle + 1.25 + 2.25 = rear axle + 3.5
  EXPECT_FALSE(cc.inCollision({8.8, 5.0, 0.0}));   // bumper at 12.30
  EXPECT_TRUE(cc.inCollision({8.91, 5.0, 0.0}));   // bumper at 12.41
  // facing away from the wall, the rear bumper is 1.0 m behind the axle
  EXPECT_FALSE(cc.inCollision({11.3, 5.0, M_PI}));
  EXPECT_TRUE(cc.inCollision({11.41, 5.0, M_PI}));
}
