// Compares the collision checks on random poses over the S5 maps.
//
//   collision_bench [poses_per_map] [scenarios/S5]
//
// For each check: how often it says free where the exact check says
// collision (must be 0 for the circles), how often it says collision where
// the exact check says free (over-cautious), and the time per pose.

#include "hybrid_astar/collision.hpp"
#include "hybrid_astar/scenario.hpp"

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace hybrid_astar;
using Clock = std::chrono::steady_clock;

namespace {

double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct Tally {
  std::string name;
  long missed = 0;        // free here, collision in the exact check
  long cautious = 0;      // collision here, free in the exact check
  double ms = 0.0;
  double build_ms = 0.0;
};

} // namespace

int main(int argc, char** argv) {
  const int per_map = argc > 1 ? std::stoi(argv[1]) : 2000;
  const std::string dir = argc > 2 ? argv[2] : "scenarios/S5";

  std::vector<Tally> tallies = {
    {"outline (v1)"}, {"circles x3, k-d tree"}, {"circles x3, distance map"},
    {"circles x4, k-d tree"}, {"circles x4, distance map"},
    {"circles x4 + exact (mode 2)"}};
  double exact_ms = 0.0;
  long total = 0, exact_hits = 0;
  std::mt19937 rng(1);

  for (int seed = 0; seed < 50; ++seed) {
    char name[64];
    std::snprintf(name, sizeof(name), "%s/seed_%02d.txt", dir.c_str(), seed);
    Scenario sc = loadScenario(name);
    const GridMap& g = sc.grid;

    std::uniform_real_distribution<double> px(g.origin_x, g.origin_x + g.width * g.resolution);
    std::uniform_real_distribution<double> py(g.origin_y, g.origin_y + g.height * g.resolution);
    std::uniform_real_distribution<double> pa(-M_PI, M_PI);
    std::vector<Pose2D> poses(per_map);
    for (auto& p : poses) p = {px(rng), py(rng), pa(rng)};

    PlannerConfig c3, c4;
    c3.collision_circles = 3;
    c4.collision_circles = 4;
    c4.collision_mode = 2;

    auto t0 = Clock::now();
    CollisionChecker exact(g, PlannerConfig{});
    std::vector<char> ref(poses.size());
    t0 = Clock::now();
    for (size_t i = 0; i < poses.size(); ++i) ref[i] = exact.exactCollision(poses[i]);
    exact_ms += msSince(t0);
    for (char r : ref) exact_hits += r;
    total += long(poses.size());

    auto build = [&](const PlannerConfig& cfg, CollisionChecker::Lookup lk, Tally& t) {
      auto tb = Clock::now();
      auto cc = std::make_unique<CollisionChecker>(g, cfg, lk);
      t.build_ms += msSince(tb);
      return cc;
    };
    auto run = [&](Tally& t, const std::function<bool(const Pose2D&)>& check) {
      std::vector<char> out(poses.size());
      auto tr = Clock::now();
      for (size_t i = 0; i < poses.size(); ++i) out[i] = check(poses[i]);
      t.ms += msSince(tr);
      for (size_t i = 0; i < poses.size(); ++i) {
        if (!out[i] && ref[i]) ++t.missed;
        if (out[i] && !ref[i]) ++t.cautious;
      }
    };

    run(tallies[0], [&](const Pose2D& p) { return exact.outlineCollision(p); });
    auto k3 = build(c3, CollisionChecker::Lookup::KdTree, tallies[1]);
    run(tallies[1], [&](const Pose2D& p) { return k3->circleCollision(p); });
    auto d3 = build(c3, CollisionChecker::Lookup::DistanceMap, tallies[2]);
    run(tallies[2], [&](const Pose2D& p) { return d3->circleCollision(p); });
    auto k4 = build(c4, CollisionChecker::Lookup::KdTree, tallies[3]);
    run(tallies[3], [&](const Pose2D& p) { return k4->circleCollision(p); });
    auto d4 = build(c4, CollisionChecker::Lookup::DistanceMap, tallies[4]);
    run(tallies[4], [&](const Pose2D& p) { return d4->circleCollision(p); });
    tallies[5].build_ms = tallies[4].build_ms;
    run(tallies[5], [&](const Pose2D& p) { return d4->inCollision(p); });
  }

  long exact_free = total - exact_hits;
  std::printf("%ld poses on 50 maps, %ld (%.1f%%) collide in the exact check\n\n",
              total, exact_hits, 100.0 * exact_hits / total);
  std::printf("| check | misses a collision | over-cautious (%% of free poses) "
              "| ns per pose | build ms per map |\n|---|---|---|---|---|\n");
  std::printf("| exact (every touched cell) | - | - | %.0f | 0 |\n", 1e6 * exact_ms / total);
  for (const auto& t : tallies) {
    std::printf("| %s | %ld | %ld (%.2f%%) | %.0f | %.2f |\n", t.name.c_str(), t.missed,
                t.cautious, 100.0 * t.cautious / exact_free, 1e6 * t.ms / total,
                t.build_ms / 50);
  }
  return 0;
}
