#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"

#include <chrono>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>

using hybrid_astar::HybridAStarPlanner;

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: hybrid_astar_cli <scenario.txt> [path.csv]\n";
    return 2;
  }

  hybrid_astar::Scenario sc;
  try {
    sc = hybrid_astar::loadScenario(argv[1]);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }

  // Same settings the ROS node used.
  VehicleModel vm;
  const double yaw_res = M_PI / 12.0;
  const int yaw_bins = int(2 * M_PI / yaw_res);
  HybridAStarPlanner planner(vm, yaw_res, yaw_bins);
  planner.setLogger([](const std::string& m) { std::cerr << m << "\n"; });

  auto t0 = std::chrono::steady_clock::now();
  auto path = planner.plan(sc.start, sc.goal, sc.grid);
  auto t1 = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::printf("scenario=%s success=%d expansions=%d time_ms=%.3f poses=%zu\n",
              sc.name.c_str(), path ? 1 : 0, planner.lastExpansions(), ms,
              path ? path->size() : size_t(0));

  if (path && argc == 3) {
    std::ofstream out(argv[2]);
    out << "x,y,theta\n";
    char buf[96];
    for (auto& p : *path) {
      std::snprintf(buf, sizeof(buf), "%.6f,%.6f,%.6f\n", p.x, p.y, p.theta);
      out << buf;
    }
  }
  return path ? 0 : 1;
}
