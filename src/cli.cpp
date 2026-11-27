#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"

#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

using namespace hybrid_astar;

namespace {

const char* kUsage =
  "usage: hybrid_astar_cli --scenario FILE [--out FILE] [-v]\n"
  "\n"
  "  --scenario FILE   scenario text file (see scenarios/)\n"
  "  --out FILE        write the path as CSV: x,y,theta,direction\n"
  "  -v                print planner log messages to stderr\n";

void writePath(const std::string& file, const PlanResult& r) {
  std::ofstream out(file);
  if (!out) throw std::runtime_error("cannot write " + file);
  out << "x,y,theta,direction\n";
  char buf[128];
  for (size_t i = 0; i < r.poses.size(); ++i) {
    const auto& p = r.poses[i];
    std::snprintf(buf, sizeof(buf), "%.6f,%.6f,%.6f,%d\n",
                  p.x, p.y, p.theta, r.direction[i]);
    out << buf;
  }
}

} // namespace

int main(int argc, char** argv) {
  std::string scenario_file, out_file;
  bool verbose = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << a << " needs a value\n" << kUsage;
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--scenario") scenario_file = next();
    else if (a == "--out") out_file = next();
    else if (a == "-v") verbose = true;
    else if (a == "-h" || a == "--help") { std::cout << kUsage; return 0; }
    else { std::cerr << "unknown option " << a << "\n" << kUsage; return 2; }
  }
  if (scenario_file.empty()) { std::cerr << kUsage; return 2; }

  try {
    Scenario sc = loadScenario(scenario_file);
    PlannerConfig cfg;
    HybridAStarPlanner planner(cfg);
    if (verbose) planner.setLogger([](const std::string& m) { std::cerr << m << "\n"; });

    PlanResult r = planner.plan(sc.grid, sc.start, sc.goal);
    std::printf("scenario=%s success=%d expansions=%d search_ms=%.3f poses=%zu\n",
                sc.name.c_str(), r.stats.success ? 1 : 0, r.stats.expansions,
                r.stats.search_ms, r.poses.size());
    if (!r.stats.success) std::fprintf(stderr, "%s\n", r.message.c_str());
    if (r.stats.success && !out_file.empty()) writePath(out_file, r);
    return r.stats.success ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
}
