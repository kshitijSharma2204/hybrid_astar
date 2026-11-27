#include "hybrid_astar/config.hpp"
#include "hybrid_astar/metrics.hpp"
#include "hybrid_astar/planner.hpp"
#include "hybrid_astar/scenario.hpp"

#include <cstdio>
#include <algorithm>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace hybrid_astar;

namespace {

const char* kUsage =
  "usage: hybrid_astar_cli --scenario FILE [--config FILE] [--set KEY=VALUE]...\n"
  "                        [--out FILE] [--trace FILE] [--repeat N]\n"
  "                        [--stats FILE] [--label TEXT] [-v]\n"
  "\n"
  "  --scenario FILE   scenario text file (see scenarios/)\n"
  "  --config FILE     planner parameters, default config/params.yaml\n"
  "  --set KEY=VALUE   override one parameter after the file is read\n"
  "  --out FILE        write the path as CSV: x,y,theta,direction\n"
  "  --trace FILE      record the search and write it as CSV (see below)\n"
  "  --repeat N        plan N times, for timing (default 1)\n"
  "  --stats FILE      append one CSV row of measurements per run\n"
  "  --label TEXT      copied into the label column of --stats\n"
  "  -v                print planner log messages to stderr\n"
  "\n"
  "Trace CSV columns are kind,id,x,y,theta,parent,collided. A 'node' row is\n"
  "one expansion; parent is the id of the node it came from. An 'rs' row is\n"
  "one sample of Reeds-Shepp attempt <id>, started from node <parent>.\n";

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

void writeTrace(const std::string& file, const SearchTrace& t) {
  std::ofstream out(file);
  if (!out) throw std::runtime_error("cannot write " + file);
  out << "kind,id,x,y,theta,parent,collided\n";
  char buf[160];
  for (size_t i = 0; i < t.nodes.size(); ++i) {
    const auto& n = t.nodes[i];
    std::snprintf(buf, sizeof(buf), "node,%zu,%.4f,%.4f,%.4f,%d,\n",
                  i, n.x, n.y, n.theta, n.parent);
    out << buf;
  }
  for (size_t i = 0; i < t.rs_attempts.size(); ++i) {
    const auto& a = t.rs_attempts[i];
    for (const auto& p : a.samples) {
      std::snprintf(buf, sizeof(buf), "rs,%zu,%.4f,%.4f,%.4f,%d,%d\n",
                    i, p.x, p.y, p.theta, a.node, a.collided ? 1 : 0);
      out << buf;
    }
  }
}

const char* kStatsHeader =
  "scenario,label,run,success,expansions,search_ms,smooth_ms,length,"
  "reverse_length,switches,max_curvature_raw,max_curvature,min_clearance,"
  "goal_pos_error,goal_heading_error,cost,setup_ms,smooth_fallbacks\n";

void appendStats(const std::string& file, const std::string& scenario,
                 const std::string& label, int run, const PlanResult& r,
                 const PathMetrics& m, double max_curv_raw, double setup_ms) {
  bool fresh = !std::ifstream(file).good();
  std::ofstream out(file, std::ios::app);
  if (!out) throw std::runtime_error("cannot write " + file);
  if (fresh) out << kStatsHeader;
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    "%s,%s,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%.5f,%.5f,%.3f,%.3g,%.3g,%.3f,%.3f,%d\n",
    scenario.c_str(), label.c_str(), run, r.stats.success ? 1 : 0,
    r.stats.expansions, r.stats.search_ms, r.stats.smooth_ms, m.length,
    m.reverse_length, m.direction_switches, max_curv_raw, m.max_curvature,
    m.min_clearance, m.goal_pos_error, m.goal_heading_error, m.cost, setup_ms,
    r.stats.smooth_fallbacks);
  out << buf;
}

} // namespace

int main(int argc, char** argv) {
  std::string scenario_file, out_file, trace_file, stats_file, label;
  int repeat = 1;
  std::string config_file = "config/params.yaml";
  std::vector<std::string> overrides;
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
    else if (a == "--config") config_file = next();
    else if (a == "--trace") trace_file = next();
    else if (a == "--stats") stats_file = next();
    else if (a == "--label") label = next();
    else if (a == "--repeat") repeat = std::max(1, std::stoi(next()));
    else if (a == "--set") overrides.push_back(next());
    else if (a == "-v") verbose = true;
    else if (a == "-h" || a == "--help") { std::cout << kUsage; return 0; }
    else { std::cerr << "unknown option " << a << "\n" << kUsage; return 2; }
  }
  if (scenario_file.empty()) { std::cerr << kUsage; return 2; }

  try {
    Scenario sc = loadScenario(scenario_file);
    PlannerConfig cfg = loadConfig(config_file);
    for (const auto& o : overrides) {
      auto eq = o.find('=');
      if (eq == std::string::npos) throw std::runtime_error("--set needs KEY=VALUE, got " + o);
      setConfigValue(cfg, o.substr(0, eq), std::stod(o.substr(eq + 1)));
    }
    HybridAStarPlanner planner(cfg);
    if (verbose) planner.setLogger([](const std::string& m) { std::cerr << m << "\n"; });

    // The collision lookups are built once; every run reuses them.
    planner.setMap(sc.grid);
    PlanResult r;
    for (int run = 0; run < repeat; ++run) {
      PlanOptions opts;
      opts.record_trace = !trace_file.empty() && run == 0;
      r = planner.plan(sc.start, sc.goal, opts);
      PathMetrics m = computeMetrics(r.poses, r.direction, sc.grid, cfg, sc.goal);
      std::printf("scenario=%s run=%d success=%d expansions=%d search_ms=%.3f "
                  "smooth_ms=%.3f length=%.2f switches=%d poses=%zu\n",
                  sc.name.c_str(), run, r.stats.success ? 1 : 0, r.stats.expansions,
                  r.stats.search_ms, r.stats.smooth_ms, m.length,
                  m.direction_switches, r.poses.size());
      if (!r.stats.success) std::fprintf(stderr, "%s\n", r.message.c_str());
      if (!stats_file.empty()) {
        double raw_curv = r.raw_poses.empty() ? m.max_curvature
                                              : maxCurvature(r.raw_poses, r.raw_direction);
        appendStats(stats_file, sc.name, label, run, r, m, raw_curv,
                    run == 0 ? planner.setupMs() : 0.0);
      }
      if (run == 0) {
        if (r.stats.success && !out_file.empty()) writePath(out_file, r);
        if (r.trace) writeTrace(trace_file, *r.trace);
      }
    }
    return r.stats.success ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
}
