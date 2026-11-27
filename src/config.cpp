#include "hybrid_astar/config.hpp"

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace hybrid_astar {

namespace {

struct Field {
  const char* name;
  double PlannerConfig::*d;   // set for real-valued fields
  int PlannerConfig::*i;      // set for integer fields
  double min;                 // smallest allowed value
};

const std::vector<Field>& fields() {
  static const std::vector<Field> f = {
    {"wheel_base",            &PlannerConfig::wheel_base,            nullptr, 1e-3},
    {"max_steer",             &PlannerConfig::max_steer,             nullptr, 1e-3},
    {"vehicle_length",        &PlannerConfig::vehicle_length,        nullptr, 1e-3},
    {"vehicle_width",         &PlannerConfig::vehicle_width,         nullptr, 1e-3},
    {"rear_axle_to_center",   &PlannerConfig::rear_axle_to_center,   nullptr, 0},
    {"step_size",             &PlannerConfig::step_size,             nullptr, 1e-3},
    {"collision_step",        &PlannerConfig::collision_step,        nullptr, 1e-3},
    {"steer_samples",         nullptr, &PlannerConfig::steer_samples,         2},
    {"xy_resolution",         &PlannerConfig::xy_resolution,         nullptr, 1e-3},
    {"heading_bins",          nullptr, &PlannerConfig::heading_bins,          4},
    {"max_expansions",        nullptr, &PlannerConfig::max_expansions,        1},
    {"goal_pos_tol",          &PlannerConfig::goal_pos_tol,          nullptr, 0},
    {"goal_heading_tol",      &PlannerConfig::goal_heading_tol,      nullptr, 0},
    {"analytic_expansion_interval", nullptr, &PlannerConfig::analytic_expansion_interval, 0},
    {"analytic_expansion_distance", &PlannerConfig::analytic_expansion_distance, nullptr, 0},
    {"rs_sample_step",        &PlannerConfig::rs_sample_step,        nullptr, 1e-3},
    {"reverse_penalty",       &PlannerConfig::reverse_penalty,       nullptr, 1},
    {"direction_change_cost", &PlannerConfig::direction_change_cost, nullptr, 0},
    {"steer_cost",            &PlannerConfig::steer_cost,            nullptr, 0},
    {"steer_change_cost",     &PlannerConfig::steer_change_cost,     nullptr, 0},
    {"heuristic_weight",      &PlannerConfig::heuristic_weight,      nullptr, 1},
  };
  return f;
}

std::string trim(const std::string& s) {
  auto b = s.find_first_not_of(" \t\r");
  if (b == std::string::npos) return {};
  return s.substr(b, s.find_last_not_of(" \t\r") - b + 1);
}

} // namespace

void setConfigValue(PlannerConfig& cfg, const std::string& key, double value) {
  for (const auto& f : fields()) {
    if (key != f.name) continue;
    if (!std::isfinite(value) || value < f.min) {
      std::ostringstream msg;
      msg << key << " must be a number >= " << f.min;
      throw std::runtime_error(msg.str());
    }
    if (f.i) {
      if (value != std::floor(value)) throw std::runtime_error(key + " must be an integer");
      cfg.*f.i = int(value);
    } else {
      cfg.*f.d = value;
    }
    return;
  }
  throw std::runtime_error("unknown config key '" + key + "'");
}

double getConfigValue(const PlannerConfig& cfg, const std::string& key) {
  for (const auto& f : fields()) {
    if (key == f.name) return f.i ? double(cfg.*f.i) : cfg.*f.d;
  }
  throw std::runtime_error("unknown config key '" + key + "'");
}

std::vector<std::string> configKeys() {
  std::vector<std::string> keys;
  for (const auto& f : fields()) keys.push_back(f.name);
  return keys;
}

PlannerConfig parseConfig(const std::string& text, const std::string& source) {
  PlannerConfig cfg;
  std::set<std::string> seen;
  std::istringstream in(text);
  std::string line;
  for (int lineno = 1; std::getline(in, line); ++lineno) {
    std::string where = source + ":" + std::to_string(lineno) + ": ";
    line = trim(line.substr(0, line.find('#')));
    if (line.empty()) continue;
    auto colon = line.find(':');
    if (colon == std::string::npos) throw std::runtime_error(where + "expected 'key: value'");
    std::string key = trim(line.substr(0, colon));
    std::string val = trim(line.substr(colon + 1));
    if (!seen.insert(key).second) throw std::runtime_error(where + "'" + key + "' set twice");

    size_t used = 0;
    double v = 0.0;
    try {
      v = std::stod(val, &used);
    } catch (const std::exception&) {
      used = 0;
    }
    if (val.empty() || used != val.size()) {
      throw std::runtime_error(where + "'" + val + "' is not a number");
    }
    try {
      setConfigValue(cfg, key, v);
    } catch (const std::runtime_error& e) {
      throw std::runtime_error(where + e.what());
    }
  }
  return cfg;
}

PlannerConfig loadConfig(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open " + path);
  std::stringstream ss;
  ss << in.rdbuf();
  return parseConfig(ss.str(), path);
}

} // namespace hybrid_astar
