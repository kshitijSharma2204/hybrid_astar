#pragma once

#include "hybrid_astar/types.hpp"

#include <string>
#include <vector>

namespace hybrid_astar {

// params.yaml is a flat "key: value" file with numbers and # comments only.
// Keys missing from the file keep their PlannerConfig defaults. Unknown keys,
// repeated keys and bad values throw std::runtime_error.
PlannerConfig loadConfig(const std::string& path);
PlannerConfig parseConfig(const std::string& text, const std::string& source = "config");

// Sets one field by name, with the same checks as the file parser.
void setConfigValue(PlannerConfig& cfg, const std::string& key, double value);
std::vector<std::string> configKeys();

} // namespace hybrid_astar
