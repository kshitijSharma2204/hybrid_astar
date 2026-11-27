#pragma once

#include "hybrid_astar/utils.hpp"

#include <string>

namespace hybrid_astar {

// A scenario is a PGM image (black = obstacle) plus a small text file:
//
//   map: S1.pgm            # relative to the text file
//   resolution: 0.1        # meters per pixel
//   origin: 0.0 0.0        # world position of the bottom-left corner
//   start: 35.0 55.0 0     # x y heading, heading in degrees
//   goal: 50.0 17.0 180
struct Scenario {
  std::string name;
  GridMap grid;
  Pose2D start;
  Pose2D goal;
};

// Both throw std::runtime_error with the file name on any problem.
GridMap loadPgm(const std::string& path, double resolution,
                double origin_x, double origin_y);
Scenario loadScenario(const std::string& path);

} // namespace hybrid_astar
