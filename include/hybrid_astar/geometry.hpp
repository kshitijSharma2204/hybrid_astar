#pragma once

#include "hybrid_astar/types.hpp"

#include <cmath>

namespace hybrid_astar {

// Wraps to (-pi, pi].
inline double wrapAngle(double a) {
  a = std::fmod(a, 2.0 * M_PI);
  if (a <= -M_PI) a += 2.0 * M_PI;
  else if (a > M_PI) a -= 2.0 * M_PI;
  return a;
}

// Smallest signed difference a - b, in (-pi, pi].
inline double angleDiff(double a, double b) { return wrapAngle(a - b); }

// The one world -> cell conversion. Cell (ix, iy) covers
// [origin + ix*res, origin + (ix+1)*res), so this is a floor.
inline int worldToCellX(const GridMap& g, double x) {
  return int(std::floor((x - g.origin_x) / g.resolution));
}
inline int worldToCellY(const GridMap& g, double y) {
  return int(std::floor((y - g.origin_y) / g.resolution));
}
inline double cellCenterX(const GridMap& g, int ix) {
  return g.origin_x + (ix + 0.5) * g.resolution;
}
inline double cellCenterY(const GridMap& g, int iy) {
  return g.origin_y + (iy + 0.5) * g.resolution;
}
inline bool inBounds(const GridMap& g, int ix, int iy) {
  return ix >= 0 && iy >= 0 && ix < g.width && iy < g.height;
}
inline bool isOccupied(const GridMap& g, int ix, int iy) {
  return g.data[size_t(iy) * g.width + ix] >= 100;
}

} // namespace hybrid_astar
