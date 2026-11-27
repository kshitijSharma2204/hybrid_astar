#pragma once

#include "hybrid_astar/types.hpp"

#include <memory>
#include <vector>

namespace hybrid_astar {

// Shortest Reeds-Shepp curves for a car with the given turning radius.
// This is the only place OMPL is used.
class ReedsShepp {
public:
  explicit ReedsShepp(double radius);
  ~ReedsShepp();
  ReedsShepp(const ReedsShepp&) = delete;
  ReedsShepp& operator=(const ReedsShepp&) = delete;

  // Length in meters of the shortest curve from a to b, ignoring obstacles.
  double length(const Pose2D& a, const Pose2D& b) const;

  // Samples the shortest curve at most step meters apart, not including a
  // itself. Each segment's end is a sample, so cusps are always samples.
  // direction[i] is +1/-1 for the motion that arrives at poses[i].
  void sample(const Pose2D& a, const Pose2D& b, double step,
              std::vector<Pose2D>& poses, std::vector<int>& direction) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace hybrid_astar
