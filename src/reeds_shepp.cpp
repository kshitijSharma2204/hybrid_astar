#include "hybrid_astar/reeds_shepp.hpp"

#include "hybrid_astar/geometry.hpp"

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <cmath>

namespace hybrid_astar {

namespace ob = ompl::base;
using RsSpace = ob::ReedsSheppStateSpace;

struct ReedsShepp::Impl {
  double radius;
  RsSpace space;
  ob::State* from;
  ob::State* to;
  ob::State* tmp;

  explicit Impl(double r)
    : radius{r}, space{r},
      from{space.allocState()}, to{space.allocState()}, tmp{space.allocState()} {}
  ~Impl() {
    space.freeState(from);
    space.freeState(to);
    space.freeState(tmp);
  }
  void set(const Pose2D& a, const Pose2D& b) const {
    auto* s = from->as<RsSpace::StateType>();
    s->setXY(a.x, a.y);
    s->setYaw(a.theta);
    auto* t = to->as<RsSpace::StateType>();
    t->setXY(b.x, b.y);
    t->setYaw(b.theta);
  }
};

ReedsShepp::ReedsShepp(double radius) : impl_{std::make_unique<Impl>(radius)} {}
ReedsShepp::~ReedsShepp() = default;

double ReedsShepp::length(const Pose2D& a, const Pose2D& b) const {
  impl_->set(a, b);
  return impl_->space.distance(impl_->from, impl_->to);
}

void ReedsShepp::sample(const Pose2D& a, const Pose2D& b, double step,
                        std::vector<Pose2D>& poses,
                        std::vector<int>& direction) const {
  poses.clear();
  direction.clear();
  impl_->set(a, b);
  RsSpace::ReedsSheppPath path = impl_->space.reedsShepp(impl_->from, impl_->to);
  // Segment lengths are in units of the radius; negative means reverse.
  const double total = path.length();
  if (total <= 0.0) return;

  double done = 0.0;
  for (double seg : path.length_) {
    double len = std::fabs(seg);
    if (len < 1e-12) continue;
    int n = std::max(1, int(std::ceil(len * impl_->radius / step)));
    int dir = seg < 0 ? -1 : 1;
    for (int j = 1; j <= n; ++j) {
      double t = (done + len * j / n) / total;
      bool first = false;
      impl_->space.interpolate(impl_->from, impl_->to, std::min(t, 1.0), first, path,
                               impl_->tmp);
      const auto* s = impl_->tmp->as<RsSpace::StateType>();
      poses.push_back({s->getX(), s->getY(), wrapAngle(s->getYaw())});
      direction.push_back(dir);
    }
    done += len;
  }
}

} // namespace hybrid_astar
