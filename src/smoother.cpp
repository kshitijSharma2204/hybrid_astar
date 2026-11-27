#include "hybrid_astar/smoother.hpp"

#include "hybrid_astar/geometry.hpp"
#include "hybrid_astar/metrics.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace hybrid_astar {

namespace {

struct Vec {
  double x = 0.0, y = 0.0;
};

// Signed curvature of the circle through three points, positive when the
// path turns left in its direction of travel.
double signedCurvature(const Pose2D& a, const Pose2D& b, const Pose2D& c) {
  double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y;
  double l1 = std::hypot(ux, uy), l2 = std::hypot(vx, vy);
  double l3 = std::hypot(c.x - a.x, c.y - a.y);
  if (l1 * l2 * l3 < 1e-12) return 0.0;
  return 2.0 * (ux * vy - uy * vx) / (l1 * l2 * l3);
}

void descend(std::vector<Vec>& c, int dir, double spacing, const CollisionChecker& cc,
             const PlannerConfig& cfg) {
  const int n = int(c.size());
  const double limit = 0.98 / cfg.minTurningRadius() * spacing * spacing;
  const double dmax = cfg.smooth_obstacle_dmax;
  const double r = cc.circleRadius();
  const double h = cc.grid().resolution;
  const auto& offsets = cc.circleOffsets();
  // The second-difference terms have a Hessian with eigenvalues up to
  // 32 * weight, so a step above 1 / (16 * weights) diverges. smooth_rate
  // is a fraction of that limit.
  const double step = cfg.smooth_rate /
    (16.0 * (cfg.smooth_w_smooth + cfg.smooth_w_curvature) + 4.0 * cfg.smooth_w_length);
  std::vector<Vec> grad(n);

  for (int it = 0; it < cfg.smooth_iterations; ++it) {
    std::fill(grad.begin(), grad.end(), Vec{});
    for (int i = 1; i + 1 < n; ++i) {
      Vec d2{c[i+1].x - 2*c[i].x + c[i-1].x, c[i+1].y - 2*c[i].y + c[i-1].y};

      // smoothness: the gradient of |d2|^2 is 2*d2 times (1, -2, 1)
      double k = 2 * cfg.smooth_w_smooth;
      // curvature: the same stencil, only while |d2| is over the limit
      double len = std::hypot(d2.x, d2.y);
      if (len > limit) k += 2 * cfg.smooth_w_curvature * (len - limit) / len;
      grad[i-1].x += k * d2.x;  grad[i-1].y += k * d2.y;
      grad[i].x -= 2*k * d2.x;  grad[i].y -= 2*k * d2.y;
      grad[i+1].x += k * d2.x;  grad[i+1].y += k * d2.y;

      // spacing: keep |c[i] - c[i-1]| near the original spacing. Without
      // this the smoothness term shortens the path, and an arc that is
      // already at full steer has to turn tighter to fit.
      Vec d1{c[i].x - c[i-1].x, c[i].y - c[i-1].y};
      double l = std::hypot(d1.x, d1.y);
      if (l > 1e-12) {
        double g = 2 * cfg.smooth_w_length * (l - spacing) / l;
        grad[i].x += g * d1.x;    grad[i].y += g * d1.y;
        grad[i-1].x -= g * d1.x;  grad[i-1].y -= g * d1.y;
      }

      // obstacles: push each footprint disc out to d_max. The heading is
      // held fixed for this step, so a disc moves with its point.
      double th = std::atan2(c[i+1].y - c[i-1].y, c[i+1].x - c[i-1].x);
      if (dir < 0) th += M_PI;
      double co = std::cos(th), si = std::sin(th);
      for (double o : offsets) {
        double x = c[i].x + o * co, y = c[i].y + o * si;
        double d = cc.obstacleDistance(x, y) - r;
        if (d >= dmax) continue;
        double gx = (cc.obstacleDistance(x + h, y) - cc.obstacleDistance(x - h, y)) / (2*h);
        double gy = (cc.obstacleDistance(x, y + h) - cc.obstacleDistance(x, y - h)) / (2*h);
        grad[i].x -= 2 * cfg.smooth_w_obstacle * (dmax - d) * gx;
        grad[i].y -= 2 * cfg.smooth_w_obstacle * (dmax - d) * gy;
      }
    }
    for (int i = 2; i + 2 < n; ++i) {
      c[i].x -= step * grad[i].x;
      c[i].y -= step * grad[i].y;
    }
  }
}

// End conditions for a uniform cubic B-spline with knot spacing s. At an
// end knot the curve is at (q + 4 c0 + c1) / 6, has velocity (c1 - q) / 2
// and acceleration q - 2 c0 + c1, where q is a point past the end. Asking
// for position p, velocity s * t and acceleration k s^2 * n (t the unit
// direction of travel, n its left normal, k the signed curvature) gives
// the three points below. Returned as {q, c0, c1}; for the far end pass
// t pointing backwards along the path and -k.
std::array<Vec, 3> endPoints(Vec p, double tx, double ty, double k, double s) {
  double nx = -ty, ny = tx;
  double a = k * s * s;
  return {Vec{p.x - s*tx + a*nx/3, p.y - s*ty + a*ny/3},
          Vec{p.x - a*nx/6, p.y - a*ny/6},
          Vec{p.x + s*tx + a*nx/3, p.y + s*ty + a*ny/3}};
}

// Uniform cubic B-spline over q_start, c[0..n-1], q_end. A B-spline does
// not pass through its inner control points, but it does not overshoot
// either: its curvature stays close to |c[i+1] - 2 c[i] + c[i-1]| / s^2,
// which is the quantity the curvature term limits.
std::vector<Pose2D> rebuild(const std::vector<Vec>& c, Vec q_start, Vec q_end,
                            const Pose2D& first, const Pose2D& last, int dir,
                            double step) {
  const int n = int(c.size());
  std::vector<Vec> q(n + 2);
  q[0] = q_start;
  for (int i = 0; i < n; ++i) q[i+1] = c[i];
  q[n+1] = q_end;

  std::vector<Pose2D> out;
  out.push_back(first);
  for (int i = 1; i + 2 < n + 2; ++i) {
    // piece between knots i and i+1 uses q[i-1..i+2]
    const Vec &p0 = q[i-1], &p1 = q[i], &p2 = q[i+1], &p3 = q[i+2];
    double chord = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int k = std::max(1, int(std::ceil(chord / (0.9 * step))));
    for (int j = 1; j <= k; ++j) {
      double t = double(j) / k, t2 = t*t, t3 = t2*t;
      double b0 = (1 - t) * (1 - t) * (1 - t) / 6;
      double b1 = (3*t3 - 6*t2 + 4) / 6;
      double b2 = (-3*t3 + 3*t2 + 3*t + 1) / 6;
      double b3 = t3 / 6;
      double d0 = -(1 - t) * (1 - t) / 2, d1 = (3*t2 - 4*t) / 2;
      double d2 = (-3*t2 + 2*t + 1) / 2, d3 = t2 / 2;
      double x = b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x;
      double y = b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y;
      double dx = d0*p0.x + d1*p1.x + d2*p2.x + d3*p3.x;
      double dy = d0*p0.y + d1*p1.y + d2*p2.y + d3*p3.y;
      double th = std::atan2(dy, dx) + (dir < 0 ? M_PI : 0.0);
      out.push_back({x, y, wrapAngle(th)});
    }
  }
  out.back() = last;
  return out;
}

} // namespace

SmoothResult smoothPath(const std::vector<Pose2D>& poses,
                        const std::vector<int>& direction,
                        const CollisionChecker& cc,
                        const PlannerConfig& cfg) {
  SmoothResult out;
  const int n = int(poses.size());
  if (n < 3) {
    out.poses = poses;
    out.direction = direction;
    return out;
  }
  const double kmax = 1.0 / cfg.minTurningRadius();
  out.poses.push_back(poses[0]);
  out.direction.push_back(direction[0]);

  // direction[i] is the motion into pose i, so pose i is a cusp when the
  // motion out of it (direction[i+1]) differs.
  int a = 0;
  for (int i = 1; i < n; ++i) {
    if (i + 1 < n && direction[i+1] == direction[i]) continue;
    const int b = i;
    const int dir = direction[a + 1];
    ++out.segments;

    // control points about smooth_spacing apart, always including both ends
    std::vector<double> s(b - a + 1, 0.0);
    for (int k = a + 1; k <= b; ++k) {
      s[k - a] = s[k - a - 1] + std::hypot(poses[k].x - poses[k-1].x, poses[k].y - poses[k-1].y);
    }
    const double total = s.back();
    const int pieces = int(std::round(total / cfg.smooth_spacing));
    std::vector<Pose2D> seg(poses.begin() + a, poses.begin() + b + 1);

    if (pieces >= 4) {
      const double spacing = total / pieces;
      std::vector<Vec> ctrl;
      size_t k = 0;
      for (int j = 0; j <= pieces; ++j) {
        double target = j * spacing;
        while (k + 1 < s.size() && s[k + 1] < target) ++k;
        size_t k1 = std::min(k + 1, s.size() - 1);
        double u = s[k1] > s[k] ? (target - s[k]) / (s[k1] - s[k]) : 0.0;
        u = std::clamp(u, 0.0, 1.0);
        ctrl.push_back({seg[k].x + u * (seg[k1].x - seg[k].x),
                        seg[k].y + u * (seg[k1].y - seg[k].y)});
      }
      // The two control points at each end are set so the spline starts and
      // ends at the original poses with the original curvature, and they
      // stay fixed while the rest move.
      const Pose2D& f = seg.front();
      const Pose2D& l = seg.back();
      const int m = int(seg.size());
      double k0 = signedCurvature(seg[0], seg[1], seg[2]);
      double k1 = signedCurvature(seg[m-3], seg[m-2], seg[m-1]);
      auto e0 = endPoints({f.x, f.y}, dir * std::cos(f.theta), dir * std::sin(f.theta), k0, spacing);
      auto e1 = endPoints({l.x, l.y}, -dir * std::cos(l.theta), -dir * std::sin(l.theta), -k1, spacing);
      ctrl[0] = e0[1];
      ctrl[1] = e0[2];
      ctrl[pieces] = e1[1];
      ctrl[pieces - 1] = e1[2];

      descend(ctrl, dir, spacing, cc, cfg);
      std::vector<Pose2D> smooth = rebuild(ctrl, e0[0], e1[0], seg.front(), seg.back(),
                                           dir, cfg.collision_step);
      std::vector<int> sdir(smooth.size(), dir);

      bool collides = std::any_of(smooth.begin(), smooth.end(),
                                  [&](const Pose2D& q) { return cc.exactCollision(q); });
      if (collides) {
        ++out.collision_fallbacks;
      } else if (maxCurvature(smooth, sdir) > cfg.smooth_curvature_limit * kmax) {
        ++out.curvature_fallbacks;
      } else {
        seg = std::move(smooth);
      }
    }
    out.poses.insert(out.poses.end(), seg.begin() + 1, seg.end());
    out.direction.insert(out.direction.end(), seg.size() - 1, dir);
    a = b;
  }
  return out;
}

} // namespace hybrid_astar
