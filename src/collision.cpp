#include "hybrid_astar/collision.hpp"

#include "hybrid_astar/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace hybrid_astar {

std::vector<float> distanceTransform(const GridMap& g) {
  // Felzenszwalb & Huttenlocher: 1D squared distance along each column,
  // then along each row using the lower envelope of parabolas.
  const int W = g.width, H = g.height;
  const double inf = 1e20;
  std::vector<double> f(size_t(W) * H);
  for (size_t i = 0; i < f.size(); ++i) f[i] = g.data[i] >= 100 ? 0.0 : inf;

  auto pass1d = [inf](const double* in, double* out, int n, std::vector<int>& v,
                      std::vector<double>& z) {
    int k = 0;
    v[0] = 0;
    z[0] = -inf;
    z[1] = inf;
    for (int q = 1; q < n; ++q) {
      double s;
      while (true) {
        int p = v[k];
        s = ((in[q] + double(q) * q) - (in[p] + double(p) * p)) / (2.0 * (q - p));
        if (s <= z[k] && k > 0) { --k; continue; }
        if (s <= z[k]) { k = -1; }
        break;
      }
      ++k;
      v[k] = q;
      z[k] = s;
      z[k+1] = inf;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
      while (z[k+1] < q) ++k;
      out[q] = double(q - v[k]) * (q - v[k]) + in[v[k]];
    }
  };

  const int n = std::max(W, H);
  std::vector<int> v(n);
  std::vector<double> z(n + 1), in(n), out(n);
  for (int x = 0; x < W; ++x) {
    for (int y = 0; y < H; ++y) in[y] = f[size_t(y) * W + x];
    pass1d(in.data(), out.data(), H, v, z);
    for (int y = 0; y < H; ++y) f[size_t(y) * W + x] = out[y];
  }
  std::vector<float> d(f.size());
  for (int y = 0; y < H; ++y) {
    pass1d(&f[size_t(y) * W], out.data(), W, v, z);
    for (int x = 0; x < W; ++x) {
      d[size_t(y) * W + x] = out[x] >= inf / 2
        ? std::numeric_limits<float>::infinity()
        : float(std::sqrt(out[x]) * g.resolution);
    }
  }
  return d;
}

CollisionChecker::CollisionChecker(const GridMap& grid, const PlannerConfig& cfg)
  : grid_{grid},
    length_{cfg.vehicle_length},
    width_{cfg.vehicle_width},
    offset_{cfg.rear_axle_to_center},
    mode_{cfg.collision_mode}
{
  // n equal slices of the rectangle, one disc around each slice.
  const int n = cfg.collision_circles;
  const double half_slice = length_ / (2.0 * n);
  radius_ = std::hypot(half_slice, width_ / 2);
  for (int i = 0; i < n; ++i) {
    circle_offsets_.push_back(offset_ - length_ / 2 + half_slice * (2 * i + 1));
  }
  // The map stores distances between cell centres. A disc centre can be up
  // to half a diagonal from its cell's centre, and an obstacle cell reaches
  // half a diagonal past its own centre, hence one full diagonal.
  threshold_ = radius_ + grid_.resolution * std::sqrt(2.0);
  dist_ = distanceTransform(grid_);
}

bool CollisionChecker::inCollision(const Pose2D& p) const {
  if (mode_ == 0) return exactCollision(p);
  if (mode_ == 1) return circleCollision(p);
  return circleCollision(p) && exactCollision(p);
}

// Every cell the rectangle touches, row by row: for each grid row, the
// rectangle's x extent inside that row's y band gives a run of cells.
bool CollisionChecker::exactCollision(const Pose2D& p) const {
  const double c = std::cos(p.theta), s = std::sin(p.theta);
  const double cx = p.x + offset_ * c, cy = p.y + offset_ * s;
  const double hl = length_ / 2, hw = width_ / 2;
  const double vx[4] = {cx + hl*c - hw*s, cx + hl*c + hw*s, cx - hl*c + hw*s, cx - hl*c - hw*s};
  const double vy[4] = {cy + hl*s + hw*c, cy + hl*s - hw*c, cy - hl*s - hw*c, cy - hl*s + hw*c};
  double ymin = std::min({vy[0], vy[1], vy[2], vy[3]});
  double ymax = std::max({vy[0], vy[1], vy[2], vy[3]});
  int iy0 = worldToCellY(grid_, ymin), iy1 = worldToCellY(grid_, ymax);
  if (iy0 < 0 || iy1 >= grid_.height) return true;

  for (int iy = iy0; iy <= iy1; ++iy) {
    double b0 = std::max(ymin, grid_.origin_y + iy * grid_.resolution);
    double b1 = std::min(ymax, grid_.origin_y + (iy + 1) * grid_.resolution);
    double xa = std::numeric_limits<double>::infinity(), xb = -xa;
    for (int k = 0; k < 4; ++k) {
      if (vy[k] >= b0 && vy[k] <= b1) {
        xa = std::min(xa, vx[k]);
        xb = std::max(xb, vx[k]);
      }
      int j = (k + 1) % 4;
      if (vy[k] == vy[j]) continue;
      for (double yy : {b0, b1}) {
        if ((vy[k] - yy) * (vy[j] - yy) <= 0.0) {
          double x = vx[k] + (yy - vy[k]) * (vx[j] - vx[k]) / (vy[j] - vy[k]);
          xa = std::min(xa, x);
          xb = std::max(xb, x);
        }
      }
    }
    if (xa > xb) continue;
    int ix0 = worldToCellX(grid_, xa), ix1 = worldToCellX(grid_, xb);
    if (ix0 < 0 || ix1 >= grid_.width) return true;
    const int8_t* row = &grid_.data[size_t(iy) * grid_.width];
    for (int ix = ix0; ix <= ix1; ++ix) {
      if (row[ix] >= 100) return true;
    }
  }
  return false;
}

double CollisionChecker::obstacleDistance(double x, double y) const {
  // bilinear between the four nearest cell centres, clamped to the map
  double fx = (x - grid_.origin_x) / grid_.resolution - 0.5;
  double fy = (y - grid_.origin_y) / grid_.resolution - 0.5;
  fx = std::clamp(fx, 0.0, grid_.width - 1.0);
  fy = std::clamp(fy, 0.0, grid_.height - 1.0);
  int x0 = std::min(int(fx), grid_.width - 2), y0 = std::min(int(fy), grid_.height - 2);
  double tx = fx - x0, ty = fy - y0;
  // capped so an empty map gives a large number instead of 0 * inf
  auto at = [&](int ix, int iy) {
    return std::min(double(dist_[size_t(iy) * grid_.width + ix]), 1e6);
  };
  return (1 - ty) * ((1 - tx) * at(x0, y0) + tx * at(x0 + 1, y0)) +
         ty * ((1 - tx) * at(x0, y0 + 1) + tx * at(x0 + 1, y0 + 1));
}

bool CollisionChecker::circleCollision(const Pose2D& p) const {
  const double c = std::cos(p.theta), s = std::sin(p.theta);
  const double xmin = grid_.origin_x, xmax = grid_.origin_x + grid_.width * grid_.resolution;
  const double ymin = grid_.origin_y, ymax = grid_.origin_y + grid_.height * grid_.resolution;
  for (double d : circle_offsets_) {
    double x = p.x + d * c, y = p.y + d * s;
    // a disc that leaves the map counts as a collision, like the exact check
    if (x - radius_ < xmin || x + radius_ > xmax ||
        y - radius_ < ymin || y + radius_ > ymax) return true;
    int ix = worldToCellX(grid_, x), iy = worldToCellY(grid_, y);
    if (dist_[size_t(iy) * grid_.width + ix] <= threshold_) return true;
  }
  return false;
}

} // namespace hybrid_astar
