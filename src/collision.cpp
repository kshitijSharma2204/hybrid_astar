#include "hybrid_astar/collision.hpp"

#include "hybrid_astar/geometry.hpp"

#include <nanoflann.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace hybrid_astar {

struct CollisionChecker::KdIndex {
  struct Cloud {
    std::vector<std::array<double,2>> points;
    size_t kdtree_get_point_count() const { return points.size(); }
    double kdtree_get_pt(size_t idx, size_t dim) const { return points[idx][dim]; }
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
  };
  using Tree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, Cloud>, Cloud, 2, size_t>;

  Cloud cloud;
  std::unique_ptr<Tree> tree;
};

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

CollisionChecker::CollisionChecker(const GridMap& grid, const PlannerConfig& cfg,
                                   Lookup lookup)
  : grid_{grid},
    length_{cfg.vehicle_length},
    width_{cfg.vehicle_width},
    offset_{cfg.rear_axle_to_center},
    mode_{cfg.collision_mode},
    lookup_{lookup}
{
  // n equal slices of the rectangle, one disc around each slice.
  const int n = cfg.collision_circles;
  const double half_slice = length_ / (2.0 * n);
  radius_ = std::hypot(half_slice, width_ / 2);
  for (int i = 0; i < n; ++i) {
    circle_offsets_.push_back(offset_ - length_ / 2 + half_slice * (2 * i + 1));
  }
  threshold_ = radius_ + grid_.resolution * std::sqrt(2.0) / 2;

  if (mode_ == 0) return;
  if (lookup_ == Lookup::DistanceMap) {
    dist_ = distanceTransform(grid_);
  } else {
    kd_ = std::make_unique<KdIndex>();
    for (int iy = 0; iy < grid_.height; ++iy)
      for (int ix = 0; ix < grid_.width; ++ix)
        if (isOccupied(grid_, ix, iy))
          kd_->cloud.points.push_back({cellCenterX(grid_, ix), cellCenterY(grid_, iy)});
    kd_->tree = std::make_unique<KdIndex::Tree>(
      2, kd_->cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kd_->tree->buildIndex();
  }
}

CollisionChecker::~CollisionChecker() = default;

bool CollisionChecker::inCollision(const Pose2D& p) const {
  if (mode_ == 0) return exactCollision(p);
  if (mode_ == 1) return circleCollision(p);
  return circleCollision(p) && exactCollision(p);
}

bool CollisionChecker::isPointInObstacle(double x, double y) const {
  int ix = worldToCellX(grid_, x);
  int iy = worldToCellY(grid_, y);
  if (!inBounds(grid_, ix, iy)) return true;
  return isOccupied(grid_, ix, iy);
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

bool CollisionChecker::outlineCollision(const Pose2D& p) const {
  double L = length_, W = width_;
  double c = std::cos(p.theta), s_ = std::sin(p.theta);
  const Pose2D s{p.x + offset_*c, p.y + offset_*s_, p.theta};
  const std::pair<double,double> corners[4] = {
    { L/2,  W/2}, { L/2, -W/2},
    {-L/2, -W/2}, {-L/2,  W/2}
  };
  for (auto &q : corners) {
    if (isPointInObstacle(s.x + q.first*c - q.second*s_,
                          s.y + q.first*s_ + q.second*c)) return true;
  }
  for (double v=-W/2; v<=W/2; v+=0.2) {
    if (isPointInObstacle(
          s.x + (L/2)*c - v*s_,
          s.y + (L/2)*s_ + v*c)) return true;
    if (isPointInObstacle(
          s.x +(-L/2)*c - v*s_,
          s.y +(-L/2)*s_ + v*c)) return true;
  }
  for (double u=-L/2; u<=L/2; u+=0.2) {
    if (isPointInObstacle(
          s.x + u*c - (W/2)*s_,
          s.y + u*s_ + (W/2)*c)) return true;
    if (isPointInObstacle(
          s.x + u*c -(-W/2)*s_,
          s.y + u*s_ +(-W/2)*c)) return true;
  }
  return false;
}

// Distance from a point to the nearest occupied cell centre. The distance
// map only knows cell centres, so it subtracts the half-diagonal to stay a
// lower bound.
double CollisionChecker::nearestObstacle(double x, double y) const {
  if (lookup_ == Lookup::DistanceMap) {
    int ix = worldToCellX(grid_, x), iy = worldToCellY(grid_, y);
    return dist_[size_t(iy) * grid_.width + ix] - grid_.resolution * std::sqrt(2.0) / 2;
  }
  if (kd_->cloud.points.empty()) return std::numeric_limits<double>::infinity();
  const double q[2] = {x, y};
  size_t idx;
  double d2;
  kd_->tree->knnSearch(q, 1, &idx, &d2);
  return std::sqrt(d2);
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
    if (nearestObstacle(x, y) <= threshold_) return true;
  }
  return false;
}

} // namespace hybrid_astar
