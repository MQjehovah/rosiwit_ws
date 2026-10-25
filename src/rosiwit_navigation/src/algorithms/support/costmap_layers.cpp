#include "rosiwit_navigation/nav_core/costmap_layers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <queue>
#include <utility>

namespace rosiwit_navigation
{
namespace core
{

namespace
{
inline bool isLethal(int8_t v) { return v >= kCostLethal; }

inline void raise(std::vector<int8_t> & grid, size_t idx, int8_t value)
{
  if (idx < grid.size() && grid[idx] < value) { grid[idx] = value; }
}
}  // namespace

// ---------------- StaticLayer ----------------
void StaticLayer::setOccupancy(const std::vector<int8_t> & data,
                               unsigned int width, unsigned int height)
{
  data_ = data;
  src_width_ = width;
  src_height_ = height;
}

void StaticLayer::stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const
{
  if (!cfg.valid() || data_.empty()) { return; }
  for (unsigned int y = 0; y < cfg.height; ++y) {
    const int my = cfg.window_y + static_cast<int>(y);
    if (my < 0 || my >= static_cast<int>(src_height_)) { continue; }
    for (unsigned int x = 0; x < cfg.width; ++x) {
      const int mx = cfg.window_x + static_cast<int>(x);
      if (mx < 0 || mx >= static_cast<int>(src_width_)) { continue; }
      const int8_t v = data_[static_cast<size_t>(my) * src_width_ + mx];
      // unknown 保持 -1, 不做"更高代价"合成: 否则会被视图默认值 0 吃掉.
      int8_t out = kCostFree;
      if (v >= 100) {
        out = kCostLethal;
      } else if (v < 0) {
        out = unknown_as_blocked_ ? kCostUnknown : kCostFree;
      }
      // 静态层是"底图", 对窗口内每个格子都是赋值 (不是 max).
      grid[static_cast<size_t>(y) * cfg.width + x] = out;
    }
  }
}

// ---------------- PointsLayer ----------------
void PointsLayer::stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const
{
  if (!cfg.valid()) { return; }
  const int r = (radius_ > 0.0)
    ? std::max(1, static_cast<int>(std::ceil(radius_ / cfg.resolution)))
    : 0;
  for (const auto & p : points_) {
    int gx = 0, gy = 0;
    if (!worldToIndex(cfg, p.x, p.y, gx, gy)) { continue; }
    if (r == 0) {
      raise(grid, static_cast<size_t>(gy) * cfg.width + gx, kCostLethal);
      continue;
    }
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) { continue; }
        const int nx = gx + dx, ny = gy + dy;
        if (nx < 0 || nx >= static_cast<int>(cfg.width) ||
            ny < 0 || ny >= static_cast<int>(cfg.height)) { continue; }
        raise(grid, static_cast<size_t>(ny) * cfg.width + nx, kCostLethal);
      }
    }
  }
}

// ---------------- CirclesLayer ----------------
void CirclesLayer::stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const
{
  if (!cfg.valid()) { return; }
  for (const auto & c : circles_) {
    int gx = 0, gy = 0;
    if (!worldToIndex(cfg, c.x, c.y, gx, gy)) { continue; }
    const int r = std::max(1, static_cast<int>(std::ceil(c.radius / cfg.resolution)));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) { continue; }
        const int nx = gx + dx, ny = gy + dy;
        if (nx < 0 || nx >= static_cast<int>(cfg.width) ||
            ny < 0 || ny >= static_cast<int>(cfg.height)) { continue; }
        raise(grid, static_cast<size_t>(ny) * cfg.width + nx, kCostLethal);
      }
    }
  }
}

// ---------------- KeepoutLayer ----------------
void KeepoutLayer::setPolygons(const std::vector<std::vector<Point2D>> & polygons)
{
  polygons_.clear();
  polygons_.reserve(polygons.size());
  for (const auto & poly : polygons) {
    if (poly.size() >= 3) { polygons_.push_back(poly); }
  }
  ++version_;
}

bool pointInPolygon(const std::vector<Point2D> & polygon, double wx, double wy)
{
  if (polygon.size() < 3) { return false; }
  bool inside = false;
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double yi = polygon[i].y, yj = polygon[j].y;
    const double xi = polygon[i].x, xj = polygon[j].x;
    if (((yi > wy) != (yj > wy)) &&
        (wx < (xj - xi) * (wy - yi) / (yj - yi + 1e-12) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

bool KeepoutLayer::contains(double wx, double wy) const
{
  if (!active_) { return false; }
  for (const auto & poly : polygons_) {
    if (pointInPolygon(poly, wx, wy)) { return true; }
  }
  return false;
}

void KeepoutLayer::stamp(const LayerConfig & cfg, std::vector<int8_t> & grid) const
{
  last_marked_ = 0;
  if (!active_ || polygons_.empty() || !cfg.valid()) { return; }
  for (const auto & poly : polygons_) {
    double min_x = poly[0].x, max_x = poly[0].x;
    double min_y = poly[0].y, max_y = poly[0].y;
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    // 视图左上角的世界坐标 (window_x/y 是整图栅格偏移).
    const double ox = cfg.map_origin_x + cfg.window_x * cfg.resolution;
    const double oy = cfg.map_origin_y + cfg.window_y * cfg.resolution;
    int x0 = static_cast<int>(std::floor((min_x - ox) / cfg.resolution));
    int y0 = static_cast<int>(std::floor((min_y - oy) / cfg.resolution));
    int x1 = static_cast<int>(std::ceil((max_x - ox) / cfg.resolution));
    int y1 = static_cast<int>(std::ceil((max_y - oy) / cfg.resolution));
    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    x1 = std::min(static_cast<int>(cfg.width) - 1, x1);
    y1 = std::min(static_cast<int>(cfg.height) - 1, y1);
    for (int y = y0; y <= y1; ++y) {
      const double wy = oy + (y + 0.5) * cfg.resolution;
      for (int x = x0; x <= x1; ++x) {
        const double wx = ox + (x + 0.5) * cfg.resolution;
        if (pointInPolygon(poly, wx, wy)) {
          raise(grid, static_cast<size_t>(y) * cfg.width + x, kCostLethal);
          ++last_marked_;
        }
      }
    }
  }
}

// ---------------- SpeedLimitLayer ----------------
bool SpeedLimitLayer::contains(double wx, double wy) const
{
  for (const auto & zone : zones_) {
    if (pointInPolygon(zone.polygon, wx, wy)) { return true; }
  }
  return false;
}

double SpeedLimitLayer::maxLinearAt(double wx, double wy, double fallback) const
{
  double limit = fallback;
  for (const auto & zone : zones_) {
    if (pointInPolygon(zone.polygon, wx, wy)) {
      limit = std::min(limit, zone.max_linear);
    }
  }
  return limit;
}

bool SpeedLimitLayer::parseRectZone(const std::string & spec, Zone & zone, double fallback_v)
{
  double x = 0.0, y = 0.0, w = 0.0, h = 0.0, v = fallback_v;
  const int n = std::sscanf(spec.c_str(), "%lf,%lf,%lf,%lf,%lf", &x, &y, &w, &h, &v);
  if (n < 4) { return false; }
  if (n < 5) { v = fallback_v; }
  if (w <= 0.0 || h <= 0.0) { return false; }
  zone.polygon.clear();
  zone.polygon.push_back({x, y});
  zone.polygon.push_back({x + w, y});
  zone.polygon.push_back({x + w, y + h});
  zone.polygon.push_back({x, y + h});
  zone.max_linear = std::max(0.0, v);
  return true;
}

// ---------------- 膨胀 ----------------
std::vector<int8_t> inflatedCostmap(
  const std::vector<int8_t> & raw,
  unsigned int width, unsigned int height,
  int inscribed_cells, int inflate_cells)
{
  const size_t n = static_cast<size_t>(width) * height;
  std::vector<int8_t> out(n, kCostFree);
  if (n == 0 || raw.size() != n) { return out; }

  inflate_cells = std::max(1, inflate_cells);
  inscribed_cells = std::max(0, std::min(inscribed_cells, inflate_cells));

  std::vector<float> dist(n, std::numeric_limits<float>::max());
  std::queue<std::pair<int, int>> q;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if (isLethal(raw[static_cast<size_t>(y) * width + x])) {
        dist[static_cast<size_t>(y) * width + x] = 0.0f;
        q.push({static_cast<int>(x), static_cast<int>(y)});
      }
    }
  }

  const int dx4[4] = {-1, 0, 1, 0};
  const int dy4[4] = {0, -1, 0, 1};
  while (!q.empty()) {
    const std::pair<int, int> c = q.front();
    q.pop();
    const float cd = dist[static_cast<size_t>(c.second) * width + c.first];
    if (cd >= inflate_cells) { continue; }
    for (int i = 0; i < 4; ++i) {
      const int nx = c.first + dx4[i];
      const int ny = c.second + dy4[i];
      if (nx < 0 || nx >= static_cast<int>(width) ||
          ny < 0 || ny >= static_cast<int>(height)) { continue; }
      float & target = dist[static_cast<size_t>(ny) * width + nx];
      if (cd + 1.0f < target) {
        target = cd + 1.0f;
        q.push({nx, ny});
      }
    }
  }

  const float inv = 1.0f / std::max(1, inflate_cells - inscribed_cells);
  for (size_t i = 0; i < n; ++i) {
    if (raw[i] == kCostUnknown) { out[i] = kCostUnknown; continue; }
    if (isLethal(raw[i])) { out[i] = kCostLethal; continue; }
    const float d = dist[i];
    if (d <= static_cast<float>(inscribed_cells)) {
      out[i] = 99;
    } else if (d <= static_cast<float>(inflate_cells)) {
      out[i] = static_cast<int8_t>((1.0f - (d - inscribed_cells) * inv) * 99.0f);
    } else {
      out[i] = kCostFree;
    }
  }
  return out;
}

}  // namespace core
}  // namespace rosiwit_navigation
