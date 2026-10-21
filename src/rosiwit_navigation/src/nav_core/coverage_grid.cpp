// ============================================================================
// coverage_grid.cpp — 见 coverage_grid.hpp
// ============================================================================
#include "rosiwit_navigation/nav_core/coverage_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace rosiwit_navigation
{
namespace core
{
namespace
{

constexpr double kEps = 1e-9;

}  // namespace

bool CoverageGrid::initialize(const CostmapGridInfo & info,
                              const std::vector<Pose2D> & polygon,
                              const CoverageConfig & config)
{
  if (polygon.size() < 3 || info.width == 0 || info.height == 0 ||
      info.resolution <= 0.0) {
    ready_ = false;
    return false;
  }
  info_ = info;
  config_ = config;
  if (!(config_.sweep_width > 0.0)) { config_.sweep_width = 0.5; }
  if (!(config_.obstacle_dilation >= 0.0)) { config_.obstacle_dilation = 0.1; }
  polygon_ = polygon;
  const size_t n = static_cast<size_t>(info_.width) * info_.height;
  cells_.assign(n, CoverageCell::OUTSIDE);
  dynamic_.assign(n, 0);
  inset_.assign(n, 0);
  for (unsigned int my = 0; my < info_.height; ++my) {
    for (unsigned int mx = 0; mx < info_.width; ++mx) {
      const double x = info_.origin.position.x + (mx + 0.5) * info_.resolution;
      const double y = info_.origin.position.y + (my + 0.5) * info_.resolution;
      cells_[index(static_cast<int>(mx), static_cast<int>(my))] =
        pointInPolygon(x, y) ? CoverageCell::UNCOVERED : CoverageCell::OUTSIDE;
    }
  }
  // 扫描方向: 区域长边方向优先 (2x1 区域沿 x 扫).
  {
    double min_x = polygon_[0].x, max_x = polygon_[0].x;
    double min_y = polygon_[0].y, max_y = polygon_[0].y;
    for (const auto & p : polygon_) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    scan_vertical_ = (max_y - min_y) > (max_x - min_x);
  }

  // center_inset: 从区域边界做 BFS 距离场, 边界内 inset 米内的格子
  // 不参与目标选择/路径规划, 但仍算在覆盖统计里 (由清扫宽度覆盖).
  if (config_.center_inset > 0.0) {
    const int inset_cells = std::max(1, static_cast<int>(
      std::ceil(config_.center_inset / info_.resolution)));
    std::vector<int> dist(n, -1);
    std::queue<std::pair<int, int>> queue;
    for (unsigned int my = 0; my < info_.height; ++my) {
      for (unsigned int mx = 0; mx < info_.width; ++mx) {
        if (cells_[index(static_cast<int>(mx), static_cast<int>(my))] ==
            CoverageCell::OUTSIDE) {
          dist[index(static_cast<int>(mx), static_cast<int>(my))] = 0;
          queue.push({static_cast<int>(mx), static_cast<int>(my)});
        }
      }
    }
    const int dx[4] = {-1, 0, 1, 0};
    const int dy[4] = {0, -1, 0, 1};
    while (!queue.empty()) {
      const auto [cx, cy] = queue.front();
      queue.pop();
      const int d = dist[index(cx, cy)];
      if (d >= inset_cells) { continue; }
      for (int i = 0; i < 4; ++i) {
        const int nx = cx + dx[i];
        const int ny = cy + dy[i];
        if (!cellInBounds(nx, ny)) { continue; }
        const int nidx = index(nx, ny);
        if (dist[nidx] >= 0) { continue; }
        dist[nidx] = d + 1;
        queue.push({nx, ny});
      }
    }
    for (size_t i = 0; i < n; ++i) {
      if (cells_[i] != CoverageCell::OUTSIDE && dist[i] >= 0 &&
          dist[i] < inset_cells) {
        inset_[i] = 1;
      }
    }
  }
  has_last_pose_ = false;
  ready_ = true;
  return true;
}

bool CoverageGrid::cellInBounds(int mx, int my) const
{
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(info_.width) && my < static_cast<int>(info_.height);
}

bool CoverageGrid::worldToCell(double x, double y, int & mx, int & my) const
{
  if (info_.resolution <= 0.0) { return false; }
  mx = static_cast<int>(std::floor((x - info_.origin.position.x) / info_.resolution));
  my = static_cast<int>(std::floor((y - info_.origin.position.y) / info_.resolution));
  return cellInBounds(mx, my);
}

Pose2D CoverageGrid::cellToWorld(int mx, int my) const
{
  Pose2D p;
  p.x = info_.origin.position.x + (mx + 0.5) * info_.resolution;
  p.y = info_.origin.position.y + (my + 0.5) * info_.resolution;
  return p;
}

bool CoverageGrid::pointInPolygon(double x, double y) const
{
  bool inside = false;
  const size_t n = polygon_.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon_[i].x;
    const double yi = polygon_[i].y;
    const double xj = polygon_[j].x;
    const double yj = polygon_[j].y;
    const bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi + kEps) + xi);
    if (intersect) { inside = !inside; }
  }
  return inside;
}

void CoverageGrid::markDisk(double x, double y, double radius, CoverageCell state,
                            bool only_uncovered, bool skip_dynamic)
{
  if (!ready_ || radius <= 0.0) { return; }
  int cx = 0;
  int cy = 0;
  if (!worldToCell(x, y, cx, cy)) { return; }
  const double res = info_.resolution;
  const int cells = std::max(1, static_cast<int>(std::ceil(radius / res)));
  const int r2 = cells * cells;
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      if (dx * dx + dy * dy > r2) { continue; }
      const int mx = cx + dx;
      const int my = cy + dy;
      if (!cellInBounds(mx, my)) { continue; }
      const int idx = index(mx, my);
      if (cells_[idx] == CoverageCell::OUTSIDE) { continue; }
      if (skip_dynamic && dynamic_[idx] != 0) { continue; }
      if (only_uncovered && cells_[idx] != CoverageCell::UNCOVERED) { continue; }
      if (state == CoverageCell::OBSTACLE &&
          cells_[idx] == CoverageCell::COVERED) {
        continue;  // 已经覆盖过的格子不再被静态障碍覆盖
      }
      cells_[idx] = state;
    }
  }
}

void CoverageGrid::updateStaticObstacles(const std::vector<int8_t> & map_data)
{
  if (!ready_ || map_data.size() < cells_.size()) { return; }
  for (size_t i = 0; i < cells_.size(); ++i) {
    const int v = map_data[i];
    if (cells_[i] == CoverageCell::OUTSIDE) { continue; }
    if (v >= 100) {
      cells_[i] = CoverageCell::OBSTACLE;
    } else if (v < 0) {
      if (cells_[i] == CoverageCell::UNCOVERED) {
        cells_[i] = CoverageCell::UNKNOWN;
      }
    }
  }
}

void CoverageGrid::updateDynamicObstacles(const std::vector<Pose2D> & points)
{
  if (!ready_) { return; }
  dynamic_.assign(cells_.size(), 0);
  if (config_.obstacle_dilation <= 0.0) { return; }
  const double res = info_.resolution;
  const int cells = std::max(1, static_cast<int>(std::ceil(config_.obstacle_dilation / res)));
  const int r2 = cells * cells;
  for (const auto & p : points) {
    int cx = 0;
    int cy = 0;
    if (!worldToCell(p.x, p.y, cx, cy)) { continue; }
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        if (dx * dx + dy * dy > r2) { continue; }
        const int mx = cx + dx;
        const int my = cy + dy;
        if (!cellInBounds(mx, my)) { continue; }
        dynamic_[index(mx, my)] = 1;
      }
    }
  }
}

void CoverageGrid::updateCovered(const Pose2D & pose)
{
  if (!ready_) { return; }
  const double radius = config_.sweep_width * 0.5;
  if (!has_last_pose_) {
    markDisk(pose.x, pose.y, radius, CoverageCell::COVERED, true, true);
    last_pose_ = pose;
    has_last_pose_ = true;
    return;
  }
  const double dx = pose.x - last_pose_.x;
  const double dy = pose.y - last_pose_.y;
  const double distance = std::hypot(dx, dy);
  const double step = std::max(info_.resolution * 0.5, 0.01);
  const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int i = 1; i <= count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(count);
    markDisk(last_pose_.x + ratio * dx, last_pose_.y + ratio * dy,
             radius, CoverageCell::COVERED, true, true);
  }
  last_pose_ = pose;
}

void CoverageGrid::resetPose(const Pose2D & pose)
{
  last_pose_ = pose;
  has_last_pose_ = true;
}

bool CoverageGrid::passable(int mx, int my, bool avoid_dynamic) const
{
  if (!cellInBounds(mx, my)) { return false; }
  const int idx = index(mx, my);
  if (inset_[idx] != 0) { return false; }
  if (avoid_dynamic && dynamic_[idx] != 0) { return false; }
  const CoverageCell cell = cells_[idx];
  return cell == CoverageCell::UNCOVERED || cell == CoverageCell::COVERED;
}

bool CoverageGrid::bfsStartCell(const Pose2D & pose, int & mx, int & my) const
{
  if (!worldToCell(pose.x, pose.y, mx, my)) { return false; }
  if (passable(mx, my, false)) { return true; }
  // 起点在障碍/边界上时, 在 1m 内找最近的可通行格.
  const int radius = std::max(1, static_cast<int>(std::ceil(1.0 / info_.resolution)));
  double best = std::numeric_limits<double>::max();
  int best_x = -1;
  int best_y = -1;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int nx = mx + dx;
      const int ny = my + dy;
      if (!passable(nx, ny, false)) { continue; }
      const double d = static_cast<double>(dx * dx + dy * dy);
      if (d < best) { best = d; best_x = nx; best_y = ny; }
    }
  }
  if (best_x < 0) { return false; }
  mx = best_x;
  my = best_y;
  return true;
}

bool CoverageGrid::bfsReach(const Pose2D & pose, bool avoid_dynamic,
                            std::vector<uint8_t> * visited) const
{
  if (!ready_) { return false; }
  int sx = 0;
  int sy = 0;
  if (!bfsStartCell(pose, sx, sy)) { return false; }
  const size_t n = cells_.size();
  std::vector<uint8_t> seen(n, 0);
  std::queue<std::pair<int, int>> queue;
  queue.push({sx, sy});
  seen[index(sx, sy)] = 1;
  bool found_uncovered = false;
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    const int idx = index(cx, cy);
    if (cells_[idx] == CoverageCell::UNCOVERED) { found_uncovered = true; }
    for (int i = 0; i < 4; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (!cellInBounds(nx, ny)) { continue; }
      const int nidx = index(nx, ny);
      if (seen[nidx] != 0) { continue; }
      if (!passable(nx, ny, avoid_dynamic)) { continue; }
      seen[nidx] = 1;
      queue.push({nx, ny});
    }
  }
  if (visited != nullptr) { *visited = std::move(seen); }
  return found_uncovered;
}

bool CoverageGrid::nearestUncovered(const Pose2D & pose, Pose2D & target,
                                    bool avoid_dynamic) const
{
  if (!ready_) { return false; }
  int sx = 0;
  int sy = 0;
  if (!bfsStartCell(pose, sx, sy)) { return false; }
  std::vector<uint8_t> seen(cells_.size(), 0);
  std::queue<std::pair<int, int>> queue;
  queue.push({sx, sy});
  seen[index(sx, sy)] = 1;
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    const int idx = index(cx, cy);
    if (cells_[idx] == CoverageCell::UNCOVERED) {
      const Pose2D p = cellToWorld(cx, cy);
      target.x = p.x;
      target.y = p.y;
      target.theta = 0.0;
      return true;
    }
    for (int i = 0; i < 4; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (!cellInBounds(nx, ny)) { continue; }
      const int nidx = index(nx, ny);
      if (seen[nidx] != 0) { continue; }
      if (!passable(nx, ny, avoid_dynamic)) { continue; }
      seen[nidx] = 1;
      queue.push({nx, ny});
    }
  }
  return false;
}

bool CoverageGrid::farthestUncovered(const Pose2D & pose, Pose2D & target,
                                     double max_distance, bool avoid_dynamic) const
{
  if (!ready_ || max_distance <= 0.0) {
    return nearestUncovered(pose, target, avoid_dynamic);
  }
  int sx = 0;
  int sy = 0;
  if (!bfsStartCell(pose, sx, sy)) { return false; }
  const int max_cells = std::max(1, static_cast<int>(
    std::ceil(max_distance / info_.resolution)));
  std::vector<int> depth(cells_.size(), -1);
  std::queue<std::pair<int, int>> queue;
  queue.push({sx, sy});
  depth[index(sx, sy)] = 0;
  int best_depth = -1;
  int best_x = -1;
  int best_y = -1;
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    const int d = depth[index(cx, cy)];
    if (cells_[index(cx, cy)] == CoverageCell::UNCOVERED) {
      if (d <= max_cells && d > best_depth) {
        best_depth = d;
        best_x = cx;
        best_y = cy;
      }
    }
    if (d >= max_cells) { continue; }
    for (int i = 0; i < 4; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (!cellInBounds(nx, ny)) { continue; }
      const int nidx = index(nx, ny);
      if (depth[nidx] >= 0) { continue; }
      if (!passable(nx, ny, avoid_dynamic)) { continue; }
      depth[nidx] = d + 1;
      queue.push({nx, ny});
    }
  }
  if (best_x < 0) { return false; }
  const Pose2D p = cellToWorld(best_x, best_y);
  target.x = p.x;
  target.y = p.y;
  target.theta = 0.0;
  return true;
}

bool CoverageGrid::nextCoverageSegment(const Pose2D & pose, double max_length,
                                       std::vector<Pose2D> & segment) const
{
  segment.clear();
  if (!ready_) { return false; }
  // 最近未覆盖格作为种子
  int sx = 0;
  int sy = 0;
  if (!bfsStartCell(pose, sx, sy)) { return false; }
  std::vector<uint8_t> seen(cells_.size(), 0);
  std::queue<std::pair<int, int>> queue;
  queue.push({sx, sy});
  seen[index(sx, sy)] = 1;
  int seed_x = -1;
  int seed_y = -1;
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    if (cells_[index(cx, cy)] == CoverageCell::UNCOVERED) {
      seed_x = cx; seed_y = cy; break;
    }
    for (int i = 0; i < 4; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (!cellInBounds(nx, ny)) { continue; }
      const int nidx = index(nx, ny);
      if (seen[nidx] != 0) { continue; }
      if (!passable(nx, ny, true)) { continue; }
      seen[nidx] = 1;
      queue.push({nx, ny});
    }
  }
  if (seed_x < 0) { return false; }

  int lo = 0;
  int hi = 0;
  if (scan_vertical_) {
    lo = seed_y; hi = seed_y;
    while (lo - 1 >= 0 && passable(seed_x, lo - 1, true)) { --lo; }
    while (hi + 1 < static_cast<int>(info_.height) && passable(seed_x, hi + 1, true)) { ++hi; }
  } else {
    lo = seed_x; hi = seed_x;
    while (lo - 1 >= 0 && passable(lo - 1, seed_y, true)) { --lo; }
    while (hi + 1 < static_cast<int>(info_.width) && passable(hi + 1, seed_y, true)) { ++hi; }
  }

  const double start_x = scan_vertical_ ? seed_x : lo;
  const double start_y = scan_vertical_ ? lo : seed_y;
  const double end_x = scan_vertical_ ? seed_x : hi;
  const double end_y = scan_vertical_ ? hi : seed_y;
  const Pose2D a = cellToWorld(static_cast<int>(start_x), static_cast<int>(start_y));
  const Pose2D b = cellToWorld(static_cast<int>(end_x), static_cast<int>(end_y));
  const bool near_a = pose.distanceTo(a) <= pose.distanceTo(b);
  const Pose2D from = near_a ? a : b;
  const Pose2D to = near_a ? b : a;
  const double length = from.distanceTo(to);
  if (length < info_.resolution) { return false; }

  const int steps = std::max(1, static_cast<int>(std::ceil(length / info_.resolution)));
  const double limit = (max_length > 0.0) ? max_length : length;
  double traveled = 0.0;
  Pose2D prev = from;
  segment.push_back(from);
  for (int i = 1; i <= steps; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(steps);
    Pose2D p;
    p.x = from.x + ratio * (to.x - from.x);
    p.y = from.y + ratio * (to.y - from.y);
    p.theta = std::atan2(to.y - from.y, to.x - from.x);
    traveled += prev.distanceTo(p);
    prev = p;
    if (traveled > limit) {
      // 线性插值到 max_length 处
      const double over = traveled - limit;
      const double seg = std::max(1e-6, prev.distanceTo(segment.back()));
      p.x -= (p.x - segment.back().x) * (over / std::max(seg, 1e-6));
      p.y -= (p.y - segment.back().y) * (over / std::max(seg, 1e-6));
      segment.push_back(p);
      break;
    }
    segment.push_back(p);
  }
  return segment.size() >= 2;
}

bool CoverageGrid::hasUncoveredReachable(const Pose2D & pose, bool avoid_dynamic) const
{
  return bfsReach(pose, avoid_dynamic, nullptr);
}

bool CoverageGrid::hasAnyUncovered() const
{
  for (const auto cell : cells_) {
    if (cell == CoverageCell::UNCOVERED) { return true; }
  }
  return false;
}

bool CoverageGrid::contains(const Pose2D & pose) const
{
  int mx = 0;
  int my = 0;
  if (!ready_ || !worldToCell(pose.x, pose.y, mx, my)) { return false; }
  const CoverageCell cell = cells_[index(mx, my)];
  return cell == CoverageCell::UNCOVERED || cell == CoverageCell::COVERED;
}

size_t CoverageGrid::uncoveredCount() const
{
  size_t count = 0;
  for (const auto cell : cells_) {
    if (cell == CoverageCell::UNCOVERED) { ++count; }
  }
  return count;
}

void CoverageGrid::markUnreachable(const Pose2D & pose)
{
  if (!ready_) { return; }
  std::vector<uint8_t> reachable;
  bfsReach(pose, false, &reachable);
  if (reachable.size() != cells_.size()) {
    // 起点完全不可用: 所有未覆盖都视为不可达.
    markAllUncoveredBlocked();
    return;
  }
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (cells_[i] == CoverageCell::UNCOVERED && reachable[i] == 0) {
      cells_[i] = CoverageCell::BLOCKED;
    }
  }
}

size_t CoverageGrid::markComponentBlocked(const Pose2D & seed)
{
  int sx = 0;
  int sy = 0;
  if (!ready_ || !worldToCell(seed.x, seed.y, sx, sy)) { return 0; }
  if (cells_[index(sx, sy)] != CoverageCell::UNCOVERED) { return 0; }
  std::queue<std::pair<int, int>> queue;
  queue.push({sx, sy});
  cells_[index(sx, sy)] = CoverageCell::BLOCKED;
  size_t count = 0;
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    ++count;
    for (int i = 0; i < 4; ++i) {
      const int nx = cx + dx[i];
      const int ny = cy + dy[i];
      if (!cellInBounds(nx, ny)) { continue; }
      const int idx = index(nx, ny);
      if (cells_[idx] != CoverageCell::UNCOVERED) { continue; }
      cells_[idx] = CoverageCell::BLOCKED;
      queue.push({nx, ny});
    }
  }
  return count;
}

void CoverageGrid::markAllUncoveredBlocked()
{
  if (!ready_) { return; }
  for (auto & cell : cells_) {
    if (cell == CoverageCell::UNCOVERED) { cell = CoverageCell::BLOCKED; }
  }
}

std::vector<std::vector<Pose2D>> CoverageGrid::blockedRegions(size_t min_cells) const
{
  std::vector<std::vector<Pose2D>> regions;
  if (!ready_) { return regions; }
  std::vector<uint8_t> seen(cells_.size(), 0);
  const int dx[4] = {-1, 0, 1, 0};
  const int dy[4] = {0, -1, 0, 1};
  for (unsigned int my = 0; my < info_.height; ++my) {
    for (unsigned int mx = 0; mx < info_.width; ++mx) {
      const int start = index(static_cast<int>(mx), static_cast<int>(my));
      if (seen[start] != 0 || cells_[start] != CoverageCell::BLOCKED) { continue; }
      std::queue<std::pair<int, int>> queue;
      queue.push({static_cast<int>(mx), static_cast<int>(my)});
      seen[start] = 1;
      size_t count = 0;
      int min_x = static_cast<int>(mx);
      int max_x = static_cast<int>(mx);
      int min_y = static_cast<int>(my);
      int max_y = static_cast<int>(my);
      while (!queue.empty()) {
        const auto [cx, cy] = queue.front();
        queue.pop();
        ++count;
        min_x = std::min(min_x, cx);
        max_x = std::max(max_x, cx);
        min_y = std::min(min_y, cy);
        max_y = std::max(max_y, cy);
        for (int i = 0; i < 4; ++i) {
          const int nx = cx + dx[i];
          const int ny = cy + dy[i];
          if (!cellInBounds(nx, ny)) { continue; }
          const int nidx = index(nx, ny);
          if (seen[nidx] != 0 || cells_[nidx] != CoverageCell::BLOCKED) { continue; }
          seen[nidx] = 1;
          queue.push({nx, ny});
        }
      }
      if (count < min_cells) { continue; }
      const double x0 = info_.origin.position.x + min_x * info_.resolution;
      const double y0 = info_.origin.position.y + min_y * info_.resolution;
      const double x1 = info_.origin.position.x + (max_x + 1) * info_.resolution;
      const double y1 = info_.origin.position.y + (max_y + 1) * info_.resolution;
      std::vector<Pose2D> box;
      box.push_back({x0, y0, 0.0});
      box.push_back({x1, y0, 0.0});
      box.push_back({x1, y1, 0.0});
      box.push_back({x0, y1, 0.0});
      regions.push_back(std::move(box));
    }
  }
  return regions;
}

CoverageStats CoverageGrid::stats() const
{
  CoverageStats s;
  for (const auto cell : cells_) {
    switch (cell) {
      case CoverageCell::COVERED: ++s.covered; break;
      case CoverageCell::UNCOVERED: ++s.uncovered; break;
      case CoverageCell::BLOCKED: ++s.blocked; break;
      case CoverageCell::OBSTACLE: ++s.obstacle; break;
      case CoverageCell::UNKNOWN: ++s.unknown; break;
      case CoverageCell::OUTSIDE: ++s.outside; break;
    }
  }
  s.total = s.covered + s.uncovered + s.blocked;
  return s;
}

size_t CoverageGrid::dynamicCount() const
{
  size_t count = 0;
  for (const auto v : dynamic_) {
    if (v != 0) { ++count; }
  }
  return count;
}

std::vector<int8_t> CoverageGrid::plannerCostmap(
  const std::vector<int8_t> & /*map_data*/) const
{
  std::vector<int8_t> out(cells_.size(), 100);
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (dynamic_[i] != 0 || inset_[i] != 0) { continue; }
    switch (cells_[i]) {
      case CoverageCell::UNCOVERED:
      case CoverageCell::COVERED:
        out[i] = 0;
        break;
      case CoverageCell::OUTSIDE:
        out[i] = 100;  // 区域边界是车体中心的硬边界
        break;
      default:
        out[i] = 100;
        break;
    }
  }
  return out;
}

std::vector<int8_t> CoverageGrid::encode() const
{
  std::vector<int8_t> out(cells_.size(), -1);
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (dynamic_[i] != 0) {
      out[i] = 100;
      continue;
    }
    switch (cells_[i]) {
      case CoverageCell::UNCOVERED: out[i] = 0; break;
      case CoverageCell::COVERED: out[i] = 50; break;
      case CoverageCell::BLOCKED: out[i] = 80; break;
      case CoverageCell::OBSTACLE: out[i] = 100; break;
      case CoverageCell::UNKNOWN:
      case CoverageCell::OUTSIDE: out[i] = -1; break;
    }
  }
  return out;
}

}  // namespace core
}  // namespace rosiwit_navigation
