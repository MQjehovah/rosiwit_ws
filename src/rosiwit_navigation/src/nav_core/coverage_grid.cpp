// ============================================================================
// coverage_grid.cpp — 见 coverage_grid.hpp
// ============================================================================
#include "rosiwit_navigation/nav_core/coverage_grid.hpp"

#include <iostream>

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
  // 静态障碍的车体半径 halo: 车体中心不能进到这里 (但清扫宽度能盖到).
  // 不做这一步, 行会贴着墙选, 车根本站不下 -> 同一行反复规划到超时.
  if (config_.robot_inset > 0.0) {
    halo_.assign(cells_.size(), 0);
    const int hcells = std::max(1, static_cast<int>(
      std::ceil(config_.robot_inset / info_.resolution)));
    std::vector<int> dist(cells_.size(), -1);
    std::queue<std::pair<int, int>> queue;
    for (unsigned int my = 0; my < info_.height; ++my) {
      for (unsigned int mx = 0; mx < info_.width; ++mx) {
        const int idx = index(static_cast<int>(mx), static_cast<int>(my));
        if (cells_[idx] == CoverageCell::OBSTACLE) {
          dist[idx] = 0;
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
      if (d >= hcells) { continue; }
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
    for (size_t i = 0; i < cells_.size(); ++i) {
      if (cells_[i] != CoverageCell::OUTSIDE && dist[i] > 0 && dist[i] <= hcells) {
        halo_[i] = 1;
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
  if (!halo_.empty() && halo_[idx] != 0) { return false; }   // 车体半径内不进
  if (avoid_dynamic && dynamic_[idx] != 0) { return false; }
  const CoverageCell cell = cells_[idx];
  return cell == CoverageCell::UNCOVERED || cell == CoverageCell::COVERED;
}

bool CoverageGrid::bfsStartCell(const Pose2D & pose, int & mx, int & my) const
{
  if (!worldToCell(pose.x, pose.y, mx, my)) { return false; }
  if (passable(mx, my, false)) { return true; }
  // 起点在障碍/边界上时, 在附近找最近的可通行格.
  // 半径 2.5m: 车体偶尔冲出区域边界 (控制超调) 时还能把清扫拉回来,
  // 以前 1m 会直接判定"无路可走" -> 整个覆盖任务提前结束.
  const int radius = std::max(1, static_cast<int>(std::ceil(2.5 / info_.resolution)));
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
  if (!bfsStartCell(pose, sx, sy)) {
    int wx = 0;
    int wy = 0;
    const bool in_map = worldToCell(pose.x, pose.y, wx, wy);
    std::cout << "[CovGrid] bfsStartCell failed: pose=(" << pose.x << ","
              << pose.y << ") in_map=" << (in_map ? 1 : 0);
    if (in_map) {
      std::cout << " cell=(" << wx << "," << wy << ") state="
                << static_cast<int>(cells_[index(wx, wy)]);
    }
    std::cout << std::endl;
    return false;
  }
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

// 按固定行距生成平行行 (boustrophedon), 在机器人可达集合里挑离它最近、
// 且长度够跟的一行; 端点按 passable 裁剪.
//
// 旧实现是"从最近的未覆盖格做 BFS 找种子再沿轴扩展": 在区域边缘会啃出
// 只有 3 个点的碎行 (row=(1.49,0.66)->(1.39,0.66)), approach 又判
// Goal in obstacle, 于是同一位置反复重试 -> 覆盖率卡在 10~30%.
bool CoverageGrid::nextCoverageSegment(const Pose2D & pose, double max_length,
                                       std::vector<Pose2D> & segment) const
{
  segment.clear();
  if (!ready_ || polygon_.size() < 3) { return false; }

  int sx = 0;
  int sy = 0;
  if (!bfsStartCell(pose, sx, sy)) { return false; }

  // ---- 1) 机器人可达格集合: 只在可达集合里选行, 避免"可行但到不了" ----
  const size_t n = cells_.size();
  std::vector<uint8_t> reach(n, 0);
  const int ddx[4] = {-1, 0, 1, 0};
  const int ddy[4] = {0, -1, 0, 1};
  {
    std::queue<std::pair<int, int>> queue;
    queue.push({sx, sy});
    reach[index(sx, sy)] = 1;
    while (!queue.empty()) {
      const auto [cx, cy] = queue.front();
      queue.pop();
      for (int i = 0; i < 4; ++i) {
        const int nx = cx + ddx[i];
        const int ny = cy + ddy[i];
        if (!cellInBounds(nx, ny)) { continue; }
        const int nidx = index(nx, ny);
        if (reach[nidx] != 0) { continue; }
        if (!passable(nx, ny, true)) { continue; }
        reach[nidx] = 1;
        queue.push({nx, ny});
      }
    }
  }

  // ---- 2) 区域 bbox ----
  int bx0 = 0, by0 = 0, bx1 = 0, by1 = 0;
  if (!worldToCell(polygon_[0].x, polygon_[0].y, bx0, by0)) { return false; }
  bx1 = bx0;
  by1 = by0;
  for (const auto & p : polygon_) {
    int mx = 0, my = 0;
    if (!worldToCell(p.x, p.y, mx, my)) { continue; }
    bx0 = std::min(bx0, mx);
    bx1 = std::max(bx1, mx);
    by0 = std::min(by0, my);
    by1 = std::max(by1, my);
  }
  const int rows_lo = scan_vertical_ ? bx0 : by0;
  const int rows_hi = scan_vertical_ ? bx1 : by1;
  const int cols_lo = scan_vertical_ ? by0 : bx0;
  const int cols_hi = scan_vertical_ ? by1 : bx1;
  const int row_step = std::max(1, static_cast<int>(
    std::lround(config_.sweep_width / info_.resolution)));
  // 行线不要贴着区域 bbox 边界: 贴边时机器人一路压线, 频繁触发"出界限速"
  // (实测平均速度只有 0.107 m/s, 预期 0.45). 内缩半个清扫宽度, 外圈由
  // sweep_width/2 的清扫盘覆盖, 不会漏.
  const int row_margin = std::max(1, static_cast<int>(
    std::lround(0.5 * config_.sweep_width / info_.resolution)));
  const int r_lo = rows_lo + row_margin;
  const int r_hi = rows_hi - row_margin;

  // ---- 3) 挑行: 有"可达且未覆盖"格子的行, 取离机器人最近的;
  //         优先长度够跟的 (>= 0.3m), 碎行只在没有别的选择时才用 ----
  const double kMinUsableLen = 0.30;
  int best_row = -1;
  double best_score = 0.0;
  bool best_long = false;
  for (int r = r_lo; r <= r_hi; r += row_step) {
    int lo_c = -1;
    int hi_c = -1;
    for (int c = cols_lo; c <= cols_hi; ++c) {
      const int mx = scan_vertical_ ? r : c;
      const int my = scan_vertical_ ? c : r;
      const int idx = index(mx, my);
      if (cells_[idx] == CoverageCell::UNCOVERED && reach[idx] != 0) {
        if (lo_c < 0) { lo_c = c; }
        hi_c = c;
      }
    }
    if (lo_c < 0) { continue; }
    const double len = (hi_c - lo_c) * info_.resolution;
    const bool usable = (len >= kMinUsableLen);
    if (best_row >= 0 && best_long && !usable) { continue; }   // 已有的更好
    const double row_world = scan_vertical_
      ? cellToWorld(r, 0).x : cellToWorld(0, r).y;
    const double dist = std::fabs((scan_vertical_ ? pose.x : pose.y) - row_world);
    if (best_row < 0 || (usable && !best_long) ||
        (usable == best_long && dist < best_score)) {
      best_row = r;
      best_score = dist;
      best_long = usable;
    }
  }
  // 兜底: 固定行线之间可能残留条带 (行距 0.4m, 清扫盘刚好相切, 边缘会有
  // 几格漏掉 — 实测剩 0.23 m2 / 3%). 这时退回到"最近的可达未覆盖格"临时
  // 定一行, 保证整体能收尾, 而不是留下一条缝隙直到超时.
  if (best_row < 0) {
    int seed_x = -1;
    int seed_y = -1;
    double best_d2 = std::numeric_limits<double>::max();
    for (int my = by0; my <= by1; ++my) {
      for (int mx = bx0; mx <= bx1; ++mx) {
        const int idx = index(mx, my);
        if (cells_[idx] != CoverageCell::UNCOVERED || reach[idx] == 0) { continue; }
        const double ddx = mx - sx;
        const double ddy = my - sy;
        const double d2 = ddx * ddx + ddy * ddy;
        if (d2 < best_d2) { best_d2 = d2; seed_x = mx; seed_y = my; }
      }
    }
    if (seed_x >= 0) {
      best_row = scan_vertical_ ? seed_x : seed_y;
    }
  }

  if (best_row < 0) {
    // 诊断: 为什么没有可选的行 (可达集合被什么截断)
    size_t reach_n = 0;
    size_t unc_reach = 0;
    for (size_t i = 0; i < n; ++i) {
      if (reach[i] != 0) {
        ++reach_n;
        if (cells_[i] == CoverageCell::UNCOVERED) { ++unc_reach; }
      }
    }
    std::cout << "[CovGrid] no row: reach=" << reach_n
              << " uncovered_reachable=" << unc_reach
              << " dynamic=" << dynamicCount()
              << " rows=" << rows_lo << ".." << rows_hi
              << " step=" << row_step
              << " start=(" << sx << "," << sy << ")"
              << " state=" << static_cast<int>(cells_[index(sx, sy)]) << std::endl;
    return false;
  }

  // ---- 4) 该行上覆盖 [最左, 最右] 的未覆盖可达格, 端点按 passable 外扩 ----
  int lo = -1;
  int hi = -1;
  for (int c = cols_lo; c <= cols_hi; ++c) {
    const int mx = scan_vertical_ ? best_row : c;
    const int my = scan_vertical_ ? c : best_row;
    const int idx = index(mx, my);
    if (cells_[idx] == CoverageCell::UNCOVERED && reach[idx] != 0) {
      if (lo < 0) { lo = c; }
      hi = c;
    }
  }
  if (lo < 0) { return false; }

  // 外扩到清扫半径内 (让清扫盘盖住端点格), 但只能在 passable 上
  const int grow = std::max(0, static_cast<int>(
    std::lround(config_.sweep_width * 0.5 / info_.resolution)) - 1);
  for (int k = 1; k <= grow; ++k) {
    const int c = lo - k;
    if (c < cols_lo) { break; }
    const int mx = scan_vertical_ ? best_row : c;
    const int my = scan_vertical_ ? c : best_row;
    if (!passable(mx, my, true)) { break; }
    lo = c;
  }
  for (int k = 1; k <= grow; ++k) {
    const int c = hi + k;
    if (c > cols_hi) { break; }
    const int mx = scan_vertical_ ? best_row : c;
    const int my = scan_vertical_ ? c : best_row;
    if (!passable(mx, my, true)) { break; }
    hi = c;
  }

  const Pose2D a = cellToWorld(scan_vertical_ ? best_row : lo,
                               scan_vertical_ ? lo : best_row);
  const Pose2D b = cellToWorld(scan_vertical_ ? best_row : hi,
                               scan_vertical_ ? hi : best_row);
  const bool near_a = pose.distanceTo(a) <= pose.distanceTo(b);
  Pose2D from = near_a ? a : b;
  Pose2D to = near_a ? b : a;

  // 已经在这一行上时, 从投影点直接往远端扫, 不要先退回行端
  const double perp = scan_vertical_ ? std::fabs(pose.x - a.x) : std::fabs(pose.y - a.y);
  if (perp <= config_.sweep_width * 0.6) {
    const double lo_w = scan_vertical_ ? std::min(a.y, b.y) : std::min(a.x, b.x);
    const double hi_w = scan_vertical_ ? std::max(a.y, b.y) : std::max(a.x, b.x);
    const double along = scan_vertical_ ? pose.y : pose.x;
    if (along > lo_w - info_.resolution && along < hi_w + info_.resolution) {
      Pose2D projected;
      if (scan_vertical_) {
        projected = Pose2D(a.x, std::max(lo_w, std::min(hi_w, along)), 0.0);
      } else {
        projected = Pose2D(std::max(lo_w, std::min(hi_w, along)), a.y, 0.0);
      }
      const double da = projected.distanceTo(a);
      const double db = projected.distanceTo(b);
      if (std::max(da, db) > info_.resolution) {
        from = projected;
        to = db > da ? b : a;
      }
    }
  }
  from.theta = std::atan2(to.y - from.y, to.x - from.x);
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

Pose2D CoverageGrid::centroid() const
{
  Pose2D c;
  if (polygon_.empty()) { return c; }
  for (const auto & p : polygon_) { c.x += p.x; c.y += p.y; }
  c.x /= static_cast<double>(polygon_.size());
  c.y /= static_cast<double>(polygon_.size());
  return c;
}

void CoverageGrid::markBlockedAround(const Pose2D & pose, double radius)
{
  if (!ready_ || radius <= 0.0) { return; }
  // only_uncovered=true: 不能把已覆盖格改成 BLOCKED, 否则统计里的 covered
  // 会往下掉 (实测 89.8% -> 89.0%).
  markDisk(pose.x, pose.y, radius, CoverageCell::BLOCKED, true, false);
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
