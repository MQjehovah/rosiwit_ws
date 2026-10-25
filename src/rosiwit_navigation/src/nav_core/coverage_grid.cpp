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


// ============================================================
// 一次生成剩余整条覆盖路线 (boustrophedon)
//   参考 fields2cover: SwathGenerator (headland 内缩的平行行) + Route (排序/连接)
//   参考 opennav_coverage: 一次出完整 route, 再交给控制器跟踪
// 点序即执行序; 位置与前一点相同的点 = "在该点原地转到该点朝向".
// ============================================================
bool CoverageGrid::buildRoute(const Pose2D & pose, std::vector<Pose2D> & route) const
{
  route.clear();
  if (!ready_ || polygon_.size() < 3) { return false; }

  int sx = 0;
  int sy = 0;
  if (!bfsStartCell(pose, sx, sy)) { return false; }

  const double res = std::max(1e-3, info_.resolution);
  const int W = static_cast<int>(info_.width);
  const int H = static_cast<int>(info_.height);

  // ---- 可达集合 (与 nextCoverageSegment 一致) ----
  std::vector<uint8_t> reach(cells_.size(), 0);
  {
    const int ddx[4] = {-1, 0, 1, 0};
    const int ddy[4] = {0, -1, 0, 1};
    std::queue<std::pair<int, int>> queue;
    queue.push({sx, sy});
    reach[index(sx, sy)] = 1;
    while (!queue.empty()) {
      const std::pair<int, int> cur = queue.front();
      queue.pop();
      for (int i = 0; i < 4; ++i) {
        const int nx = cur.first + ddx[i];
        const int ny = cur.second + ddy[i];
        if (!cellInBounds(nx, ny)) { continue; }
        const int nidx = index(nx, ny);
        if (reach[nidx] != 0) { continue; }
        if (!passable(nx, ny, true)) { continue; }
        reach[nidx] = 1;
        queue.push({nx, ny});
      }
    }
  }

  // ---- 区域 bbox (clamp 到栅格) ----
  double px0 = polygon_[0].x;
  double px1 = polygon_[0].x;
  double py0 = polygon_[0].y;
  double py1 = polygon_[0].y;
  for (const auto & p : polygon_) {
    px0 = std::min(px0, p.x);
    px1 = std::max(px1, p.x);
    py0 = std::min(py0, p.y);
    py1 = std::max(py1, p.y);
  }
  auto toCell = [&](double v, double origin) {
    return static_cast<int>(std::floor((v - origin) / res));
  };
  const int bx0 = std::max(0, toCell(px0, info_.origin.position.x));
  const int bx1 = std::min(W - 1, toCell(px1, info_.origin.position.x));
  const int by0 = std::max(0, toCell(py0, info_.origin.position.y));
  const int by1 = std::min(H - 1, toCell(py1, info_.origin.position.y));
  if (bx1 <= bx0 || by1 <= by0) { return false; }

  // ---- 1) 距离场: 每格到"区域外"的距离 (米) ----
  // headland 环与内部分界都靠它. 两遍 3-4 chamfer 近似欧氏距离.
  const float kInf = 1e6f;
  const float kDiag = 1.41421356f;
  std::vector<float> dist(cells_.size(), kInf);
  for (int y = by0; y <= by1; ++y) {
    for (int x = bx0; x <= bx1; ++x) {
      if (cells_[index(x, y)] == CoverageCell::OUTSIDE) { dist[index(x, y)] = 0.0f; }
    }
  }
  for (int y = by0; y <= by1; ++y) {
    for (int x = bx0; x <= bx1; ++x) {
      float & v = dist[index(x, y)];
      if (v <= 0.0f) { continue; }
      float best = v;
      if (x > bx0) { best = std::min(best, dist[index(x - 1, y)] + 1.0f); }
      if (y > by0) { best = std::min(best, dist[index(x, y - 1)] + 1.0f); }
      if (x > bx0 && y > by0) { best = std::min(best, dist[index(x - 1, y - 1)] + kDiag); }
      if (x < bx1 && y > by0) { best = std::min(best, dist[index(x + 1, y - 1)] + kDiag); }
      v = best;
    }
  }
  for (int y = by1; y >= by0; --y) {
    for (int x = bx1; x >= bx0; --x) {
      float & v = dist[index(x, y)];
      if (v <= 0.0f) { continue; }
      float best = v;
      if (x < bx1) { best = std::min(best, dist[index(x + 1, y)] + 1.0f); }
      if (y < by1) { best = std::min(best, dist[index(x, y + 1)] + 1.0f); }
      if (x < bx1 && y < by1) { best = std::min(best, dist[index(x + 1, y + 1)] + kDiag); }
      if (x > bx0 && y < by1) { best = std::min(best, dist[index(x - 1, y + 1)] + kDiag); }
      v = best;
    }
  }
  for (int y = by0; y <= by1; ++y) {
    for (int x = bx0; x <= bx1; ++x) {
      dist[index(x, y)] *= static_cast<float>(res);
    }
  }

  // ---- 2) headland 环: 沿边界内缩 headland_width/2 走一圈 ----
  // 与 fields2cover 的 generate_headland / path_coverage_ros2 的 border_drive 一致:
  // 先绕边界一圈把边带扫掉, 内部再按行扫, 这样行线不用贴边, 边带不会被漏.
  std::vector<std::vector<std::pair<int, int>>> ring_segs;
  const double headland = std::max(0.0, config_.headland_width);
  if (headland > 0.0) {
    const double target = 0.5 * headland;
    // 取"跨过 target 等值线"的那一层格子 (d>=target 且有一个 4 邻域 d<target).
    // 直接用 |d-target| < res/2 会同时命中相邻两层 (栅格中心到边界是 0.225/0.275),
    // 环变成两格厚, 贪心走位会在两格之间来回抖.
    std::vector<std::pair<int, int>> cand;
    cand.reserve(4096);
    for (int y = by0; y <= by1; ++y) {
      for (int x = bx0; x <= bx1; ++x) {
        const int i = index(x, y);
        if (dist[i] < target) { continue; }
        bool crossing = (x <= bx0 || x >= bx1 || y <= by0 || y >= by1);
        const int nbx[4] = {x - 1, x + 1, x, x};
        const int nby[4] = {y, y, y - 1, y + 1};
        for (int k = 0; k < 4 && !crossing; ++k) {
          if (nbx[k] < bx0 || nbx[k] > bx1 || nby[k] < by0 || nby[k] > by1) {
            crossing = true;
            break;
          }
          if (dist[index(nbx[k], nby[k])] < target) { crossing = true; }
        }
        if (!crossing) { continue; }
        if (!passable(x, y, true)) { continue; }
        cand.push_back({x, y});
      }
    }
    // 贪心最近邻走环 (步长 <= 0.15m): 遇到障碍/断口就另起一段.
    std::vector<uint8_t> used(cand.size(), 0);
    size_t remaining = cand.size();
    while (remaining >= 8) {
      size_t cur = cand.size();
      double best = std::numeric_limits<double>::max();
      const Pose2D ref = ring_segs.empty() ? pose :
        cellToWorld(ring_segs.back().back().first, ring_segs.back().back().second);
      for (size_t i = 0; i < cand.size(); ++i) {
        if (used[i] != 0) { continue; }
        const double d = ref.distanceTo(cellToWorld(cand[i].first, cand[i].second));
        if (d < best) { best = d; cur = i; }
      }
      if (cur >= cand.size()) { break; }
      used[cur] = 1;
      --remaining;
      ring_segs.push_back({cand[cur]});
      while (true) {
        size_t nb = cand.size();
        double bd = 3.0 * res;
        for (size_t j = 0; j < cand.size(); ++j) {
          if (used[j] != 0) { continue; }
          const double dx = static_cast<double>(cand[j].first - cand[cur].first);
          const double dy = static_cast<double>(cand[j].second - cand[cur].second);
          const double d = std::hypot(dx, dy) * res;
          if (d < bd) { bd = d; nb = j; }
        }
        if (nb >= cand.size()) { break; }
        used[nb] = 1;
        --remaining;
        ring_segs.back().push_back(cand[nb]);
        cur = nb;
      }
    }
  }

  // ---- 3) 单元分解: x-monotone (Boustrophedon / Choset) ----
  // 沿扫掠方向切片; 可通行区间 1:1 延续 -> 同一单元; 分叉/合并/端点台阶 -> 切单元.
  // 每个单元内每条行线都是一整段, 这是"每行只取最长一段"那个 bug 的正解.
  const bool slice_along_x = !scan_vertical_;
  const int sweep_cells = std::max(1, static_cast<int>(std::lround(config_.sweep_width / res)));
  const int doorway_cells = (config_.doorway_width > 0.0)
    ? std::max(1, static_cast<int>(std::lround(config_.doorway_width / res)))
    : 0;
  const int s_lo = slice_along_x ? bx0 : by0;
  const int s_hi = slice_along_x ? bx1 : by1;
  const int a_lo = slice_along_x ? by0 : bx0;
  const int a_hi = slice_along_x ? by1 : bx1;
  auto cellOf = [&](int s, int a, int & mx, int & my) {
    if (slice_along_x) {
      mx = s;
      my = a;
    } else {
      mx = a;
      my = s;
    }
  };
  const float interior_thresh = static_cast<float>(headland) - 0.5f * static_cast<float>(res);
  auto interiorAt = [&](int s, int a) -> bool {
    int mx = 0;
    int my = 0;
    cellOf(s, a, mx, my);
    if (!cellInBounds(mx, my)) { return false; }
    const int i = index(mx, my);
    if (dist[i] < interior_thresh) { return false; }
    return passable(mx, my, true) && reach[i] != 0;
  };

  struct Iv { int lo; int hi; };
  struct DCell {
    int s0 = 0;
    std::vector<Iv> iv;   // 与切片 s0..s0+iv.size()-1 对齐
  };
  std::vector<DCell> cells;
  std::vector<int> label(cells_.size(), -1);   // 每格属于哪个单元 (-1 = 不是内部)
  {
    std::vector<Iv> prev;
    std::vector<int> prev_owner;
    for (int s = s_lo; s <= s_hi; ++s) {
      std::vector<Iv> cur;
      int run = -1;
      for (int a = a_lo; a <= a_hi; ++a) {
        const bool ok = interiorAt(s, a);
        if (ok && run < 0) { run = a; }
        if (!ok && run >= 0) {
          cur.push_back({run, a - 1});
          run = -1;
        }
      }
      if (run >= 0) { cur.push_back({run, a_hi}); }

      std::vector<int> cur_owner(cur.size(), -1);
      std::vector<int> cur_hits(cur.size(), 0);
      std::vector<int> prev_hits(prev.size(), 0);
      std::vector<int> pair_prev(cur.size(), -1);
      for (size_t ci = 0; ci < cur.size(); ++ci) {
        for (size_t pi = 0; pi < prev.size(); ++pi) {
          if (cur[ci].lo <= prev[pi].hi && prev[pi].lo <= cur[ci].hi) {
            ++cur_hits[ci];
            ++prev_hits[pi];
            pair_prev[ci] = static_cast<int>(pi);
          }
        }
      }
      for (size_t ci = 0; ci < cur.size(); ++ci) {
        const int pi = pair_prev[ci];
        bool cont = (cur_hits[ci] == 1 && pi >= 0 &&
                     prev_hits[static_cast<size_t>(pi)] == 1 &&
                     prev_owner[static_cast<size_t>(pi)] >= 0);
        // 门洞: 这一片可通行宽度只有 doorway 那么窄 -> 在此切单元.
        // 该切片自己会成为一个小单元 (门洞本身仍会被覆盖), 两侧房间各自成单元.
        if (cont && doorway_cells > 0 &&
            (cur[ci].hi - cur[ci].lo) <= doorway_cells) {
          cont = false;
        }
        if (cont) {
          // 凹角/台阶: 区间端点突然跳变 (>2 格) 说明边界在这里拐了个直角.
          // 不切的话 L 形会被当成一个单元 (宽度从 3.25 突增到 5.55), 行长参差.
          const Iv & pv = prev[static_cast<size_t>(pi)];
          if (std::abs(cur[ci].lo - pv.lo) > 2 || std::abs(cur[ci].hi - pv.hi) > 2) {
            cont = false;
          }
        }
        int id = -1;
        if (cont) {
          id = prev_owner[static_cast<size_t>(pi)];
          cells[static_cast<size_t>(id)].iv.push_back(cur[ci]);
        } else {
          DCell c;
          c.s0 = s;
          c.iv.push_back(cur[ci]);
          cells.push_back(c);
          id = static_cast<int>(cells.size()) - 1;
        }
        cur_owner[ci] = id;
        for (int a = cur[ci].lo; a <= cur[ci].hi; ++a) {
          int mx = 0;
          int my = 0;
          cellOf(s, a, mx, my);
          label[index(mx, my)] = id;
        }
      }
      prev = cur;
      prev_owner = cur_owner;
    }
  }

  // ---- 4) 每个单元按**自己的长边**生成 swath ----
  // F2C 的 swath angle 目标是"行数最少", 对矩形单元就是沿长边走.
  // 行线首条内缩 sweep/2, 末条贴边 (与上一条重叠 <= sweep/2, F2C allow_overlap).
  struct Swath { int mx0; int my0; int mx1; int my1; };
  std::vector<std::vector<Swath>> cell_swaths(cells.size());
  for (size_t ci = 0; ci < cells.size(); ++ci) {
    const int want = static_cast<int>(ci);
    int x0 = W;
    int x1 = -1;
    int y0 = H;
    int y1 = -1;
    for (int y = by0; y <= by1; ++y) {
      for (int x = bx0; x <= bx1; ++x) {
        if (label[index(x, y)] != want) { continue; }
        x0 = std::min(x0, x);
        x1 = std::max(x1, x);
        y0 = std::min(y0, y);
        y1 = std::max(y1, y);
      }
    }
    if (x1 < x0 || y1 < y0) { continue; }
    const bool along_x = (x1 - x0) >= (y1 - y0);
    // 车心留边: 行线/端点都要离区域边界留一点, 否则控制超调一步就出界
    // (实测 4x3 区域行末冲到 x=2.62, 边界 2.0, 每次行末都 OUT_OF_AREA 卡住).
    // 清扫盘半径 sweep/2 >= 留边, 所以留边不吃覆盖面积.
    const int margin = std::max(0, static_cast<int>(std::lround(
      std::min(0.5 * config_.sweep_width, 0.15) / res)));
    auto narrow = [&](int lo, int hi) {
      const int a = lo + margin;
      const int b = hi - margin;
      if (b - a < 2) {
        const int mid = (lo + hi) / 2;
        return std::make_pair(mid, mid);
      }
      return std::make_pair(a, b);
    };
    const auto row_range = narrow(along_x ? x0 : y0, along_x ? x1 : y1);
    const auto lane_range = narrow(along_x ? y0 : x0, along_x ? y1 : x1);
    const int row_lo = row_range.first;
    const int row_hi = row_range.second;
    const int lane_lo = lane_range.first;
    const int lane_hi = lane_range.second;
    const int lane_edge_hi = along_x ? y1 : x1;

    std::vector<int> lanes;
    // 第一条就直接放在"留边"上: 清扫盘半径 0.25 > 留边 0.15, 所以覆盖仍然到边界,
    // 而车心不会贴边. (以前从 lane_lo + sweep/2 开始, 加上留边就双重内缩,
    // 4x3 区域底部 0.15m 一条带扫不到, 覆盖率掉到 94.9%.)
    for (int a = lane_lo; a <= lane_hi; a += sweep_cells) {
      lanes.push_back(a);
    }
    // 收尾贴边: 上一条还没够到边界时补一条 (与上一条重叠 <= sweep/2, F2C allow_overlap)
    if (lanes.empty() ||
        (lanes.back() + sweep_cells / 2 < lane_edge_hi && lane_hi > lanes.back())) {
      lanes.push_back(lane_hi);
    }

    std::vector<Swath> sws;
    for (size_t li = 0; li < lanes.size(); ++li) {
      const int a = lanes[li];
      std::vector<Swath> lane_sws;
      int run = -1;
      for (int s = row_lo; s <= row_hi + 1; ++s) {
        const bool ok = (s <= row_hi) &&
          (label[index(along_x ? s : a, along_x ? a : s)] == want);
        if (ok && run < 0) { run = s; }
        if (!ok && run >= 0) {
          Swath sw;
          if (along_x) {
            sw.mx0 = run; sw.my0 = a; sw.mx1 = s - 1; sw.my1 = a;
          } else {
            sw.mx0 = a; sw.my0 = run; sw.mx1 = a; sw.my1 = s - 1;
          }
          lane_sws.push_back(sw);
          run = -1;
        }
      }
      // 同一 lane 上可能有多段 (单元非凸): 先按蛇形定顺序, 再按蛇形定方向
      const bool flip = (li % 2 == 1);
      if (flip) { std::reverse(lane_sws.begin(), lane_sws.end()); }
      for (auto sw : lane_sws) {
        if (flip) {
          std::swap(sw.mx0, sw.mx1);
          std::swap(sw.my0, sw.my1);
        }
        // 还有没覆盖的格子才保留
        bool need = false;
        const int sx0 = std::min(sw.mx0, sw.mx1);
        const int sx1 = std::max(sw.mx0, sw.mx1);
        const int sy0 = std::min(sw.my0, sw.my1);
        const int sy1 = std::max(sw.my0, sw.my1);
        const int dx = (sx1 > sx0) ? 1 : 0;
        const int dy = (sy1 > sy0) ? 1 : 0;
        for (int x = sx0, y = sy0; x <= sx1 && y <= sy1 && !need; x += dx, y += dy) {
          if (cells_[index(x, y)] == CoverageCell::UNCOVERED) { need = true; }
        }
        if (!need) { continue; }
        sws.push_back(sw);
      }
    }
    // 太碎的 swath (< 0.10m) 会让执行端原地抖, 单元里只剩碎段时才保留
    std::vector<Swath> keep;
    std::vector<Swath> tiny;
    for (const auto & sw : sws) {
      const double len = std::hypot(sw.mx1 - sw.mx0, sw.my1 - sw.my0) * res;
      if (len >= 0.10) { keep.push_back(sw); } else { tiny.push_back(sw); }
    }
    if (keep.empty()) { keep = tiny; }
    cell_swaths[ci] = keep;
  }

  // ---- 5) 组装: headland 环 -> 单元最近邻 -> 单元内蛇形 -> 圆弧/原地转连接 ----
  Pose2D prev = pose;
  double prev_heading = pose.theta;
  auto appendLine = [&](const Pose2D & from, const Pose2D & to) {
    const double len = std::hypot(to.x - from.x, to.y - from.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / res)));
    const double theta = std::atan2(to.y - from.y, to.x - from.x);
    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      Pose2D p;
      p.x = from.x + t * (to.x - from.x);
      p.y = from.y + t * (to.y - from.y);
      p.theta = theta;
      route.push_back(p);
    }
  };
  auto appendTurnAndLine = [&](const Pose2D & target) {
    if (std::hypot(target.x - prev.x, target.y - prev.y) <= res) { return; }
    const double bearing = std::atan2(target.y - prev.y, target.x - prev.x);
    Pose2D turn = prev;
    turn.theta = bearing;
    route.push_back(turn);
    appendLine(prev, target);
    prev_heading = bearing;
    prev = target;
  };

  // ---- 转场: 直线能走就直走, 不能走就用栅格 BFS + 视线捷径绕开障碍 ----
  auto cellPassable = [&](int mx, int my) {
    return cellInBounds(mx, my) && passable(mx, my, true);
  };
  auto lineClear = [&](const Pose2D & a, const Pose2D & b) {
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / (0.5 * res))));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      int mx = 0;
      int my = 0;
      if (!worldToCell(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), mx, my)) { return false; }
      if (!cellPassable(mx, my)) { return false; }
    }
    return true;
  };
  auto gridPath = [&](const Pose2D & a, const Pose2D & b, std::vector<Pose2D> & out) {
    int ax = 0;
    int ay = 0;
    int bx = 0;
    int by = 0;
    if (!worldToCell(a.x, a.y, ax, ay) || !worldToCell(b.x, b.y, bx, by)) { return false; }
    if (!cellPassable(ax, ay) || !cellPassable(bx, by)) { return false; }
    std::vector<int> parent(cells_.size(), -1);
    std::queue<int> q;
    const int start = index(ax, ay);
    const int goal = index(bx, by);
    parent[start] = start;
    q.push(start);
    bool found = (start == goal);
    const int ddx[4] = {-1, 1, 0, 0};
    const int ddy[4] = {0, 0, -1, 1};
    while (!q.empty() && !found) {
      const int cur = q.front();
      q.pop();
      const int cx = cur % W;
      const int cy = cur / W;
      for (int k = 0; k < 4; ++k) {
        const int nx = cx + ddx[k];
        const int ny = cy + ddy[k];
        if (!cellInBounds(nx, ny)) { continue; }
        const int ni = index(nx, ny);
        if (parent[ni] >= 0) { continue; }
        if (!passable(nx, ny, true)) { continue; }
        parent[ni] = cur;
        if (ni == goal) {
          found = true;
          break;
        }
        q.push(ni);
      }
    }
    if (!found) { return false; }
    std::vector<Pose2D> raw;
    for (int c = goal; c != start; c = parent[c]) {
      raw.push_back(cellToWorld(c % W, c / W));
    }
    raw.push_back(cellToWorld(ax, ay));
    std::reverse(raw.begin(), raw.end());
    // 视线捷径: 只保留能直连的转折点 (BFS 4-邻接的锯齿会被吃掉)
    out.clear();
    out.push_back(raw.front());
    size_t i = 0;
    while (i + 1 < raw.size()) {
      size_t j = i + 1;
      for (size_t k = raw.size() - 1; k > i + 1; --k) {
        if (lineClear(raw[i], raw[k])) {
          j = k;
          break;
        }
      }
      out.push_back(raw[j]);
      i = j;
    }
    return out.size() >= 2;
  };
  auto appendTransit = [&](const Pose2D & target) {
    if (std::hypot(target.x - prev.x, target.y - prev.y) <= res) { return; }
    if (lineClear(prev, target)) {
      appendTurnAndLine(target);
      return;
    }
    std::vector<Pose2D> pts;
    if (!gridPath(prev, target, pts)) {
      appendTurnAndLine(target);
      return;
    }
    // 起点原地对正, 其余折点直接走 (不插原地转标记, 否则每个折点都要停一下)
    const Pose2D p1 = pts[1];
    const double bearing = std::atan2(p1.y - prev.y, p1.x - prev.x);
    Pose2D turn = prev;
    turn.theta = bearing;
    route.push_back(turn);
    for (size_t i = 1; i < pts.size(); ++i) {
      const Pose2D p = pts[i];
      appendLine(prev, p);
      prev_heading = std::atan2(p.y - prev.y, p.x - prev.x);
      prev = p;
    }
  };

  for (const auto & seg : ring_segs) {
    if (seg.size() < 2) { continue; }
    const Pose2D r0 = cellToWorld(seg.front().first, seg.front().second);
    appendTransit(r0);
    Pose2D cur_p = r0;
    for (size_t k = 1; k < seg.size(); ++k) {
      Pose2D p = cellToWorld(seg[k].first, seg[k].second);
      prev_heading = std::atan2(p.y - cur_p.y, p.x - cur_p.x);
      p.theta = prev_heading;
      route.push_back(p);
      cur_p = p;
    }
    prev = cur_p;
  }

  auto appendArcTurn = [&](const Pose2D & from_pose, const Pose2D & to_pose,
                           double h0) -> bool {
    const double r_max = config_.turn_radius;
    if (!(r_max > 0.0)) { return false; }
    const double dx = to_pose.x - from_pose.x;
    const double dy = to_pose.y - from_pose.y;
    const double along = dx * std::cos(h0) + dy * std::sin(h0);
    const double lat = -dx * std::sin(h0) + dy * std::cos(h0);   // 左正
    if (std::fabs(along) > 0.30) { return false; }
    const double R = 0.5 * std::fabs(lat);
    if (R < 0.10 || R > r_max + 1e-6) { return false; }
    const double s = (lat > 0.0) ? 1.0 : -1.0;   // +1 左转(逆时针) / -1 右转
    const double cx = from_pose.x + s * (-std::sin(h0)) * R;
    const double cy = from_pose.y + s * (std::cos(h0)) * R;
    const double a0 = std::atan2(from_pose.y - cy, from_pose.x - cx);
    const double sweep = s * M_PI;
    const int steps = std::max(6, static_cast<int>(
      std::ceil(std::fabs(sweep) * R / res)));
    // 弧线整段都必须在可通行区内, 否则说明行端没有转弯余量 (多边形贴墙),
    // 退回"原地转 + 直线" —— 不能让车头扫到区域外.
    for (int i = 0; i <= steps; ++i) {
      const double a = a0 + sweep * (static_cast<double>(i) / steps);
      int mx = 0;
      int my = 0;
      if (!worldToCell(cx + R * std::cos(a), cy + R * std::sin(a), mx, my)) { return false; }
      if (!passable(mx, my, true)) { return false; }
    }
    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      const double a = a0 + sweep * t;
      Pose2D p;
      p.x = cx + R * std::cos(a);
      p.y = cy + R * std::sin(a);
      p.theta = std::remainder(a + s * (M_PI / 2.0), 2.0 * M_PI);
      route.push_back(p);
    }
    return true;
  };

  std::vector<size_t> pending;
  for (size_t i = 0; i < cell_swaths.size(); ++i) {
    if (!cell_swaths[i].empty()) { pending.push_back(i); }
  }
  while (!pending.empty()) {
    size_t pick = pending.size();
    double best = std::numeric_limits<double>::max();
    bool reversed = false;
    for (size_t k = 0; k < pending.size(); ++k) {
      const auto & sws = cell_swaths[pending[k]];
      const double d_forward = prev.distanceTo(
        cellToWorld(sws.front().mx0, sws.front().my0));
      const double d_backward = prev.distanceTo(
        cellToWorld(sws.back().mx1, sws.back().my1));
      if (std::min(d_forward, d_backward) < best) {
        best = std::min(d_forward, d_backward);
        pick = k;
        reversed = (d_backward < d_forward);
      }
    }
    if (pick >= pending.size()) { break; }
    const size_t cid = pending[pick];
    pending.erase(pending.begin() + static_cast<long>(pick));

    std::vector<std::pair<Pose2D, Pose2D>> strokes;
    for (const auto & sw : cell_swaths[cid]) {
      strokes.push_back(std::make_pair(cellToWorld(sw.mx0, sw.my0),
                                       cellToWorld(sw.mx1, sw.my1)));
    }
    if (reversed) {
      std::reverse(strokes.begin(), strokes.end());
      for (auto & st : strokes) { std::swap(st.first, st.second); }
    }
    for (const auto & st : strokes) {
      bool arc_done = false;
      if (std::hypot(st.first.x - prev.x, st.first.y - prev.y) > 0.5 * res) {
        arc_done = appendArcTurn(prev, st.first, prev_heading);
      }
      if (arc_done) {
        const Pose2D end = route.back();
        if (std::hypot(st.first.x - end.x, st.first.y - end.y) > 0.5 * res) {
          appendLine(end, st.first);
        }
      } else {
        appendTransit(st.first);
      }
      appendLine(st.first, st.second);
      prev = st.second;
      prev_heading = std::atan2(st.second.y - st.first.y, st.second.x - st.first.x);
    }
  }

  // 末尾补一个"原地转点", 朝向保持 (让最后一段能被 isGoalReached 正确判到)
  if (!route.empty()) {
    Pose2D end = prev;
    end.theta = route.back().theta;
    route.push_back(end);
  }
  return route.size() >= 2;
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
