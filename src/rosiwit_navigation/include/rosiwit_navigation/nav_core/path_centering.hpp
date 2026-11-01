// ============================================================================
// path_centering.hpp — 把路径推到"离障碍尽量远"的位置 (ESDF 居中)
//
// 为什么需要: hybrid A* 只保证"足迹不碰障碍", 不保证"走在通道中间"。
// 0.75 m 门洞 + 0.66 m 车体 = 单边只有 4.5 cm 余量, 路径偏 7 cm 就必蹭。
//
// 为什么不用"逐点梯度上升" (v1 的做法):
//   距离场是 0.05 m 栅格, 梯度在格子边界上量化跳变; 每个点独立迭代 25 次后,
//   相邻点会被推到互相矛盾的位置 —— 实测门洞段相邻点 x 在 -0.08/+0.09 之间
//   来回跳 (间距只有 4 cm), 控制器只能跟着甩头 (航向偏 27°), 比不居中更糟。
//
// 现在的做法 (确定、单调、平滑):
//   1. 只处理"间隙不足"的点 (clearance < min_clearance), 开阔处一律不动;
//   2. 沿该点的**法向**做一维扫描 d ∈ [-max_shift, +max_shift], 取
//      (clearance - tie_break*|d|) 最大的 d —— 间隙平坦时优先少移动;
//   3. 对整条偏移序列做多次 [0.25, 0.5, 0.25] 平滑, 并限制相邻点的偏移差
//      (max_step), 从根本上消除锯齿;
//   4. 起点/终点固定, 朝向按新切线重算。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__PATH_CENTERING_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__PATH_CENTERING_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include "rosiwit_navigation/nav_core/footprint.hpp"
#include "rosiwit_navigation/nav_core/obstacle_distance.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

struct CenteringConfig
{
  /// 足迹间隙低于它才移动 (m): 开阔处不动, 避免无意义的路径漂移
  double min_clearance = 0.10;
  /// 单点相对原路径的最大横向偏移 (m): 防止把路径推进门的另一侧
  double max_shift = 0.15;
  /// 法向扫描步长 (m)
  double scan_step = 0.02;
  /// 相邻路径点允许的偏移差 (m): 平滑后仍超限就按斜坡压缩, 保证无锯齿
  double max_step = 0.03;
  /// [0.25,0.5,0.25] 平滑次数
  int smooth_passes = 8;
  /// 间隙"足够好"的容差 (m): 间隙在最大值的这个范围内就选移动最少的偏移
  double clearance_slack = 0.01;
};

namespace detail
{
inline double wrapPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

/// 相邻两段之间的最大折角 (度): 用来量化"路径是否锯齿"
inline double pathMaxTurnDegrees(const Path & path)
{
  double worst = 0.0;
  for (size_t i = 1; i + 1 < path.points.size(); ++i) {
    const auto & a = path.points[i - 1].pose;
    const auto & b = path.points[i].pose;
    const auto & c = path.points[i + 1].pose;
    const double a1 = std::atan2(b.y - a.y, b.x - a.x);
    const double a2 = std::atan2(c.y - b.y, c.x - b.x);
    worst = std::max(worst, std::fabs(wrapPi(a2 - a1)) * 180.0 / M_PI);
  }
  return worst;
}
}  // namespace detail

/// 沿间隙把路径推向"离障碍更远"的位置。
/// @return 是否发生了有效移动 (移动量 > 1 mm 的点数 > 0)
inline bool centerPathAlongClearance(Path * path, const ObstacleDistance & esdf,
                                     const Footprint & footprint,
                                     const CenteringConfig & cfg)
{
  if (path == nullptr || path->points.size() < 4 || !esdf.ready()) { return false; }
  const std::vector<std::pair<double, double>> samples = sampleFootprint(footprint, 0.05);
  if (samples.empty()) { return false; }

  const size_t n = path->points.size();
  const double scan = std::max(0.005, cfg.scan_step);
  const double max_shift = std::max(0.0, cfg.max_shift);
  if (max_shift < 1e-3) { return false; }

  std::vector<double> shift(n, 0.0);
  std::vector<std::pair<double, double>> normal(n, {0.0, 0.0});
  std::vector<bool> usable(n, false);

  for (size_t i = 1; i + 1 < n; ++i) {
    const auto & a = path->points[i - 1].pose;
    const auto & b = path->points[i + 1].pose;
    const double tx = b.x - a.x;
    const double ty = b.y - a.y;
    const double len = std::hypot(tx, ty);
    if (len < 1e-6) { continue; }
    // 法向 (前进方向左手侧)
    normal[i] = {-ty / len, tx / len};
    usable[i] = true;

    Pose2D here = path->points[i].pose;
    if (esdf.footprintClearance(here, samples) < 0.0) { continue; }   // 出窗口: 不动
    if (esdf.footprintClearance(here, samples) >= cfg.min_clearance) { continue; }  // 够宽: 不动

    double best_c = -1e18;
    double best_d = 0.0;
    // 目标函数用**中心点**距离场沿法向的极值 (= 通道几何中轴), 而不是足迹最小
    // 间隙: 后者是 min over 采样点, 在 0.05m 栅格上是阶梯函数, 极值位置会被
    // 量化到 ±3cm 之外。足迹间隙只用来做"要不要动"的门槛。
    std::vector<std::pair<double, double>> cand;   // (d, center distance)
    for (double d = -max_shift; d <= max_shift + 1e-9; d += scan) {
      const double qx = here.x + d * normal[i].first;
      const double qy = here.y + d * normal[i].second;
      const double c = esdf.distanceAt(qx, qy);
      if (c < 0.0) { continue; }
      cand.emplace_back(d, c);
      best_c = std::max(best_c, c);
    }
    if (cand.empty()) { continue; }
    // 取"间隙接近最大"的候选区间的**中点**: 距离场在两个格心之间是平的,
    // 若用"移动最少"规则会偏向进入时的那一侧, 残留固定偏差 (实测 12cm 偏
    // 只修到 3cm)。取中点则对称回到几何中心。
    double d_lo = 0.0, d_hi = 0.0;
    bool have = false;
    for (const auto & cd : cand) {
      if (cd.second < best_c - cfg.clearance_slack) { continue; }
      if (!have) {
        d_lo = d_hi = cd.first;
        have = true;
      } else {
        d_lo = std::min(d_lo, cd.first);
        d_hi = std::max(d_hi, cd.first);
      }
    }
    if (have) { shift[i] = 0.5 * (d_lo + d_hi); }
  }

  // 端点固定: 从两端 3 个点内线性过渡到 0, 保证拼接处连续
  const size_t ramp = std::min<size_t>(3, n / 4);
  for (size_t k = 0; k <= ramp && k < n; ++k) {
    const double w = static_cast<double>(k) / static_cast<double>(ramp + 1);
    shift[k] *= w;
    shift[n - 1 - k] *= w;
  }
  shift.front() = 0.0;
  shift.back() = 0.0;

  // 沿路径平滑 (消除栅格量化带来的锯齿)
  for (int p = 0; p < std::max(0, cfg.smooth_passes); ++p) {
    std::vector<double> next = shift;
    for (size_t i = 1; i + 1 < n; ++i) {
      if (!usable[i]) { continue; }
      next[i] = 0.25 * shift[i - 1] + 0.5 * shift[i] + 0.25 * shift[i + 1];
    }
    shift.swap(next);
  }
  // 斜率限幅: 平滑后相邻差仍超过 max_step 就按斜坡压缩 (保证不存在跳变)
  if (cfg.max_step > 0.0) {
    for (size_t i = 1; i < n; ++i) {
      shift[i] = std::min(shift[i], shift[i - 1] + cfg.max_step);
      shift[i] = std::max(shift[i], shift[i - 1] - cfg.max_step);
    }
    for (size_t i = n - 1; i > 0; --i) {
      shift[i - 1] = std::min(shift[i - 1], shift[i] + cfg.max_step);
      shift[i - 1] = std::max(shift[i - 1], shift[i] - cfg.max_step);
    }
  }

  int moved = 0;
  for (size_t i = 1; i + 1 < n; ++i) {
    const double d = shift[i];
    if (std::fabs(d) < 1e-3) { continue; }
    path->points[i].pose.x += d * normal[i].first;
    path->points[i].pose.y += d * normal[i].second;
    ++moved;
  }
  if (moved == 0) { return false; }

  // 朝向跟随新切线 (起点/终点朝向不动)
  for (size_t i = 1; i + 1 < n; ++i) {
    const auto & a = path->points[i - 1].pose;
    const auto & b = path->points[i + 1].pose;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) > 1e-6) { path->points[i].pose.theta = std::atan2(dy, dx); }
  }
  return moved > 0;
}

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__PATH_CENTERING_HPP_
