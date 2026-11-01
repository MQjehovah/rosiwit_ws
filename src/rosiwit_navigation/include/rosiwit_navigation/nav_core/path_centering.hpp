// ============================================================================
// path_centering.hpp — 把路径推到"离障碍尽量远"的位置 (ESDF 居中)
//
// 为什么需要: hybrid A* 只保证"足迹不碰障碍", 不保证"走在通道中间"。
// 0.75 m 门洞 + 0.66 m 车体 = 单边只有 4.5 cm 余量, 路径偏 7 cm 就必蹭。
//
// 做法(不引入新依赖, 只用本仓库已有的件):
//   * 间隙与梯度来自 core::ObstacleDistance (已建好的 EDT 距离场);
//   * 每个内部路径点受三股力: 间隙不够 -> 沿"距离增大"方向推; 平滑项 -> 拉向邻居中点;
//     参考项 -> 不要离原路径太远;
//   * 单点累计偏移不超过 max_shift (默认 0.15 m), 起点/终点固定, 朝向按切线重算。
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
  /// 期望的足迹最小间隙 (m); 低于它才施加"往外推"的力
  double min_clearance = 0.10;
  double weight_clearance = 1.0;
  double weight_smooth = 0.5;
  double weight_reference = 0.3;
  int iterations = 25;
  /// 单点相对原路径的最大偏移 (m): 防止把路径推进门的另一侧
  double max_shift = 0.15;
  /// 每次迭代的步长系数
  double step = 0.05;
};

/// 沿间隙梯度把路径推向"离障碍更远"的位置。
/// @return 是否发生了有效移动 (移动量 > 1 mm 的点数 > 0)
inline bool centerPathAlongClearance(Path * path, const ObstacleDistance & esdf,
                                     const Footprint & footprint,
                                     const CenteringConfig & cfg)
{
  if (path == nullptr || path->points.size() < 4 || !esdf.ready()) { return false; }
  const std::vector<std::pair<double, double>> samples = sampleFootprint(footprint, 0.05);
  if (samples.empty()) { return false; }

  const size_t n = path->points.size();
  std::vector<Pose2D> orig(n);
  for (size_t i = 0; i < n; ++i) { orig[i] = path->points[i].pose; }

  const double eps = 0.02;
  int moved_points = 0;
  for (int it = 0; it < cfg.iterations; ++it) {
    moved_points = 0;
    for (size_t i = 1; i + 1 < n; ++i) {
      Pose2D & p = path->points[i].pose;
      const double c = esdf.footprintClearance(p, samples);
      if (c < 0.0) { continue; }   // 出窗口(不知道): 不动

      double fx = 0.0, fy = 0.0;
      // 纯梯度上升: 一直朝"间隙更大"的方向走 (居中 = 局部间隙最大处),
      // 而不是"够 min_clearance 就停" —— 后者只会停在离墙 min_clearance 的地方。
      Pose2D pp = p;
      pp.x += eps;
      const double cx1 = esdf.footprintClearance(pp, samples);
      pp.x -= 2.0 * eps;
      const double cx0 = esdf.footprintClearance(pp, samples);
      pp = p;
      pp.y += eps;
      const double cy1 = esdf.footprintClearance(pp, samples);
      pp.y -= 2.0 * eps;
      const double cy0 = esdf.footprintClearance(pp, samples);
      if (cx1 >= 0.0 && cx0 >= 0.0 && cy1 >= 0.0 && cy0 >= 0.0) {
        const double gx = (cx1 - cx0) / (2.0 * eps);
        const double gy = (cy1 - cy0) / (2.0 * eps);
        // 间隙低于期望时加大权重 (靠近墙时更积极)
        const double boost = 1.0 + 2.0 * std::max(0.0, cfg.min_clearance - c) /
                                   std::max(1e-3, cfg.min_clearance);
        fx += cfg.weight_clearance * boost * gx;
        fy += cfg.weight_clearance * boost * gy;
      }
      // 平滑: 拉向邻居中点
      fx += cfg.weight_smooth * (0.5 * (path->points[i - 1].pose.x + path->points[i + 1].pose.x) - p.x);
      fy += cfg.weight_smooth * (0.5 * (path->points[i - 1].pose.y + path->points[i + 1].pose.y) - p.y);
      // 参考: 别跑太远
      fx += cfg.weight_reference * (orig[i].x - p.x);
      fy += cfg.weight_reference * (orig[i].y - p.y);

      double nx = p.x + cfg.step * fx;
      double ny = p.y + cfg.step * fy;
      // 限幅 (相对原始路径)
      const double sx = nx - orig[i].x;
      const double sy = ny - orig[i].y;
      const double s = std::hypot(sx, sy);
      if (s > cfg.max_shift) {
        nx = orig[i].x + sx * cfg.max_shift / s;
        ny = orig[i].y + sy * cfg.max_shift / s;
      }
      if (std::hypot(nx - p.x, ny - p.y) > 1e-4) { ++moved_points; }
      p.x = nx;
      p.y = ny;
    }
    if (moved_points == 0) { break; }
  }

  // 朝向跟随新切线 (起点/终点朝向不动)
  for (size_t i = 1; i + 1 < n; ++i) {
    const Pose2D & a = path->points[i - 1].pose;
    const Pose2D & b = path->points[i + 1].pose;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) > 1e-6) { path->points[i].pose.theta = std::atan2(dy, dx); }
  }
  return moved_points > 0;
}

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__PATH_CENTERING_HPP_
