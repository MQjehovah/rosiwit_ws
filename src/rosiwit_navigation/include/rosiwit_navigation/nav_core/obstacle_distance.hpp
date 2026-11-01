// ============================================================================
// obstacle_distance.hpp — 局部代价地图上的"到最近障碍的距离"查询
//
// TEB 的障碍项需要"最近障碍点的距离 + 方向"。逐个障碍物遍历太慢也不准 (局部图里
// 没有障碍物列表), 所以用现有精确欧氏距离变换 (core::euclideanDistanceCells)
// 建一张距离场, 再按格双线性插值。
//
// 语义约定 (很重要):
//   * **窗口外返回 -1**, 绝不用一个很大的数假装"很安全" —— 调用方必须区分
//     "知道很远" 与 "不知道";
//   * **unknown(-1) 也算障碍**: 建图不全时贴着未知区走, 与"贴着墙走"同样危险。
// ============================================================================
#ifndef ROSIWIT_NAVIGATION__NAV_CORE__OBSTACLE_DISTANCE_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__OBSTACLE_DISTANCE_HPP_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "rosiwit_navigation/nav_core/distance_field.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

class ObstacleDistance
{
public:
  /// 用一张局部代价地图重建距离场 (格代价 >=100 或 <0 都当障碍)
  void update(const CostmapGrid & grid)
  {
    ready_ = false;
    dist_m_.clear();
    w_ = static_cast<int>(grid.info.width);
    h_ = static_cast<int>(grid.info.height);
    res_ = grid.info.resolution;
    ox_ = grid.info.origin.position.x;
    oy_ = grid.info.origin.position.y;
    if (w_ <= 0 || h_ <= 0 || res_ <= 0.0 ||
        grid.data.size() != static_cast<size_t>(w_) * static_cast<size_t>(h_)) {
      return;
    }
    std::vector<uint8_t> lethal(grid.data.size(), 0);
    for (size_t i = 0; i < grid.data.size(); ++i) {
      const int v = static_cast<int>(grid.data[i]);
      lethal[i] = (v >= 100 || v < 0) ? 1 : 0;
    }
    const std::vector<float> cells = euclideanDistanceCells(lethal, static_cast<unsigned>(w_),
                                                            static_cast<unsigned>(h_));
    if (cells.size() != grid.data.size()) { return; }
    dist_m_.resize(cells.size());
    for (size_t i = 0; i < cells.size(); ++i) {
      dist_m_[i] = static_cast<float>(cells[i] * res_);
    }
    ready_ = true;
  }

  bool ready() const { return ready_; }
  bool empty() const { return !ready_; }

  bool inside(double x, double y) const
  {
    if (!ready_) { return false; }
    const double fx = (x - ox_) / res_ - 0.5;
    const double fy = (y - oy_) / res_ - 0.5;
    return fx >= -0.5 && fy >= -0.5 &&
           fx <= static_cast<double>(w_) - 0.5 && fy <= static_cast<double>(h_) - 0.5;
  }

  /// 到最近障碍的距离 (米); 窗口外返回 -1
  double distanceAt(double x, double y) const
  {
    if (!ready_ || !inside(x, y)) { return -1.0; }
    const double fx = (x - ox_) / res_ - 0.5;
    const double fy = (y - oy_) / res_ - 0.5;
    const int x0 = std::max(0, std::min(w_ - 1, static_cast<int>(std::floor(fx))));
    const int y0 = std::max(0, std::min(h_ - 1, static_cast<int>(std::floor(fy))));
    const int x1 = std::min(w_ - 1, x0 + 1);
    const int y1 = std::min(h_ - 1, y0 + 1);
    const double tx = std::max(0.0, std::min(1.0, fx - x0));
    const double ty = std::max(0.0, std::min(1.0, fy - y0));
    const double d00 = dist_m_[static_cast<size_t>(y0) * w_ + x0];
    const double d10 = dist_m_[static_cast<size_t>(y0) * w_ + x1];
    const double d01 = dist_m_[static_cast<size_t>(y1) * w_ + x0];
    const double d11 = dist_m_[static_cast<size_t>(y1) * w_ + x1];
    return (1 - ty) * ((1 - tx) * d00 + tx * d10) + ty * ((1 - tx) * d01 + tx * d11);
  }

  /// 车体足迹到最近障碍的最小距离 (米); 任一点出窗口或未就绪 -> -1
  double footprintClearance(const Pose2D & pose,
                            const std::vector<std::pair<double, double>> & samples) const
  {
    if (!ready_) { return -1.0; }
    if (samples.empty()) { return distanceAt(pose.x, pose.y); }
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    double best = 1e9;
    for (const auto & q : samples) {
      const double wx = pose.x + c * q.first - s * q.second;
      const double wy = pose.y + s * q.first + c * q.second;
      const double d = distanceAt(wx, wy);
      if (d < 0.0) { return -1.0; }
      if (d < best) { best = d; }
    }
    return best;
  }

private:
  bool ready_ = false;
  int w_ = 0;
  int h_ = 0;
  double res_ = 0.0;
  double ox_ = 0.0;
  double oy_ = 0.0;
  std::vector<float> dist_m_;   ///< 每格中心到最近障碍的距离 (米)
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__OBSTACLE_DISTANCE_HPP_
