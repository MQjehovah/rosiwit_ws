// ============================================================
// rosiwit_navigation - 路径平滑器
// ============================================================

#ifndef ROSIWIT_NAVIGATION__PLANNERS__PATH_SMOOTHER_HPP_
#define ROSIWIT_NAVIGATION__PLANNERS__PATH_SMOOTHER_HPP_

#include <vector>
#include <cmath>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace rosiwit_navigation
{
namespace planners
{

/// 平滑器配置
struct SmootherConfig
{
    int max_iterations = 100;       // 最大迭代次数
    double alpha_start = 0.5;       // 初始平滑率
    double alpha_end = 0.1;         // 最终平滑率（逐渐减小以保持路径形状）
};

/**
 * @brief 对路径应用梯度下降平滑
 * @param raw_path 原始路径（如 A* 输出的栅格路径）
 * @param costmap 代价地图（用于碰撞检测，可为 nullptr）
 * @param config 平滑器配置
 * @return 平滑后的路径
 *
 * 算法：对每个内点，施加指向相邻点中点的拉力，
 * 多次迭代后路径趋于平滑。若平滑后进入障碍物则回退。
 */
nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & raw_path,
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap,
    const SmootherConfig & config = SmootherConfig());

}  // namespace planners
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__PLANNERS__PATH_SMOOTHER_HPP_
