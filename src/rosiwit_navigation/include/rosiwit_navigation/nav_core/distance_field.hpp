#pragma once
// 精确欧氏距离变换 (Felzenszwalb & Huttenlocher, 2012)。
//
// 为什么要单独一个模块: 之前 Hybrid A* 的膨胀场和 inflatedCostmap 都用
// **4 邻域 BFS**, 得到的是曼哈顿距离, 却被当成欧氏距离直接和"外接圆半径"比较
// (hybrid_astar_planner: obstacle_dist_m_ >= circumscribed_radius_ 就放行)。
// 45° 方向曼哈顿距离是欧氏的 sqrt(2) 倍, 等效阈值从 0.585 m 降到 0.41 m,
// 车体角部最多 13 cm 侵入被判成"任何朝向都安全"; 膨胀等值线也变成菱形,
// 让"斜穿墙角"比"绕直角"便宜 —— 这是窄处切角的几何根源。
// 这里用两遍一维平方距离变换得到精确欧氏距离 (O(N), 无近似误差)。

#include <cstdint>
#include <vector>

namespace rosiwit_navigation
{
namespace core
{

/// 返回每个格中心到最近 lethal 格中心的距离, 单位=格; 没有 lethal 时返回很大的值。
/// lethal 与返回值都按 row-major (idx = y * width + x)。
std::vector<float> euclideanDistanceCells(
    const std::vector<uint8_t> & lethal, unsigned int width, unsigned int height);

}  // namespace core
}  // namespace rosiwit_navigation
