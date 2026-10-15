// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__SPIRAL_PLANNER_HPP_
#define COVERAGE_PLANNER__SPIRAL_PLANNER_HPP_

#include <vector>
#include <string>
#include "coverage_planner/i_planner.hpp"
#include "coverage_planner/coverage_utils.hpp"
#include "coverage_planner/turn_optimizer.hpp"

namespace coverage_planner
{

/**
 * @brief 螺旋方向枚举
 */
enum class SpiralDirection
{
    CLOCKWISE,          // 顺时针
    COUNTER_CLOCKWISE,  // 逆时针
};

/**
 * @brief 区域边界结构
 */
struct RegionBoundary
{
    int x_min;
    int x_max;
    int y_min;
    int y_max;

    RegionBoundary() : x_min(0), x_max(0), y_min(0), y_max(0) {}
    RegionBoundary(int xmin, int xmax, int ymin, int ymax)
        : x_min(xmin), x_max(xmax), y_min(ymin), y_max(ymax) {}

    int width() const { return x_max - x_min + 1; }
    int height() const { return y_max - y_min + 1; }

    bool contains(const Point2D & p) const {
        return p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max;
    }
};

/**
 * @brief 回字形路径规划器（Spiral Coverage）
 *
 * 实现从外向内的螺旋覆盖路径规划：
 * - 螺旋生成算法
 * - 非凸区域分解
 * - 降级策略（失败时切换到弓字形）
 */
class SpiralPlanner : public IPlanner
{
public:
    SpiralPlanner();
    ~SpiralPlanner() override = default;

    /**
     * @brief 规划全覆盖路径
     */
    PlannerResult plan(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config) override;

    /**
     * @brief 获取规划器名称
     */
    std::string getName() const override { return "SpiralPlanner"; }

    /**
     * @brief 重置规划器状态
     */
    void reset() override;

private:
    // 规划参数
    double robot_radius_;
    double coverage_resolution_;
    SpiralDirection spiral_direction_;
    bool enable_fallback_;

    // P0优化参数 - 转弯优化
    bool enable_turn_optimization_;
    double turn_angle_threshold_;
    double turn_merge_distance_;

    // 转弯优化器实例
    TurnOptimizer turn_optimizer_;

    // 内部状态
    nav_msgs::msg::OccupancyGrid inflated_map_;
    std::vector<std::vector<bool>> coverage_mask_;

    /**
     * @brief 执行螺旋规划
     */
    std::vector<geometry_msgs::msg::PoseStamped> performSpiralPlanning(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & start_grid,
        const RegionBoundary & region);

    /**
     * @brief 生成螺旋路径
     */
    std::vector<Point2D> generateSpiralPath(
        const nav_msgs::msg::OccupancyGrid & map,
        const RegionBoundary & region,
        const Point2D & start);

    /**
     * @brief 分解非凸区域
     */
    std::vector<RegionBoundary> decomposeRegion(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 检测区域是否为凸区域
     */
    bool isConvexRegion(
        const nav_msgs::msg::OccupancyGrid & map,
        const RegionBoundary & region);

    /**
     * @brief 获取区域的边界
     */
    RegionBoundary getRegionBoundary(const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 沿着边界移动一格
     */
    Point2D moveAlongBoundary(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & current,
        const RegionBoundary & boundary,
        int & direction);

    /**
     * @brief 缩小边界（螺旋向内）
     */
    RegionBoundary shrinkBoundary(
        const RegionBoundary & boundary,
        int steps);

    /**
     * @brief 检查边界是否有效
     */
    bool isValidBoundary(
        const nav_msgs::msg::OccupancyGrid & map,
        const RegionBoundary & boundary);

    /**
     * @brief 连接多个螺旋区域
     */
    std::vector<Point2D> connectSpiralRegions(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<std::vector<Point2D>> & spiral_paths,
        const Point2D & start);

    /**
     * @brief BFS寻路连接两点
     */
    std::vector<Point2D> findPath(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & start,
        const Point2D & goal);

    /**
     * @brief 将栅格点转换为路径点
     */
    std::vector<geometry_msgs::msg::PoseStamped> convertToPath(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<Point2D> & grid_points);

    /**
     * @brief 简化路径
     */
    std::vector<Point2D> simplifyPath(const std::vector<Point2D> & path);

    /**
     * @brief 降级到弓字形算法
     */
    PlannerResult fallbackToZigzag(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config);

    /**
     * @brief 填充螺旋内部未被覆盖的区域
     */
    std::vector<Point2D> fillUncoveredAreas(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<Point2D> & spiral_path);
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__SPIRAL_PLANNER_HPP_