// Copyright (c)  2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__ZIGZAG_PLANNER_HPP_
#define COVERAGE_PLANNER__ZIGZAG_PLANNER_HPP_

#include <vector>
#include <string>
#include <chrono>
#include "coverage_planner/i_planner.hpp"
#include "coverage_planner/coverage_utils.hpp"
#include "coverage_planner/map_preprocessor.hpp"
#include "coverage_planner/scan_direction_optimizer.hpp"
#include "coverage_planner/zone_decomposer.hpp"
#include "coverage_planner/turn_optimizer.hpp"

namespace coverage_planner
{

/**
 * @brief 扫描线段结构
 */
struct ScanLine
{
    int y;              // Y坐标（水平扫描）或X坐标（垂直扫描）
    int x_start;        // 起始X坐标
    int x_end;          // 结束X坐标
    bool is_forward;    // 扫描方向

    ScanLine() : y(0), x_start(0), x_end(0), is_forward(true) {}
    ScanLine(int y_coord, int x_s, int x_e, bool forward = true)
        : y(y_coord), x_start(x_s), x_end(x_e), is_forward(forward) {}
};

/**
 * @brief 弓字形路径规划器（Boustrophedon Scanline Algorithm）
 *
 * 实现水平来回扫描模式的全覆盖路径规划：
 * - BSA扫描线分割
 * - 障碍物打断恢复
 * - 自动选择最优扫描方向
 * - P0优化：地图预处理（形态学去噪）
 * - P0优化：长边优先策略（PCA方向检测）
 */
class ZigzagPlanner : public IPlanner
{
public:
    ZigzagPlanner();
    ~ZigzagPlanner() override = default;

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
    std::string getName() const override { return "ZigzagPlanner"; }

    /**
     * @brief 重置规划器状态
     */
    void reset() override;

private:
    // 规划参数
    double robot_radius_;
    double coverage_resolution_;
    bool enable_optimization_;

    // P0优化参数
    bool enable_map_preprocessing_;
    bool enable_pca_direction_;

    // 新增P0优化参数
    bool enable_zone_decomposition_;
    bool enable_turn_optimization_;

    // ZoneDecomposer配置参数
    int zone_min_area_;
    int zone_max_count_;
    double zone_merge_threshold_;

    // TurnOptimizer配置参数
    double turn_angle_threshold_;
    double turn_merge_distance_;

    // 内部状态
    nav_msgs::msg::OccupancyGrid inflated_map_;
    nav_msgs::msg::OccupancyGrid preprocessed_map_;
    std::vector<std::vector<bool>> visited_mask_;

    // 优化器实例
    MapPreprocessor map_preprocessor_;
    ScanDirectionOptimizer scan_optimizer_;

    // 新增优化器实例
    ZoneDecomposer zone_decomposer_;
    TurnOptimizer turn_optimizer_;

    /**
     * @brief 执行扫描线规划
     */
    std::vector<geometry_msgs::msg::PoseStamped> performScanlinePlanning(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & start_grid,
        int scan_direction);

    /**
     * @brief 提取扫描线段
     */
    std::vector<ScanLine> extractScanlines(
        const nav_msgs::msg::OccupancyGrid & map,
        int scan_direction);

    /**
     * @brief 连接扫描线段形成完整路径
     */
    std::vector<Point2D> connectScanlines(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<ScanLine> & scanlines,
        const Point2D & start);

    /**
     * @brief BFS寻路连接两点
     */
    std::vector<Point2D> findConnectionPath(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & start,
        const Point2D & goal);

    /**
     * @brief 找到最近的可达扫描线起点
     */
    Point2D findNearestScanlineStart(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<ScanLine> & scanlines,
        const Point2D & start);

    /**
     * @brief 选择最优扫描方向（传统方法）
     */
    int selectOptimalDirection(const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 选择最优扫描方向（P0优化方法）
     */
    int selectOptimalDirectionOptimized(
        const nav_msgs::msg::OccupancyGrid & map,
        const ScanDirectionConfig & config,
        ScanDirectionResult & result_info);

    /**
     * @brief 简化路径（去除同方向的冗余点）
     */
    std::vector<Point2D> simplifyPath(const std::vector<Point2D> & path);

    /**
     * @brief 将栅格路径转换为世界坐标路径
     */
    std::vector<geometry_msgs::msg::PoseStamped> convertToPath(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<Point2D> & grid_path);

    /**
     * @brief 检查两点之间是否有视线（无障碍物）
     */
    bool hasLineOfSight(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & p1,
        const Point2D & p2);

    /**
     * @brief 预处理地图（P0优化）
     */
    nav_msgs::msg::OccupancyGrid preprocessMap(
        const nav_msgs::msg::OccupancyGrid & map,
        const PlannerConfig & config);

    /**
     * @brief 执行分区规划（P0优化）
     *
     * 将复杂地图分解为多个简单区域，分别规划
     */
    std::vector<geometry_msgs::msg::PoseStamped> performZonePlanning(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config,
        int scan_direction);

    /**
     * @brief 优化路径转弯（P0优化）
     */
    std::vector<geometry_msgs::msg::PoseStamped> optimizeTurns(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        const nav_msgs::msg::OccupancyGrid & map,
        const PlannerConfig & config);
};  // class ZigzagPlanner

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__ZIGZAG_PLANNER_HPP_