// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__I_PLANNER_HPP_
#define COVERAGE_PLANNER__I_PLANNER_HPP_

#include <vector>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace coverage_planner
{

/**
 * @brief 规划结果结构体
 */
struct PlannerResult
{
    bool success;                                          // 规划是否成功
    std::vector<geometry_msgs::msg::PoseStamped> path;    // 规划路径
    double coverage_rate;                                  // 覆盖率 (0.0-1.0)
    double path_length;                                    // 路径长度 (米)
    int turn_count;                                        // 转弯次数
    std::string error_message;                             // 错误信息

    // 扩展统计信息（用于优化效果分析）
    double planning_time_ms;                               // 规划耗时 (毫秒)
    int original_turn_count;                               // 优化前的转弯次数（用于对比）
    double principal_angle;                                // PCA主方向角度
    double aspect_ratio;                                   // 地图长宽比
    std::string direction_method;                          // 方向选择方法
};

/**
 * @brief 规划器配置参数
 */
struct PlannerConfig
{
    // === 基础参数 ===
    double robot_radius;           // 机器人半径 (米)
    double coverage_resolution;    // 覆盖路径分辨率 (米)
    double inflation_radius;        // 障碍物膨胀半径 (米)
    bool enable_optimization;       // 是否启用路径优化
    int direction_optimization;     // 扫描方向优化
                                  // 0: 水平, 1: 垂直, 2: 扫描线统计
                                  // 3: PCA分析, 4: 长边优先(综合)

    // === P0优化参数 - 地图预处理 ===
    bool enable_map_preprocessing;     // 是否启用地图预处理
    int morphology_kernel_size;         // 形态学核大小
    int opening_iterations;             // 开运算迭代次数
    int closing_iterations;             // 闭运算迭代次数
    int min_obstacle_size;              // 最小障碍物尺寸（去噪）
    int max_hole_size;                  // 最大空洞尺寸（填充）
    double obstacle_merge_distance;     // 障碍物合并距离

    // === P0优化参数 - 扫描方向 ===
    bool enable_pca_direction;          // 是否启用PCA方向检测
    bool enable_mbr_analysis;           // 是否启用最小外接矩形分析
    double aspect_ratio_threshold;      // 长宽比阈值
    double pca_confidence_threshold;    // PCA置信度阈值
    bool fallback_to_scanline;          // PCA失败时回退到扫描线统计

    // === 性能参数 ===
    int max_planning_time_ms;           // 最大规划时间限制
    bool enable_statistics_output;      // 是否输出详细统计

    // === P0优化参数 - 分区规划 ===
    bool enable_zone_decomposition;     // 是否启用分区规划
    int zone_min_area;                  // 最小区域面积（栅格数）
    int zone_max_count;                 // 最大分区数量
    double zone_merge_threshold;        // 区域合并阈值

    // === P0优化参数 - 转弯优化 ===
    bool enable_turn_optimization;      // 是否启用转弯优化
    double turn_angle_threshold;         // 转弯检测角度阈值（弧度）
    double turn_merge_distance;          // 转弯合并距离阈值（栅格数）

    // 默认构造函数
    PlannerConfig()
    : robot_radius(0.3),
      coverage_resolution(0.1),
      inflation_radius(0.25),
      enable_optimization(true),
      direction_optimization(4),

      // 地图预处理默认值
      enable_map_preprocessing(true),
      morphology_kernel_size(3),
      opening_iterations(1),
      closing_iterations(1),
      min_obstacle_size(3),
      max_hole_size(10),
      obstacle_merge_distance(2.0),

      // 扫描方向默认值
      enable_pca_direction(true),
      enable_mbr_analysis(true),
      aspect_ratio_threshold(2.0),
      pca_confidence_threshold(0.6),
      fallback_to_scanline(true),

      // 性能默认值
      max_planning_time_ms(3000),
      enable_statistics_output(true),

      // 分区规划默认值
      enable_zone_decomposition(false),
      zone_min_area(100),
      zone_max_count(20),
      zone_merge_threshold(0.2),

      // 转弯优化默认值
      enable_turn_optimization(false),
      turn_angle_threshold(0.1),
      turn_merge_distance(10.0)
    {}
};

/**
 * @brief 规划器抽象接口
 *
 * 定义全覆盖路径规划器的统一接口，支持多种算法实现（策略模式）
 */
class IPlanner
{
public:
    virtual ~IPlanner() = default;

    /**
     * @brief 规划全覆盖路径
     *
     * @param map 栅格地图
     * @param start_pose 起始位姿
     * @param config 规划配置参数
     * @return PlannerResult 规划结果
     */
    virtual PlannerResult plan(
        const nav_msgs::msg::OccupancyGrid & map,
        const geometry_msgs::msg::Pose & start_pose,
        const PlannerConfig & config) = 0;

    /**
     * @brief 获取规划器名称
     *
     * @return std::string 规划器名称
     */
    virtual std::string getName() const = 0;

    /**
     * @brief 重置规划器状态
     */
    virtual void reset() = 0;
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__I_PLANNER_HPP_