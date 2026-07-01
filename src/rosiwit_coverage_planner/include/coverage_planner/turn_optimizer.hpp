// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__TURN_OPTIMIZER_HPP_
#define COVERAGE_PLANNER__TURN_OPTIMIZER_HPP_

#include <vector>
#include <cmath>
#include <memory>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace coverage_planner
{

/**
 * @brief 转弯点类型
 */
enum class TurnType
{
    NONE,           // 无转弯（直线段）
    SHARP,          // 急转弯 (>90°)
    MEDIUM,         // 中等转弯 (45°-90°)
    GENTLE,         // 缓转弯 (<45°)
    U_TURN,         // U形转弯 (180°)
    SCANLINE_END    // 扫描线末端转弯
};

/**
 * @brief 转弯点信息
 */
struct TurnPoint
{
    size_t index;                   // 在路径中的索引
    TurnType type;                  // 转弯类型
    double angle;                   // 转弯角度 (弧度)
    double angle_change;            // 方向变化量 (弧度)
    geometry_msgs::msg::Point position; // 转弯位置
    
    // 优化相关
    bool can_merge;                 // 是否可合并
    bool can_smooth;                // 是否可平滑
    size_t merge_candidate_index;   // 可合并的转弯点索引
    
    TurnPoint()
    : index(0), type(TurnType::NONE), angle(0.0), angle_change(0.0),
      can_merge(false), can_smooth(false), merge_candidate_index(0)
    {}
};

/**
 * @brief 转弯优化配置
 */
struct TurnOptimizerConfig
{
    // 转弯检测参数
    double angle_threshold;          // 转弯检测角度阈值（弧度）
    double merge_distance_threshold; // 转弯点合并距离阈值（栅格数）
    double merge_angle_threshold;    // 转弯合并角度阈值（弧度）
    
    // 转弯优化策略
    bool enable_merge;               // 是否启用转弯合并
    bool enable_smooth;              // 是否启用转弯平滑
    bool enable_skip_obsolete;       // 是否跳过废弃转弯点
    
    // 平滑参数
    double smooth_radius;            // 平滑半径（米）
    int smooth_samples;              // 平滑采样点数
    
    // 默认构造函数
    TurnOptimizerConfig()
    : angle_threshold(0.1),           // 约5.7度
      merge_distance_threshold(10.0), // 10个栅格
      merge_angle_threshold(0.35),    // 约20度
      enable_merge(true),
      enable_smooth(false),           // 默认关闭平滑（需要更复杂的路径重建）
      enable_skip_obsolete(true),
      smooth_radius(0.3),
      smooth_samples(10)
    {}
};

/**
 * @brief 转弯优化结果
 */
struct TurnOptimizeResult
{
    bool success;                           // 优化是否成功
    std::vector<geometry_msgs::msg::PoseStamped> optimized_path; // 优化后的路径
    int original_turn_count;                // 原始转弯次数
    int optimized_turn_count;               // 优化后转弯次数
    double reduction_rate;                  // 转弯减少率
    
    // 统计信息
    int merged_count;                       // 合并的转弯次数
    int smoothed_count;                     // 平滑的转弯次数
    int skipped_count;                      // 跳过的废弃转弯数
    
    // 转弯点列表
    std::vector<TurnPoint> original_turns;  // 原始转弯点
    std::vector<TurnPoint> optimized_turns; // 优化后转弯点
    
    double optimization_time_ms;            // 优化耗时
    
    TurnOptimizeResult()
    : success(false), original_turn_count(0), optimized_turn_count(0),
      reduction_rate(0.0), merged_count(0), smoothed_count(0),
      skipped_count(0), optimization_time_ms(0.0)
    {}
};

/**
 * @brief 转弯优化器类
 *
 * 实现路径转弯优化功能：
 * - 转弯点检测与分类
 * - 相邻转弯点合并
 * - 路径重建
 */
class TurnOptimizer
{
public:
    /**
     * @brief 默认构造函数
     */
    TurnOptimizer();
    
    /**
     * @brief 析构函数
     */
    ~TurnOptimizer() = default;
    
    /**
     * @brief 优化路径转弯
     *
     * @param path 输入路径
     * @param map 地图（用于碰撞检测）
     * @param config 优化配置
     * @return TurnOptimizeResult 优化结果
     */
    TurnOptimizeResult optimize(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        const nav_msgs::msg::OccupancyGrid & map,
        const TurnOptimizerConfig & config = TurnOptimizerConfig());
    
    /**
     * @brief 计算路径转弯次数
     *
     * @param path 输入路径
     * @param angle_threshold 转弯检测角度阈值
     * @return int 转弯次数
     */
    int countTurns(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double angle_threshold = 0.1) const;
    
    /**
     * @brief 检测路径中的转弯点
     *
     * @param path 输入路径
     * @param angle_threshold 转弯检测角度阈值
     * @return std::vector<TurnPoint> 转弯点列表
     */
    std::vector<TurnPoint> detectTurnPoints(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double angle_threshold = 0.1) const;
    
    /**
     * @brief 重置优化器状态
     */
    void reset();

private:
    // 内部状态
    std::vector<TurnPoint> turn_points_;
    
    /**
     * @brief 计算两点之间的方向角
     *
     * @param p1 起点
     * @param p2 终点
     * @return double 方向角（弧度，范围[0, 2π)）
     */
    double computeDirection(
        const geometry_msgs::msg::Point & p1,
        const geometry_msgs::msg::Point & p2) const;
    
    /**
     * @brief 计算两个方向角之间的角度差
     *
     * @param angle1 第一个方向角
     * @param angle2 第二个方向角
     * @return double 角度差（弧度，范围[0, π]）
     */
    double computeAngleDifference(double angle1, double angle2) const;
    
    /**
     * @brief 分类转弯类型
     *
     * @param angle_change 方向变化量
     * @return TurnType 转弯类型
     */
    TurnType classifyTurn(double angle_change) const;
    
    /**
     * @brief 合并相邻转弯点
     *
     * @param turns 转弯点列表
     * @param config 优化配置
     * @return std::vector<TurnPoint> 合并后的转弯点列表
     */
    std::vector<TurnPoint> mergeTurnPoints(
        const std::vector<TurnPoint> & turns,
        const TurnOptimizerConfig & config) const;
    
    /**
     * @brief 重建路径（跳过合并的转弯点）
     *
     * @param original_path 原始路径
     * @param turns 原始转弯点
     * @param merged_turns 合并后的转弯点
     * @return std::vector<geometry_msgs::msg::PoseStamped> 重建后的路径
     */
    std::vector<geometry_msgs::msg::PoseStamped> rebuildPath(
        const std::vector<geometry_msgs::msg::PoseStamped> & original_path,
        const std::vector<TurnPoint> & turns,
        const std::vector<TurnPoint> & merged_turns) const;
    
    /**
     * @brief 计算两点之间的距离
     */
    double distance(
        const geometry_msgs::msg::Point & p1,
        const geometry_msgs::msg::Point & p2) const
    {
        return std::sqrt(
            std::pow(p1.x - p2.x, 2) +
            std::pow(p1.y - p2.y, 2) +
            std::pow(p1.z - p2.z, 2));
    }
    
    /**
     * @brief 计算路径段的方向角
     *
     * @param path 路径
     * @param index 起始索引
     * @param window_size 窗口大小
     * @return double 方向角
     */
    double computeSegmentDirection(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        size_t index,
        int window_size = 3) const;
    
    /**
     * @brief 插值生成平滑路径
     *
     * @param p1 起点
     * @param p2 终点
     * @param angle1 起点方向
     * @param angle2 终点方向
     * @param num_points 插值点数
     * @return std::vector<geometry_msgs::msg::Point> 插值点列表
     */
    std::vector<geometry_msgs::msg::Point> interpolateSmooth(
        const geometry_msgs::msg::Point & p1,
        const geometry_msgs::msg::Point & p2,
        double angle1,
        double angle2,
        int num_points) const;
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__TURN_OPTIMIZER_HPP_