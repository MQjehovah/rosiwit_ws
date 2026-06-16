// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__COVERAGE_UTILS_HPP_
#define COVERAGE_PLANNER__COVERAGE_UTILS_HPP_

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace coverage_planner
{

/**
 * @brief 2D点结构
 */
struct Point2D
{
    int x;
    int y;
    
    Point2D() : x(0), y(0) {}
    Point2D(int px, int py) : x(px), y(py) {}
    
    bool operator==(const Point2D & other) const
    {
        return x == other.x && y == other.y;
    }
    
    bool operator!=(const Point2D & other) const
    {
        return !(*this == other);
    }
    
    double distanceTo(const Point2D & other) const
    {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

/**
 * @brief 覆盖统计信息
 */
struct CoverageStats
{
    int total_cells;           // 总栅格数
    int free_cells;            // 空闲栅格数
    int covered_cells;         // 已覆盖栅格数
    double coverage_rate;      // 覆盖率
    double path_length;        // 路径长度(米)
    int turn_count;            // 转弯次数
};

/**
 * @brief 地图工具类
 */
class MapUtils
{
public:
    /**
     * @brief 检查点是否在地图范围内
     */
    static bool isInBounds(const nav_msgs::msg::OccupancyGrid & map, int x, int y);
    
    /**
     * @brief 检查点是否为障碍物
     */
    static bool isObstacle(const nav_msgs::msg::OccupancyGrid & map, int x, int y, int threshold = 50);
    
    /**
     * @brief 检查点是否为空闲区域
     */
    static bool isFree(const nav_msgs::msg::OccupancyGrid & map, int x, int y, int threshold = 50);
    
    /**
     * @brief 地图膨胀（障碍物膨胀）
     */
    static nav_msgs::msg::OccupancyGrid inflateMap(
        const nav_msgs::msg::OccupancyGrid & map,
        double robot_radius);
    
    /**
     * @brief BFS可达性检查
     */
    static std::vector<Point2D> getReachableCells(
        const nav_msgs::msg::OccupancyGrid & map,
        const Point2D & start);
    
    /**
     * @brief 从世界坐标转换为栅格坐标
     */
    static Point2D worldToGrid(
        const nav_msgs::msg::OccupancyGrid & map,
        double world_x, double world_y);
    
    /**
     * @brief 从栅格坐标转换为世界坐标
     */
    static void gridToWorld(
        const nav_msgs::msg::OccupancyGrid & map,
        int grid_x, int grid_y,
        double & world_x, double & world_y);
    
    /**
     * @brief 计算地图中的空闲区域
     */
    static std::vector<Point2D> getFreeCells(const nav_msgs::msg::OccupancyGrid & map);
    
    /**
     * @brief 计算扫描方向（选择转弯次数最少的方向）
     */
    static int getOptimalScanDirection(const nav_msgs::msg::OccupancyGrid & map);
};

/**
 * @brief 路径工具类
 */
class PathUtils
{
public:
    /**
     * @brief 计算路径长度
     */
    static double calculatePathLength(const std::vector<geometry_msgs::msg::PoseStamped> & path);
    
    /**
     * @brief 计算转弯次数
     */
    static int calculateTurnCount(const std::vector<geometry_msgs::msg::PoseStamped> & path);
    
    /**
     * @brief 路径平滑（简化）
     */
    static std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double resolution);
    
    /**
     * @brief 插值路径点
     */
    static std::vector<geometry_msgs::msg::PoseStamped> interpolatePath(
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double max_distance);
    
    /**
     * @brief 计算两点之间的欧氏距离
     */
    static double distanceBetween(
        const geometry_msgs::msg::PoseStamped & pose1,
        const geometry_msgs::msg::PoseStamped & pose2);
    
    /**
     * @brief 计算两点之间的角度差
     */
    static double angleBetween(
        const geometry_msgs::msg::PoseStamped & pose1,
        const geometry_msgs::msg::PoseStamped & pose2);
    
    /**
     * @brief 创建PoseStamped消息
     */
    static geometry_msgs::msg::PoseStamped createPoseStamped(
        double x, double y, double yaw,
        const std::string & frame_id);
};

/**
 * @brief 覆盖率计算类
 */
class CoverageCalculator
{
public:
    /**
     * @brief 计算覆盖统计信息
     */
    static CoverageStats calculateCoverage(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double robot_radius);
    
    /**
     * @brief 标记路径覆盖的区域
     */
    static std::vector<std::vector<bool>> markCoverage(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<geometry_msgs::msg::PoseStamped> & path,
        double robot_radius);
    
    /**
     * @brief 计算单点覆盖率
     */
    static double calculateCoverageRate(
        const nav_msgs::msg::OccupancyGrid & map,
        const std::vector<std::vector<bool>> & coverage_mask);
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__COVERAGE_UTILS_HPP_