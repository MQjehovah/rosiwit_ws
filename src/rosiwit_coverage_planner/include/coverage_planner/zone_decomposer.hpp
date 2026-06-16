// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__ZONE_DECOMPOSER_HPP_
#define COVERAGE_PLANNER__ZONE_DECOMPOSER_HPP_

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <memory>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/opencv.hpp"

namespace coverage_planner
{

/**
 * @brief 分区类型枚举
 */
enum class ZoneType
{
    RECTANGULAR,      // 矩形区域
    CORRIDOR,         // 走廊型区域
    COMPLEX,          // 复杂区域（需要进一步分解）
    OBSTACLE          // 障碍物区域
};

/**
 * @brief 分区区域结构
 */
struct Zone
{
    int id;                           // 区域ID
    ZoneType type;                    // 区域类型
    cv::Rect bounding_box;            // 外接矩形（栅格坐标）
    std::vector<cv::Point> contour;   // 区域轮廓
    std::vector<cv::Point> free_cells; // 可通行栅格点
    int area;                          // 区域面积（栅格数）
    cv::Point centroid;                // 质心
    
    // 扫描方向优化
    int optimal_scan_direction;        // 最优扫描方向 (0:水平, 1:垂直)
    double principal_angle;            // PCA主方向角度
    
    // 连通性信息
    std::vector<int> neighbor_zones;   // 相邻区域ID列表
    std::vector<cv::Point> connection_points; // 连接点列表
    
    // 默认构造函数
    Zone()
    : id(-1), type(ZoneType::RECTANGULAR), area(0),
      optimal_scan_direction(0), principal_angle(0.0)
    {}
};

/**
 * @brief 区域连接通道结构
 */
struct ConnectionChannel
{
    int zone_a_id;                     // 区域A的ID
    int zone_b_id;                     // 区域B的ID
    cv::Point point_a;                 // 区域A侧的连接点
    cv::Point point_b;                 // 区域B侧的连接点
    double distance;                   // 连接距离
    bool is_reachable;                 // 是否可达
    
    ConnectionChannel()
    : zone_a_id(-1), zone_b_id(-1), distance(0.0), is_reachable(false)
    {}
};

/**
 * @brief 分区配置参数
 */
struct ZoneDecomposerConfig
{
    // 连通域分析参数
    int min_zone_area;                 // 最小区域面积（栅格数）
    bool enable_rectangular_split;     // 是否启用矩形分割
    bool enable_pca_direction;         // 是否启用PCA方向检测
    
    // 区域合并参数
    double merge_threshold;            // 区域合并阈值（面积比）
    int max_zone_count;                // 最大分区数量
    
    // 连接通道检测参数
    int connection_search_radius;      // 连接点搜索半径
    double channel_width_threshold;    // 通道宽度阈值
    
    // 默认构造函数
    ZoneDecomposerConfig()
    : min_zone_area(100),
      enable_rectangular_split(true),
      enable_pca_direction(true),
      merge_threshold(0.2),
      max_zone_count(20),
      connection_search_radius(5),
      channel_width_threshold(2.0)
    {}
};

/**
 * @brief 分区分解结果
 */
struct DecompositionResult
{
    bool success;                      // 分解是否成功
    std::vector<Zone> zones;           // 分区列表
    std::vector<ConnectionChannel> channels; // 连接通道列表
    std::vector<int> visit_order;      // 推荐访问顺序
    int total_free_cells;              // 总空闲栅格数
    double decomposition_time_ms;      // 分解耗时
    std::string error_message;         // 错误信息
    
    DecompositionResult()
    : success(false), total_free_cells(0), decomposition_time_ms(0.0)
    {}
};

/**
 * @brief 分区规划器类
 *
 * 实现复杂地图的分区分解功能：
 * - 连通域分析：识别独立的可通行区域
 * - 矩形分区：将复杂区域分解为简单矩形区域
 * - 连接通道识别：确定区域之间的连接点
 * - 访问顺序优化：TSP算法规划最优访问顺序
 */
class ZoneDecomposer
{
public:
    /**
     * @brief 默认构造函数
     */
    ZoneDecomposer();

    /**
     * @brief 析构函数
     */
    ~ZoneDecomposer() = default;

    /**
     * @brief 分解地图为多个区域
     *
     * @param map 输入地图
     * @param config 分解配置
     * @return DecompositionResult 分解结果
     */
    DecompositionResult decompose(
        const nav_msgs::msg::OccupancyGrid & map,
        const ZoneDecomposerConfig & config = ZoneDecomposerConfig());

    /**
     * @brief 获取区域内的覆盖路径
     *
     * @param map 地图
     * @param zone 区域
     * @param scan_direction 扫描方向 (0:水平, 1:垂直)
     * @return std::vector<cv::Point> 栅格路径点
     */
    std::vector<cv::Point> getZonePath(
        const nav_msgs::msg::OccupancyGrid & map,
        const Zone & zone,
        int scan_direction);

    /**
     * @brief 获取连通域分析结果（用于可视化）
     *
     * @return cv::Mat 连通域标签图
     */
    cv::Mat getConnectedComponentsImage() const { return connected_components_img_; }

    /**
     * @brief 重置分解器状态
     */
    void reset();

private:
    // 内部状态
    cv::Mat connected_components_img_;
    std::vector<std::vector<bool>> visited_mask_;
    
    /**
     * @brief 连通域分析
     *
     * 使用泛洪填充算法识别地图中的所有连通区域
     *
     * @param map 输入地图
     * @param min_area 最小区域面积
     * @return std::vector<Zone> 连通域列表
     */
    std::vector<Zone> findConnectedComponents(
        const nav_msgs::msg::OccupancyGrid & map,
        int min_area);
    
    /**
     * @brief 泛洪填充查找连通区域
     *
     * @param map 地图
     * @param start_x 起始X坐标
     * @param start_y 起始Y坐标
     * @param label 区域标签
     * @param visited 访问标记矩阵
     * @return std::vector<cv::Point> 区域内的所有点
     */
    std::vector<cv::Point> floodFill(
        const nav_msgs::msg::OccupancyGrid & map,
        int start_x,
        int start_y,
        int label,
        std::vector<std::vector<int>> & labels);
    
    /**
     * @brief 计算区域的最优扫描方向
     *
     * 使用PCA分析区域主方向，选择最优扫描方向以减少转弯
     *
     * @param zone 区域
     * @param map 地图
     * @return int 扫描方向 (0:水平, 1:垂直)
     */
    int computeOptimalScanDirection(
        const Zone & zone,
        const nav_msgs::msg::OccupancyGrid & map);
    
    /**
     * @brief 执行PCA分析
     *
     * @param points 点集
     * @param principal_angle 输出：主方向角度
     * @param aspect_ratio 输出：长宽比
     */
    void performPCA(
        const std::vector<cv::Point> & points,
        double & principal_angle,
        double & aspect_ratio);
    
    /**
     * @brief 查找区域连接通道
     *
     * 检测相邻区域之间的连接通道和连接点
     *
     * @param zones 区域列表
     * @param map 地图
     * @param search_radius 搜索半径
     * @return std::vector<ConnectionChannel> 连接通道列表
     */
    std::vector<ConnectionChannel> findConnectionChannels(
        const std::vector<Zone> & zones,
        const nav_msgs::msg::OccupancyGrid & map,
        int search_radius);
    
    /**
     * @brief 计算访问顺序（简化TSP）
     *
     * 使用贪心算法计算区域访问顺序
     *
     * @param zones 区域列表
     * @param channels 连接通道列表
     * @param start_point 起点
     * @return std::vector<int> 访问顺序（区域ID列表）
     */
    std::vector<int> computeVisitOrder(
        const std::vector<Zone> & zones,
        const std::vector<ConnectionChannel> & channels,
        const cv::Point & start_point);
    
    /**
     * @brief 计算两点之间的距离
     */
    double distance(const cv::Point & a, const cv::Point & b) const
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }
    
    /**
     * @brief 检查点是否在地图范围内
     */
    bool isInBounds(
        const nav_msgs::msg::OccupancyGrid & map,
        int x, int y) const
    {
        return x >= 0 && x < static_cast<int>(map.info.width) &&
               y >= 0 && y < static_cast<int>(map.info.height);
    }
    
    /**
     * @brief 检查点是否为空闲区域
     */
    bool isFree(
        const nav_msgs::msg::OccupancyGrid & map,
        int x, int y,
        int threshold = 50) const
    {
        if (!isInBounds(map, x, y)) return false;
        int index = y * map.info.width + x;
        return map.data[index] < threshold;
    }
    
    /**
     * @brief 计算区域的质心
     */
    cv::Point computeCentroid(const std::vector<cv::Point> & points) const;
    
    /**
     * @brief 计算区域的外接矩形
     */
    cv::Rect computeBoundingBox(const std::vector<cv::Point> & points) const;
    
    /**
     * @brief 判断两个区域是否相邻
     */
    bool areNeighbors(
        const Zone & zone_a,
        const Zone & zone_b,
        const nav_msgs::msg::OccupancyGrid & map,
        int search_radius);
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__ZONE_DECOMPOSER_HPP_