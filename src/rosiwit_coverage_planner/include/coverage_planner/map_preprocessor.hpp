// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__MAP_PREPROCESSOR_HPP_
#define COVERAGE_PLANNER__MAP_PREPROCESSOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace coverage_planner
{

/**
 * @brief 预处理配置参数
 */
struct PreprocessConfig
{
    bool enable_morphology;      // 是否启用形态学处理
    int morphology_kernel_size;   // 形态学核大小
    int opening_iterations;       // 开运算迭代次数
    int closing_iterations;      // 闭运算迭代次数
    
    bool enable_obstacle_merge;  // 是否启用障碍物合并
    double obstacle_merge_distance; // 障碍物合并距离阈值（栅格数）
    int min_obstacle_size;       // 最小障碍物尺寸（小于此值的噪点将被去除）
    
    bool enable_hole_filling;    // 是否启用空洞填充
    int max_hole_size;           // 最大空洞尺寸（小于此值的空洞将被填充）
    
    // 默认构造函数
    PreprocessConfig()
    : enable_morphology(true),
      morphology_kernel_size(3),
      opening_iterations(1),
      closing_iterations(1),
      enable_obstacle_merge(true),
      obstacle_merge_distance(3.0),
      min_obstacle_size(2),
      enable_hole_filling(true),
      max_hole_size(5)
    {}
};

/**
 * @brief 区域信息结构
 */
struct RegionInfo
{
    int id;                      // 区域ID
    int area;                    // 区域面积
    cv::Rect bounding_box;       // 外接矩形
    cv::Point centroid;          // 质心
    std::vector<cv::Point> contour; // 轮廓点
};

/**
 * @brief 地图预处理类
 * 
 * 提供地图预处理功能，包括：
 * - 形态学处理（开运算去噪点，闭运算填充空洞）
 * - 障碍物简化与合并
 * - 区域分割与连通性分析
 */
class MapPreprocessor
{
public:
    /**
     * @brief 默认构造函数
     */
    MapPreprocessor();

    /**
     * @brief 析构函数
     */
    ~MapPreprocessor() = default;

    /**
     * @brief 预处理地图
     * 
     * @param map 输入地图
     * @param config 预处理配置
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid preprocess(
        const nav_msgs::msg::OccupancyGrid & map,
        const PreprocessConfig & config = PreprocessConfig());

    /**
     * @brief 形态学开运算（去除噪点）
     * 
     * @param map 输入地图
     * @param kernel_size 核大小
     * @param iterations 迭代次数
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid morphologicalOpening(
        const nav_msgs::msg::OccupancyGrid & map,
        int kernel_size = 3,
        int iterations = 1);

    /**
     * @brief 形态学闭运算（填充空洞）
     * 
     * @param map 输入地图
     * @param kernel_size 核大小
     * @param iterations 迭代次数
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid morphologicalClosing(
        const nav_msgs::msg::OccupancyGrid & map,
        int kernel_size = 3,
        int iterations = 1);

    /**
     * @brief 合并邻近障碍物
     * 
     * @param map 输入地图
     * @param merge_distance 合并距离阈值（栅格数）
     * @param min_obstacle_size 最小障碍物尺寸
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid mergeNearbyObstacles(
        const nav_msgs::msg::OccupancyGrid & map,
        double merge_distance = 3.0,
        int min_obstacle_size = 2);

    /**
     * @brief 填充小空洞
     * 
     * @param map 输入地图
     * @param max_hole_size 最大空洞尺寸
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid fillSmallHoles(
        const nav_msgs::msg::OccupancyGrid & map,
        int max_hole_size = 5);

    /**
     * @brief 去除孤立噪点
     * 
     * @param map 输入地图
     * @param min_neighbors 最小邻居数（少于此值视为噪点）
     * @return nav_msgs::msg::OccupancyGrid 处理后的地图
     */
    nav_msgs::msg::OccupancyGrid removeIsolatedNoise(
        const nav_msgs::msg::OccupancyGrid & map,
        int min_neighbors = 2);

    /**
     * @brief 提取障碍物轮廓
     * 
     * @param map 输入地图
     * @return std::vector<RegionInfo> 障碍物区域信息列表
     */
    std::vector<RegionInfo> extractObstacleRegions(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 提取空闲区域
     * 
     * @param map 输入地图
     * @return std::vector<RegionInfo> 空闲区域信息列表
     */
    std::vector<RegionInfo> extractFreeRegions(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 将OccupancyGrid转换为OpenCV Mat
     * 
     * @param map 输入地图
     * @return cv::Mat OpenCV图像（0=空闲，255=障碍物）
     */
    static cv::Mat occupancyGridToMat(const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 将OpenCV Mat转换为OccupancyGrid
     * 
     * @param mat OpenCV图像（0=空闲，255=障碍物）
     * @param original_map 原始地图（用于获取元数据）
     * @return nav_msgs::msg::OccupancyGrid 输出地图
     */
    static nav_msgs::msg::OccupancyGrid matToOccupancyGrid(
        const cv::Mat & mat,
        const nav_msgs::msg::OccupancyGrid & original_map);

private:
    /**
     * @brief 分析连通区域
     * 
     * @param binary_map 二值图像
     * @param connectivity 连通性（4或8）
     * @return std::vector<RegionInfo> 区域信息列表
     */
    std::vector<RegionInfo> analyzeConnectedComponents(
        const cv::Mat & binary_map,
        int connectivity = 8);

    /**
     * @brief 膨胀障碍物
     * 
     * @param mat 输入图像
     * @param kernel_size 核大小
     * @return cv::Mat 处理后的图像
     */
    cv::Mat dilateObstacles(const cv::Mat & mat, int kernel_size);

    /**
     * @brief 腐蚀障碍物
     * 
     * @param mat 输入图像
     * @param kernel_size 核大小
     * @return cv::Mat 处理后的图像
     */
    cv::Mat erodeObstacles(const cv::Mat & mat, int kernel_size);
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__MAP_PREPROCESSOR_HPP_