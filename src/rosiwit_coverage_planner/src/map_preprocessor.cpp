// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/map_preprocessor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace coverage_planner
{

MapPreprocessor::MapPreprocessor()
{
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::preprocess(
    const nav_msgs::msg::OccupancyGrid & map,
    const PreprocessConfig & config)
{
    nav_msgs::msg::OccupancyGrid result = map;

    // 步骤1：去除孤立噪点
    if (config.min_obstacle_size > 0) {
        result = removeIsolatedNoise(result, config.min_obstacle_size);
    }

    // 步骤2：形态学处理
    if (config.enable_morphology) {
        // 开运算去除噪点
        if (config.opening_iterations > 0) {
            result = morphologicalOpening(
                result,
                config.morphology_kernel_size,
                config.opening_iterations);
        }
        // 闭运算填充空洞
        if (config.closing_iterations > 0) {
            result = morphologicalClosing(
                result,
                config.morphology_kernel_size,
                config.closing_iterations);
        }
    }

    // 步骤3：合并邻近障碍物
    if (config.enable_obstacle_merge) {
        result = mergeNearbyObstacles(
            result,
            config.obstacle_merge_distance,
            config.min_obstacle_size);
    }

    // 步骤4：填充小空洞
    if (config.enable_hole_filling && config.max_hole_size > 0) {
        result = fillSmallHoles(result, config.max_hole_size);
    }

    return result;
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::morphologicalOpening(
    const nav_msgs::msg::OccupancyGrid & map,
    int kernel_size,
    int iterations)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 创建结构元素
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    
    // 执行开运算（先腐蚀后膨胀）
    cv::Mat result;
    cv::morphologyEx(mat, result, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), iterations);
    
    // 转换回OccupancyGrid
    return matToOccupancyGrid(result, map);
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::morphologicalClosing(
    const nav_msgs::msg::OccupancyGrid & map,
    int kernel_size,
    int iterations)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 创建结构元素
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    
    // 执行闭运算（先膨胀后腐蚀）
    cv::Mat result;
    cv::morphologyEx(mat, result, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), iterations);
    
    // 转换回OccupancyGrid
    return matToOccupancyGrid(result, map);
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::mergeNearbyObstacles(
    const nav_msgs::msg::OccupancyGrid & map,
    double merge_distance,
    int min_obstacle_size)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 膨胀障碍物以合并邻近区域
    int dilate_size = static_cast<int>(std::ceil(merge_distance / 2.0));
    if (dilate_size > 0) {
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1));
        
        cv::Mat dilated;
        cv::dilate(mat, dilated, kernel);
        
        // 然后腐蚀恢复原尺寸
        cv::Mat eroded;
        cv::erode(dilated, eroded, kernel);
        
        mat = eroded;
    }
    
    // 去除小障碍物
    if (min_obstacle_size > 1) {
        // 查找连通区域
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(
            mat, labels, stats, centroids, 8, CV_32S);
        
        // 创建输出图像
        cv::Mat result = cv::Mat::zeros(mat.size(), mat.type());
        
        // 保留面积大于阈值的区域
        for (int i = 1; i < num_labels; ++i) {  // 跳过背景(0)
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area >= min_obstacle_size) {
                // 保留该区域
                cv::Mat mask = (labels == i);
                result.setTo(255, mask);
            }
        }
        
        mat = result;
    }
    
    // 转换回OccupancyGrid
    return matToOccupancyGrid(mat, map);
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::fillSmallHoles(
    const nav_msgs::msg::OccupancyGrid & map,
    int max_hole_size)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 反转图像：空洞变为白色
    cv::Mat inverted;
    cv::bitwise_not(mat, inverted);
    
    // 查找连通区域（空洞）
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(
        inverted, labels, stats, centroids, 8, CV_32S);
    
    // 创建输出图像
    cv::Mat result = mat.clone();
    
    // 填充小于阈值的空洞
    for (int i = 1; i < num_labels; ++i) {  // 跳过背景(0)
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area <= max_hole_size) {
            // 检查是否真的是空洞（不是背景区域）
            // 空洞应该是被障碍物包围的区域
            int cx = static_cast<int>(centroids.at<double>(i, 0));
            int cy = static_cast<int>(centroids.at<double>(i, 1));
            
            // 检查边界（边界上的区域可能是地图边界，不是空洞）
            bool is_border = (stats.at<int>(i, cv::CC_STAT_LEFT) == 0 ||
                             stats.at<int>(i, cv::CC_STAT_TOP) == 0 ||
                             stats.at<int>(i, cv::CC_STAT_LEFT) + stats.at<int>(i, cv::CC_STAT_WIDTH) >= mat.cols - 1 ||
                             stats.at<int>(i, cv::CC_STAT_TOP) + stats.at<int>(i, cv::CC_STAT_HEIGHT) >= mat.rows - 1);
            
            if (!is_border) {
                // 填充该空洞
                cv::Mat mask = (labels == i);
                result.setTo(255, mask);  // 填充为障碍物
            }
        }
    }
    
    // 转换回OccupancyGrid
    return matToOccupancyGrid(result, map);
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::removeIsolatedNoise(
    const nav_msgs::msg::OccupancyGrid & map,
    int min_neighbors)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 创建输出图像
    cv::Mat result = mat.clone();
    
    // 检查每个障碍物像素
    for (int y = 0; y < mat.rows; ++y) {
        for (int x = 0; x < mat.cols; ++x) {
            if (mat.at<uchar>(y, x) == 255) {  // 障碍物像素
                // 统计邻居障碍物数量
                int neighbor_count = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int ny = y + dy;
                        int nx = x + dx;
                        if (ny >= 0 && ny < mat.rows && nx >= 0 && nx < mat.cols) {
                            if (mat.at<uchar>(ny, nx) == 255) {
                                neighbor_count++;
                            }
                        }
                    }
                }
                
                // 如果邻居数少于阈值，去除该噪点
                if (neighbor_count < min_neighbors) {
                    result.at<uchar>(y, x) = 0;
                }
            }
        }
    }
    
    // 转换回OccupancyGrid
    return matToOccupancyGrid(result, map);
}

std::vector<RegionInfo> MapPreprocessor::extractObstacleRegions(
    const nav_msgs::msg::OccupancyGrid & map)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 分析障碍物连通区域
    return analyzeConnectedComponents(mat, 8);
}

std::vector<RegionInfo> MapPreprocessor::extractFreeRegions(
    const nav_msgs::msg::OccupancyGrid & map)
{
    // 转换为OpenCV格式
    cv::Mat mat = occupancyGridToMat(map);
    
    // 反转图像：空闲区域变为白色
    cv::Mat inverted;
    cv::bitwise_not(mat, inverted);
    
    // 分析空闲区域连通性
    return analyzeConnectedComponents(inverted, 8);
}

cv::Mat MapPreprocessor::occupancyGridToMat(const nav_msgs::msg::OccupancyGrid & map)
{
    // 创建输出图像
    cv::Mat mat(map.info.height, map.info.width, CV_8UC1);
    
    // 转换数据：障碍物(>=50 或 <0) -> 255, 空闲(0-49) -> 0
    for (size_t i = 0; i < map.data.size(); ++i) {
        int8_t value = map.data[i];
        if (value >= 50 || value < 0) {
            mat.data[i] = 255;  // 障碍物
        } else {
            mat.data[i] = 0;    // 空闲
        }
    }
    
    return mat;
}

nav_msgs::msg::OccupancyGrid MapPreprocessor::matToOccupancyGrid(
    const cv::Mat & mat,
    const nav_msgs::msg::OccupancyGrid & original_map)
{
    nav_msgs::msg::OccupancyGrid result;
    
    // 复制元数据
    result.header = original_map.header;
    result.info = original_map.info;
    
    // 调整数据大小
    result.data.resize(mat.rows * mat.cols);
    result.info.width = mat.cols;
    result.info.height = mat.rows;
    
    // 转换数据：255 -> 100 (障碍物), 0 -> 0 (空闲)
    for (int i = 0; i < mat.rows * mat.cols; ++i) {
        if (mat.data[i] == 255) {
            result.data[i] = 100;  // 障碍物
        } else {
            result.data[i] = 0;    // 空闲
        }
    }
    
    return result;
}

std::vector<RegionInfo> MapPreprocessor::analyzeConnectedComponents(
    const cv::Mat & binary_map,
    int connectivity)
{
    std::vector<RegionInfo> regions;
    
    // 使用OpenCV的连通区域分析
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(
        binary_map, labels, stats, centroids, connectivity, CV_32S);
    
    // 提取每个区域的信息
    for (int i = 1; i < num_labels; ++i) {  // 跳过背景(0)
        RegionInfo info;
        info.id = i;
        info.area = stats.at<int>(i, cv::CC_STAT_AREA);
        
        int left = stats.at<int>(i, cv::CC_STAT_LEFT);
        int top = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        info.bounding_box = cv::Rect(left, top, width, height);
        
        info.centroid = cv::Point(
            static_cast<int>(centroids.at<double>(i, 0)),
            static_cast<int>(centroids.at<double>(i, 1)));
        
        // 提取轮廓点
        cv::Mat mask = (labels == i);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            info.contour = contours[0];
        }
        
        regions.push_back(info);
    }
    
    return regions;
}

cv::Mat MapPreprocessor::dilateObstacles(const cv::Mat & mat, int kernel_size)
{
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    
    cv::Mat result;
    cv::dilate(mat, result, kernel);
    return result;
}

cv::Mat MapPreprocessor::erodeObstacles(const cv::Mat & mat, int kernel_size)
{
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(kernel_size, kernel_size));
    
    cv::Mat result;
    cv::erode(mat, result, kernel);
    return result;
}

}  // namespace coverage_planner