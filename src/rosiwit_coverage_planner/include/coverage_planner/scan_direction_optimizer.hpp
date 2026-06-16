// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#ifndef COVERAGE_PLANNER__SCAN_DIRECTION_OPTIMIZER_HPP_
#define COVERAGE_PLANNER__SCAN_DIRECTION_OPTIMIZER_HPP_

#include <vector>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"

namespace coverage_planner
{

/**
 * @brief 扫描方向优化配置
 */
struct ScanDirectionConfig
{
    bool enable_pca;                // 是否启用PCA方向检测
    bool enable_mbr;                // 是否启用最小外接矩形
    double aspect_ratio_threshold;  // 长宽比阈值（超过此值使用长边优先）
    double pca_threshold;           // PCA主方向置信度阈值
    bool fallback_to_scanline;      // PCA/MBR失败时是否回退到扫描线统计
    
    // 默认构造函数
    ScanDirectionConfig()
    : enable_pca(true),
      enable_mbr(true),
      aspect_ratio_threshold(2.0),   // 长宽比>2时使用长边优先
      pca_threshold(0.1),            // PCA主方向分量差异阈值
      fallback_to_scanline(true)
    {}
};

/**
 * @brief 扫描方向分析结果
 */
struct ScanDirectionResult
{
    int direction;               // 推荐扫描方向 (0:水平, 1:垂直)
    double confidence;           // 推荐置信度 (0.0-1.0)
    double principal_angle;      // PCA主方向角度 (弧度)
    double aspect_ratio;         // 地图长宽比
    double pca_variance_ratio;   // PCA方差比 (主成分/次成分)
    std::string method_used;     // 使用的方法名称
    
    // 默认构造函数
    ScanDirectionResult()
    : direction(0),
      confidence(0.0),
      principal_angle(0.0),
      aspect_ratio(1.0),
      pca_variance_ratio(1.0),
      method_used("unknown")
    {}
};

/**
 * @brief 扫描方向优化器
 * 
 * 实现长边优先策略，通过PCA主方向检测和最小外接矩形分析，
 * 自动选择最优扫描方向以减少转弯次数。
 */
class ScanDirectionOptimizer
{
public:
    /**
     * @brief 默认构造函数
     */
    ScanDirectionOptimizer();

    /**
     * @brief 析构函数
     */
    ~ScanDirectionOptimizer() = default;

    /**
     * @brief 分析最优扫描方向
     * 
     * 综合使用PCA、最小外接矩形、长宽比等多种方法，
     * 选择最适合的扫描方向。
     * 
     * @param map 输入地图
     * @param config 优化配置
     * @return ScanDirectionResult 分析结果
     */
    ScanDirectionResult analyzeOptimalDirection(
        const nav_msgs::msg::OccupancyGrid & map,
        const ScanDirectionConfig & config = ScanDirectionConfig());

    /**
     * @brief 使用PCA检测主方向
     * 
     * 通过主成分分析检测空闲区域的分布主方向。
     * 
     * @param map 输入地图
     * @return double 主方向角度 (弧度，0表示水平方向)
     */
    double detectPrincipalDirectionPCA(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 计算最小外接矩形
     * 
     * 计算空闲区域的最小外接矩形，获取其长轴方向。
     * 
     * @param map 输入地图
     * @return double 最小外接矩形长轴角度 (弧度)
     */
    double calculateMinBoundingRectAngle(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 计算地图长宽比
     * 
     * 计算空闲区域的长宽比。
     * 
     * @param map 输入地图
     * @return double 长宽比 (>=1.0)
     */
    double calculateAspectRatio(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 根据角度确定扫描方向
     * 
     * 将角度转换为扫描方向（水平或垂直）。
     * 
     * @param angle 角度 (弧度)
     * @return int 扫描方向 (0:水平, 1:垂直)
     */
    static int angleToScanDirection(double angle);

    /**
     * @brief 获取空闲区域的点集
     * 
     * @param map 输入地图
     * @return std::vector<cv::Point2f> 空闲区域点集
     */
    static std::vector<cv::Point2f> getFreeRegionPoints(
        const nav_msgs::msg::OccupancyGrid & map);

    /**
     * @brief 将OccupancyGrid转换为OpenCV Mat
     * 
     * @param map 输入地图
     * @return cv::Mat 二值图像 (0=空闲，255=障碍物)
     */
    static cv::Mat occupancyGridToMat(
        const nav_msgs::msg::OccupancyGrid & map);

private:
    /**
     * @brief 执行PCA分析
     * 
     * @param points 点集
     * @return std::pair<double, double> 主方向角度和方差比
     */
    std::pair<double, double> performPCA(
        const std::vector<cv::Point2f> & points);

    /**
     * @brief 计算点集的最小外接矩形
     * 
     * @param points 点集
     * @return std::pair<double, cv::RotatedRect> 长轴角度和旋转矩形
     */
    std::pair<double, cv::RotatedRect> calculateMinRect(
        const std::vector<cv::Point2f> & points);

    /**
     * @brief 综合多个方法的结果
     * 
     * @param pca_result PCA结果
     * @param mbr_result MBR结果
     * @param aspect_ratio 长宽比
     * @param config 配置参数
     * @return ScanDirectionResult 最终结果
     */
    ScanDirectionResult combineResults(
        const std::pair<double, double> & pca_result,
        const std::pair<double, cv::RotatedRect> & mbr_result,
        double aspect_ratio,
        const ScanDirectionConfig & config);

    /**
     * @brief 计算角度到最近轴方向的距离
     * 
     * @param angle 角度 (弧度)
     * @return double 到水平(0)或垂直(PI/2)的最近距离
     */
    static double angleToAxisDistance(double angle);
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__SCAN_DIRECTION_OPTIMIZER_HPP_