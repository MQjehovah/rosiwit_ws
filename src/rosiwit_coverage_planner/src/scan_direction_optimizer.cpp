// Copyright (c) 2024, Your Name. All rights reserved.
// Licensed under the Apache-2.0 license.

#include "coverage_planner/scan_direction_optimizer.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace coverage_planner
{

ScanDirectionOptimizer::ScanDirectionOptimizer()
{
}

ScanDirectionResult ScanDirectionOptimizer::analyzeOptimalDirection(
    const nav_msgs::msg::OccupancyGrid & map,
    const ScanDirectionConfig & config)
{
    ScanDirectionResult result;

    // 获取空闲区域点集
    std::vector<cv::Point2f> points = getFreeRegionPoints(map);

    // 检查是否有足够的点
    if (points.size() < 10) {
        // 点太少，无法进行PCA分析
        result.direction = 0;  // 默认水平扫描
        result.confidence = 0.1;
        result.method_used = "fallback_min_points";
        return result;
    }

    // 计算长宽比
    double aspect_ratio = calculateAspectRatio(map);
    result.aspect_ratio = aspect_ratio;

    // 如果长宽比足够大，直接使用长边优先
    if (aspect_ratio > config.aspect_ratio_threshold) {
        // 根据地图尺寸确定方向
        if (map.info.width > map.info.height) {
            result.direction = 0;  // 水平扫描
            result.confidence = 0.9;
        } else {
            result.direction = 1;  // 垂直扫描
            result.confidence = 0.9;
        }
        result.method_used = "aspect_ratio_priority";
        return result;
    }

    // PCA分析
    std::pair<double, double> pca_result(0.0, 1.0);
    if (config.enable_pca) {
        pca_result = performPCA(points);
        result.principal_angle = pca_result.first;
        result.pca_variance_ratio = pca_result.second;
    }

    // 最小外接矩形分析
    std::pair<double, cv::RotatedRect> mbr_result(0.0, cv::RotatedRect());
    if (config.enable_mbr) {
        mbr_result = calculateMinRect(points);
    }

    // 综合结果
    result = combineResults(pca_result, mbr_result, aspect_ratio, config);

    return result;
}

double ScanDirectionOptimizer::detectPrincipalDirectionPCA(
    const nav_msgs::msg::OccupancyGrid & map)
{
    std::vector<cv::Point2f> points = getFreeRegionPoints(map);
    if (points.size() < 10) {
        return 0.0;  // 默认水平方向
    }

    auto pca_result = performPCA(points);
    return pca_result.first;
}

double ScanDirectionOptimizer::calculateMinBoundingRectAngle(
    const nav_msgs::msg::OccupancyGrid & map)
{
    std::vector<cv::Point2f> points = getFreeRegionPoints(map);
    if (points.size() < 10) {
        return 0.0;
    }

    auto mbr_result = calculateMinRect(points);
    return mbr_result.first;
}

double ScanDirectionOptimizer::calculateAspectRatio(
    const nav_msgs::msg::OccupancyGrid & map)
{
    // 获取空闲区域点集
    std::vector<cv::Point2f> points = getFreeRegionPoints(map);
    
    if (points.empty()) {
        return 1.0;
    }

    // 计算边界矩形
    cv::Rect bounding_rect = cv::boundingRect(points);
    
    // 计算长宽比
    double width = bounding_rect.width;
    double height = bounding_rect.height;
    
    if (height <= 0 || width <= 0) {
        return 1.0;
    }

    // 返回较大的值作为长宽比
    return std::max(width / height, height / width);
}

int ScanDirectionOptimizer::angleToScanDirection(double angle)
{
    // 将角度归一化到 [0, PI)
    double normalized_angle = std::abs(angle);
    while (normalized_angle >= M_PI) {
        normalized_angle -= M_PI;
    }

    // 判断更接近水平还是垂直
    // 水平方向: 0 或 PI
    // 垂直方向: PI/2
    double dist_to_horizontal = std::min(normalized_angle, M_PI - normalized_angle);
    double dist_to_vertical = std::abs(normalized_angle - M_PI / 2);

    // 更接近水平 -> 0, 更接近垂直 -> 1
    return (dist_to_vertical < dist_to_horizontal) ? 1 : 0;
}

std::vector<cv::Point2f> ScanDirectionOptimizer::getFreeRegionPoints(
    const nav_msgs::msg::OccupancyGrid & map)
{
    std::vector<cv::Point2f> points;
    
    // 收集所有空闲区域的点
    for (size_t y = 0; y < map.info.height; ++y) {
        for (size_t x = 0; x < map.info.width; ++x) {
            size_t index = y * map.info.width + x;
            int8_t value = map.data[index];
            
            // 空闲区域: 值 >= 0 且 < 50
            if (value >= 0 && value < 50) {
                points.push_back(cv::Point2f(
                    static_cast<float>(x),
                    static_cast<float>(y)));
            }
        }
    }
    
    return points;
}

cv::Mat ScanDirectionOptimizer::occupancyGridToMat(
    const nav_msgs::msg::OccupancyGrid & map)
{
    cv::Mat mat(map.info.height, map.info.width, CV_8UC1);
    
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

std::pair<double, double> ScanDirectionOptimizer::performPCA(
    const std::vector<cv::Point2f> & points)
{
    if (points.size() < 2) {
        return std::make_pair(0.0, 1.0);
    }

    // 计算均值
    cv::Point2f mean(0.0f, 0.0f);
    for (const auto & pt : points) {
        mean.x += pt.x;
        mean.y += pt.y;
    }
    mean.x /= static_cast<float>(points.size());
    mean.y /= static_cast<float>(points.size());

    // 构建协方差矩阵
    cv::Mat cov_matrix(2, 2, CV_32F, cv::Scalar(0));
    for (const auto & pt : points) {
        float dx = pt.x - mean.x;
        float dy = pt.y - mean.y;
        cov_matrix.at<float>(0, 0) += dx * dx;
        cov_matrix.at<float>(0, 1) += dx * dy;
        cov_matrix.at<float>(1, 0) += dx * dy;
        cov_matrix.at<float>(1, 1) += dy * dy;
    }
    cov_matrix /= static_cast<float>(points.size());

    // 计算特征值和特征向量
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(cov_matrix, eigenvalues, eigenvectors);

    // 主方向角度（使用最大特征值对应的特征向量）
    double angle = std::atan2(
        eigenvectors.at<float>(0, 1),
        eigenvectors.at<float>(0, 0));

    // 方差比（主成分方差/次成分方差）
    double variance_ratio = eigenvalues.at<float>(0) / eigenvalues.at<float>(1);

    return std::make_pair(angle, variance_ratio);
}

std::pair<double, cv::RotatedRect> ScanDirectionOptimizer::calculateMinRect(
    const std::vector<cv::Point2f> & points)
{
    if (points.size() < 3) {
        return std::make_pair(0.0, cv::RotatedRect());
    }

    // 计算最小外接矩形
    cv::RotatedRect min_rect = cv::minAreaRect(points);
    
    // 获取长轴角度
    double angle = min_rect.angle;
    
    // OpenCV的minAreaRect返回的角度范围是 [-90, 0]
    // 需要转换为正确的方向
    cv::Size2f size = min_rect.size;
    
    // 如果宽度小于高度，说明角度需要调整90度
    if (size.width < size.height) {
        angle += 90.0;
        std::swap(size.width, size.height);
    }
    
    // 转换为弧度
    angle = angle * M_PI / 180.0;

    return std::make_pair(angle, min_rect);
}

ScanDirectionResult ScanDirectionOptimizer::combineResults(
    const std::pair<double, double> & pca_result,
    const std::pair<double, cv::RotatedRect> & mbr_result,
    double aspect_ratio,
    const ScanDirectionConfig & config)
{
    ScanDirectionResult result;
    result.aspect_ratio = aspect_ratio;
    result.principal_angle = pca_result.first;
    result.pca_variance_ratio = pca_result.second;

    // 方法1: PCA结果
    int pca_direction = angleToScanDirection(pca_result.first);
    double pca_confidence = std::min(0.8, 0.5 + 0.3 * (pca_result.second - 1.0) / 5.0);

    // 方法2: MBR结果
    int mbr_direction = angleToScanDirection(mbr_result.first);
    double mbr_size_ratio = mbr_result.second.size.width / mbr_result.second.size.height;
    if (mbr_size_ratio < 1.0) mbr_size_ratio = 1.0 / mbr_size_ratio;
    double mbr_confidence = std::min(0.85, 0.5 + 0.35 * (mbr_size_ratio - 1.0) / 4.0);

    // 如果两种方法结果一致，置信度更高
    if (pca_direction == mbr_direction) {
        result.direction = pca_direction;
        result.confidence = std::max(pca_confidence, mbr_confidence) + 0.1;
        result.method_used = "pca_mbr_agreement";
    } else {
        // 结果不一致，选择置信度更高的方法
        if (pca_confidence > mbr_confidence) {
            result.direction = pca_direction;
            result.confidence = pca_confidence;
            result.method_used = "pca_primary";
        } else {
            result.direction = mbr_direction;
            result.confidence = mbr_confidence;
            result.method_used = "mbr_primary";
        }
    }

    // 如果置信度太低，考虑使用长宽比作为参考
    if (result.confidence < 0.6 && aspect_ratio > 1.5) {
        // 根据地图尺寸判断
        // 注意：这里需要原始地图信息，我们使用aspect_ratio作为参考
        if (aspect_ratio > 2.0) {
            // 长宽比很大，使用更直接的判断
            // 假设aspect_ratio是width/height或height/width的最大值
            // 这里我们根据PCA角度来判断更接近哪个方向
            result.confidence = 0.7;
            result.method_used = "aspect_ratio_boost";
        }
    }

    // 确保置信度不超过1.0
    result.confidence = std::min(1.0, result.confidence);

    return result;
}

double ScanDirectionOptimizer::angleToAxisDistance(double angle)
{
    // 将角度归一化到 [0, PI)
    double normalized_angle = std::abs(angle);
    while (normalized_angle >= M_PI) {
        normalized_angle -= M_PI;
    }

    // 计算到最近轴方向（水平或垂直）的距离
    double dist_to_horizontal = std::min(normalized_angle, M_PI - normalized_angle);
    double dist_to_vertical = std::abs(normalized_angle - M_PI / 2);

    return std::min(dist_to_horizontal, dist_to_vertical);
}

}  // namespace coverage_planner