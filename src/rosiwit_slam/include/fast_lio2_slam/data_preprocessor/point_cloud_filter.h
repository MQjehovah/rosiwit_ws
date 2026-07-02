/**
 * @file point_cloud_filter.h
 * @brief FAST-LIO2 SLAM - 点云滤波处理
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/common/config.h"

namespace fast_lio2_slam {

/**
 * @brief 点云滤波器配置
 */
struct FilterConfig {
    double min_range = 0.5;          // 最小距离
    double max_range = 100.0;        // 最大距离
    double min_z = -3.0;             // 最小Z值
    double max_z = 3.0;              // 最大Z值
    double voxel_size = 0.2;         // 体素滤波大小
    int sor_mean_k = 50;             // 统计离群点滤波K值
    double sor_stddev_mul = 1.0;     // 统计离群点滤波标准差阈值
    bool enable_sor = false;         // 是否启用统计离群点滤波
    int scan_line = 16;              // 激光线数
};

/**
 * @brief 点云滤波与预处理类
 */
class PointCloudFilter {
public:
    PointCloudFilter();
    explicit PointCloudFilter(const FilterConfig& config);
    ~PointCloudFilter() = default;

    /**
     * @brief 初始化滤波器
     */
    void initialize(const FilterConfig& config);

    /**
     * @brief 设置滤波器配置
     */
    void setConfig(const FilterConfig& config) { config_ = config; }

    /**
     * @brief 处理点云 (完整处理流程)
     */
    PointCloudPtr process(const PointCloudPtr& cloud);

    /**
     * @brief 移除NaN点
     */
    PointCloudPtr removeNaNPoints(const PointCloudPtr& cloud);

    /**
     * @brief 按距离滤波
     */
    PointCloudPtr filterByRange(const PointCloudPtr& cloud);

    /**
     * @brief 按高度滤波
     */
    PointCloudPtr filterByHeight(const PointCloudPtr& cloud);

    /**
     * @brief 体素滤波降采样
     */
    PointCloudPtr voxelDownsample(const PointCloudPtr& cloud);

    /**
     * @brief 统计离群点滤波
     */
    PointCloudPtr statisticalOutlierRemoval(const PointCloudPtr& cloud);

    /**
     * @brief 特征提取 (平面点和边缘点)
     */
    void extractFeatures(const PointCloudPtr& cloud,
                         PointCloudPtr& corner_points,
                         PointCloudPtr& planar_points,
                         int corner_num = 2,
                         int planar_num = 4);

    /**
     * @brief 计算点曲率 (用于特征提取)
     */
    std::vector<double> computeCurvature(const PointCloudPtr& cloud);

    /**
     * @brief 运动畸变校正
     */
    bool motionUndistort(PointCloudPtr& cloud,
                         const std::vector<ImuData>& imu_buffer,
                         const State& state_begin,
                         double scan_time_start,
                         double scan_time_end);

private:
    FilterConfig config_;
    bool initialized_;
};

} // namespace fast_lio2_slam
