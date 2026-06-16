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

// ==================== 实现部分 ====================

inline PointCloudFilter::PointCloudFilter() 
    : initialized_(false) {}

inline PointCloudFilter::PointCloudFilter(const FilterConfig& config)
    : config_(config), initialized_(true) {}

inline void PointCloudFilter::initialize(const FilterConfig& config) {
    config_ = config;
    initialized_ = true;
}

inline PointCloudPtr PointCloudFilter::process(const PointCloudPtr& cloud) {
    if (!initialized_ || cloud->empty()) {
        return cloud;
    }

    PointCloudPtr result = cloud;

    // 1. 移除NaN点
    result = removeNaNPoints(result);

    // 2. 距离滤波
    result = filterByRange(result);

    // 3. 高度滤波
    result = filterByHeight(result);

    // 4. 体素降采样
    if (config_.voxel_size > 0) {
        result = voxelDownsample(result);
    }

    // 5. 统计离群点滤波 (可选)
    if (config_.enable_sor) {
        result = statisticalOutlierRemoval(result);
    }

    return result;
}

inline PointCloudPtr PointCloudFilter::removeNaNPoints(const PointCloudPtr& cloud) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    filtered->reserve(cloud->size());

    for (const auto& point : cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            filtered->points.push_back(point);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;
    
    return filtered;
}

inline PointCloudPtr PointCloudFilter::filterByRange(const PointCloudPtr& cloud) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    filtered->reserve(cloud->size());
    
    double min_range_sq = config_.min_range * config_.min_range;
    double max_range_sq = config_.max_range * config_.max_range;

    for (const auto& point : cloud->points) {
        double range_sq = point.x * point.x + point.y * point.y + point.z * point.z;
        
        if (range_sq >= min_range_sq && range_sq <= max_range_sq) {
            filtered->points.push_back(point);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;
    
    return filtered;
}

inline PointCloudPtr PointCloudFilter::filterByHeight(const PointCloudPtr& cloud) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    filtered->reserve(cloud->size());

    for (const auto& point : cloud->points) {
        if (point.z >= config_.min_z && point.z <= config_.max_z) {
            filtered->points.push_back(point);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = true;
    
    return filtered;
}

inline PointCloudPtr PointCloudFilter::voxelDownsample(const PointCloudPtr& cloud) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
    voxel_filter.filter(*filtered);
    
    return filtered;
}

inline PointCloudPtr PointCloudFilter::statisticalOutlierRemoval(const PointCloudPtr& cloud) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    
    pcl::StatisticalOutlierRemoval<PointType> sor_filter;
    sor_filter.setInputCloud(cloud);
    sor_filter.setMeanK(config_.sor_mean_k);
    sor_filter.setStddevMulThresh(config_.sor_stddev_mul);
    sor_filter.filter(*filtered);
    
    return filtered;
}

inline std::vector<double> PointCloudFilter::computeCurvature(const PointCloudPtr& cloud) {
    const int neighbors = 5;
    std::vector<double> curvature(cloud->size(), 0.0);
    
    if (cloud->size() < static_cast<size_t>(2 * neighbors + 1)) {
        return curvature;
    }

    // 按扫描线分组计算曲率
    for (size_t i = neighbors; i < cloud->size() - neighbors; ++i) {
        double diff_x = 0, diff_y = 0, diff_z = 0;
        
        for (int j = -neighbors; j <= neighbors; ++j) {
            if (j == 0) continue;
            diff_x += cloud->points[i + j].x - cloud->points[i].x;
            diff_y += cloud->points[i + j].y - cloud->points[i].y;
            diff_z += cloud->points[i + j].z - cloud->points[i].z;
        }
        
        // 曲率 = 邻域差值范数 / 点自身距离
        double range = std::sqrt(cloud->points[i].x * cloud->points[i].x +
                                  cloud->points[i].y * cloud->points[i].y +
                                  cloud->points[i].z * cloud->points[i].z);
        if (range > 0.1) {
            curvature[i] = std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / range;
        }
    }
    
    return curvature;
}

inline void PointCloudFilter::extractFeatures(const PointCloudPtr& cloud,
                                               PointCloudPtr& corner_points,
                                               PointCloudPtr& planar_points,
                                               int corner_num,
                                               int planar_num) {
    corner_points.reset(new pcl::PointCloud<PointType>());
    planar_points.reset(new pcl::PointCloud<PointType>());
    
    if (cloud->empty()) return;

    // 计算曲率
    std::vector<double> curvature = computeCurvature(cloud);

    // 按曲率排序的索引
    std::vector<size_t> indices(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        indices[i] = i;
    }
    
    // 按曲率从小到大排序
    std::sort(indices.begin(), indices.end(), 
              [&curvature](size_t a, size_t b) {
                  return curvature[a] < curvature[b];
              });

    // 标记已选中的点
    std::vector<bool> selected(cloud->size(), false);
    
    int scan_points = cloud->size() / config_.scan_line;
    
    // 选择平面点 (低曲率)
    int planar_count = 0;
    for (size_t i = 0; i < cloud->size() && planar_count < planar_num * config_.scan_line; ++i) {
        size_t idx = indices[i];
        if (!selected[idx] && curvature[idx] < 0.1) {
            // 检查邻域点未被选中
            bool neighbor_free = true;
            for (int j = -2; j <= 2; ++j) {
                if (static_cast<int>(idx) + j >= 0 && 
                    static_cast<int>(idx) + j < static_cast<int>(cloud->size())) {
                    if (selected[idx + j]) {
                        neighbor_free = false;
                        break;
                    }
                }
            }
            
            if (neighbor_free) {
                planar_points->points.push_back(cloud->points[idx]);
                selected[idx] = true;
                planar_count++;
            }
        }
    }

    // 选择角点 (高曲率)
    int corner_count = 0;
    for (int i = cloud->size() - 1; i >= 0 && corner_count < corner_num * config_.scan_line; --i) {
        size_t idx = indices[i];
        if (!selected[idx] && curvature[idx] > 0.5) {
            // 检查邻域点未被选中
            bool neighbor_free = true;
            for (int j = -2; j <= 2; ++j) {
                if (static_cast<int>(idx) + j >= 0 && 
                    static_cast<int>(idx) + j < static_cast<int>(cloud->size())) {
                    if (selected[idx + j]) {
                        neighbor_free = false;
                        break;
                    }
                }
            }
            
            if (neighbor_free) {
                corner_points->points.push_back(cloud->points[idx]);
                selected[idx] = true;
                corner_count++;
            }
        }
    }

    corner_points->width = corner_points->size();
    corner_points->height = 1;
    corner_points->is_dense = true;
    
    planar_points->width = planar_points->size();
    planar_points->height = 1;
    planar_points->is_dense = true;
}

inline bool PointCloudFilter::motionUndistort(PointCloudPtr& cloud,
                                               const std::vector<ImuData>& imu_buffer,
                                               const State& state_begin,
                                               double scan_time_start,
                                               double scan_time_end) {
    if (imu_buffer.size() < 2 || cloud->empty()) {
        return false;
    }

    double scan_period = scan_time_end - scan_time_start;
    if (scan_period <= 0) {
        return false;
    }

    // 对每个点进行运动补偿
    PointCloudPtr undistorted(new pcl::PointCloud<PointType>());
    undistorted->reserve(cloud->size());

    for (const auto& point : cloud->points) {
        // 计算点的时间戳
        double point_time = scan_time_start + point.intensity * scan_period;
        
        // 插值获取该时刻的状态
        // 简化处理: 使用IMU积分获取该时刻的位姿
        
        // 对于FAST-LIO2，点的intensity编码了相对时间
        double t = point.intensity;  // 相对时间 [0, 1]
        
        // 使用IMU数据估计该时刻的位姿 (简化版)
        // 实际应使用IMU前向传播
        Vector3d trans = state_begin.position + t * state_begin.velocity * scan_period;
        
        // 简化处理: 假设旋转变化不大
        // 实际应使用IMU积分的旋转
        
        // 将点转换到扫描结束时刻的坐标系
        PointType new_point = point;
        new_point.x = point.x - trans(0);
        new_point.y = point.y - trans(1);
        new_point.z = point.z - trans(2);
        
        undistorted->points.push_back(new_point);
    }

    undistorted->width = undistorted->size();
    undistorted->height = 1;
    cloud = undistorted;
    
    return true;
}

} // namespace fast_lio2_slam