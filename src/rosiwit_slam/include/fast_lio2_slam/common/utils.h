/**
 * @file utils.h
 * @brief FAST-LIO2 SLAM - 工具函数
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
// 使用自定义的Sophus简化实现，避免与系统库冲突
#include "sophus_se3.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

// ============== 数学工具 ==============

/**
 * @brief 角度转弧度
 */
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

/**
 * @brief 弧度转角度
 */
inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

/**
 * @brief 欧拉角转旋转矩阵 (ZYX顺序)
 */
inline Matrix3d eulerToRotationMatrix(const Vector3d& euler) {
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    Matrix3d R;
    R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
         sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
         -sp,   cp*sr,            cp*cr;

    return R;
}

/**
 * @brief 旋转矩阵转欧拉角 (ZYX顺序)
 */
inline Vector3d rotationMatrixToEuler(const Matrix3d& R) {
    double roll = std::atan2(R(2, 1), R(2, 2));
    double pitch = std::asin(-R(2, 0));
    double yaw = std::atan2(R(1, 0), R(0, 0));
    return Vector3d(roll, pitch, yaw);
}

/**
 * @brief 四元数转欧拉角
 */
inline Vector3d quaternionToEuler(const Quaterniond& q) {
    return rotationMatrixToEuler(q.toRotationMatrix());
}

/**
 * @brief 欧拉角转四元数
 */
inline Quaterniond eulerToQuaternion(const Vector3d& euler) {
    return Quaterniond(eulerToRotationMatrix(euler));
}

/**
 * @brief 叉乘矩阵 (skew-symmetric matrix)
 */
inline Matrix3d skewSymmetric(const Vector3d& v) {
    Matrix3d S;
    S << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return S;
}

/**
 * @brief 插值位姿
 */
inline SE3d interpolatePose(const SE3d& pose1, const SE3d& pose2, double t) {
    // 球面线性插值旋转
    Quaterniond q1 = pose1.unit_quaternion();
    Quaterniond q2 = pose2.unit_quaternion();
    Quaterniond q_interp = q1.slerp(t, q2);

    // 线性插值平移
    Vector3d t_interp = (1 - t) * pose1.translation() + t * pose2.translation();

    return SE3d(q_interp, t_interp);
}

/**
 * @brief 计算两点云之间的ICP变换
 */
inline bool computeIcpTransform(const std::vector<Vector3d>& src_points,
                                 const std::vector<Vector3d>& tgt_points,
                                 SE3d& transform,
                                 int max_iterations = 20,
                                 double tolerance = 1e-6) {
    if (src_points.size() != tgt_points.size() || src_points.empty()) {
        return false;
    }

    // 计算质心
    Vector3d src_centroid = Vector3d::Zero();
    Vector3d tgt_centroid = Vector3d::Zero();

    for (size_t i = 0; i < src_points.size(); ++i) {
        src_centroid += src_points[i];
        tgt_centroid += tgt_points[i];
    }
    src_centroid /= src_points.size();
    tgt_centroid /= tgt_points.size();

    // 构建协方差矩阵
    Matrix3d H = Matrix3d::Zero();
    for (size_t i = 0; i < src_points.size(); ++i) {
        Vector3d src_centered = src_points[i] - src_centroid;
        Vector3d tgt_centered = tgt_points[i] - tgt_centroid;
        H += src_centered * tgt_centered.transpose();
    }

    // SVD分解
    Eigen::JacobiSVD<Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

    // 处理反射情况
    if (R.determinant() < 0) {
        Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    // 计算平移
    Vector3d t = tgt_centroid - R * src_centroid;

    transform = SE3d(Quaterniond(R), t);
    return true;
}

// ============== 点云工具 ==============

/**
 * @brief 体素滤波
 */
inline PointCloudPtr voxelFilter(const PointCloudPtr& cloud, double voxel_size) {
    if (cloud->empty() || voxel_size <= 0) {
        return cloud;
    }

    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    pcl::VoxelGrid<PointType> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.filter(*filtered);

    return filtered;
}

/**
 * @brief 统计离群点滤波
 */
inline PointCloudPtr statisticalOutlierRemoval(const PointCloudPtr& cloud,
                                                int mean_k = 50,
                                                double stddev_mul_thresh = 1.0) {
    if (cloud->empty()) {
        return cloud;
    }

    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    pcl::StatisticalOutlierRemoval<PointType> filter;
    filter.setInputCloud(cloud);
    filter.setMeanK(mean_k);
    filter.setStddevMulThresh(stddev_mul_thresh);
    filter.filter(*filtered);

    return filtered;
}

/**
 * @brief 移除NaN点
 */
inline PointCloudPtr removeNaNPoints(const PointCloudPtr& cloud) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
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

/**
 * @brief 移除过远点
 */
inline PointCloudPtr removeFarPoints(const PointCloudPtr& cloud, double max_distance) {
    PointCloudPtr filtered(new pcl::PointCloud<PointType>());
    double max_dist_sq = max_distance * max_distance;

    for (const auto& point : cloud->points) {
        double dist_sq = point.x * point.x + point.y * point.y + point.z * point.z;
        if (dist_sq <= max_dist_sq) {
            filtered->points.push_back(point);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    return filtered;
}

/**
 * @brief 变换点云
 */
inline void transformPointCloud(const PointCloudPtr& cloud_in,
                                 PointCloudPtr& cloud_out,
                                 const SE3d& transform) {
    cloud_out->clear();
    cloud_out->reserve(cloud_in->size());

    for (const auto& point : cloud_in->points) {
        Vector3d p(point.x, point.y, point.z);
        Vector3d p_transformed = transform * p;

        PointType new_point;
        new_point.x = p_transformed(0);
        new_point.y = p_transformed(1);
        new_point.z = p_transformed(2);
        new_point.intensity = point.intensity;
        new_point.normal_x = point.normal_x;
        new_point.normal_y = point.normal_y;
        new_point.normal_z = point.normal_z;

        cloud_out->points.push_back(new_point);
    }

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
}

/**
 * @brief 计算点云曲率 (用于特征提取)
 */
inline double computeCurvature(const PointCloudPtr& cloud, int idx, int neighbors = 5) {
    if (idx < neighbors || idx >= static_cast<int>(cloud->size()) - neighbors) {
        return 0.0;
    }

    Vector3d center(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
    Vector3d diff_sum = Vector3d::Zero();

    for (int i = -neighbors; i <= neighbors; ++i) {
        if (i == 0) continue;
        Vector3d neighbor(cloud->points[idx + i].x,
                          cloud->points[idx + i].y,
                          cloud->points[idx + i].z);
        diff_sum += (neighbor - center);
    }

    return diff_sum.norm();
}

// ============== 时间工具 ==============

/**
 * @brief 获取当前时间戳 (秒)
 */
inline double getCurrentTimestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

/**
 * @brief 计算时间差 (毫秒)
 */
inline double timeDiffMs(double t1, double t2) {
    return std::abs(t2 - t1) * 1000.0;
}

/**
 * @brief 线性插值IMU数据
 */
inline ImuData interpolateImu(const ImuData& imu1, const ImuData& imu2, double timestamp) {
    if (imu1.timestamp == imu2.timestamp) {
        return imu1;
    }

    double t = (timestamp - imu1.timestamp) / (imu2.timestamp - imu1.timestamp);
    t = std::max(0.0, std::min(1.0, t));

    ImuData result;
    result.timestamp = timestamp;
    result.acc = (1 - t) * imu1.acc + t * imu2.acc;
    result.gyro = (1 - t) * imu1.gyro + t * imu2.gyro;

    return result;
}

// ============== 日志工具 ==============

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

/**
 * @brief 简单日志输出
 */
inline void logPrint(LogLevel level, const std::string& message) {
    const char* level_str[] = {"[DEBUG]", "[INFO]", "[WARN]", "[ERROR]", "[FATAL]"};

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time_t);

    char time_buf[32];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm);

    printf("%s %s %s\n", time_buf, level_str[static_cast<int>(level)], message.c_str());
}

#define LOG_DEBUG(msg) logPrint(LogLevel::DEBUG, msg)
#define LOG_INFO(msg) logPrint(LogLevel::INFO, msg)
#define LOG_WARN(msg) logPrint(LogLevel::WARN, msg)
#define LOG_ERROR(msg) logPrint(LogLevel::ERROR, msg)
#define LOG_FATAL(msg) logPrint(LogLevel::FATAL, msg)

} // namespace fast_lio2_slam