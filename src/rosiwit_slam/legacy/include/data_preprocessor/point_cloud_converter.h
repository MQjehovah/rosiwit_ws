/**
 * @file point_cloud_converter.h
 * @brief FAST-LIO2 SLAM - 点云格式转换器
 * @author AI Development Team
 * @date 2026-04-26
 *
 * @description
 * 该模块负责将不同厂商的点云格式转换为FAST-LIO2所需的Livox格式。
 *
 * 支持的输入格式：
 * - Ouster (x, y, z, intensity, ring, timestamp)
 * - Velodyne (x, y, z, intensity, ring)
 * - Livox (x, y, z, intensity, normal_x, normal_y, normal_z, curvature)
 * - 标准 PointXYZI (x, y, z, intensity)
 *
 * 输出格式：
 * - pcl::PointXYZINormal (x, y, z, intensity, normal_x, normal_y, normal_z, curvature)
 *
 * 关键问题修复：
 * BUG-002: Ouster点云缺少normal_x, normal_y, normal_z, curvature字段
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <set>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

/**
 * @brief 点云格式类型枚举
 */
enum class PointCloudFormat {
    UNKNOWN = 0,
    LIVOX,          // Livox格式: x, y, z, intensity, normal_x, normal_y, normal_z, curvature
    OUSTER,         // Ouster格式: x, y, z, intensity, ring, timestamp
    VELODYNE,       // Velodyne格式: x, y, z, intensity, ring
    HESAI,          // Hesai格式: x, y, z, intensity, ring, timestamp
    STANDARD_XYZI,  // 标准PointXYZI: x, y, z, intensity
    STANDARD_XYZ    // 标准PointXYZ: x, y, z
};

/**
 * @brief 点云格式信息
 */
struct PointCloudFormatInfo {
    PointCloudFormat format = PointCloudFormat::UNKNOWN;
    std::string format_name = "unknown";
    bool has_intensity = false;
    bool has_ring = false;
    bool has_timestamp = false;
    bool has_normal = false;
    bool has_curvature = false;
    int point_step = 0;          // 每个点的字节数
    std::map<std::string, int> field_offsets;  // 字段偏移量
};

/**
 * @brief 点云格式转换配置
 */
struct ConverterConfig {
    // 曲率计算方法
    enum class CurvatureMethod {
        NEIGHBOR_BASED,  // 基于邻域点的曲率计算
        RING_BASED,      // 基于扫描线的曲率计算（需要ring字段）
        ZERO             // 直接设为0（快速但不精确）
    };

    // 法向量计算方法
    enum class NormalMethod {
        PCA_BASED,       // 基于PCA的法向量估计
        RING_BASED,      // 基于扫描线的法向量估计
        ZERO             // 直接设为[0,0,1]（快速但不精确）
    };

    CurvatureMethod curvature_method = CurvatureMethod::RING_BASED;
    NormalMethod normal_method = NormalMethod::RING_BASED;

    // 曲率计算参数
    int curvature_neighbors = 5;     // 曲率计算的邻域点数
    double normal_radius = 0.5;     // 法向量估计的搜索半径

    // 扫描线参数
    int scan_lines = 64;             // LiDAR扫描线数
    double scan_period = 0.1;        // 扫描周期(秒)

    // 是否保留原始时间戳
    bool preserve_timestamp = true;

    // 是否打印格式信息
    bool verbose = true;
};

/**
 * @brief 点云格式转换器
 *
 * 负责将不同格式的点云转换为FAST-LIO2所需的PointXYZINormal格式
 */
class PointCloudConverter {
public:
    PointCloudConverter() = default;
    explicit PointCloudConverter(const ConverterConfig& config);
    ~PointCloudConverter() = default;

    /**
     * @brief 设置转换配置
     */
    void setConfig(const ConverterConfig& config) { config_ = config; }

    /**
     * @brief 获取当前配置
     */
    const ConverterConfig& getConfig() const { return config_; }

    /**
     * @brief 从ROS2 PointCloud2消息转换为PCL点云
     *
     * 这是主要的转换接口，自动检测输入格式并进行转换。
     *
     * @param msg ROS2 PointCloud2消息
     * @param cloud 输出的PCL点云（PointXYZINormal格式）
     * @return bool 转换是否成功
     */
    bool fromROSMsg(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                    PointCloudPtr& cloud);

    /**
     * @brief 检测点云格式
     *
     * @param msg ROS2 PointCloud2消息
     * @return PointCloudFormatInfo 检测到的格式信息
     */
    PointCloudFormatInfo detectFormat(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    /**
     * @brief 转换Ouster格式点云
     *
     * @param msg ROS2 PointCloud2消息
     * @param cloud 输出点云
     * @param format_info 格式信息
     * @return bool 转换是否成功
     */
    bool convertOuster(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                       PointCloudPtr& cloud,
                       const PointCloudFormatInfo& format_info);

    /**
     * @brief 转换Velodyne格式点云
     */
    bool convertVelodyne(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                         PointCloudPtr& cloud,
                         const PointCloudFormatInfo& format_info);

    /**
     * @brief 转换标准PointXYZI格式
     */
    bool convertStandardXYZI(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                             PointCloudPtr& cloud,
                             const PointCloudFormatInfo& format_info);

    /**
     * @brief 转换Livox格式点云（直接转换，无需填充）
     */
    bool convertLivox(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                      PointCloudPtr& cloud);

    /**
     * @brief 计算点云曲率（基于扫描线）
     *
     * @param cloud 点云
     * @param ring_indices 每条扫描线上的点索引
     */
    void computeCurvatureRingBased(PointCloudPtr& cloud,
                                   const std::vector<std::vector<int>>& ring_indices);

    /**
     * @brief 计算点云曲率（基于邻域）
     */
    void computeCurvatureNeighborBased(PointCloudPtr& cloud);

    /**
     * @brief 计算法向量（基于PCA）
     */
    void computeNormalPCA(PointCloudPtr& cloud);

    /**
     * @brief 计算法向量（基于扫描线）
     */
    void computeNormalRingBased(PointCloudPtr& cloud,
                                const std::vector<std::vector<int>>& ring_indices);

    /**
     * @brief 按扫描线分组
     *
     * @param cloud 点云
     * @param ring_field ring字段数据（如果有）
     * @param format_info 格式信息
     * @return std::vector<std::vector<int>> 每条扫描线的点索引
     */
    std::vector<std::vector<int>> groupByRing(
        const PointCloudPtr& cloud,
        const std::vector<int>& ring_field,
        const PointCloudFormatInfo& format_info);

    /**
     * @brief 设置默认法向量和曲率
     *
     * 快速模式，不进行精确计算
     */
    void setDefaultNormalAndCurvature(PointCloudPtr& cloud);

private:
    ConverterConfig config_;

    /**
     * @brief 从字节数据提取字段值
     */
    template<typename T>
    T extractField(const uint8_t* data, int offset) const {
        return *reinterpret_cast<const T*>(data + offset);
    }

    /**
     * @brief 获取ring字段数据
     */
    std::vector<int> getRingField(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                  const PointCloudFormatInfo& format_info);

    /**
     * @brief 根据时间戳排序（用于无ring字段的扫描线分组）
     */
    std::vector<std::vector<int>> groupByTimestamp(
        const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
        const PointCloudPtr& cloud,
        const PointCloudFormatInfo& format_info);
};

} // namespace fast_lio2_slam
