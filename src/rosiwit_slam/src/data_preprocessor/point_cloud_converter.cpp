/**
 * @file point_cloud_converter.cpp
 * @brief FAST-LIO2 SLAM - 点云格式转换器实现
 */

#include "fast_lio2_slam/data_preprocessor/point_cloud_converter.h"

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace fast_lio2_slam {

PointCloudConverter::PointCloudConverter(const ConverterConfig& config)
    : config_(config) {}

bool PointCloudConverter::fromROSMsg(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    PointCloudPtr& cloud) {

    // 检测输入格式
    PointCloudFormatInfo format_info = detectFormat(msg);

    if (config_.verbose) {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudConverter"),
                    "Detected point cloud format: %s, point_step=%d, fields=%zu",
                    format_info.format_name.c_str(),
                    format_info.point_step,
                    msg->fields.size());
    }

    // 根据格式选择转换方法
    bool success = false;
    switch (format_info.format) {
        case PointCloudFormat::LIVOX:
            success = convertLivox(msg, cloud);
            break;
        case PointCloudFormat::OUSTER:
            success = convertOuster(msg, cloud, format_info);
            break;
        case PointCloudFormat::VELODYNE:
            success = convertVelodyne(msg, cloud, format_info);
            break;
        case PointCloudFormat::HESAI:
            // Hesai格式类似Ouster
            success = convertOuster(msg, cloud, format_info);
            break;
        case PointCloudFormat::STANDARD_XYZI:
            success = convertStandardXYZI(msg, cloud, format_info);
            break;
        case PointCloudFormat::STANDARD_XYZ:
        case PointCloudFormat::UNKNOWN:
        default:
            // 尝试使用PCL标准转换
            RCLCPP_WARN(rclcpp::get_logger("PointCloudConverter"),
                        "Unknown format, attempting standard PCL conversion");
            success = convertStandardXYZI(msg, cloud, format_info);
            break;
    }

    return success;
}

PointCloudFormatInfo PointCloudConverter::detectFormat(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {

    PointCloudFormatInfo info;
    info.point_step = msg->point_step;

    // 收集所有字段名
    std::set<std::string> field_names;
    for (const auto& field : msg->fields) {
        std::string name = field.name;
        // 转换为小写进行匹配
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        field_names.insert(name);
        info.field_offsets[name] = field.offset;

        // 检查特定字段
        if (name == "intensity" || name == "i") info.has_intensity = true;
        if (name == "ring" || name == "scan_line" || name == "scan_id") info.has_ring = true;
        if (name == "timestamp" || name == "time") info.has_timestamp = true;
        if (name == "normal_x" || name == "nx") info.has_normal = true;
        if (name == "curvature" || name == "curv") info.has_curvature = true;
    }

    // 判断格式
    if (info.has_normal && info.has_curvature) {
        // Livox格式：有法向量和曲率
        info.format = PointCloudFormat::LIVOX;
        info.format_name = "Livox";
    } else if (info.has_ring && info.has_timestamp && info.has_intensity) {
        // Ouster格式：有ring、timestamp和intensity
        info.format = PointCloudFormat::OUSTER;
        info.format_name = "Ouster";
    } else if (info.has_ring && info.has_intensity) {
        // Velodyne格式：有ring和intensity
        info.format = PointCloudFormat::VELODYNE;
        info.format_name = "Velodyne";
    } else if (info.has_intensity) {
        // 标准PointXYZI
        info.format = PointCloudFormat::STANDARD_XYZI;
        info.format_name = "Standard_XYZI";
    } else {
        // 标准PointXYZ
        info.format = PointCloudFormat::STANDARD_XYZ;
        info.format_name = "Standard_XYZ";
    }

    return info;
}

bool PointCloudConverter::convertOuster(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    PointCloudPtr& cloud,
    const PointCloudFormatInfo& format_info) {

    // 预分配空间
    cloud->clear();
    cloud->reserve(msg->width * msg->height);
    cloud->is_dense = true;

    // 获取字段偏移量（使用at()方法访问const map）
    int offset_x = format_info.field_offsets.count("x") ? format_info.field_offsets.at("x") : 0;
    int offset_y = format_info.field_offsets.count("y") ? format_info.field_offsets.at("y") : 4;
    int offset_z = format_info.field_offsets.count("z") ? format_info.field_offsets.at("z") : 8;
    int offset_intensity = format_info.field_offsets.count("intensity") ? format_info.field_offsets.at("intensity") : 12;
    int offset_ring = format_info.field_offsets.count("ring") ? format_info.field_offsets.at("ring") : -1;

    // 提取ring字段用于后续计算
    std::vector<int> ring_values;
    if (offset_ring >= 0) {
        ring_values.reserve(msg->width * msg->height);
    }

    // 遍历所有点
    const uint8_t* data_ptr = msg->data.data();
    for (size_t i = 0; i < msg->width * msg->height; ++i) {
        const uint8_t* point_data = data_ptr + i * msg->point_step;

        PointType point;

        // 提取坐标
        point.x = extractField<float>(point_data, offset_x);
        point.y = extractField<float>(point_data, offset_y);
        point.z = extractField<float>(point_data, offset_z);

        // 检查有效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            cloud->is_dense = false;
            continue;
        }

        // 提取强度
        if (format_info.has_intensity && format_info.field_offsets.count("intensity")) {
            point.intensity = extractField<float>(point_data, format_info.field_offsets.at("intensity"));
        } else {
            point.intensity = 0.0f;
        }

        // 暂时设置默认法向量和曲率，稍后计算
        point.normal_x = 0.0f;
        point.normal_y = 0.0f;
        point.normal_z = 1.0f;
        point.curvature = 0.0f;

        cloud->points.push_back(point);

        // 提取ring值
        if (offset_ring >= 0) {
            int ring = static_cast<int>(extractField<uint16_t>(point_data, offset_ring));
            ring_values.push_back(ring);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    // 根据配置计算曲率和法向量
    if (config_.normal_method != ConverterConfig::NormalMethod::ZERO ||
        config_.curvature_method != ConverterConfig::CurvatureMethod::ZERO) {

        // 按扫描线分组
        std::vector<std::vector<int>> ring_indices;
        if (!ring_values.empty()) {
            ring_indices = groupByRing(cloud, ring_values, format_info);
        }

        // 计算法向量
        switch (config_.normal_method) {
            case ConverterConfig::NormalMethod::RING_BASED:
                if (!ring_indices.empty()) {
                    computeNormalRingBased(cloud, ring_indices);
                } else {
                    setDefaultNormalAndCurvature(cloud);
                }
                break;
            case ConverterConfig::NormalMethod::PCA_BASED:
                computeNormalPCA(cloud);
                break;
            case ConverterConfig::NormalMethod::ZERO:
            default:
                // 已设置默认值
                break;
        }

        // 计算曲率
        switch (config_.curvature_method) {
            case ConverterConfig::CurvatureMethod::RING_BASED:
                if (!ring_indices.empty()) {
                    computeCurvatureRingBased(cloud, ring_indices);
                } else {
                    computeCurvatureNeighborBased(cloud);
                }
                break;
            case ConverterConfig::CurvatureMethod::NEIGHBOR_BASED:
                computeCurvatureNeighborBased(cloud);
                break;
            case ConverterConfig::CurvatureMethod::ZERO:
            default:
                // 已设置默认值
                break;
        }
    }

    if (config_.verbose) {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudConverter"),
                    "Converted %zu Ouster points to Livox format (original: %u)",
                    cloud->size(), msg->width * msg->height);
    }

    return true;
}

bool PointCloudConverter::convertVelodyne(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    PointCloudPtr& cloud,
    const PointCloudFormatInfo& format_info) {

    // Velodyne格式与Ouster类似，只是没有timestamp字段
    return convertOuster(msg, cloud, format_info);
}

bool PointCloudConverter::convertStandardXYZI(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    PointCloudPtr& cloud,
    const PointCloudFormatInfo& format_info) {

    // 预分配空间
    cloud->clear();
    cloud->reserve(msg->width * msg->height);
    cloud->is_dense = true;

    // 获取字段偏移量
    int offset_x = format_info.field_offsets.count("x") ? format_info.field_offsets.at("x") : 0;
    int offset_y = format_info.field_offsets.count("y") ? format_info.field_offsets.at("y") : 4;
    int offset_z = format_info.field_offsets.count("z") ? format_info.field_offsets.at("z") : 8;

    // 遍历所有点
    const uint8_t* data_ptr = msg->data.data();
    for (size_t i = 0; i < msg->width * msg->height; ++i) {
        const uint8_t* point_data = data_ptr + i * msg->point_step;

        PointType point;

        // 提取坐标
        point.x = extractField<float>(point_data, offset_x);
        point.y = extractField<float>(point_data, offset_y);
        point.z = extractField<float>(point_data, offset_z);

        // 检查有效点
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            cloud->is_dense = false;
            continue;
        }

        // 提取强度
        if (format_info.has_intensity && format_info.field_offsets.count("intensity")) {
            point.intensity = extractField<float>(point_data, format_info.field_offsets.at("intensity"));
        } else {
            point.intensity = 0.0f;
        }

        // 设置默认法向量和曲率
        point.normal_x = 0.0f;
        point.normal_y = 0.0f;
        point.normal_z = 1.0f;
        point.curvature = 0.0f;

        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    // 对于标准格式，使用邻域法计算曲率和法向量
    if (config_.curvature_method == ConverterConfig::CurvatureMethod::NEIGHBOR_BASED) {
        computeCurvatureNeighborBased(cloud);
    }
    if (config_.normal_method == ConverterConfig::NormalMethod::PCA_BASED) {
        computeNormalPCA(cloud);
    }

    if (config_.verbose) {
        RCLCPP_INFO(rclcpp::get_logger("PointCloudConverter"),
                    "Converted %zu XYZI points to Livox format",
                    cloud->size());
    }

    return true;
}

bool PointCloudConverter::convertLivox(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    PointCloudPtr& cloud) {

    // Livox格式直接使用PCL转换
    try {
        pcl::fromROSMsg(*msg, *cloud);

        if (config_.verbose) {
            RCLCPP_INFO(rclcpp::get_logger("PointCloudConverter"),
                        "Direct conversion of %zu Livox points", cloud->size());
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("PointCloudConverter"),
                     "Failed to convert Livox format: %s", e.what());
        return false;
    }
}

void PointCloudConverter::computeCurvatureRingBased(
    PointCloudPtr& cloud,
    const std::vector<std::vector<int>>& ring_indices) {

    const int neighbors = config_.curvature_neighbors;

    // 对每条扫描线计算曲率
    for (const auto& ring : ring_indices) {
        if (ring.size() < static_cast<size_t>(2 * neighbors + 1)) {
            continue;
        }

        for (size_t i = neighbors; i < ring.size() - neighbors; ++i) {
            double diff_x = 0, diff_y = 0, diff_z = 0;

            // 计算与邻域点的差值
            for (int j = -neighbors; j <= neighbors; ++j) {
                if (j == 0) continue;
                int neighbor_idx = ring[i + j];
                diff_x += cloud->points[neighbor_idx].x - cloud->points[ring[i]].x;
                diff_y += cloud->points[neighbor_idx].y - cloud->points[ring[i]].y;
                diff_z += cloud->points[neighbor_idx].z - cloud->points[ring[i]].z;
            }

            // 计算曲率（邻域差值范数 / 点距离）
            double range = std::sqrt(cloud->points[ring[i]].x * cloud->points[ring[i]].x +
                                     cloud->points[ring[i]].y * cloud->points[ring[i]].y +
                                     cloud->points[ring[i]].z * cloud->points[ring[i]].z);
            if (range > 0.1) {
                cloud->points[ring[i]].curvature = static_cast<float>(
                    std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / range);
            }
        }
    }
}

void PointCloudConverter::computeCurvatureNeighborBased(PointCloudPtr& cloud) {

    if (cloud->size() < static_cast<size_t>(2 * config_.curvature_neighbors + 1)) {
        return;
    }

    // 构建KdTree
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud);

    const int k = config_.curvature_neighbors;
    std::vector<int> point_idx(k);
    std::vector<float> point_sqr_dist(k);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (kdtree.nearestKSearch(cloud->points[i], k + 1, point_idx, point_sqr_dist) > 0) {
            double diff_x = 0, diff_y = 0, diff_z = 0;
            int count = 0;

            for (int j = 1; j <= k && j < static_cast<int>(point_idx.size()); ++j) {
                diff_x += cloud->points[point_idx[j]].x - cloud->points[i].x;
                diff_y += cloud->points[point_idx[j]].y - cloud->points[i].y;
                diff_z += cloud->points[point_idx[j]].z - cloud->points[i].z;
                count++;
            }

            if (count > 0) {
                double range = std::sqrt(cloud->points[i].x * cloud->points[i].x +
                                         cloud->points[i].y * cloud->points[i].y +
                                         cloud->points[i].z * cloud->points[i].z);
                if (range > 0.1) {
                    cloud->points[i].curvature = static_cast<float>(
                        std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / range);
                }
            }
        }
    }
}

void PointCloudConverter::computeNormalPCA(PointCloudPtr& cloud) {

    if (cloud->size() < 10) {
        return;
    }

    // 构建KdTree
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud);

    const int k = 10;
    std::vector<int> point_idx(k);
    std::vector<float> point_sqr_dist(k);

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (kdtree.nearestKSearch(cloud->points[i], k, point_idx, point_sqr_dist) > 2) {
            // 计算邻域点的协方差矩阵
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (int j = 0; j < static_cast<int>(point_idx.size()); ++j) {
                centroid += Eigen::Vector3d(cloud->points[point_idx[j]].x,
                                            cloud->points[point_idx[j]].y,
                                            cloud->points[point_idx[j]].z);
            }
            centroid /= point_idx.size();

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (int j = 0; j < static_cast<int>(point_idx.size()); ++j) {
                Eigen::Vector3d diff(cloud->points[point_idx[j]].x - centroid.x(),
                                      cloud->points[point_idx[j]].y - centroid.y(),
                                      cloud->points[point_idx[j]].z - centroid.z());
                covariance += diff * diff.transpose();
            }

            // SVD分解获取法向量（最小特征值对应的特征向量）
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
            Eigen::Vector3d normal = solver.eigenvectors().col(0);

            // 确保法向量朝上（z分量为正）
            if (normal.z() < 0) {
                normal = -normal;
            }

            cloud->points[i].normal_x = static_cast<float>(normal.x());
            cloud->points[i].normal_y = static_cast<float>(normal.y());
            cloud->points[i].normal_z = static_cast<float>(normal.z());
        }
    }
}

void PointCloudConverter::computeNormalRingBased(
    PointCloudPtr& cloud,
    const std::vector<std::vector<int>>& ring_indices) {

    // 对每条扫描线计算法向量
    for (const auto& ring : ring_indices) {
        if (ring.size() < 3) {
            continue;
        }

        for (size_t i = 1; i < ring.size() - 1; ++i) {
            // 使用前后两个点计算法向量
            Eigen::Vector3d p0(cloud->points[ring[i - 1]].x,
                               cloud->points[ring[i - 1]].y,
                               cloud->points[ring[i - 1]].z);
            Eigen::Vector3d p1(cloud->points[ring[i]].x,
                               cloud->points[ring[i]].y,
                               cloud->points[ring[i]].z);
            Eigen::Vector3d p2(cloud->points[ring[i + 1]].x,
                               cloud->points[ring[i + 1]].y,
                               cloud->points[ring[i + 1]].z);

            // 计算切向量
            Eigen::Vector3d tangent = (p2 - p0).normalized();

            // 计算近似法向量（垂直于切向量，大致朝外）
            Eigen::Vector3d radial = p1.normalized();
            Eigen::Vector3d normal = tangent.cross(Eigen::Vector3d(0, 0, 1));
            if (normal.norm() < 0.1) {
                normal = tangent.cross(radial);
            }

            if (normal.norm() > 0.1) {
                normal.normalize();
                cloud->points[ring[i]].normal_x = static_cast<float>(normal.x());
                cloud->points[ring[i]].normal_y = static_cast<float>(normal.y());
                cloud->points[ring[i]].normal_z = static_cast<float>(normal.z());
            }
        }
    }
}

std::vector<std::vector<int>> PointCloudConverter::groupByRing(
    const PointCloudPtr& cloud,
    const std::vector<int>& ring_field,
    const PointCloudFormatInfo& format_info) {

    std::vector<std::vector<int>> ring_indices;

    if (ring_field.empty() || cloud->size() != ring_field.size()) {
        return ring_indices;
    }

    // 找出最大ring值
    int max_ring = 0;
    for (int r : ring_field) {
        max_ring = std::max(max_ring, r);
    }

    ring_indices.resize(max_ring + 1);

    // 按ring分组
    for (size_t i = 0; i < ring_field.size(); ++i) {
        if (ring_field[i] >= 0 && ring_field[i] <= max_ring) {
            ring_indices[ring_field[i]].push_back(i);
        }
    }

    return ring_indices;
}

void PointCloudConverter::setDefaultNormalAndCurvature(PointCloudPtr& cloud) {
    // 设置默认法向量（向上）和曲率（0）
    for (auto& point : cloud->points) {
        point.normal_x = 0.0f;
        point.normal_y = 0.0f;
        point.normal_z = 1.0f;
        point.curvature = 0.0f;
    }
}

std::vector<int> PointCloudConverter::getRingField(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    const PointCloudFormatInfo& format_info) {

    std::vector<int> ring_values;

    if (!format_info.has_ring) {
        return ring_values;
    }

    ring_values.reserve(msg->width * msg->height);

    // 查找ring字段偏移量
    int ring_offset = -1;
    for (const auto& field : msg->fields) {
        std::string name = field.name;
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        if (name == "ring" || name == "scan_line" || name == "scan_id") {
            ring_offset = field.offset;
            break;
        }
    }

    if (ring_offset < 0) {
        return ring_values;
    }

    // 提取ring值
    const uint8_t* data_ptr = msg->data.data();
    for (size_t i = 0; i < msg->width * msg->height; ++i) {
        const uint8_t* point_data = data_ptr + i * msg->point_step;
        uint16_t ring = extractField<uint16_t>(point_data, ring_offset);
        ring_values.push_back(static_cast<int>(ring));
    }

    return ring_values;
}

} // namespace fast_lio2_slam
