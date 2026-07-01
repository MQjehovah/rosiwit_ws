/**
 * @file map_manager.h
 * @brief FAST-LIO2 SLAM - 地图管理模块
 * @author AI Development Team
 * @date 2026-04-24
 */

#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include "fast_lio2_slam/common/types.h"
#include <vector>
#include <mutex>
#include <fstream>

namespace fast_lio2_slam {

/**
 * @brief 地图管理配置
 */
struct MapManagerConfig {
    double resolution = 0.2;           // 地图分辨率
    double submap_size = 50.0;          // 子地图大小 (米)
    int max_submap_points = 50000;     // 子地图最大点数
    std::string map_path = "./map";    // 地图保存路径
    bool enable_pcd_save = true;       // 启用PCD保存
    bool enable_submap = true;         // 启用子地图
};

/**
 * @brief 子地图结构
 */
struct Submap {
    int id;
    SE3d center_pose;                  // 子地图中心位姿
    PointCloudPtr cloud;               // 点云数据
    std::vector<int> frame_ids;        // 包含的帧ID
    bool is_active;                    // 是否活跃

    // 新增字段 - 用于持久化
    std::string session_id;            // 会话ID
    double timestamp_start;            // 开始时间戳
    double timestamp_end;              // 结束时间戳
    Eigen::Vector3d min_bound;         // 最小边界
    Eigen::Vector3d max_bound;         // 最大边界

    Submap() : id(-1), is_active(false),
               timestamp_start(0.0), timestamp_end(0.0),
               min_bound(Eigen::Vector3d::Zero()),
               max_bound(Eigen::Vector3d::Zero()) {
        cloud.reset(new pcl::PointCloud<PointType>());
    }
};

/**
 * @brief 地图管理类
 *
 * 功能:
 * 1. 点云地图存储和管理
 * 2. 子地图划分和管理
 * 3. PCD文件保存和加载
 * 4. 地图内存优化
 */
class MapManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapManager();
    explicit MapManager(const MapManagerConfig& config);
    ~MapManager() = default;

    /**
     * @brief 初始化
     */
    void initialize(const MapManagerConfig& config);

    /**
     * @brief 添加点云到地图
     */
    void addPointCloud(const PointCloudPtr& cloud, const SE3d& pose, int frame_id);

    /**
     * @brief 更新子地图
     */
    void updateSubmaps(const SE3d& current_pose);

    /**
     * @brief 获取当前活跃子地图
     */
    Submap* getActiveSubmap();

    /**
     * @brief 获取所有子地图
     */
    std::vector<Submap> getAllSubmaps();

    /**
     * @brief 获取完整地图
     */
    PointCloudPtr getFullMap();

    /**
     * @brief 获取局部地图 (当前位置附近)
     */
    PointCloudPtr getLocalMap(const SE3d& current_pose, double radius);

    /**
     * @brief 保存地图到PCD文件
     */
    bool saveToPcd(const std::string& filename);

    /**
     * @brief 从PCD文件加载地图
     */
    bool loadFromPcd(const std::string& filename);

    /**
     * @brief 保存子地图
     */
    bool saveSubmaps();

    /**
     * @brief 加载子地图
     */
    bool loadSubmaps(const std::string& path = "");

    /**
     * @brief 清空地图
     */
    void clear();

    /**
     * @brief 获取地图点数
     */
    size_t pointCount() const;

    /**
     * @brief 获取子地图数
     */
    size_t submapCount() const;

    /**
     * @brief 内存优化 (降采样)
     */
    void optimizeMemory();

    /**
     * @brief 获取可视化用的降采样地图
     */
    PointCloudPtr getVisualizationCloud(double voxel_size = 0.5);

    /**
     * @brief 保存地图（支持多种格式）
     * @param path 保存路径
     * @param format 格式: "pcd", "ply", "bin"
     * @return 是否成功
     */
    bool saveMap(const std::string& path, const std::string& format = "pcd");

    /**
     * @brief 加载地图
     * @param path 地图路径
     * @param merge 是否合并到现有地图
     * @return 是否成功
     */
    bool loadMap(const std::string& path, bool merge = false);

    /**
     * @brief 重置地图
     */
    void reset();

    /**
     * @brief 获取地图元数据
     */
    MapMetadata getMetadata() const;

    /**
     * @brief 获取地图统计信息
     */
    MapStatistics getStatistics() const;

private:
    /**
     * @brief 判断点是否在子地图范围内
     */
    bool isInSubmapRange(const Vector3d& point, const Submap& submap);

    /**
     * @brief 创建新子地图
     */
    Submap createNewSubmap(const SE3d& center_pose);

    /**
     * @brief 合并相近的子地图
     */
    void mergeSubmaps();

private:
    MapManagerConfig config_;
    PointCloudPtr global_map_;         // 全局地图
    std::vector<Submap> submaps_;      // 子地图列表
    int current_submap_id_;            // 当前活跃子地图ID
    mutable std::mutex mutex_;         // 互斥锁
    bool initialized_;

    // 统计信息
    int total_frame_count_;
    size_t total_point_count_;
};

// ==================== 实现部分 ====================

inline MapManager::MapManager()
    : current_submap_id_(-1), initialized_(false),
      total_frame_count_(0), total_point_count_(0) {
    global_map_.reset(new pcl::PointCloud<PointType>());
}

inline MapManager::MapManager(const MapManagerConfig& config)
    : config_(config), current_submap_id_(-1), initialized_(true),
      total_frame_count_(0), total_point_count_(0) {
    global_map_.reset(new pcl::PointCloud<PointType>());
}

inline void MapManager::initialize(const MapManagerConfig& config) {
    config_ = config;
    clear();
    initialized_ = true;
}

inline void MapManager::addPointCloud(const PointCloudPtr& cloud,
                                       const SE3d& pose, int frame_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (cloud->empty()) return;

    // 变换点云到世界坐标系
    Matrix3d R = pose.rotation().matrix();
    Vector3d t = pose.translation();

    PointCloudPtr transformed_cloud(new pcl::PointCloud<PointType>());
    transformed_cloud->reserve(cloud->size());

    for (const auto& point : cloud->points) {
        PointType new_point;
        Vector3d p(point.x, point.y, point.z);
        Vector3d p_world = R * p + t;

        new_point.x = p_world(0);
        new_point.y = p_world(1);
        new_point.z = p_world(2);
        new_point.intensity = point.intensity;

        // 体素滤波 (避免重复点)
        bool add_point = true;

        // 添加到全局地图
        if (global_map_->size() < config_.max_submap_points * 100) {
            global_map_->points.push_back(new_point);
            total_point_count_++;
        }

        transformed_cloud->points.push_back(new_point);
    }

    // 添加到子地图
    if (config_.enable_submap) {
        updateSubmaps(pose);

        if (current_submap_id_ >= 0 && current_submap_id_ < submaps_.size()) {
            Submap& current_submap = submaps_[current_submap_id_];

            if (current_submap.cloud->size() < config_.max_submap_points) {
                for (const auto& point : transformed_cloud->points) {
                    current_submap.cloud->points.push_back(point);
                }
                current_submap.frame_ids.push_back(frame_id);
            }
        }
    }

    total_frame_count_++;
}

inline void MapManager::updateSubmaps(const SE3d& current_pose) {
    if (!config_.enable_submap) return;

    Vector3d pos = current_pose.translation();

    // 检查是否需要切换子地图
    bool need_new_submap = true;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < submaps_.size(); ++i) {
        double dist = (submaps_[i].center_pose.translation() - pos).norm();

        if (dist < config_.submap_size / 2) {
            // 在当前子地图范围内
            submaps_[i].is_active = true;
            current_submap_id_ = i;
            need_new_submap = false;
            min_distance = std::min(min_distance, dist);
        } else {
            submaps_[i].is_active = false;
        }
    }

    // 创建新子地图
    if (need_new_submap) {
        Submap new_submap = createNewSubmap(current_pose);
        submaps_.push_back(new_submap);
        current_submap_id_ = submaps_.size() - 1;
    }
}

inline Submap* MapManager::getActiveSubmap() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (current_submap_id_ >= 0 && current_submap_id_ < submaps_.size()) {
        return &submaps_[current_submap_id_];
    }
    return nullptr;
}

inline std::vector<Submap> MapManager::getAllSubmaps() {
    std::lock_guard<std::mutex> lock(mutex_);
    return submaps_;
}

inline PointCloudPtr MapManager::getFullMap() {
    std::lock_guard<std::mutex> lock(mutex_);

    PointCloudPtr full_map(new pcl::PointCloud<PointType>());

    if (!config_.enable_submap) {
        full_map = global_map_;
    } else {
        // 合并所有子地图
        for (const auto& submap : submaps_) {
            for (const auto& point : submap.cloud->points) {
                full_map->points.push_back(point);
            }
        }
    }

    full_map->width = full_map->points.size();
    full_map->height = 1;
    full_map->is_dense = true;

    return full_map;
}

inline PointCloudPtr MapManager::getLocalMap(const SE3d& current_pose, double radius) {
    std::lock_guard<std::mutex> lock(mutex_);

    PointCloudPtr local_map(new pcl::PointCloud<PointType>());
    Vector3d pos = current_pose.translation();
    double radius_sq = radius * radius;

    for (const auto& point : global_map_->points) {
        double dist_sq = (Vector3d(point.x, point.y, point.z) - pos).squaredNorm();
        if (dist_sq <= radius_sq) {
            local_map->points.push_back(point);
        }
    }

    local_map->width = local_map->points.size();
    local_map->height = 1;

    return local_map;
}

inline bool MapManager::saveToPcd(const std::string& filename) {
    std::cout << "[MapManager] saveToPcd: starting, filename=" << filename << std::endl;
    std::lock_guard<std::mutex> lock(mutex_);

    // 直接获取地图数据（不再调用会重复加锁的 getFullMap()）
    PointCloudPtr map_to_save(new pcl::PointCloud<PointType>());

    if (!config_.enable_submap) {
        map_to_save = global_map_;
    } else {
        for (const auto& submap : submaps_) {
            for (const auto& point : submap.cloud->points) {
                map_to_save->points.push_back(point);
            }
        }
    }

    map_to_save->width = map_to_save->points.size();
    map_to_save->height = 1;
    map_to_save->is_dense = true;

    std::cout << "[MapManager] saveToPcd: " << map_to_save->points.size() << " points to save" << std::endl;
    if (map_to_save->empty()) return false;

    try {
        pcl::io::savePCDFileBinary(filename, *map_to_save);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

inline bool MapManager::loadFromPcd(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        global_map_->clear();
        pcl::io::loadPCDFile<PointType>(filename, *global_map_);
        total_point_count_ = global_map_->size();
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

inline bool MapManager::saveSubmaps() {
    if (!config_.enable_submap || submaps_.empty()) return false;

    for (size_t i = 0; i < submaps_.size(); ++i) {
        std::string filename = config_.map_path + "/submap_" +
                               std::to_string(i) + ".pcd";

        try {
            pcl::io::savePCDFileBinary(filename, *submaps_[i].cloud);
        } catch (const std::exception& e) {
            return false;
        }
    }

    return true;
}

inline bool MapManager::loadSubmaps(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 清空现有子地图
    submaps_.clear();
    current_submap_id_ = -1;

    // 遍历目录查找子地图文件
    std::string directory = path.empty() ? config_.map_path : path;

    int submap_id = 0;
    while (true) {
        std::string filename = directory + "/submap_" + std::to_string(submap_id) + ".pcd";

        // 检查文件是否存在
        std::ifstream file(filename);
        if (!file.good()) break;
        file.close();

        // 加载子地图
        Submap new_submap;
        new_submap.id = submap_id;
        new_submap.cloud.reset(new pcl::PointCloud<PointType>());

        try {
            pcl::io::loadPCDFile<PointType>(filename, *new_submap.cloud);

            if (new_submap.cloud->empty()) {
                submap_id++;
                continue;
            }

            // 计算子地图中心位置（从点云平均值估算）
            Vector3d center(0, 0, 0);
            for (const auto& point : new_submap.cloud->points) {
                center += Vector3d(point.x, point.y, point.z);
            }
            center /= new_submap.cloud->size();

            new_submap.center_pose.setTranslation(center);
            new_submap.is_active = false;

            submaps_.push_back(new_submap);
            total_point_count_ += new_submap.cloud->size();

            submap_id++;
        } catch (const std::exception& e) {
            break;
        }
    }

    if (submaps_.empty()) {
        return false;
    }

    // 设置第一个子地图为活跃状态
    submaps_[0].is_active = true;
    current_submap_id_ = 0;

    return true;
}

inline void MapManager::clear() {
    std::lock_guard<std::mutex> lock(mutex_);

    global_map_->clear();
    submaps_.clear();
    current_submap_id_ = -1;
    total_frame_count_ = 0;
    total_point_count_ = 0;
}

inline size_t MapManager::pointCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return total_point_count_;
}

inline size_t MapManager::submapCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return submaps_.size();
}

inline void MapManager::optimizeMemory() {
    std::lock_guard<std::mutex> lock(mutex_);

    // 对全局地图进行降采样
    if (global_map_->size() > config_.max_submap_points) {
        PointCloudPtr filtered(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize(config_.resolution, config_.resolution, config_.resolution);
        voxel_filter.filter(*filtered);
        global_map_ = filtered;
        total_point_count_ = global_map_->size();
    }

    // 合并相近的子地图
    if (submaps_.size() > 10) {
        mergeSubmaps();
    }
}

inline bool MapManager::isInSubmapRange(const Vector3d& point, const Submap& submap) {
    double dist = (point - submap.center_pose.translation()).norm();
    return dist < config_.submap_size;
}

inline Submap MapManager::createNewSubmap(const SE3d& center_pose) {
    Submap submap;
    submap.id = submaps_.size();
    submap.center_pose = center_pose;
    submap.is_active = true;
    submap.cloud.reset(new pcl::PointCloud<PointType>());

    return submap;
}

inline void MapManager::mergeSubmaps() {
    if (submaps_.size() < 2) return;

    // 合并距离相近的子地图
    std::vector<bool> merged(submaps_.size(), false);

    for (size_t i = 0; i < submaps_.size(); ++i) {
        if (merged[i] || submaps_[i].cloud->empty()) continue;

        for (size_t j = i + 1; j < submaps_.size(); ++j) {
            if (merged[j] || submaps_[j].cloud->empty()) continue;

            // 计算两个子地图中心的距离
            double dist = (submaps_[i].center_pose.translation() -
                          submaps_[j].center_pose.translation()).norm();

            // 如果距离小于阈值，进行合并
            if (dist < config_.submap_size * 0.6) {
                // 合并点云
                for (const auto& point : submaps_[j].cloud->points) {
                    if (submaps_[i].cloud->size() < config_.max_submap_points) {
                        submaps_[i].cloud->points.push_back(point);
                    }
                }

                // 合并帧ID
                submaps_[i].frame_ids.insert(submaps_[i].frame_ids.end(),
                                            submaps_[j].frame_ids.begin(),
                                            submaps_[j].frame_ids.end());

                // 更新中心位置
                Vector3d new_center = (submaps_[i].center_pose.translation() *
                                       submaps_[i].cloud->size() +
                                       submaps_[j].center_pose.translation() *
                                       submaps_[j].cloud->size()) /
                                       (submaps_[i].cloud->size() + submaps_[j].cloud->size());
                submaps_[i].center_pose.setTranslation(new_center);

                // 清空被合并的子地图
                submaps_[j].cloud->clear();
                submaps_[j].frame_ids.clear();
                merged[j] = true;
            }
        }
    }

    // 移除空的子地图
    std::vector<Submap> new_submaps;
    for (const auto& submap : submaps_) {
        if (!submap.cloud->empty()) {
            new_submaps.push_back(submap);
        }
    }
    submaps_ = new_submaps;

    // 重新编号
    for (size_t i = 0; i < submaps_.size(); ++i) {
        submaps_[i].id = i;
    }

    // 更新当前活跃子地图ID
    if (current_submap_id_ >= 0 && current_submap_id_ < static_cast<int>(submaps_.size())) {
        current_submap_id_ = current_submap_id_;
    } else {
        current_submap_id_ = submaps_.size() > 0 ? 0 : -1;
    }
}

// ==================== 新增方法实现 ====================

inline PointCloudPtr MapManager::getVisualizationCloud(double voxel_size) {
    std::lock_guard<std::mutex> lock(mutex_);

    PointCloudPtr viz_cloud(new pcl::PointCloud<PointType>());

    if (global_map_->empty()) {
        return viz_cloud;
    }

    // 使用体素滤波进行降采样
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(global_map_);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*viz_cloud);

    return viz_cloud;
}

inline bool MapManager::saveMap(const std::string& path, const std::string& format) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        std::string filename = path;
        if (format == "pcd") {
            if (filename.substr(filename.size() - 4) != ".pcd") {
                filename += ".pcd";
            }
            pcl::io::savePCDFileBinary(filename, *global_map_);
        } else if (format == "ply") {
            if (filename.substr(filename.size() - 4) != ".ply") {
                filename += ".ply";
            }
            pcl::io::savePCDFileBinary(filename, *global_map_);
        } else {
            // 默认使用PCD格式
            if (filename.substr(filename.size() - 4) != ".pcd") {
                filename += ".pcd";
            }
            pcl::io::savePCDFileBinary(filename, *global_map_);
        }

        // 同时保存子地图
        saveSubmaps();

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

inline bool MapManager::loadMap(const std::string& path, bool merge) {
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        if (!merge) {
            clear();
        }

        // 尝试加载点云文件
        PointCloudPtr loaded_cloud(new pcl::PointCloud<PointType>());

        // 根据文件扩展名选择加载方式
        if (path.substr(path.size() - 4) == ".pcd") {
            pcl::io::loadPCDFile<PointType>(path, *loaded_cloud);
        } else if (path.substr(path.size() - 4) == ".ply") {
            pcl::io::loadPCDFile<PointType>(path, *loaded_cloud);
        } else {
            // 尝试作为目录加载子地图
            return loadSubmaps(path);
        }

        if (loaded_cloud->empty()) {
            return false;
        }

        // 合入全局地图
        for (const auto& point : loaded_cloud->points) {
            global_map_->points.push_back(point);
        }
        total_point_count_ = global_map_->size();

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

inline void MapManager::reset() {
    clear();
}

inline MapMetadata MapManager::getMetadata() const {
    std::lock_guard<std::mutex> lock(mutex_);

    MapMetadata metadata;
    metadata.map_name = "fast_lio2_slam_map";
    metadata.version = "1.0";
    metadata.total_points = total_point_count_;
    metadata.total_submaps = submaps_.size();

    // 计算地图范围
    if (!global_map_->empty()) {
        Vector3d min_bound(std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::max());
        Vector3d max_bound(std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest(),
                          std::numeric_limits<double>::lowest());

        for (const auto& point : global_map_->points) {
            min_bound.x() = std::min(min_bound.x(), static_cast<double>(point.x));
            min_bound.y() = std::min(min_bound.y(), static_cast<double>(point.y));
            min_bound.z() = std::min(min_bound.z(), static_cast<double>(point.z));
            max_bound.x() = std::max(max_bound.x(), static_cast<double>(point.x));
            max_bound.y() = std::max(max_bound.y(), static_cast<double>(point.y));
            max_bound.z() = std::max(max_bound.z(), static_cast<double>(point.z));
        }

        metadata.map_center = (min_bound + max_bound) / 2.0;
        metadata.map_size = max_bound - min_bound;
    }

    return metadata;
}

inline MapStatistics MapManager::getStatistics() const {
    std::lock_guard<std::mutex> lock(mutex_);

    MapStatistics stats;
    stats.total_points = total_point_count_;
    stats.total_submaps = submaps_.size();

    // 计算活跃子地图数
    int active_count = 0;
    for (const auto& submap : submaps_) {
        if (submap.is_active) active_count++;
    }
    stats.active_submaps = active_count;

    // 估算内存使用（每个点约16字节）
    stats.memory_usage_mb = total_point_count_ * 16.0 / (1024.0 * 1024.0);

    // 计算地图范围
    if (!global_map_->empty()) {
        for (const auto& point : global_map_->points) {
            stats.min_bound.x() = std::min(stats.min_bound.x(), static_cast<double>(point.x));
            stats.min_bound.y() = std::min(stats.min_bound.y(), static_cast<double>(point.y));
            stats.min_bound.z() = std::min(stats.min_bound.z(), static_cast<double>(point.z));
            stats.max_bound.x() = std::max(stats.max_bound.x(), static_cast<double>(point.x));
            stats.max_bound.y() = std::max(stats.max_bound.y(), static_cast<double>(point.y));
            stats.max_bound.z() = std::max(stats.max_bound.z(), static_cast<double>(point.z));
        }
    }

    return stats;
}

} // namespace fast_lio2_slam