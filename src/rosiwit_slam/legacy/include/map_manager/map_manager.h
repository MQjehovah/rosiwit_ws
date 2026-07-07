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

} // namespace fast_lio2_slam
