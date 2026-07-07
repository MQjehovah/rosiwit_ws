/**
 * @file map_persistence.h
 * @brief FAST-LIO2 SLAM - 地图持久化模块
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 提供地图持久化功能，包括:
 * - 地图文件读写 (PCD, PLY, BIN)
 * - 元数据管理 (YAML)
 * - 地图项目导入导出
 * - 子地图序列化
 * - 位姿图存储 (G2O)
 * - 多会话数据管理
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <filesystem>

#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/map_manager/map_manager.h"

namespace fast_lio2_slam {

/**
 * @brief 地图持久化配置
 */
struct PersistenceConfig {
    // 文件格式
    std::string default_format = "pcd";  // pcd, ply, bin

    // 压缩选项
    bool compress_pcd = false;           // PCD压缩
    bool binary_mode = true;             // 二进制模式

    // 存储路径
    std::string map_directory = "./map";
    std::string submap_directory = "submaps";
    std::string session_directory = "sessions";
    std::string trajectory_directory = "trajectory";

    // 元数据文件名
    std::string metadata_file = "metadata.yaml";
    std::string pose_graph_file = "pose_graph.g2o";

    // 自动备份
    bool auto_backup = true;
    int max_backups = 5;
    std::string backup_directory = "backup";
};

/**
 * @brief 地图项目结构
 */
struct MapProject {
    std::string name;
    std::string path;
    std::string version;

    // 文件列表
    std::string global_map_file;
    std::string pose_graph_file;
    std::vector<std::string> submap_files;
    std::vector<std::string> session_files;
    std::string metadata_file;

    // 轨迹数据
    std::string poses_file;
    std::string timestamps_file;
    std::string keyframes_file;

    /**
     * @brief 检查项目完整性
     */
    bool checkIntegrity() const;
};

/**
 * @brief 地图持久化管理器
 *
 * 处理地图数据的持久化存储和加载
 */
class MapPersistence {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<MapPersistence>;

    /**
     * @brief 构造函数
     */
    MapPersistence();
    explicit MapPersistence(const PersistenceConfig& config);

    /**
     * @brief 初始化
     */
    void initialize(const PersistenceConfig& config);

    // ============ 地图保存 ============

    /**
     * @brief 保存点云地图
     * @param cloud 点云
     * @param path 文件路径
     * @param format 格式: "pcd", "ply", "bin"
     * @return 是否成功
     */
    bool savePointCloud(const PointCloudPtr& cloud,
                        const std::string& path,
                        const std::string& format = "pcd");

    /**
     * @brief 保存子地图
     * @param submap 子地图
     * @param path 文件路径
     * @return 是否成功
     */
    bool saveSubmap(const Submap& submap, const std::string& path);

    /**
     * @brief 保存元数据
     * @param metadata 元数据
     * @param path 文件路径
     * @return 是否成功
     */
    bool saveMetadata(const MapMetadata& metadata, const std::string& path);

    /**
     * @brief 保存会话信息
     * @param session 会话信息
     * @param path 文件路径
     * @return 是否成功
     */
    bool saveSession(const SessionInfo& session, const std::string& path);

    /**
     * @brief 保存位姿图
     * @param poses 位姿序列
     * @param constraints 约束
     * @param path 文件路径 (G2O格式)
     * @return 是否成功
     */
    bool savePoseGraph(const std::vector<SE3d>& poses,
                       const std::vector<std::tuple<int, int, SE3d>>& constraints,
                       const std::string& path);

    /**
     * @brief 保存轨迹
     * @param poses 位姿序列
     * @param timestamps 时间戳序列
     * @param path 目录路径
     * @return 是否成功
     */
    bool saveTrajectory(const std::vector<SE3d>& poses,
                        const std::vector<double>& timestamps,
                        const std::string& path);

    /**
     * @brief 导出完整地图项目
     * @param map_manager 地图管理器
     * @param directory 导出目录
     * @return 项目结构
     */
    MapProject exportMapProject(std::shared_ptr<MapManager> map_manager,
                                const std::string& directory);

    // ============ 地图加载 ============

    /**
     * @brief 加载点云地图
     * @param path 文件路径
     * @return 点云
     */
    PointCloudPtr loadPointCloud(const std::string& path);

    /**
     * @brief 加载子地图
     * @param path 文件路径
     * @return 子地图
     */
    Submap loadSubmap(const std::string& path);

    /**
     * @brief 加载元数据
     * @param path 文件路径
     * @return 元数据
     */
    MapMetadata loadMetadata(const std::string& path);

    /**
     * @brief 加载会话信息
     * @param path 文件路径
     * @return 会话信息
     */
    SessionInfo loadSession(const std::string& path);

    /**
     * @brief 加载位姿图
     * @param path 文件路径
     * @return [poses, constraints]
     */
    std::pair<std::vector<SE3d>, std::vector<std::tuple<int, int, SE3d>>>
    loadPoseGraph(const std::string& path);

    /**
     * @brief 加载轨迹
     * @param path 目录路径
     * @return [poses, timestamps]
     */
    std::pair<std::vector<SE3d>, std::vector<double>>
    loadTrajectory(const std::string& path);

    /**
     * @brief 导入地图项目
     * @param directory 项目目录
     * @param map_manager 地图管理器 (用于填充)
     * @return 是否成功
     */
    bool importMapProject(const std::string& directory,
                          std::shared_ptr<MapManager> map_manager);

    // ============ 文件管理 ============

    /**
     * @brief 创建地图项目目录结构
     * @param directory 目录路径
     * @return 是否成功
     */
    bool createProjectDirectory(const std::string& directory);

    /**
     * @brief 检查文件是否存在
     */
    bool fileExists(const std::string& path) const;

    /**
     * @brief 检查目录是否存在
     */
    bool directoryExists(const std::string& path) const;

    /**
     * @brief 创建备份
     * @param source 源目录
     * @return 备份路径
     */
    std::string createBackup(const std::string& source);

    /**
     * @brief 列出所有备份
     * @param directory 备份目录
     * @return 备份列表
     */
    std::vector<std::string> listBackups(const std::string& directory) const;

    /**
     * @brief 获取文件大小
     */
    size_t getFileSize(const std::string& path) const;

    /**
     * @brief 删除文件
     */
    bool deleteFile(const std::string& path);

    // ============ 工具函数 ============

    /**
     * @brief 获取支持的文件格式
     */
    std::vector<std::string> getSupportedFormats() const;

    /**
     * @brief 解析文件格式
     */
    std::string parseFormat(const std::string& path) const;

    /**
     * @brief 验证地图文件
     */
    bool validateMapFile(const std::string& path) const;

    /**
     * @brief 获取配置
     */
    PersistenceConfig getConfig() const { return config_; }

private:
    // 配置
    PersistenceConfig config_;

    // 辅助函数
    bool savePcd(const PointCloudPtr& cloud, const std::string& path, bool binary);
    bool savePly(const PointCloudPtr& cloud, const std::string& path, bool binary);
    bool saveBin(const PointCloudPtr& cloud, const std::string& path);

    PointCloudPtr loadPcd(const std::string& path);
    PointCloudPtr loadPly(const std::string& path);
    PointCloudPtr loadBin(const std::string& path);

    void writeYamlNode(YAML::Emitter& out, const std::string& key, const Eigen::Vector3d& vec);
    Eigen::Vector3d readYamlVector(const YAML::Node& node);
    SE3d readYamlSE3(const YAML::Node& node);
    void writeYamlSE3(YAML::Emitter& out, const SE3d& pose);
};

} // namespace fast_lio2_slam
