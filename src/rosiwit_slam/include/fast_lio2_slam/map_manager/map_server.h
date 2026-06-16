/**
 * @file map_server.h
 * @brief FAST-LIO2 SLAM - 地图服务模块
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 提供ROS2服务接口用于地图的保存、加载、查询等操作
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/map_manager/map_manager.h"

#include <memory>
#include <string>
#include <functional>

namespace fast_lio2_slam {

/**
 * @brief 地图服务配置
 */
struct MapServerConfig {
    // 服务名称
    std::string save_map_service = "/save_map";
    std::string load_map_service = "/load_map";
    std::string clear_map_service = "/clear_map";
    std::string get_map_service = "/get_map";
    std::string save_pcd_service = "/save_pcd";
    std::string get_metadata_service = "/get_map_metadata";
    
    // 发布话题
    std::string global_map_topic = "/global_map";
    std::string local_map_topic = "/local_map";
    std::string submap_marker_topic = "/submap_markers";
    std::string map_info_topic = "/map_info";
    
    // 发布频率
    double map_publish_rate = 0.5;     // Hz
    double marker_publish_rate = 1.0;  // Hz
    
    // 地图保存
    std::string default_save_path = "./map";
    std::string default_format = "pcd";
    bool auto_save_on_shutdown = true;
    double auto_save_interval = 60.0;  // 秒
    
    // 可视化
    double visualization_voxel_size = 0.5;
    bool show_submap_boundaries = true;
    bool show_trajectory = true;
};

/**
 * @brief 地图元数据消息 (用于发布)
 */
struct MapMetadataMsg {
    std::string map_name;
    std::string version;
    double created_time;
    double modified_time;
    int64_t total_points;
    int64_t total_submaps;
    int64_t total_sessions;
    std::vector<double> bounds_min;   // [x, y, z]
    std::vector<double> bounds_max;
    std::vector<double> center;
    double quality_score;
    double coverage_area;
    double memory_usage_mb;
};

/**
 * @brief 地图服务类
 * 
 * 提供ROS2服务接口用于地图管理:
 * - 保存/加载地图
 * - 清空地图
 * - 获取地图元数据
 * - 发布地图可视化
 */
class MapServer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using Ptr = std::shared_ptr<MapServer>;
    
    /**
     * @brief 构造函数
     */
    MapServer(rclcpp::Node::SharedPtr node, 
              std::shared_ptr<MapManager> map_manager);
    
    /**
     * @brief 析构函数
     */
    ~MapServer();
    
    /**
     * @brief 初始化服务
     */
    void initialize(const MapServerConfig& config);
    
    /**
     * @brief 启动服务
     */
    void start();
    
    /**
     * @brief 停止服务
     */
    void stop();
    
    // ============ 发布接口 ============
    
    /**
     * @brief 发布全局地图
     */
    void publishGlobalMap();
    
    /**
     * @brief 发布局部地图
     * @param center 中心位置
     * @param radius 半径
     */
    void publishLocalMap(const Eigen::Vector3d& center, double radius);
    
    /**
     * @brief 发布子地图边界可视化
     */
    void publishSubmapMarkers();
    
    /**
     * @brief 发布地图元数据
     */
    void publishMetadata();
    
    /**
     * @brief 发布轨迹
     * @param poses 位姿序列
     */
    void publishTrajectory(const std::vector<SE3d>& poses);
    
    // ============ 服务接口 ============
    
    /**
     * @brief 保存地图 (手动触发)
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
     * @brief 清空地图
     */
    void clearMap();
    
    /**
     * @brief 获取地图元数据
     * @return 元数据消息
     */
    MapMetadataMsg getMetadata() const;

private:
    // ============ 服务回调 ============
    
    void handleSaveMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void handleLoadMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void handleClearMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void handleGetMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void handleSavePcd(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // ============ 定时器回调 ============
    
    void mapPublishTimerCallback();
    void markerPublishTimerCallback();
    void autoSaveTimerCallback();
    
    // ============ 辅助函数 ============
    
    /**
     * @brief 创建子地图边界Marker
     */
    visualization_msgs::msg::Marker createSubmapMarker(
        const Submap& submap, int id);
    
    /**
     * @brief 点云转ROS消息
     */
    sensor_msgs::msg::PointCloud2 toPointCloud2Msg(
        const PointCloudPtr& cloud, 
        const std::string& frame_id,
        double timestamp);
    
    /**
     * @brief 记录日志
     */
    template<typename... Args>
    void logInfo(const char* format, Args... args);
    
    template<typename... Args>
    void logWarn(const char* format, Args... args);
    
    template<typename... Args>
    void logError(const char* format, Args... args);

private:
    // ROS2节点
    rclcpp::Node::SharedPtr node_;
    
    // 地图管理器
    std::shared_ptr<MapManager> map_manager_;
    
    // 配置
    MapServerConfig config_;
    
    // ROS2服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_service_;
    
    // ROS2发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    rclcpp::TimerBase::SharedPtr marker_publish_timer_;
    rclcpp::TimerBase::SharedPtr auto_save_timer_;
    
    // 当前位置 (用于局部地图发布)
    Eigen::Vector3d current_position_;
    std::mutex position_mutex_;
    
    // 运行标志
    std::atomic<bool> running_{false};
};

// ============ 内联实现 ============

inline MapServer::MapServer(rclcpp::Node::SharedPtr node,
                            std::shared_ptr<MapManager> map_manager)
    : node_(node), map_manager_(map_manager) {
}

inline MapServer::~MapServer() {
    stop();
}

inline void MapServer::initialize(const MapServerConfig& config) {
    config_ = config;
}

inline void MapServer::start() {
    running_ = true;
    
    // 创建服务
    save_map_service_ = node_->create_service<std_srvs::srv::Trigger>(
        config_.save_map_service,
        std::bind(&MapServer::handleSaveMap, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    load_map_service_ = node_->create_service<std_srvs::srv::Trigger>(
        config_.load_map_service,
        std::bind(&MapServer::handleLoadMap, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    clear_map_service_ = node_->create_service<std_srvs::srv::Trigger>(
        config_.clear_map_service,
        std::bind(&MapServer::handleClearMap, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    get_map_service_ = node_->create_service<std_srvs::srv::Trigger>(
        config_.get_map_service,
        std::bind(&MapServer::handleGetMap, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    save_pcd_service_ = node_->create_service<std_srvs::srv::Trigger>(
        config_.save_pcd_service,
        std::bind(&MapServer::handleSavePcd, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // 创建发布者
    global_map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.global_map_topic, 1);
    
    local_map_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config_.local_map_topic, 1);
    
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        config_.submap_marker_topic, 1);
    
    // 创建定时器
    map_publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / config_.map_publish_rate)),
        std::bind(&MapServer::mapPublishTimerCallback, this));
    
    marker_publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / config_.marker_publish_rate)),
        std::bind(&MapServer::markerPublishTimerCallback, this));
    
    if (config_.auto_save_interval > 0) {
        auto_save_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 * config_.auto_save_interval)),
            std::bind(&MapServer::autoSaveTimerCallback, this));
    }
    
    RCLCPP_INFO(node_->get_logger(), "MapServer started");
}

inline void MapServer::stop() {
    running_ = false;
    
    if (config_.auto_save_on_shutdown && map_manager_) {
        RCLCPP_INFO(node_->get_logger(), "Auto-saving map before shutdown...");
        saveMap(config_.default_save_path, config_.default_format);
    }
}

inline void MapServer::publishGlobalMap() {
    if (!map_manager_) return;
    
    auto cloud = map_manager_->getVisualizationCloud(config_.visualization_voxel_size);
    if (!cloud || cloud->empty()) return;
    
    auto msg = toPointCloud2Msg(cloud, "map", node_->now().seconds());
    global_map_publisher_->publish(msg);
}

inline void MapServer::publishLocalMap(const Eigen::Vector3d& center, double radius) {
    if (!map_manager_) return;
    
    SE3d pose = SE3d::trans(center.x(), center.y(), center.z());
    auto cloud = map_manager_->getLocalMap(pose, radius);
    if (!cloud || cloud->empty()) return;
    
    auto msg = toPointCloud2Msg(cloud, "map", node_->now().seconds());
    local_map_publisher_->publish(msg);
}

inline void MapServer::mapPublishTimerCallback() {
    publishGlobalMap();
}

inline void MapServer::markerPublishTimerCallback() {
    publishSubmapMarkers();
}

inline void MapServer::autoSaveTimerCallback() {
    if (map_manager_) {
        RCLCPP_INFO(node_->get_logger(), "Auto-saving map...");
        saveMap(config_.default_save_path, config_.default_format);
    }
}

inline void MapServer::handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    bool success = saveMap(config_.default_save_path, config_.default_format);
    response->success = success;
    response->message = success ? "Map saved successfully" : "Failed to save map";
}

inline void MapServer::handleLoadMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    bool success = loadMap(config_.default_save_path, false);
    response->success = success;
    response->message = success ? "Map loaded successfully" : "Failed to load map";
}

inline void MapServer::handleClearMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    clearMap();
    response->success = true;
    response->message = "Map cleared";
}

inline void MapServer::handleGetMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    publishGlobalMap();
    response->success = true;
    response->message = "Map published";
}

inline void MapServer::handleSavePcd(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    bool success = saveMap(config_.default_save_path, "pcd");
    response->success = success;
    response->message = success ? "PCD saved successfully" : "Failed to save PCD";
}

inline sensor_msgs::msg::PointCloud2 MapServer::toPointCloud2Msg(
    const PointCloudPtr& cloud,
    const std::string& frame_id,
    double timestamp) {
    
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    return msg;
}

inline bool MapServer::saveMap(const std::string& path, const std::string& format) {
    if (!map_manager_) return false;
    return map_manager_->saveMap(path, format);
}

inline bool MapServer::loadMap(const std::string& path, bool merge) {
    if (!map_manager_) return false;
    return map_manager_->loadMap(path, merge);
}

inline void MapServer::clearMap() {
    if (map_manager_) {
        map_manager_->reset();
    }
}

inline MapMetadataMsg MapServer::getMetadata() const {
    MapMetadataMsg msg;
    if (!map_manager_) return msg;
    
    auto metadata = map_manager_->getMetadata();
    msg.map_name = metadata.map_name;
    msg.version = metadata.version;
    msg.created_time = metadata.created_time;
    msg.modified_time = metadata.modified_time;
    msg.total_points = metadata.total_points;
    msg.total_submaps = metadata.total_submaps;
    msg.total_sessions = metadata.total_sessions;
    msg.bounds_min = {metadata.map_center.x() - metadata.map_size.x()/2,
                       metadata.map_center.y() - metadata.map_size.y()/2,
                       metadata.map_center.z() - metadata.map_size.z()/2};
    msg.bounds_max = {metadata.map_center.x() + metadata.map_size.x()/2,
                       metadata.map_center.y() + metadata.map_size.y()/2,
                       metadata.map_center.z() + metadata.map_size.z()/2};
    msg.center = {metadata.map_center.x(), metadata.map_center.y(), metadata.map_center.z()};
    msg.quality_score = metadata.map_quality_score;
    msg.coverage_area = metadata.avg_point_density;
    
    auto stats = map_manager_->getStatistics();
    msg.memory_usage_mb = stats.memory_usage_mb;
    
    return msg;
}

} // namespace fast_lio2_slam