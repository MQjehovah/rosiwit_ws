/**
 * @file map_server.cpp
 * @brief FAST-LIO2 SLAM - 地图服务模块实现
 */

#include "fast_lio2_slam/map_manager/map_server.h"

#include <chrono>

namespace fast_lio2_slam {

MapServer::MapServer(rclcpp::Node::SharedPtr node,
                     std::shared_ptr<MapManager> map_manager)
    : node_(node), map_manager_(map_manager) {
}

MapServer::~MapServer() {
    stop();
}

void MapServer::initialize(const MapServerConfig& config) {
    config_ = config;
}

void MapServer::start() {
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

void MapServer::stop() {
    running_ = false;

    if (config_.auto_save_on_shutdown && map_manager_) {
        RCLCPP_INFO(node_->get_logger(), "Auto-saving map before shutdown...");
        saveMap(config_.default_save_path, config_.default_format);
    }
}

void MapServer::publishGlobalMap() {
    if (!map_manager_) return;

    auto cloud = map_manager_->getVisualizationCloud(config_.visualization_voxel_size);
    if (!cloud || cloud->empty()) return;

    auto msg = toPointCloud2Msg(cloud, "map", node_->now().seconds());
    global_map_publisher_->publish(msg);
}

void MapServer::publishLocalMap(const Eigen::Vector3d& center, double radius) {
    if (!map_manager_) return;

    SE3d pose = SE3d::trans(center.x(), center.y(), center.z());
    auto cloud = map_manager_->getLocalMap(pose, radius);
    if (!cloud || cloud->empty()) return;

    auto msg = toPointCloud2Msg(cloud, "map", node_->now().seconds());
    local_map_publisher_->publish(msg);
}

void MapServer::mapPublishTimerCallback() {
    publishGlobalMap();
}

void MapServer::markerPublishTimerCallback() {
    publishSubmapMarkers();
}

void MapServer::autoSaveTimerCallback() {
    if (map_manager_) {
        RCLCPP_INFO(node_->get_logger(), "Auto-saving map...");
        saveMap(config_.default_save_path, config_.default_format);
    }
}

void MapServer::handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    bool success = saveMap(config_.default_save_path, config_.default_format);
    response->success = success;
    response->message = success ? "Map saved successfully" : "Failed to save map";
}

void MapServer::handleLoadMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    bool success = loadMap(config_.default_save_path, false);
    response->success = success;
    response->message = success ? "Map loaded successfully" : "Failed to load map";
}

void MapServer::handleClearMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    clearMap();
    response->success = true;
    response->message = "Map cleared";
}

void MapServer::handleGetMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    publishGlobalMap();
    response->success = true;
    response->message = "Map published";
}

void MapServer::handleSavePcd(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    bool success = saveMap(config_.default_save_path, "pcd");
    response->success = success;
    response->message = success ? "PCD saved successfully" : "Failed to save PCD";
}

sensor_msgs::msg::PointCloud2 MapServer::toPointCloud2Msg(
    const PointCloudPtr& cloud,
    const std::string& frame_id,
    double timestamp) {

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    return msg;
}

bool MapServer::saveMap(const std::string& path, const std::string& format) {
    if (!map_manager_) return false;
    return map_manager_->saveMap(path, format);
}

bool MapServer::loadMap(const std::string& path, bool merge) {
    if (!map_manager_) return false;
    return map_manager_->loadMap(path, merge);
}

void MapServer::clearMap() {
    if (map_manager_) {
        map_manager_->reset();
    }
}

MapMetadataMsg MapServer::getMetadata() const {
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
