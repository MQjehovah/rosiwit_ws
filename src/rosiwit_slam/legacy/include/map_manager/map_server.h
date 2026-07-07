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

} // namespace fast_lio2_slam
