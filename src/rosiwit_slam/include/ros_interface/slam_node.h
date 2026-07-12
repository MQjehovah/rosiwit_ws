#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>
#include <mutex>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_types.h"
#include "rosiwit_slam/srv/save_map.hpp"
#include "rosiwit_slam/srv/load_map.hpp"
#include "rosiwit_slam/srv/save_grid_map.hpp"
#include "rosiwit_slam/srv/set_slam_mode.hpp"

namespace rosiwit_slam {

class SlamBase;    // forward declare for tryPopAndProcess

class SlamNode : public rclcpp::Node {
public:
    explicit SlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SlamNode() override = default;

private:
    struct NodeConfig {
        std::string imu_topic   = "/imu";
        std::string lidar_topic = "/velodyne_points";
        std::string body_frame  = "base_link";
        std::string world_frame = "odom";
        int    lidar_filter_num = 3;
        double lidar_min_range  = 0.5;
        double lidar_max_range  = 100.0;
        float  scan_period_ms   = 100.0f;   // LiDAR 扫描周期 (毫秒), 用于点时间戳
    };

    void loadParameters();
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timerCB();
    void mapTimerCB();
    void onOutput(const SlamOutput& out);
    void publish(const SlamOutput& out);
    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    builtin_interfaces::msg::Time toRosTime(double sec);

    void handleSaveMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::SaveMap::Request>, std::shared_ptr<rosiwit_slam::srv::SaveMap::Response>);
    void handleLoadMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::LoadMap::Request>, std::shared_ptr<rosiwit_slam::srv::LoadMap::Response>);
    void handleSaveGridMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::SaveGridMap::Request>, std::shared_ptr<rosiwit_slam::srv::SaveGridMap::Response>);
    void handleSetSlamMode(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::SetSlamMode::Request>, std::shared_ptr<rosiwit_slam::srv::SetSlamMode::Response>);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          m_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  m_lidar_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_init_pose_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub, m_world_cloud_pub, m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_map_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_nav_pub;
    rclcpp::TimerBase::SharedPtr m_process_timer, m_map_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf;
    rclcpp::Service<rosiwit_slam::srv::SaveMap>::SharedPtr       m_srv_save_map;
    rclcpp::Service<rosiwit_slam::srv::LoadMap>::SharedPtr       m_srv_load_map;
    rclcpp::Service<rosiwit_slam::srv::SaveGridMap>::SharedPtr   m_srv_save_grid_map;
    rclcpp::Service<rosiwit_slam::srv::SetSlamMode>::SharedPtr   m_srv_set_slam_mode;
    std::unique_ptr<ISlamAlgorithm> m_algo;
    ISlamAlgorithm* m_algo_raw = nullptr;       // non-owning, same as m_algo.get()
    NodeConfig m_cfg;
    std::mutex m_out_mutex;
    SlamOutput m_latest;
    bool m_have_output = false;
    nav_msgs::msg::Path m_path;
    bool m_grid_map_published = false;

    // EMA 平滑滤波（抑制 LIO 抖动，避免 costmap 中机器人中心跳进 lethal）
    PoseStamped m_smoothed_pose;
    bool m_has_smoothed = false;
    double m_ema_alpha = 0.3;
};

} // namespace rosiwit_slam
