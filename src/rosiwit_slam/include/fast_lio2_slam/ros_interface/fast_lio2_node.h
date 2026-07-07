/**
 * @file fast_lio2_node.h
 * @brief FAST-LIO2 SLAM - ROS2节点主类 (基于 FAST-LIO2_ROS2 MapBuilder 核心)
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <mutex>
#include <deque>
#include <memory>

/**
 * @brief FAST-LIO2 SLAM ROS2节点 (全局命名空间, 核心算法来自 FAST-LIO2_ROS2)
 */
class FastLio2Node : public rclcpp::Node
{
public:
    explicit FastLio2Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~FastLio2Node() override = default;

private:
    struct NodeConfig
    {
        std::string imu_topic = "/imu";
        std::string lidar_topic = "/velodyne_points";
        std::string body_frame = "base_link";
        std::string world_frame = "odom";
        bool print_time_cost = false;
    };

    struct StateData
    {
        bool lidar_pushed = false;
        std::mutex imu_mutex;
        std::mutex lidar_mutex;
        double last_lidar_time = -1.0;
        double last_imu_time = -1.0;
        std::deque<IMUData> imu_buffer;
        std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
        nav_msgs::msg::Path path;
    };

    void loadParameters();
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    bool syncPackage();
    void timerCB();
    void publishGlobalMap();

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      CloudType::Ptr cloud, std::string frame_id, const double &time);
    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                         std::string frame_id, std::string child_frame, const double &time);
    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                     std::string frame_id, const double &time);
    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                     std::string frame_id, std::string child_frame, const double &time);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::TimerBase::SharedPtr m_map_timer;
    rclcpp::TimerBase::SharedPtr m_timer;

    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};
