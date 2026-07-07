// include/ros_interface/slam_node.h
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>
#include <mutex>
#include "slam_core/i_slam_algorithm.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

// 极薄 ROS 接口层: 只做
//   1) ROS msg <-> SLAM 层 IMUSample/LidarFrame 转换, 喂给算法
//   2) 接收算法的 SlamOutput 回调, 周期发布 odom/path/cloud/tf
// 不感知任何算法内部细节 (无 IESKF/LidarProcessor 依赖)。
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
        bool   print_time_cost  = false;
    };

    void loadParameters();
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void timerCB();          // 调 algo->tryPopAndProcess + 发布最新输出
    void mapTimerCB();
    void onOutput(const SlamOutput& out);
    void publish(const SlamOutput& out);
    builtin_interfaces::msg::Time toRosTime(double sec);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          m_imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  m_lidar_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub, m_world_cloud_pub, m_global_map_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     m_path_pub;
    rclcpp::TimerBase::SharedPtr m_timer, m_map_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf;

    std::unique_ptr<ISlamAlgorithm> m_algo;
    NodeConfig m_cfg;
    std::mutex m_out_mutex;
    SlamOutput m_latest;
    bool m_have_output = false;
    nav_msgs::msg::Path m_path;
};

} // namespace rosiwit_slam
