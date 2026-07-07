// include/ros_interface/ros_utils.h
#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

// ROS 层专用工具: PointCloud2<->PCL 转换 + 时间戳转换。
// 仅 ROS 接口层 (SlamNode) 使用; 算法层 (algorithms/) 不依赖此头,
// 从而算法层与 rclcpp/sensor_msgs 解耦。
class RosUtils {
public:
    static double getSec(std_msgs::msg::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr ros2PCL(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg,
        int filter_num, double min_range = 0.5, double max_range = 20.0);
    static builtin_interfaces::msg::Time getTime(const double &sec);
};
