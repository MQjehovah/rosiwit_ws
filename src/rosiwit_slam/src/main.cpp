/**
 * @file main.cpp
 * @brief FAST-LIO2 SLAM - ROS2节点入口
 */

#include <rclcpp/rclcpp.hpp>
#include "fast_lio2_slam/ros_interface/fast_lio2_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建节点 (核心算法来自 FAST-LIO2_ROS2)
    auto node = std::make_shared<FastLio2Node>();

    // 使用多线程执行器，确保IMU/LiDAR回调与定时器处理并行工作
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
