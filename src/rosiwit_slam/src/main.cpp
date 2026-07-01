/**
 * @file main.cpp
 * @brief FAST-LIO2 SLAM - ROS2节点入口
 * @author AI Development Team
 * @date 2026-04-24
 */

#include <rclcpp/rclcpp.hpp>
#include "fast_lio2_slam/ros_interface/fast_lio2_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<fast_lio2_slam::FastLio2Node>();

    // 使用多线程执行器，确保不同线程中的publish能正常工作
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 执行节点
    executor.spin();

    // 清理
    rclcpp::shutdown();

    return 0;
}