// src/main.cpp
#include <rclcpp/rclcpp.hpp>
#include "ros_interface/slam_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // 通过 SlamFactory 在运行时按 config 的 slam_algorithm 字段选择算法
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(std::make_shared<rosiwit_slam::SlamNode>());
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
