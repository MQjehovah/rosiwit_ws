// src/main.cpp
#include <rclcpp/rclcpp.hpp>
#include "ros_interface/slam_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rosiwit_slam::SlamNode>();
    bool use_mt = false;
    node->declare_parameter("use_multi_threaded", false);
    node->get_parameter("use_multi_threaded", use_mt);
    if (use_mt) {
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        exec.spin();
    } else {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
