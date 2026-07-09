// ============================================================
// Diffbot Navigation - 主入口节点
// ============================================================

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosiwit_navigation/ros_interface/navigation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 创建生命周期节点
  auto navigation_node = std::make_shared<rosiwit_navigation::navigation::SmoothNavigation>();

  // 执行节点
  rclcpp::spin(navigation_node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}