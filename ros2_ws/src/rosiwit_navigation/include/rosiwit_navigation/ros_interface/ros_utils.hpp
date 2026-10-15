#ifndef ROSIWIT_NAVIGATION__ROS_INTERFACE__ROS_UTILS_HPP_
#define ROSIWIT_NAVIGATION__ROS_INTERFACE__ROS_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace ros_interface
{

class RosUtils
{
public:
  static core::Pose2D toCorePose(const geometry_msgs::msg::PoseStamped & pose);
  static core::VelocityCommand toCoreVelocity(const geometry_msgs::msg::Twist & twist);
  static geometry_msgs::msg::Twist toRosTwist(const core::VelocityCommand & cmd);
  static core::Path toCorePath(const nav_msgs::msg::Path & path);
  static nav_msgs::msg::Path toRosPath(
    const core::Path & path, const std::string & frame_id,
    const rclcpp::Time & stamp);
};

} // namespace ros_interface
} // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__ROS_INTERFACE__ROS_UTILS_HPP_
