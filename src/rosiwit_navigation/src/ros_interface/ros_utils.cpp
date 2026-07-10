#include "rosiwit_navigation/ros_interface/ros_utils.hpp"

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rosiwit_navigation
{
namespace ros_interface
{

core::Pose2D RosUtils::toCorePose(const geometry_msgs::msg::PoseStamped & pose)
{
  return core::Pose2D(
    pose.pose.position.x,
    pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation));
}

core::VelocityCommand RosUtils::toCoreVelocity(const geometry_msgs::msg::Twist & twist)
{
  return core::VelocityCommand(twist.linear.x, twist.linear.y, twist.angular.z);
}

geometry_msgs::msg::Twist RosUtils::toRosTwist(const core::VelocityCommand & cmd)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = cmd.linear_x;
  twist.linear.y = cmd.lateral_y;
  twist.angular.z = cmd.angular_z;
  return twist;
}

core::Path RosUtils::toCorePath(const nav_msgs::msg::Path & path)
{
  core::Path core_path;
  for (const auto & pose : path.poses) {
    core::PathPoint pt;
    pt.pose = toCorePose(pose);
    core_path.points.push_back(pt);
  }
  return core_path;
}

nav_msgs::msg::Path RosUtils::toRosPath(
  const core::Path & path, const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Path ros_path;
  ros_path.header.frame_id = frame_id;
  ros_path.header.stamp = stamp;
  for (const auto & pt : path.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = ros_path.header;
    pose.pose.position.x = pt.pose.x;
    pose.pose.position.y = pt.pose.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), pt.pose.theta));
    ros_path.poses.push_back(pose);
  }
  return ros_path;
}

} // namespace ros_interface
} // namespace rosiwit_navigation
