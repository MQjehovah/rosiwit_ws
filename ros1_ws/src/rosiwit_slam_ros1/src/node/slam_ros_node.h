// ============================================================================
// SlamRosNode —— 组装根 (composition root)
//
// 职责**只有三件**:
//   1. 创建执行资源 (rosiwit_common::ThreadPool, 按 lane 声明)
//   2. 按依赖顺序构造并注入: ThreadPool -> SlamAlgorithm -> SlamRosInterface
//   3. 管理生命周期与关停顺序 (先停 ROS 发布定时器 -> 停算法线程 -> 最后关池)
//
// 它不含算法, 也不做发布: 发布在 SlamRosInterface, 算法在 SlamAlgorithm。
// 输入订阅放在这里 —— 一个进程只允许一处 subscribe (避免同一话题被注入两次)。
// ============================================================================
#ifndef ROSIWIT_SLAM_ROS1__NODE__SLAM_ROS_NODE_H_
#define ROSIWIT_SLAM_ROS1__NODE__SLAM_ROS_NODE_H_

#include <memory>

#include <ros/ros.h>

#include "rosiwit_common/thread_pool.hpp"

#include "slam_algorithm.h"
#include "facade.h"

namespace rosiwit_slam_ros1
{

class SlamRosNode
{
public:
  SlamRosNode(ros::NodeHandle & nh, ros::NodeHandle & pnh);
  ~SlamRosNode();

  SlamRosNode(const SlamRosNode &) = delete;
  SlamRosNode & operator=(const SlamRosNode &) = delete;

  bool ok() const { return ok_; }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // 声明顺序 = 构造顺序; 析构反向 -> 算法先死, 池最后死 (算法关停时可能还要投任务)
  rosiwit_common::ThreadPool pool_;
  SlamAlgorithm algo_;
  std::unique_ptr<SlamRos1Interface> iface_;

  ros::Subscriber imu_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber vo_sub_;
  ros::Subscriber absolute_sub_;

  bool ok_ = false;
};

}  // namespace rosiwit_slam_ros1

#endif  // ROSIWIT_SLAM_ROS1__NODE__SLAM_ROS_NODE_H_
