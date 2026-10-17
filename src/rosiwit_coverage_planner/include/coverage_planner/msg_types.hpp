// Copyright (c) 2024. All rights reserved.
// Licensed under the Apache-2.0 license.
//
// ROS 无关的消息类型 (镜像 ROS 版 OccupancyGrid/PoseStamped 布局), 使算法核心
// 不依赖 nav_msgs / geometry_msgs / rclcpp。ROS1/ROS2 接口层负责与 ROS 类型互转。

#ifndef COVERAGE_PLANNER__MSG_TYPES_HPP_
#define COVERAGE_PLANNER__MSG_TYPES_HPP_

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace coverage_planner
{

struct Header
{
  double stamp = 0.0;
  std::string frame_id;
};

struct Point
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

struct PoseStamped
{
  Header header;
  Pose pose;
};

struct MapMetaData
{
  struct Origin
  {
    Point position;
    Quaternion orientation;
  } origin;
  double resolution = 0.0;
  uint32_t width = 0;
  uint32_t height = 0;
};

struct GridMap
{
  Header header;
  MapMetaData info;
  std::vector<int8_t> data;
};

inline double yawFromQuaternion(const Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline Quaternion quaternionFromYaw(double yaw)
{
  Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace coverage_planner

// ---- 轻量日志宏 (替代 RCLCPP_*, 核心不依赖 rclcpp) ----
#ifndef COV_INFO
#define COV_INFO(...)  do { std::printf("[INFO] "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define COV_WARN(...)  do { std::printf("[WARN] "); std::printf(__VA_ARGS__); std::printf("\n"); } while (0)
#define COV_ERROR(...) do { std::fprintf(stderr, "[ERROR] "); std::fprintf(stderr, __VA_ARGS__); std::fprintf(stderr, "\n"); } while (0)
#endif

#endif  // COVERAGE_PLANNER__MSG_TYPES_HPP_
