# rosiwit_slam_ros1 — rosiwit_slam 的 ROS1 接口层

与 ROS2 节点（`src/rosiwit_slam/src/ros_interface/slam_node.cpp`）**共享同一套 ROS 无关的算法核心**
（`SlamFactory` / `SlamPipeline` / `ILocalization` 等）。本包只做 ROS1 ↔ 核心的桥接，
因此 ROS1/ROS2 **同时可用、且行为一致**。

## 结构

```
rosiwit_slam              (ROS2/ament, 含算法核心 + ROS2 节点)
ros1_interface/
  rosiwit_slam_ros1       (ROS1/catkin, 复用核心源码, 仅桥接)
    src/rosiwit_slam_ros1_node.cpp
    srv/{SaveMap,LoadMap,SaveGridMap,SetSlamMode}.srv
    launch/rosiwit_slam.launch
    config/default.yaml   (与 ROS2 config 同格式)
```

构建本包时，CMake 直接**编译核心源码**（`src/rosiwit_slam/src/**/*.cpp`，排除 `ros_interface/` 与 `main.cpp`），
与 ROS2 共享同一份代码，避免分叉。

## 构建（ROS1）

本目录含 `COLCON_IGNORE`，不被 colcon(ROS2) 处理。请放入 **ROS1 工作区**构建：

```bash
# 假设 ROS1 (noetic/melodic) 环境
mkdir -p ~/ros1_ws/src && cd ~/ros1_ws/src
ln -s <this_repo>/ros1_interface/rosiwit_slam_ros1 .
# 需要同时能看到算法核心源码 (本 CMake 用相对路径 ../../src/rosiwit_slam)
#   做法: 让本包位于 <repo>/ros1_interface/ 下 (即保持当前仓库布局), 或修改 CMake 中 SLAM_PKG
cd ~/ros1_ws && catkin_make   # 或 catkin build
```

## 接口

**订阅**
- `imu` (`sensor_msgs/Imu`)：`config.imu_topic`
- `lidar` (`sensor_msgs/PointCloud2`)：`config.lidar_topic`
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`)
- `odom` (`nav_msgs/Odometry`，可选，`enable_odom`）· `/visual_odometry`（可选，`vision.enabled`）· `/absolute_pose`（可选，`absolute_enabled`）

**发布**
- `lio_odom` / `odom` (`nav_msgs/Odometry`)、`lio_path` (`nav_msgs/Path`)
- `body_cloud` / `world_cloud` / `cloud_map` (`sensor_msgs/PointCloud2`)
- `grid_map` / `map` (`nav_msgs/OccupancyGrid`，首行=最大 y)
- TF：`world_frame -> body_frame`，静态 `map -> odom`

**服务**
- `save_map` / `load_map` / `save_grid_map` / `set_slam_mode`

## 备注
- 点云不做逐点去畸变（curvature 置 0）；如需 deskew，请在驱动层/前端处理。
- 配置为 YAML，格式与 ROS2 `config/default.yaml` 完全一致（同一核心解析）。
