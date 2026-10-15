# rosiwit_slam_ros1 — rosiwit_slam 的 ROS1 接口层

与 ROS2 包（`rosiwit_slam_ros2`）**完全等价**，都只是薄接口：链接同一个 **ROS 无关的算法核心**
`../src/rosiwit_slam`（通过 CMake `add_subdirectory` 内联构建）。核心可**独立开发/测试/验证**。

## 结构

```
src/rosiwit_slam           (算法核心: 纯 C++/PCL/Eigen, 无 ROS; 可独立 cmake+ctest)
src/rosiwit_slam_ros1      (本包: ROS1/catkin 薄接口包; 含 COLCON_IGNORE)
src/rosiwit_slam_ros2      (ROS2/ament 薄接口包)
```

## 独立验证核心（无需 ROS）

```bash
cd src/rosiwit_slam
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## 构建本 ROS1 包

本包含 `COLCON_IGNORE`，不被 colcon(ROS2) 处理。放入 **ROS1 工作区**构建：

```bash
mkdir -p ~/ros1_ws/src && cd ~/ros1_ws/src
ln -s <this_repo>/src/rosiwit_slam_ros1 .
ln -s <this_repo>/src/rosiwit_slam .        # 核心 (与 ROS1 包同级)
# 本包 CMake 通过 ../rosiwit_slam 内联构建核心
cd ~/ros1_ws && catkin_make
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
