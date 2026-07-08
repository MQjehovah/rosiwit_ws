# rosiwit_slam

基于 ROS2 的**分层 SLAM 框架**,三层架构(ROS 接口层 / SLAM 接口层 / 算法层),支持运行时模式切换(mapping / localization / idle)、地图保存/加载、2D 栅格图生成。内置 **FAST-LIO2** 前端 + **Ceres** 后端 + **GICP** 定位。

---

## 快速开始

```bash
# 构建
cd ~/rosiwit_ws && colcon build --packages-select rosiwit_slam --symlink-install && source install/setup.bash

# 启动 (默认 IDLE 模式, 不处理数据)
ros2 launch rosiwit_slam slam.launch.py use_sim_time:=true
```

## 工作流

### 建图

```bash
# 1. 切换到建图模式
ros2 service call /set_slam_mode rosiwit_slam/srv/SetSlamMode "{mode: 'mapping'}"

# 2. 机器人走一圈...

# 3. 保存地图
ros2 service call /save_map rosiwit_slam/srv/SaveMap "{path: 'map.pcd'}"
ros2 service call /save_grid_map rosiwit_slam/srv/SaveGridMap "{pgm_path: 'map.pgm', yaml_path: 'map.yaml', resolution: 0.05}"

# 4. 切回 IDLE
ros2 service call /set_slam_mode rosiwit_slam/srv/SetSlamMode "{mode: 'idle'}"
```

### 定位

```bash
# 1. 加载地图 (同时设置到定位模块, 初始位姿默认 (0,0,0))
ros2 service call /load_map rosiwit_slam/srv/LoadMap "{path: 'map.pcd'}"

# 2. 切换到定位模式
ros2 service call /set_slam_mode rosiwit_slam/srv/SetSlamMode "{mode: 'localization'}"

# 3. (可选) 在 RViz 中用 2D Pose Estimate 设置初始位姿
#    → 自动通过 /initialpose 话题传给定位模块

# 4. 定位结果通过 /lio_odom /lio_path /tf 输出
```

### 模式随时切换

建图和定位可以随时切换:
- `mapping → idle`: 停止建图, 保留已积累的地图
- `idle → localization`: 加载地图后开始定位
- `localization → mapping`: 回到建图模式 (前端持续运行, 无缝切换)

## ROS 接口

| 类型 | 名称 | 说明 |
|---|---|---|
| 订阅 | `/imu`, `/velodyne_points` | IMU + LiDAR |
| 订阅 | `/initialpose` | RViz 2D Pose Estimate (定位模式设置初始位姿) |
| 发布 | `lio_odom` | Odometry |
| 发布 | `lio_path` | 累计轨迹 |
| 发布 | `body_cloud`, `world_cloud` | 当前帧点云 |
| 发布 | `cloud_map` | 全局地图 (建图=实时, 定位=加载的地图) |
| 发布 | `grid_map` | nav_msgs/OccupancyGrid (定位模式) |
| TF | `odom → base_link` | 位姿 |
| Service | `save_map` | 保存 PCD 地图 |
| Service | `load_map` | 加载 PCD 地图 (同时设置到定位模块) |
| Service | `save_grid_map` | 保存 2D 栅格图 (PGM+YAML) |
| Service | `set_slam_mode` | 切换模式 (mapping/localization/idle) |

## 架构

```
ROS 接口层 (ros_interface/)
  SlamNode — 极薄: msg 转换 + 发布 + 服务
        │ ISlamAlgorithm (SlamPipeline)
SLAM 接口层 (slam_core/)
  SlamPipeline — 编排器: IDLE/MAPPING/LOCALIZATION 三态
  IFrontend / IBackend / ILoopClosure / IMapManager / ILocalization
  SlamFactory — 运行时工厂
算法层 (algorithms/)
  fast_lio2/        — FastLio2Frontend (IESKF + iKD-Tree)
  ceres_backend/    — Ceres 位姿图优化
  gicp_localization/— GICP scan-to-map 定位
  pcd_map_manager/  — PCD 地图 + 2D 占据栅格
  scan_context_lc/  — 距离回环检测
```

## 许可证

MIT
