# rosiwit_app - 移动机器人统一调度系统

## 概述

`rosiwit_app` 是移动机器人系统的统一调度层，整合以下三个子系统：

| 子系统 | 包名 | 功能 |
|--------|------|------|
| 模拟器 | `rosiwit_simulator` | 提供仿真传感器数据 (LiDAR/IMU/Odom) |
| SLAM | `rosiwit_slam` | Fast-LIO2 建图定位 |
| 导航 | `diffbot_navigation` | Nav2 自主导航 |

## 核心功能

1. **一键启动** - `system_bringup.launch.py` 按序启动所有子系统
2. **地图初始化** - 启动时自动加载上次地图和位置，初始化 SLAM
3. **导航目标接收** - 接收用户指定的导航目标（地名或坐标）
4. **状态监控** - 实时监控系统状态、子系统和位置

## 状态机

```
INIT → INITIALIZING → READY → NAVIGATING → READY
              ↓                         ↑
         MAPPING → MAP_SAVING → READY
```

## 快速启动

### 仿真环境完整测试
```bash
ros2 launch rosiwit_app sim_slam_nav.launch.py
```

### 一键启动（使用系统launch文件）
```bash
ros2 launch rosiwit_app system_bringup.launch.py use_rviz:=true
```

### 仅启动App（其他模块已运行）
```bash
ros2 launch rosiwit_app app_only.launch.py
```

## 使用方式

### 1. 导航到命名航点
```bash
ros2 topic pub --once /rosiwit_app/go_to std_msgs/msg/String "{data: 'point_a'}"
```

### 2. 导航到指定坐标
```bash
ros2 topic pub --once /rosiwit_app/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 3. 保存当前地图
```bash
ros2 service call /rosiwit_app/save_map std_srvs/srv/Trigger
```

### 4. 查看系统状态
```bash
ros2 service call /rosiwit_app/get_status std_srvs/srv/Trigger
```

### 5. 查看可用航点
```bash
ros2 service call /rosiwit_app/list_waypoints std_srvs/srv/Trigger
```

### 6. 实时监控状态
```bash
ros2 topic echo /rosiwit_app/status
```

## ROS2 接口

### Topics (订阅)
| Topic | 类型 | 说明 |
|-------|------|------|
| `/rosiwit_app/goal_pose` | `geometry_msgs/PoseStamped` | 发送导航目标 |
| `/rosiwit_app/go_to` | `std_msgs/String` | 导航到命名航点 |
| `/odom_estimated` | `nav_msgs/Odometry` | 里程计（来自SLAM） |

### Topics (发布)
| Topic | 类型 | 说明 |
|-------|------|------|
| `/rosiwit_app/status` | `std_msgs/String` | 系统状态 JSON |
| `/initial_pose` | `geometry_msgs/Pose` | SLAM 初始位姿 |

### Services
| Service | 类型 | 说明 |
|---------|------|------|
| `/rosiwit_app/save_map` | `std_srvs/Trigger` | 保存地图 |
| `/rosiwit_app/get_status` | `std_srvs/Trigger` | 获取系统状态 |
| `/rosiwit_app/list_waypoints` | `std_srvs/Trigger` | 列出航点 |

### Action Client
| Action | 类型 | 说明 |
|--------|------|------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Nav2 导航 |

## 配置文件

- `config/app_params.yaml` - 应用参数
- `config/waypoints.yaml` - 预设航点列表

## 地图文件

地图保存在 `/tmp/rosiwit_sim_map/` 目录：
- `fast_lio2_map.pcd` - 3D 点云地图
- `fast_lio2_map.pgm` - 2D 栅格地图
- `fast_lio2_map.yaml` - 地图元数据
- `last_position.json` - 最后已知位置
