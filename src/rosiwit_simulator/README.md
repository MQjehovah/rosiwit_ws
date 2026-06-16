# rosiwit_simulator

ROS2 Humble 仿真环境包，为 mbot 差速驱动机器人提供 Gazebo 仿真支持。

## 功能概述

- **差速驱动仿真**: 基于 `libgazebo_ros_diff_drive.so` 的差速驱动插件
- **2D 激光雷达**: RPLIDAR A2 激光雷达仿真 (使用 `libgazebo_ros_laser.so`)
- **3D 激光雷达**: Velodyne VLP-16 3D 激光雷达仿真 (使用 `libgazebo_ros_ray_sensor.so`)
- **IMU 传感器**: IMU 姿态传感器仿真 (使用 `libgazebo_ros_imu_sensor.so`)
- **深度相机**: Kinect 深度相机仿真
- **RGB 相机**: 普通相机仿真

## 3D 激光雷达 (Velodyne VLP-16) 配置

### 传感器参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 传感器类型 | `gpu_ray` | GPU 加速光线追踪 |
| 扫描频率 | 10 Hz | 每秒 10 帧点云 |
| 水平采样 | 1800 点 | 水平 360° (±π) |
| 垂直采样 | 16 线 | 垂直 ±15° (±0.2618 rad) |
| 测距范围 | 0.5m ~ 100m | 有效测量距离 |
| 噪声模型 | 高斯噪声 (σ=0.01) | 模拟真实传感器噪声 |
| 外形尺寸 | 圆柱体 r=0.0516m, h=0.0717m | VLP-16 真实外形 |
| 质量 | 0.83 kg | VLP-16 真实质量 |

### 安装位置

3D 雷达安装在机器人顶部 (`base_link` 上方 0.1955m 处)：
- `x = 0.0` (居中)
- `y = 0.0` (居中)
- `z = 0.1955` (顶部)

### 发布的话题

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 10 Hz | 3D 点云数据 |
| `/imu` | `sensor_msgs/Imu` | 50 Hz | IMU 姿态数据 |
| `/odom` | `nav_msgs/Odometry` | 30 Hz | 里程计数据 |
| `/cmd_vel` | `geometry_msgs/Twist` | - | 速度控制输入 |

### TF 树

```
odom → base_footprint → base_link → velodyne_link (3D 雷达)
                                   → imu_link (IMU)
```

## 快速开始

### 环境要求

- ROS2 Humble Hawksbill
- Gazebo 11
- Python 3.10+
- Ubuntu 22.04 LTS (或 WSL2)

### 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select simulator --symlink-install
source install/setup.bash
```

### 启动 3D 激光雷达仿真

```bash
ros2 launch simulator simulator_gazebo_3d.launch.py
```

该 launch 文件将自动启动：
1. Gazebo 仿真环境 (house.world)
2. 3D 雷达机器人模型 (mbot_with_lidar3d_gazebo.xacro)
3. robot_state_publisher (50 Hz)
4. joint_state_publisher
5. RViz 可视化 (simulator_3d.rviz)

### 启动 2D 激光雷达仿真

```bash
ros2 launch simulator simulator_gazebo.launch.py
```

### 验证雷达数据

```bash
# 检查 3D 点云话题
ros2 topic echo /velodyne_points --once

# 检查话题频率
ros2 topic hz /velodyne_points

# 查看 TF 树
ros2 run tf2_tools view_frames
```

## 与 rosiwit_slam 配合使用

本项目已迁移到 ROS2 Humble，可直接与 rosiwit_slam (FAST-LIO2) 通过 DDS 通信，无需 ros1_bridge：

```bash
# 终端1: 启动 3D 仿真 (ROS2 Humble)
ros2 launch simulator simulator_gazebo_3d.launch.py

# 终端2: 启动 FAST-LIO2 (ROS2 Humble, 直接 DDS 通信)
ros2 launch rosiwit_slam fast_lio2.launch.py
```

### 兼容性说明

- **点云话题**: `/velodyne_points` (sensor_msgs/PointCloud2)
- **IMU 话题**: `/imu` (sensor_msgs/Imu)
- **雷达坐标系**: `velodyne_link`
- 以上话题和坐标系与 rosiwit_slam 的 `velodyne_vlp16.yaml` 配置完全对齐

## 项目结构

```
rosiwit_simulator/
├── CMakeLists.txt
├── package.xml
├── launch/                      # Launch 启动文件
│   ├── *.launch               # XML 版本 (ROS1, 保留)
│   ├── *.launch.py            # Python 版本 (ROS2 Humble, 推荐)
│   └── include/               # 子 launch 文件 (AMCL/SLAM/导航配置)
├── urdf/
│   └── xacro/
│       ├── gazebo/                     # Gazebo 组合模型
│       │   ├── mbot_base.xacro                # 基础底盘
│       │   ├── mbot_with_lidar3d_gazebo.xacro # 3D 雷达组合模型
│       │   ├── mbot_with_laser_gazebo.xacro   # 2D 雷达组合模型
│       │   └── ...
│       └── sensors/                    # 传感器定义
│           ├── lidar3d.xacro                  # 3D 雷达物理描述
│           ├── lidar3d_gazebo.xacro           # 3D 雷达 Gazebo 插件
│           ├── imu.xacro                      # IMU 物理描述
│           ├── imu_gazebo.xacro               # IMU Gazebo 插件
│           ├── lidar.xacro                    # 2D 雷达物理描述
│           ├── lidar_gazebo.xacro             # 2D 雷达 Gazebo 插件
│           └── ...
├── config/                      # 参数配置
│   ├── diff/                   # 差速驱动导航参数
│   └── omni/                   # 全向驱动导航参数
├── rviz/                        # RViz 配置
│   ├── simulator_3d.rviz       # 3D 仿真可视化配置
│   └── urdf.rviz               # 模型预览配置
├── world/                       # Gazebo 世界文件
├── models/                      # Gazebo 模型文件
├── meshes/                      # 3D 网格模型
├── map/                         # 地图文件
└── images/                      # 图片资源
```

## Docker 部署

### 前置条件

- Docker Engine >= 20.10
- Docker Compose >= 1.29 (或 docker-compose-plugin v2)
- NVIDIA Driver >= 470.x (推荐，用于 GPU 加速)
- nvidia-container-toolkit (Docker GPU 支持)
- X11 display (用于 Gazebo/RViz GUI)

### 快速启动 (Docker)

```bash
# 1. 构建 Docker 镜像 (ROS2 Humble)
cd docker/
./build.sh --cache

# 2. 仅启动仿真器
./run.sh

# 3. 启动完整 SLAM 栈 (仿真器 + FAST-LIO2, ROS2 原生通信)
./run.sh --slam

# 4. 开发模式 (挂载源码，实时编辑)
./run.sh --devel

# 5. 停止所有容器
./run.sh --stop
```

### Docker Compose 服务说明

| 服务 | 说明 | ROS 版本 |
|------|------|----------|
| `simulator` | ROS2 Humble + Gazebo 11 仿真 | ROS2 Humble |
| `slam` | FAST-LIO2 SLAM (ROS2 原生) | ROS2 Humble |
| `slam-viz` | RViz2 可视化 | ROS2 Humble |
| `simulator-devel` | 开发环境 (含源码挂载) | ROS2 Humble |

> **注意**: 迁移到 ROS2 后不再需要 `ros1_bridge`，所有服务通过 DDS 直接通信。

### 验证 Docker 部署

```bash
# 检查容器状态
docker-compose -f docker/docker-compose.yml ps

# 查看 3D 点云话题 (容器内)
docker exec rosiwit-simulator ros2 topic echo /velodyne_points --once

# 查看话题列表
docker exec rosiwit-simulator ros2 topic list

# 查看 SLAM 状态
docker exec rosiwit-slam ros2 topic list
docker exec rosiwit-slam ros2 topic echo /cloud_registered --once

# 查看日志
docker-compose -f docker/docker-compose.yml logs -f simulator
```

### 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DISPLAY` | `:0` | X11 display |
| `LIBGL_ALWAYS_SOFTWARE` | `0` | 软件渲染 (无 GPU 时设为 1) |
| `ROS_DOMAIN_ID` | `42` | DDS 域 ID (所有容器必须一致) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS 中间件实现 |
| `GAZEBO_MODEL_PATH` | - | Gazebo 模型搜索路径 |

### 安全加固

Docker Compose 配置已按安全审计报告建议加固：

- ✅ 使用 `cap_drop: ALL` + 最小 `cap_add`
- ✅ 启用 `no-new-privileges`
- ✅ 设置资源限制 (CPU/内存)
- ✅ 日志轮转配置 (max-size=100m, max-file=3)
- ✅ 健康检查配置

### CI/CD

| 文件 | 平台 | 说明 |
|------|------|------|
| `.gitlab-ci.yml` | GitLab CI | 4 阶段: lint → build → test → deploy |
| `Jenkinsfile` | Jenkins | 声明式 Pipeline, 支持 Docker 构建 |
| `docker/build.sh` | 手动 | 本地构建脚本 |
| `docker/run.sh` | 手动 | 本地运行脚本 |

详见 `DEPENDENCIES.md` 获取完整依赖清单。

## 新增文件 (3D 激光雷达)

### 业务代码

| 文件 | 说明 |
|------|------|
| `urdf/xacro/sensors/lidar3d.xacro` | 3D 雷达物理描述 (VLP-16 外形、惯性) |
| `urdf/xacro/sensors/lidar3d_gazebo.xacro` | 3D 雷达 Gazebo 插件配置 |
| `urdf/xacro/gazebo/mbot_with_lidar3d_gazebo.xacro` | 3D 雷达组合模型 |

### 部署配置

| 文件 | 说明 |
|------|------|
| `docker/Dockerfile` | 多阶段构建 (base → builder → workspace → runtime), 基于 `ros:humble-ros-base-jammy` |
| `docker/docker-compose.yml` | 完整 SLAM 栈编排 (ROS2 Humble 原生, 无需 ros1_bridge) |
| `docker/entrypoint.sh` | 容器入口脚本 (ROS2 环境、GPU 检测) |
| `docker/build.sh` | 构建脚本 (支持 `--devel`, `--cache`, `--push`) |
| `docker/run.sh` | 运行脚本 (支持 `--slam`, `--devel`, `--2d`) |
| `docker/.dockerignore` | Docker 构建排除规则 |
| `.gitlab-ci.yml` | GitLab CI/CD 4 阶段流水线 (lint → build → test → deploy) |
| `Jenkinsfile` | Jenkins 声明式 Pipeline (ROS2 Humble) |
| `DEPENDENCIES.md` | 完整依赖清单 (ROS2 Humble) |
| `DEPLOYMENT.md` | 部署文档 (本地/Docker/CI/CD) |
| `launch/simulator_gazebo_3d.launch.py` | 3D 仿真启动文件 (Python launch) |
| `rviz/simulator_3d.rviz` | 3D 仿真 RViz2 可视化配置 |
