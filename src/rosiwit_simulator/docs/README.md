# rosiwit_simulator — 项目文档

> **版本**: v2.0 (ROS2 Humble 迁移)
> **日期**: 2026-05-05
> **状态**: ✅ 已通过全部测试（83/83），建议发布
> **上一版本**: v1.0 (3D 激光雷达扩展, ROS1 Noetic)

---

## 1. 项目概述

**rosiwit_simulator** 是基于 **ROS2 Humble** + Gazebo 11 的机器人仿真环境包，为 mbot 差速驱动机器人提供完整的仿真支持。

### 版本演进

| 版本 | 里程碑 | ROS 版本 | 构建系统 |
|------|--------|----------|----------|
| v1.0 | 3D 激光雷达扩展 (Velodyne VLP-16) | ROS1 Noetic | catkin |
| **v2.0** | **ROS1→ROS2 Humble 迁移** | **ROS2 Humble** | **ament_cmake** |

### 核心功能

| 功能 | 传感器 | Gazebo 插件 | ROS2 Topic |
|------|--------|-------------|------------|
| 差速驱动 | 底盘电机 | `libgazebo_ros_diff_drive.so` | `/cmd_vel`, `/odom` |
| 2D 激光雷达 | RPLIDAR A2 | `libgazebo_ros_laser.so` | `/scan` |
| 3D 激光雷达 | Velodyne VLP-16 | `libgazebo_ros_ray_sensor.so` | `/velodyne_points` |
| IMU | 姿态传感器 | `libgazebo_ros_imu_sensor.so` | `/imu` |
| 深度相机 | Kinect | Gazebo depth camera | `/camera/depth/*` |
| RGB 相机 | USB Camera | Gazebo camera | `/camera/rgb/*` |

---

## 2. 快速开始

### 2.1 环境要求

| 依赖 | 版本 | 说明 |
|------|------|------|
| ROS2 | Humble Hawksbill | ament_cmake 构建系统 |
| Gazebo | 11.x | 需要 GPU 支持 (用于 `gpu_ray` 传感器) |
| Python | 3.10+ | ROS2 Humble 自带 |
| colcon | 0.12+ | ROS2 构建工具 |
| xacro | ros-humble-xacro | URDF 宏处理器 |

安装依赖：
```bash
# ROS2 Humble 基础环境（假设已安装）
source /opt/ros/humble/setup.bash

# 项目依赖
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro \
                 ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
                 ros-humble-rviz2 ros-humble-nav2-map-server ros-humble-nav2-amcl \
                 ros-humble-tf2-ros
```

### 2.2 构建步骤

```bash
# 进入工作空间
cd /path/to/rosiwit_ws

# 构建
colcon build --packages-select simulator

# 加载环境
source install/setup.bash
```

### 2.3 启动仿真

#### 3D 激光雷达仿真

```bash
ros2 launch simulator simulator_gazebo_3d.launch.py
```

启动后自动加载：
1. **Gazebo** 仿真环境 (`house.world`)
2. **3D 雷达机器人模型** (`mbot_with_lidar3d_gazebo.xacro`)
3. **robot_state_publisher** (50 Hz TF 发布)
4. **joint_state_publisher** (关节状态)
5. **RViz2** 可视化 (`simulator_3d.rviz`)

#### 2D 激光雷达仿真

```bash
ros2 launch simulator simulator_gazebo.launch.py
```

#### SLAM 建图

```bash
# GMapping
ros2 launch simulator simulator_mapping_gmaping.launch.py

# Cartographer
ros2 launch simulator simulator_mapping_cartographer.launch.py
```

#### 自主导航

```bash
# 先启动地图服务
ros2 launch simulator simulator_map_server.launch.py

# 启动 AMCL 定位 + 导航
ros2 launch simulator simulator_nav_movebase.launch.py
```

#### RViz2 可视化

```bash
ros2 launch simulator simulator_rviz.launch.py
```

### 2.4 验证数据

```bash
# 检查 3D 点云话题
ros2 topic echo /velodyne_points --once

# 检查发布频率
ros2 topic hz /velodyne_points
# 预期: average ~10 Hz

# 查看活跃节点
ros2 node list

# 查看 TF 树
ros2 run tf2_tools view_frames
# 预期: odom → base_footprint → base_link → velodyne_link
```

### 2.5 Launch 参数

所有主 launch 文件均支持以下标准参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `true` | 使用仿真时间 |
| `gui` | `true` | 启动 Gazebo GUI |
| `paused` | `false` | 启动时暂停仿真 |
| `headless` | `false` | 无头模式（不启动 GUI） |
| `debug` | `false` | 调试模式 |
| `world_name` | `house.world` | Gazebo 世界文件路径 |

---

## 3. 项目结构

```
rosiwit_simulator/
├── CMakeLists.txt              ← ament_cmake 构建 (CMake 3.8+)
├── package.xml                 ← format 3, ament_cmake buildtool
├── launch/
│   ├── simulator_gazebo.launch          [XML] 2D 仿真 (保留)
│   ├── simulator_gazebo.launch.py       [Py]  2D 仿真 (ROS2)
│   ├── simulator_gazebo_3d.launch       [XML] 3D 仿真 (保留)
│   ├── simulator_gazebo_3d.launch.py    [Py]  3D 仿真 (ROS2)
│   ├── simulator_mapping_gmaping.launch [XML] GMapping (保留)
│   ├── simulator_mapping_gmaping.launch.py [Py] GMapping (ROS2)
│   ├── simulator_mapping_cartographer.launch [XML] Cartographer (保留)
│   ├── simulator_mapping_cartographer.launch.py [Py] Cartographer (ROS2)
│   ├── simulator_amcl_diff.launch       [XML] AMCL (保留)
│   ├── simulator_amcl_diff.launch.py    [Py]  AMCL (ROS2)
│   ├── simulator_nav_movebase.launch    [XML] 导航 (保留)
│   ├── simulator_nav_movebase.launch.py [Py]  导航 (ROS2)
│   ├── simulator_map_server.launch      [XML] 地图服务 (保留)
│   ├── simulator_map_server.launch.py   [Py]  地图服务 (ROS2)
│   ├── simulator_rviz.launch            [XML] RViz (保留)
│   ├── simulator_rviz.launch.py         [Py]  RViz (ROS2)
│   └── include/
│       ├── amcl_diff.launch / .launch.py
│       ├── amcl_omni.launch / .launch.py
│       ├── gmapping_base.launch / .launch.py
│       ├── hector_mapping_base.launch / .launch.py
│       ├── teb_move_base_diff.launch / .launch.py
│       └── teb_move_base_omni.launch / .launch.py
├── config/         ← 导航参数 YAML (diff/ + omni/)
├── map/            ← 地图文件
├── models/         ← Gazebo 模型资源
├── meshes/         ← 3D 网格文件
├── world/          ← Gazebo 世界文件
├── urdf/           ← URDF/Xacro 机器人描述
├── rviz/           ← RViz2 配置文件
├── images/         ← 图片资源
├── docker/         ← Docker 部署配置
├── .gitlab-ci.yml  ← GitLab CI 流水线
├── Jenkinsfile     ← Jenkins Pipeline
├── DEPLOYMENT.md   ← 部署文档
├── DEPENDENCIES.md ← 依赖清单
└── README.md       ← 项目说明
```

---

## 4. 迁移要点

### 4.1 构建系统变更

| 方面 | ROS1 (旧) | ROS2 (新) |
|------|-----------|-----------|
| 构建系统 | catkin (CMake 3.0.2) | ament_cmake (CMake 3.8+) |
| 构建命令 | `catkin_make` | `colcon build` |
| 环境加载 | `source devel/setup.bash` | `source install/setup.bash` |
| 包格式 | package.xml format 2 | package.xml format 3 |
| 结束标记 | `catkin_package()` | `ament_package()` |

### 4.2 Launch 文件迁移

| ROS1 (XML) | ROS2 (Python) |
|------------|---------------|
| `<node pkg="x" type="y" name="z"/>` | `Node(package='x', executable='y', name='z')` |
| `<arg name="x" default="y"/>` | `DeclareLaunchArgument('x', default_value='y')` |
| `<param name="x" value="y"/>` | `parameters={'x': 'y'}` |
| `<remap from="a" to="b"/>` | `remappings=[('a', 'b')]` |
| `<include file="$(find pkg)/launch/x.launch"/>` | `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` |
| `$(find pkg)/path` | `get_package_share_directory('pkg')` + `os.path.join()` |
| `<rosparam file="..." command="load"/>` | `parameters=[os.path.join(dir, 'file.yaml')]` |

### 4.3 节点映射

| ROS1 节点 | ROS2 节点 |
|-----------|-----------|
| `gazebo_ros/empty_world.launch` | `gazebo_ros/gazebo.launch.py` |
| `gazebo_ros/spawn_model` | `gazebo_ros/spawn_entity.py` |
| `robot_state_publisher` | `robot_state_publisher/robot_state_publisher` |
| `joint_state_publisher` | `joint_state_publisher/joint_state_publisher` |
| `rviz/rviz` | `rviz2/rviz2` |
| `map_server/map_server` | `nav2_map_server/map_server` |
| `amcl/amcl` | `nav2_amcl/amcl` |
| `tf/static_transform_publisher` | `tf2_ros/static_transform_publisher` |

---

## 5. 与 v1.0 的差异

### v2.0 新增文件

| 文件路径 | 操作 | 说明 |
|---------|------|------|
| `CMakeLists.txt` | **重写** | catkin → ament_cmake (21行) |
| `package.xml` | **重写** | format 2 → format 3, ROS2 依赖 |
| `launch/*.launch.py` (8个) | **新增** | 主 launch Python 版本 |
| `launch/include/*.launch.py` (6个) | **新增** | 子 launch Python 版本 |

### v2.0 保留文件

所有 XML launch 文件、资源文件（models/, meshes/, world/, urdf/, map/, config/, rviz/, images/）和部署文件（docker/, .gitlab-ci.yml, Jenkinsfile）均未修改。

---

## 6. 测试状态

### ROS2 迁移测试 (v2.0)

| 测试套件 | 用例数 | 通过 | 通过率 |
|----------|--------|------|--------|
| 构建系统迁移 (AC1) | 7 | 7 | 100% ✅ |
| 资源安装完整性 (AC2) | 8 | 8 | 100% ✅ |
| Python Launch 文件 (AC3) | 44 | 44 | 100% ✅ |
| 向后兼容 (AC4) | 12 | 12 | 100% ✅ |
| 迁移对应关系 (AC5) | 12 | 12 | 100% ✅ |
| **总计** | **83** | **83** | **100%** ✅ |

colcon build: ✅ 通过 (0.33s)

### 3D LiDAR 功能测试 (v1.0 遗留)

| 测试套件 | 用例数 | 通过 | 通过率 |
|----------|--------|------|--------|
| 单元测试 | 81 | 81 | 100% ✅ |
| 集成测试 | 28 | 28 | 100% ✅ |
| 功能测试 | 52 | 52 | 100% ✅ |
| **总计** | **161** | **161** | **100%** ✅ |

历史缺陷（BUG-001 ~ BUG-003）已全部修复并验证通过。详见 [changelog.md](changelog.md)。

---

## 7. 注意事项

1. **GPU 要求**: 3D 雷达使用 `gpu_ray` 传感器类型，Gazebo 需要 GPU 加速支持。在无 GPU 的 WSL2 环境下可能无法正常启动。
2. **ROS2 原生**: v2.0 已完全迁移到 ROS2 Humble，不再需要 ros1_bridge 桥接。与 rosiwit_slam (FAST-LIO2) 可直接在 ROS2 生态中协同。
3. **XML 保留**: 所有原始 XML launch 文件完整保留，用于参考和过渡期兼容。
4. **use_sim_time**: SLAM 启动时务必设置 `use_sim_time:=true`。
5. **2D 兼容**: 3D 雷达功能完全独立，不影响原有 2D 雷达仿真功能。通过不同 launch 文件切换。

---

## 8. 相关文档

| 文档 | 说明 |
|------|------|
| [api.md](api.md) | API & 配置参考（Launch 文件签名、Xacro 宏、SLAM 配置） |
| [architecture.md](architecture.md) | 架构说明（系统架构、模块结构、数据流） |
| [changelog.md](changelog.md) | 版本变更记录 |
| `requirements.md` | 需求文档（ROS1→ROS2 迁移需求） |
| `test_report.md` | 测试报告（迁移测试 83/83 通过） |
| `security_report.md` | 安全审查报告 |
