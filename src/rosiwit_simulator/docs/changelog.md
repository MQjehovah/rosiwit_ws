# 变更记录 — rosiwit_simulator

> 本文档记录 rosiwit_simulator 项目的版本变更历史。

---

## [2.0.0] - 2026-05-05 — ROS1→ROS2 Humble 迁移

### 概述
将 rosiwit_simulator 从 ROS1 Noetic (catkin) 完整迁移到 ROS2 Humble (ament_cmake)。所有 14 个 XML launch 文件迁移为 Python launch，构建系统重写为 ament_cmake。

### 变更

#### 构建系统
- **重写** `CMakeLists.txt`: catkin → ament_cmake (CMake 3.8+)
  - `cmake_minimum_required(VERSION 3.8)`
  - `find_package(ament_cmake REQUIRED)`
  - `install(DIRECTORY ...)` 安装 9 个资源目录
  - `ament_package()` 替代 `catkin_package()`
- **重写** `package.xml`: format 2 → format 3
  - `<buildtool_depend>ament_cmake</buildtool_depend>`
  - 删除 catkin 相关依赖
  - 添加 ROS2 依赖: rclpy, launch, launch_ros, gazebo_ros, nav2_map_server, nav2_amcl, tf2_ros 等

#### Launch 文件 (14 个 XML → 14 个 Python)

**主 launch 文件 (8 个)**:
- `simulator_gazebo.launch.py` — 2D 仿真: Gazebo + spawn_entity + state_publishers
- `simulator_gazebo_3d.launch.py` — 3D 仿真: 增加 RViz2 + 3D 雷达模型
- `simulator_mapping_gmaping.launch.py` — GMapping SLAM
- `simulator_mapping_cartographer.launch.py` — Cartographer SLAM
- `simulator_amcl_diff.launch.py` — AMCL 差速定位
- `simulator_nav_movebase.launch.py` — 导航栈 (map_server + AMCL + move_base)
- `simulator_map_server.launch.py` — 独立地图服务
- `simulator_rviz.launch.py` — RViz2 可视化

**子 launch 文件 (6 个)**:
- `include/amcl_diff.launch.py` — AMCL 差速配置 (nav2_amcl)
- `include/amcl_omni.launch.py` — AMCL 全向配置 (nav2_amcl)
- `include/gmapping_base.launch.py` — GMapping 参数 (slam_gmapping)
- `include/hector_mapping_base.launch.py` — Hector Mapping 参数
- `include/teb_move_base_diff.launch.py` — TEB 差速规划器
- `include/teb_move_base_omni.launch.py` — TEB 全向规划器

#### 迁移映射

| ROS1 | ROS2 |
|------|------|
| `gazebo_ros/empty_world.launch` | `gazebo_ros/gazebo.launch.py` |
| `gazebo_ros/spawn_model` | `gazebo_ros/spawn_entity.py` |
| `map_server/map_server` | `nav2_map_server/map_server` |
| `amcl/amcl` | `nav2_amcl/amcl` |
| `rviz/rviz` | `rviz2/rviz2` |
| `robot_state_publisher` | `robot_state_publisher/robot_state_publisher` |

### 保留
- 所有 14 个 XML launch 文件完整保留
- 所有资源文件未修改: models/, meshes/, world/, urdf/, map/, config/, rviz/, images/
- 部署配置未修改: docker/, .gitlab-ci.yml, Jenkinsfile

### 测试
- **83/83 用例通过** (100%)
- colcon build 通过 (0.33s)
- 详见: `test_report.md`

---

## [1.0.0] - 2026-05-04 — 3D 激光雷达扩展

### 概述
在原有 2D 仿真基础上扩展 3D 激光雷达 (Velodyne VLP-16) 支持，实现 3D SLAM 仿真能力。

### 新增
- `urdf/xacro/sensors/lidar3d.xacro` — VLP-16 传感器物理描述 (gpu_ray, 16 线)
- `urdf/xacro/gazebo/mbot_with_lidar3d_gazebo.xacro` — 3D 雷达机器人模型
- `config/velodyne_vlp16.yaml` — VLP-16 参数配置
- `rviz/simulator_3d.rviz` — 3D 可视化 RViz 配置
- `launch/simulator_gazebo_3d.launch` — 3D 仿真启动文件

### 修复
- **BUG-001 (Critical)**: `lidar3d_gazebo.xacro` frameName 从 "velodyne" 修正为 "velodyne_link"，与 URDF link 名一致；同步更新 `velodyne_vlp16.yaml` 的 `lidar_frame`
- **BUG-002 (High)**: `velodyne_vlp16.yaml` 的 `imu_topic` 从 "/imu/data" 修正为 "/imu"；`fast_lio2.launch.py` 默认话题改为 "/velodyne_points" 和 "/imu"
- **BUG-003 (Low)**: `simulator_gazebo_3d.launch` 添加 RViz 节点启动

### 测试
- **161/161 用例通过** (100%)
- 3 个缺陷全部修复验证通过
- 详见: `test_report.md`

---

## [0.1.0] — 初始版本

### 功能
- ROS1 Noetic catkin 项目
- 2D 激光雷达 (RPLIDAR A2) 仿真
- GMapping / Cartographer / Hector SLAM 支持
- AMCL 定位 + TEB 导航
- 差速驱动 / 全向驱动双模式
- Gazebo 仿真环境 (house.world)
- mbot 差速驱动机器人模型
