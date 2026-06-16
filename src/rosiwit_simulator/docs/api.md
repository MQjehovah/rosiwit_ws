# API & 配置参考 — rosiwit_simulator

> **版本**: v2.0 (ROS2 Humble)
> **日期**: 2026-05-05
> **包名**: `simulator`

---

## 1. Launch 文件 API

所有 Python launch 文件使用 ROS2 `launch` / `launch_ros` API 编写，入口函数为 `generate_launch_description()`。

### 1.1 主 Launch 文件 (`launch/`)

---

#### `simulator_gazebo.launch.py`

**功能**: 启动 2D 激光雷达 Gazebo 仿真环境

**使用**:
```bash
ros2 launch simulator simulator_gazebo.launch.py [参数:=值]
```

**参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `paused` | bool | `false` | 启动时暂停仿真 |
| `use_sim_time` | bool | `true` | 使用仿真时间 |
| `gui` | bool | `true` | 启动 Gazebo GUI |
| `headless` | bool | `false` | 无头模式 |
| `debug` | bool | `false` | 调试模式 |
| `world_name` | string | `house.world` | Gazebo 世界文件 |

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `gazebo` | `gazebo_ros` | Gazebo 仿真器 (通过 `gazebo.launch.py`) |
| `spawn_entity` | `gazebo_ros` | 实例化机器人 URDF 模型 |
| `robot_state_publisher` | `robot_state_publisher` | 发布 TF (50 Hz) |
| `joint_state_publisher` | `joint_state_publisher` | 关节状态发布 |

**模型**: `urdf/xacro/gazebo/mbot_with_laser_gazebo.xacro`

---

#### `simulator_gazebo_3d.launch.py`

**功能**: 启动 3D 激光雷达 (Velodyne VLP-16) Gazebo 仿真环境

**使用**:
```bash
ros2 launch simulator simulator_gazebo_3d.launch.py [参数:=值]
```

**参数**: 同 `simulator_gazebo.launch.py`

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `gazebo` | `gazebo_ros` | Gazebo 仿真器 |
| `spawn_entity` | `gazebo_ros` | 实例化 3D 雷达机器人模型 |
| `robot_state_publisher` | `robot_state_publisher` | 发布 TF (50 Hz) |
| `joint_state_publisher` | `joint_state_publisher` | 关节状态发布 |
| `rviz2` | `rviz2` | RViz2 可视化 (`simulator_3d.rviz`) |

**模型**: `urdf/xacro/gazebo/mbot_with_lidar3d_gazebo.xacro`

**关键 Topics**:

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | ~10 Hz | 3D 点云数据 |
| `/imu` | `sensor_msgs/Imu` | ~100 Hz | IMU 数据 |
| `/scan` | `sensor_msgs/LaserScan` | ~10 Hz | 2D 激光 (仅 2D 模式) |
| `/cmd_vel` | `geometry_msgs/Twist` | - | 速度指令输入 |
| `/odom` | `nav_msgs/Odometry` | ~50 Hz | 里程计 |

---

#### `simulator_mapping_gmaping.launch.py`

**功能**: 启动 GMapping SLAM 建图

**使用**:
```bash
ros2 launch simulator simulator_mapping_gmaping.launch.py
```

**参数**: 无显式参数（使用内部默认值）

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `slam_gmapping` | `slam_gmapping` | GMapping SLAM 节点 |

**依赖**: 需先启动 `simulator_gazebo.launch.py` 或 `simulator_gazebo_3d.launch.py`

---

#### `simulator_mapping_cartographer.launch.py`

**功能**: 启动 Cartographer SLAM 建图

**使用**:
```bash
ros2 launch simulator simulator_mapping_cartographer.launch.py
```

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `cartographer_node` | `cartographer_ros` | Cartographer SLAM 核心节点 |
| `cartographer_occupancy_grid_node` | `cartographer_ros` | 栅格地图生成节点 |

**配置**: 使用 `cartographer_ros` 包内 `configuration_files/test.lua`

---

#### `simulator_amcl_diff.launch.py`

**功能**: 启动 AMCL 自适应蒙特卡洛定位（差速驱动模型）

**使用**:
```bash
ros2 launch simulator simulator_amcl_diff.launch.py
```

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `amcl` | `nav2_amcl` | AMCL 定位节点 (diff 模式) |

**关键参数** (内嵌):

| 参数 | 值 | 说明 |
|------|-----|------|
| `odom_model_type` | `diff` | 差速驱动里程计模型 |
| `min_particles` | 500 | 最小粒子数 |
| `max_particles` | 5000 | 最大粒子数 |
| `laser_model_type` | `likelihood_field` | 激光模型类型 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `transform_tolerance` | 0.1 | TF 容差 |

---

#### `simulator_nav_movebase.launch.py`

**功能**: 启动自主导航栈 (地图服务 + AMCL + move_base)

**使用**:
```bash
ros2 launch simulator simulator_nav_movebase.launch.py map_file:=/path/to/map.yaml
```

**参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_file` | string | `map/test.yaml` | 地图文件路径 |
| `initial_pose_x` | float | `0.0` | 初始位姿 X |
| `initial_pose_y` | float | `0.0` | 初始位姿 Y |
| `initial_pose_a` | float | `0.0` | 初始位姿角度 |

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `map_server` | `nav2_map_server` | 地图服务节点 |
| `amcl` | `nav2_amcl` | AMCL 定位 (via include `amcl_diff.launch.py`) |
| `move_base` | `move_base` | 导航规划器 (via include `teb_move_base_diff.launch.py`) |

---

#### `simulator_map_server.launch.py`

**功能**: 独立启动地图服务

**使用**:
```bash
ros2 launch simulator simulator_map_server.launch.py map_file:=/path/to/map.yaml
```

**参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_file` | string | `/opt/xzrobot/maps/gazebo/map.yaml` | 地图文件路径 |

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `map_server` | `nav2_map_server` | 地图服务 |

---

#### `simulator_rviz.launch.py`

**功能**: 启动 RViz2 可视化

**使用**:
```bash
ros2 launch simulator simulator_rviz.launch.py
```

**参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `model` | string | `mbot_with_laser_gazebo.xacro` | 机器人模型文件路径 |
| `gui` | bool | `false` | GUI 模式 |

**启动的节点**:

| 节点 | 包 | 说明 |
|------|-----|------|
| `rviz2` | `rviz2` | RViz2 可视化 (`rviz/urdf.rviz`) |

---

### 1.2 子 Launch 文件 (`launch/include/`)

以下文件通常不直接调用，而是被主 launch 文件通过 `IncludeLaunchDescription` 引用。

---

#### `amcl_diff.launch.py`

**功能**: AMCL 差速驱动定位配置

**参数**:

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `use_map_topic` | `true` | 使用地图话题 |
| `scan_topic` | `scan` | 激光扫描话题 |
| `initial_pose_x` | `0.0` | 初始 X |
| `initial_pose_y` | `0.0` | 初始 Y |
| `initial_pose_a` | `0.0` | 初始角度 |
| `odom_frame_id` | `odom` | 里程计 frame |
| `base_frame_id` | `base_footprint` | 机器人 frame |
| `global_frame_id` | `map` | 全局 frame |

**节点**: `nav2_amcl/amcl` — `odom_model_type: 'diff'`, 粒子数 2000~5000

---

#### `amcl_omni.launch.py`

**功能**: AMCL 全向驱动定位配置

**参数**: 同 `amcl_diff.launch.py`

**节点**: `nav2_amcl/amcl` — `odom_model_type: 'omni-corrected'`, 粒子数 300~800

---

#### `gmapping_base.launch.py`

**功能**: GMapping SLAM 基础配置

**参数**:

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `scan_topic` | `scan` | 激光扫描话题 |
| `base_frame` | `base_footprint` | 机器人 frame |
| `odom_frame` | `odom` | 里程计 frame |

**节点**: `slam_gmapping/slam_gmapping` — 含完整 GMapping 参数集 (地图分辨率 0.05, 粒子数 8, 最大范围 12m)

---

#### `hector_mapping_base.launch.py`

**功能**: Hector Mapping SLAM 配置

**参数**:

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `tf_map_scanmatch_transform_frame_name` | `scanmatcher_frame` | 扫描匹配 frame |
| `base_frame` | `base_footprint` | 机器人 frame |
| `odom_frame` | `odom` | 里程计 frame |
| `pub_map_odom_transform` | `true` | 发布 map→odom TF |
| `scan_subscriber_queue_size` | `5` | 扫描队列大小 |
| `scan_topic` | `scan` | 激光扫描话题 |
| `map_size` | `2048` | 地图尺寸 |

**节点**: `hector_mapping/hector_mapping` — 地图分辨率 0.05, 激光范围 0.4~5.5m

---

#### `teb_move_base_diff.launch.py`

**功能**: TEB 局部规划器 + move_base（差速驱动）

**节点**: `move_base/move_base` — 加载 `config/diff/` 下 6 个参数文件:
- `costmap_common_params.yaml`
- `local_costmap_params.yaml`
- `global_costmap_params.yaml`
- `base_global_planner_param.yaml`
- `teb_local_planner_params.yaml`
- `move_base_params.yaml`

---

#### `teb_move_base_omni.launch.py`

**功能**: TEB 局部规划器 + move_base（全向驱动）

**节点**: `move_base/move_base` — 加载 `config/omni/` 下 6 个参数文件（同名）

---

## 2. URDF/Xacro 模型参考

### 2.1 可用机器人模型

| Xacro 文件 | 传感器 | 用途 |
|-----------|--------|------|
| `mbot_gazebo.xacro` | 基础底盘 | 最小仿真 |
| `mbot_with_laser_gazebo.xacro` | RPLIDAR A2 | 2D SLAM/导航 |
| `mbot_with_lidar3d_gazebo.xacro` | Velodyne VLP-16 + IMU | 3D SLAM |
| `mbot_with_camera_gazebo.xacro` | USB Camera | 视觉导航 |
| `mbot_with_kinect_gazebo.xacro` | Kinect | 深度感知 |

### 2.2 关键 Link 名称

| Link | 模型 | 说明 |
|------|------|------|
| `base_footprint` | 全部 | 地面投影参考点 |
| `base_link` | 全部 | 机器人中心 |
| `laser_link` | 2D | 2D 激光雷达 |
| `velodyne_link` | 3D | 3D 激光雷达 |
| `imu_link` | 3D | IMU 传感器 |
| `camera_link` | Camera | RGB 相机 |
| `kinect_link` | Kinect | 深度相机 |

### 2.3 Gazebo 插件配置

| 插件 | 传感器 | 参数文件 |
|------|--------|---------|
| `libgazebo_ros_diff_drive.so` | 底盘 | `mbot_base.xacro` |
| `libgazebo_ros_laser.so` | RPLIDAR A2 | `sensors/lidar.xacro` |
| `libgazebo_ros_ray_sensor.so` | VLP-16 | `sensors/lidar3d.xacro` |
| `libgazebo_ros_imu_sensor.so` | IMU | `sensors/imu_gazebo.xacro` |

---

## 3. 配置文件参考

### 3.1 导航参数 (`config/`)

```
config/
├── diff/                          ← 差速驱动参数
│   ├── costmap_common_params.yaml
│   ├── local_costmap_params.yaml
│   ├── global_costmap_params.yaml
│   ├── base_global_planner_param.yaml
│   ├── teb_local_planner_params.yaml
│   └── move_base_params.yaml
└── omni/                          ← 全向驱动参数
    ├── (同上 6 个文件)
```

### 3.2 SLAM 配置 (`config/`)

| 文件 | 用途 |
|------|------|
| `velodyne_vlp16.yaml` | VLP-16 传感器参数 (topic, frame, 样本数等) |
| `fast_lio2.launch.py` (roswit_slam包) | FAST-LIO2 3D SLAM 启动配置 |

### 3.3 传感器参数 (`velodyne_vlp16.yaml`)

```yaml
lidar_topic: "/velodyne_points"
imu_topic: "/imu"
lidar_frame: "velodyne_link"
points_per_scan: 30000
scan_rate: 10
range_min: 0.1
range_max: 100.0
is_dense: true
```

---

## 4. TF 树参考

### 4.1 3D 仿真模式

```
map → odom → base_footprint → base_link → velodyne_link
                                   └──→ imu_link
                                   └──→ [wheel links]
```

### 4.2 2D 仿真模式

```
map → odom → base_footprint → base_link → laser_link
                                   └──→ [wheel links]
```

### 4.3 关键 TF 发布者

| 发布者 | 话题 | 说明 |
|--------|------|------|
| `robot_state_publisher` | `/tf` | URDF 关节 TF |
| `gazebo` | `/tf` | odom → base_footprint |
| `amcl` | `/tf` | map → odom |
| `slam_gmapping` | `/tf` | map → odom |

---

## 5. ROS2 迁移映射表

从 ROS1 XML launch 迁移到 ROS2 Python launch 的关键映射：

### 5.1 Launch 语法映射

| ROS1 (XML) | ROS2 (Python) |
|------------|---------------|
| `<node pkg="P" type="T" name="N"/>` | `Node(package='P', executable='T', name='N')` |
| `<arg name="X" default="V"/>` | `DeclareLaunchArgument('X', default_value='V')` |
| `<param name="X" value="V"/>` | `parameters={'X': 'V'}` |
| `<remap from="A" to="B"/>` | `remappings=[('A', 'B')]` |
| `<include file="$(find P)/launch/F"/>` | `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` |
| `$(find P)/path` | `get_package_share_directory('P')` + `os.path.join()` |
| `<rosparam file="F" command="load"/>` | `parameters=[os.path.join(dir, 'F.yaml')]` |
| `<group>` | `GroupAction(...)` |

### 5.2 节点包名映射

| ROS1 包/节点 | ROS2 包/节点 |
|-------------|-------------|
| `gazebo_ros/empty_world.launch` | `gazebo_ros/gazebo.launch.py` |
| `gazebo_ros/spawn_model` | `gazebo_ros/spawn_entity.py` |
| `robot_state_publisher` | `robot_state_publisher/robot_state_publisher` |
| `joint_state_publisher` | `joint_state_publisher/joint_state_publisher` |
| `rviz/rviz` | `rviz2/rviz2` |
| `map_server/map_server` | `nav2_map_server/map_server` |
| `amcl/amcl` | `nav2_amcl/amcl` |
| `gmapping/slam_gmapping` | `slam_gmapping/slam_gmapping` |
| `tf/static_transform_publisher` | `tf2_ros/static_transform_publisher` |

---

## 6. 构建系统 API

### 6.1 CMakeLists.txt 关键指令

```cmake
cmake_minimum_required(VERSION 3.8)
project(simulator)

find_package(ament_cmake REQUIRED)

# 安装目录
install(DIRECTORY
  launch config map models meshes world urdf rviz images
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 6.2 package.xml 格式

```xml
<?xml version="1.0"?>
<package format="3">
  <name>simulator</name>
  <version>2.0.0</version>
  <description>ROS2 Gazebo simulation environment for mbot robot</description>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

---

## 7. 文件完整性清单

### 7.1 Launch 文件对应关系

| XML (ROS1 原始) | Python (ROS2 迁移) | 状态 |
|-----------------|-------------------|------|
| `simulator_gazebo.launch` | `simulator_gazebo.launch.py` | ✅ |
| `simulator_gazebo_3d.launch` | `simulator_gazebo_3d.launch.py` | ✅ |
| `simulator_mapping_gmaping.launch` | `simulator_mapping_gmaping.launch.py` | ✅ |
| `simulator_mapping_cartographer.launch` | `simulator_mapping_cartographer.launch.py` | ✅ |
| `simulator_amcl_diff.launch` | `simulator_amcl_diff.launch.py` | ✅ |
| `simulator_nav_movebase.launch` | `simulator_nav_movebase.launch.py` | ✅ |
| `simulator_map_server.launch` | `simulator_map_server.launch.py` | ✅ |
| `simulator_rviz.launch` | `simulator_rviz.launch.py` | ✅ |
| `include/amcl_diff.launch` | `include/amcl_diff.launch.py` | ✅ |
| `include/amcl_omni.launch` | `include/amcl_omni.launch.py` | ✅ |
| `include/gmapping_base.launch` | `include/gmapping_base.launch.py` | ✅ |
| `include/hector_mapping_base.launch` | `include/hector_mapping_base.launch.py` | ✅ |
| `include/teb_move_base_diff.launch` | `include/teb_move_base_diff.launch.py` | ✅ |
| `include/teb_move_base_omni.launch` | `include/teb_move_base_omni.launch.py` | ✅ |

**共计**: 14 个 XML → 14 个 Python，100% 覆盖
