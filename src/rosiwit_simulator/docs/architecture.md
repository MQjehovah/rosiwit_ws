# 架构说明 — rosiwit_simulator ROS2 Humble

> **版本**: v2.0 (ROS1→ROS2 迁移)
> **日期**: 2026-05-05
> **ROS 版本**: ROS2 Humble Hawksbill
> **构建系统**: ament_cmake

---

## 1. 系统架构总览

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 Humble                             │
│                                                             │
│  ┌──────────┐   ┌──────────────────┐   ┌────────────────┐  │
│  │  Gazebo  │   │ robot_state_     │   │    RViz2       │  │
│  │  仿真器   │──▶│ publisher        │──▶│   可视化       │  │
│  │ (物理引擎)│   │ (TF/URDF发布)    │   │               │  │
│  └────┬─────┘   └──────────────────┘   └────────────────┘  │
│       │                                                     │
│       │ sensor topics                                       │
│       ▼                                                     │
│  ┌──────────┐   ┌──────────────────┐   ┌────────────────┐  │
│  │ 传感器    │   │    SLAM 节点      │   │  导航栈        │  │
│  │ Topics   │──▶│ (GMapping/       │──▶│ (AMCL +        │  │
│  │          │   │  Cartographer/   │   │  move_base)    │  │
│  │          │   │  Hector/         │   │               │  │
│  │          │   │  FAST-LIO2)      │   │               │  │
│  └──────────┘   └──────────────────┘   └────────────────┘  │
│       │                                                     │
│       │ /cmd_vel (输入)                                      │
│       ▼                                                     │
│  ┌──────────────────────────────────────────────────┐      │
│  │              DDS 通信中间层 (Fast-RTPS)           │      │
│  └──────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### v1.0 → v2.0 架构变更

| 方面 | v1.0 (ROS1 Noetic) | v2.0 (ROS2 Humble) |
|------|-------------------|-------------------|
| 通信中间件 | TCPROS/UDPROS (rosmaster) | DDS (Fast-RTPS) |
| 构建系统 | catkin (cmake 3.0.2) | ament_cmake (cmake 3.8+) |
| Launch 格式 | XML (.launch) | Python (.launch.py) |
| 环境加载 | `source devel/setup.bash` | `source install/setup.bash` |
| 桥接需求 | ros1_bridge | 不需要 (原生 ROS2) |
| 节点生命周期 | 无 | 支持 (Lifecycle Node) |

---

## 2. 模块结构

### 2.1 包依赖关系

```
simulator (ament_cmake, format 3)
├── gazebo_ros              — Gazebo 仿真器接口
├── robot_state_publisher   — URDF→TF 发布
├── joint_state_publisher   — 关节状态发布
├── xacro                   — URDF 宏处理
├── rviz2                   — 可视化
├── nav2_map_server         — 地图服务
├── nav2_amcl               — 自适应定位
├── tf2_ros                 — TF2 变换
├── slam_gmapping           — GMapping SLAM
├── cartographer_ros        — Cartographer SLAM
├── hector_mapping          — Hector SLAM
├── move_base               — 导航规划
├── launch                  — Launch 框架
└── launch_ros              — ROS2 Launch 扩展
```

### 2.2 层次结构

```
┌───────────────────────────────────────────┐
│              应用层 (Launch 文件)          │
│  simulator_gazebo / simulator_gazebo_3d   │
│  simulator_mapping_* / simulator_nav_*    │
├───────────────────────────────────────────┤
│              配置层 (YAML + RViz)         │
│  config/diff/ config/omni/ rviz/*.rviz    │
├───────────────────────────────────────────┤
│              模型层 (URDF/Xacro)          │
│  urdf/xacro/gazebo/mbot_*.xacro           │
│  urdf/xacro/sensors/*.xacro               │
├───────────────────────────────────────────┤
│              资源层 (Gazebo 资源)          │
│  models/ meshes/ world/ images/           │
├───────────────────────────────────────────┤
│              基础设施层                    │
│  docker/ .gitlab-ci.yml Jenkinsfile       │
└───────────────────────────────────────────┘
```

---

## 3. 数据流

### 3.1 3D SLAM 仿真数据流

```
Gazebo (物理仿真)
    │
    ├── /velodyne_points (PointCloud2, 10Hz)
    │      │
    │      └──▶ FAST-LIO2 (rosiwit_slam 包)
    │              │
    │              └── /cloud_registered (PointCloud2)
    │              └── /tf (map → odom → base_link)
    │
    ├── /imu (Imu, 100Hz)
    │      │
    │      └──▶ FAST-LIO2
    │
    ├── /odom (Odometry, 50Hz)
    │      │
    │      └──▶ robot_state_publisher
    │              └── /tf (odom → base_footprint → velodyne_link)
    │
    └── /cmd_vel (Twist, 输入)
           │
           └──▶ DiffDrive 插件
                   └── 驱动轮转速控制
```

### 3.2 2D 导航数据流

```
Gazebo → /scan → AMCL → /tf (map → odom)
                  │
                  └── move_base ← /move_base/goal (PoseStamped)
                        │
                        ├── Global Planner (路径规划)
                        └── TEB Local Planner (局部避障)
                              │
                              └── /cmd_vel → Gazebo DiffDrive
```

---

## 4. Launch 文件层级结构

### 4.1 主 Launch 文件

```
simulator_gazebo.launch.py
├── IncludeLaunchDescription: gazebo_ros/gazebo.launch.py
├── Node: spawn_entity (mbot_with_laser_gazebo.xacro)
├── Node: robot_state_publisher (publish_rate=50)
└── Node: joint_state_publisher

simulator_gazebo_3d.launch.py
├── IncludeLaunchDescription: gazebo_ros/gazebo.launch.py
├── Node: spawn_entity (mbot_with_lidar3d_gazebo.xacro)
├── Node: robot_state_publisher (publish_rate=50)
├── Node: joint_state_publisher
└── Node: rviz2 (-d simulator_3d.rviz)
```

### 4.2 SLAM Launch 文件

```
simulator_mapping_gmaping.launch.py
└── Node: slam_gmapping (内嵌完整参数)

simulator_mapping_cartographer.launch.py
├── Node: cartographer_node
└── Node: cartographer_occupancy_grid_node
```

### 4.3 导航 Launch 文件

```
simulator_nav_movebase.launch.py
├── Node: map_server (nav2_map_server)
├── IncludeLaunchDescription: include/amcl_diff.launch.py
│   └── Node: amcl (nav2_amcl, diff model)
└── IncludeLaunchDescription: include/teb_move_base_diff.launch.py
    └── Node: move_base (加载 config/diff/*.yaml)
```

---

## 5. Gazebo 仿真架构

### 5.1 物理引擎配置

- **引擎**: ODE (Open Dynamics Engine)
- **更新频率**: 1000 Hz (默认)
- **时间步长**: 1ms

### 5.2 传感器插件链

```
mbot_with_lidar3d_gazebo.xacro
│
├── mbot_base.xacro
│   └── libgazebo_ros_diff_drive.so
│       ├── Topic: /cmd_vel (input)
│       ├── Topic: /odom (output)
│       └── Frame: odom → base_footprint
│
├── sensors/lidar3d.xacro
│   └── libgazebo_ros_ray_sensor.so
│       ├── Topic: /velodyne_points
│       ├── Frame: velodyne_link
│       └── Type: gpu_ray, 16 线, 10Hz
│
└── sensors/imu_gazebo.xacro
    └── libgazebo_ros_imu_sensor.so
        ├── Topic: /imu
        └── Frame: imu_link
```

---

## 6. 与 rosiwit_slam 的协同

### 6.1 FAST-LIO2 集成

rosiwit_simulator 提供的仿真数据可直接被 rosiwit_slam 包的 FAST-LIO2 节点消费：

| 数据源 | Topic | FAST-LIO2 输入 |
|--------|-------|---------------|
| 3D 点云 | `/velodyne_points` | `lidar_topic` |
| IMU 数据 | `/imu` | `imu_topic` |
| TF 变换 | `/tf` | frame 变换依据 |

### 6.2 启动顺序

```bash
# 终端 1: 启动仿真
ros2 launch simulator simulator_gazebo_3d.launch.py

# 终端 2: 启动 FAST-LIO2
ros2 launch rosiwit_slam fast_lio2.launch.py
```

### 6.3 ROS2 优势

v2.0 迁移后，rosiwit_simulator 与 rosiwit_slam 均运行在 ROS2 生态中，**不再需要 ros1_bridge** 桥接，通信延迟更低、部署更简洁。

---

## 7. 部署架构

### 7.1 Docker 容器化

```
docker/
├── Dockerfile           — 多阶段构建 (base→builder→workspace→runtime)
├── docker-compose.yml   — SLAM 栈编排
├── entrypoint.sh        — 容器入口
├── build.sh            — 构建脚本
└── run.sh              — 运行脚本
```

### 7.2 CI/CD

| 平台 | 配置文件 | 阶段 |
|------|---------|------|
| GitLab CI | `.gitlab-ci.yml` | lint → build → test → deploy |
| Jenkins | `Jenkinsfile` | 声明式 Pipeline |

---

## 8. 安全架构

根据安全审查报告 (`security_report.md`)：

- **容器安全**: `cap_drop ALL` + 最小 `cap_add`, `no-new-privileges`, 资源限制
- **日志**: 日志轮转配置
- **健康检查**: Docker 健康检查机制
- **High 级问题**: 均在 Docker 基础设施层面，不影响仿真器功能

---

## 9. 测试架构

### 9.1 测试覆盖

| 测试类型 | 工具 | 用例数 |
|----------|------|--------|
| 构建系统迁移 | pytest | 7 |
| 资源安装完整性 | pytest | 8 |
| Python Launch 文件 | pytest | 44 |
| 向后兼容 | pytest | 12 |
| 迁移对应关系 | pytest | 12 |
| **总计** | | **83** |

### 9.2 测试文件

```
tests/
├── conftest.py              — pytest fixture (simulator_dir)
├── test_migration.py        — ROS1→ROS2 迁移测试
├── test_launch_imports.py   — Launch 文件导入测试
└── ...
```

---

## 10. 迁移设计决策

### 10.1 保留 XML 原文件

**决策**: 所有 14 个 XML launch 文件完整保留，新建对应的 Python 版本。

**理由**:
- 过渡期兼容性：团队成员可参照 XML 理解 Python 逻辑
- 回退能力：如遇问题可快速切回 ROS1
- 文档价值：XML 作为迁移对照参考

### 10.2 统一参数命名

**决策**: 所有 Python launch 文件使用 `DeclareLaunchArgument` 声明参数，保持与原 XML 的参数名一致。

### 10.3 配置文件不改

**决策**: `config/`, `map/`, `models/` 等资源目录不做任何修改。

**理由**: 这些是纯数据文件，不依赖 ROS 版本。导航参数 (YAML) 由运行时节点解析，构建系统只负责安装到 `share/` 目录。
