# rosiwit_slam

基于 ROS2 的**分层 SLAM 框架**——三层架构(ROS 接口层 / SLAM 接口层 / 算法层),支持运行时切换多种 SLAM 算法。内置 **FAST-LIO2**(LiDAR-Inertial 紧耦合,IESKF + iKD-Tree)。

📖 完整文档见 [`docs/rosiwit_slam.md`](docs/rosiwit_slam.md)。

---

## 特性

- **三层架构**:ROS 接口层 / SLAM 接口层 / 算法层,职责清晰、解耦
- **运行时算法切换**:config 或 launch 参数,无需重编译
- **算法层脱离 ROS**:对 rclcpp 零依赖,可独立单元测试
- **内置 FAST-LIO2**:IESKF + iKD-Tree,LiDAR+IMU 紧耦合里程计

---

## 快速开始

```bash
# 1. 安装依赖 (Ubuntu 22.04 + ROS2 Humble)
bash scripts/install_dependencies.sh

# 2. 构建
cd ~/rosiwit_ws
colcon build --packages-select rosiwit_slam --symlink-install
source install/setup.bash

# 3. 运行
ros2 launch rosiwit_slam slam.launch.py
```

启动后日志:`SlamNode ready: algo=fast_lio2 imu=/imu lidar=/velodyne_points`。

---

## 架构

```
ROS 接口层 (ros_interface/)   SlamNode: msg↔IMUSample/LidarFrame 转换 + 发布
        │ ISlamAlgorithm (unique_ptr, 工厂创建)
SLAM 接口层 (slam_core/)      ISlamAlgorithm + SlamBase(同步基类) + SlamFactory(运行时工厂)
        │ 继承 SlamBase
算法层 (algorithms/)          FastLio2Algorithm (适配 MapBuilder+IESKF)
```

Node 只通过 `SlamOutput`(位姿+点云)拿结果,看不到任何算法内部细节。数据流与各层职责详见 [`docs/rosiwit_slam.md`](docs/rosiwit_slam.md)。

---

## 切换 / 添加算法

```bash
# 运行时切换 (无需重编译)
ros2 launch rosiwit_slam slam.launch.py slam_algorithm:=<name>
# 或改 config/default.yaml 的 slam_algorithm 字段
```

**添加新算法**:在 `algorithms/<name>/` 实现 `ISlamAlgorithm`(继承 `SlamBase` 复用同步逻辑),在 `SlamFactory::create()` 加分支。ROS 层零改动。完整步骤见 [docs](docs/rosiwit_slam.md)。

---

## 话题与 TF

| 类型 | 名称 | 说明 |
|---|---|---|
| 订阅 | `/imu`, `/velodyne_points` | IMU + PointCloud2(launch 可覆盖) |
| 发布 | `lio_odom` | Odometry(world→body 位姿+速度) |
| 发布 | `lio_path` | 累计轨迹 |
| 发布 | `body_cloud`, `world_cloud` | 当前帧 body/world 系点云 |
| 发布 | `cloud_map` | 全局地图(2s 周期) |
| TF | `odom → base_link` | 可经 config 的 `world_frame`/`body_frame` 配置 |

---

## 目录结构

```
rosiwit_slam/
├── src/
│   ├── ros_interface/          SlamNode + RosUtils (极薄 ROS 层)
│   ├── slam_core/              ISlamAlgorithm + SlamBase + SlamFactory
│   ├── algorithms/fast_lio2/   FastLio2Algorithm + map_builder 核心
│   └── main.cpp                SlamFactory::create() → SlamNode
├── include/{ros_interface,slam_core,algorithms/fast_lio2}/  对应头文件
├── config/default.yaml         唯一配置(扁平格式)
├── launch/slam.launch.py       主 launch(slam_algorithm 参数化)
├── test/                       gtest: slam_factory + slam_base_sync
├── scripts/                    install_dependencies.sh + 数据工具
├── docs/rosiwit_slam.md        完整文档
├── rviz/                       可视化配置
└── CMakeLists.txt, package.xml
```

---

## 测试

```bash
colcon test --packages-select rosiwit_slam --event-handlers console_direct+
```

gtest 覆盖工厂机制 + IMU/LiDAR 时间同步,不依赖 ROS 运行时。

---



# 保存 3D 点云地图
ros2 service call /save_map rosiwit_slam/srv/SaveMap "{path: 'map.pcd'}"

# 保存 2D 占据栅格图(供 nav2 导航)
ros2 service call /save_grid_map rosiwit_slam/srv/SaveGridMap \
  "{pgm_path: 'map.pgm', yaml_path: 'map.yaml', resolution: 0.05}"

# 加载地图(定位模式使用)
ros2 service call /load_map rosiwit_slam/srv/LoadMap "{path: 'map.pcd'}"

# 切换模式
ros2 service call /set_slam_mode rosiwit_slam/srv/SetSlamMode "{mode: 'mapping'}"
ros2 service call /set_slam_mode rosiwit_slam/srv/SetSlamMode "{mode: 'localization'}"

## 许可证

MIT
