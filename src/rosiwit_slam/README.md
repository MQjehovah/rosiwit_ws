# Rosiwit SLAM

**基于 ROS2 的分层 SLAM 框架,支持运行时切换多种 SLAM 算法**

> ## 架构概览 (2026-07 三层重构)
>
> 本包采用**三层架构**,通过 `slam_algorithm` 字段在运行时切换算法:
>
> ```
> src/
> ├── ros_interface/        ROS 接口层: SlamNode (极薄, msg<->IMUSample/LidarFrame 转换 + 发布)
> ├── slam_core/            SLAM 接口层: ISlamAlgorithm + SlamBase(同步基类) + SlamFactory(运行时工厂)
> └── algorithms/fast_lio2/ 算法层: FastLio2Algorithm (封装 MapBuilder+IESKF)
> ```
>
> - 数据流: ROS msg → SlamNode 转 IMUSample/LidarFrame → 算法 onImu/onLidar → SlamOutput 回调 → SlamNode 发布 odom/path/cloud/tf
> - 切换算法: 改 `config/default.yaml` 的 `slam_algorithm`,或 `ros2 launch rosiwit_slam slam.launch.py slam_algorithm:=<name>`
> - 新增算法: 在 `algorithms/<name>/` 实现 `ISlamAlgorithm`,在 `SlamFactory::create()` 加分支,ROS 层零改动
> - 设计文档:`docs/plans/2026-07-07-layered-architecture-design.md`
>
> **下方为重构前的旧文档,目录结构与模块描述已过时。**

---

## 📋 项目概述(旧)

本项目实现了一个基于FAST-LIO2算法的ROS2 SLAM节点，支持3D激光雷达、IMU和里程计的多传感器融合定位与建图。

### 核心特性

- ✅ **FAST-LIO2 IEKF算法**: LiDAR+IMU紧耦合状态估计
- ✅ **iKD-Tree地图管理**: 增量式KD树，高效点云管理
- ✅ **Scan Context闭环检测**: 全局闭环检测与位姿修正
- ✅ **GTSAM后端优化**: 位姿图优化（可选）
- ✅ **多传感器融合**: 支持LiDAR、IMU、Odom数据融合
- ✅ **ROS2原生支持**: 完整的ROS2接口

---

## 📁 目录结构

```
rosiwit_slam/
├── CMakeLists.txt              # CMake构建文件
├── package.xml                 # ROS2包描述
├── README.md                   # 项目文档
│
├── include/fast_lio2_slam/     # 头文件目录（C++ 命名空间）
│   ├── common/
│   │   ├── types.h             # 核心类型定义
│   │   ├── config.h            # 配置管理
│   │   └── utils.h             # 工具函数
│   │
│   ├── data_preprocessor/
│   │   ├── point_cloud_filter.h  # 点云滤波
│   │   ├── imu_processor.h       # IMU处理
│   │
│   ├── fast_lio2_core/
│   │   ├── iekf_estimator.h    # IEKF估计器
│   │   ├── ikd_tree.h          # iKD-Tree
│   │
│   ├── ros_interface/
│   │   ├── fast_lio2_node.h    # ROS2节点主类
│   │
│   ├── odom_fusion/
│   │   ├── odom_fusion.h       # 里程计融合
│   │
│   ├── loop_closure/
│   │   ├── scan_context.h      # Scan Context
│   │   ├── gtsam_backend.h     # GTSAM后端
│   │
│   └── map_manager/
│   │   ├── map_manager.h       # 地图管理
│   │
│   └── config/
│       ├── default.yaml        # 默认配置
│       ├── livox_avia.yaml     # Livox配置
│       ├── velodyne_vlp16.yaml # Velodyne配置
│
├── launch/                     # Launch文件
│   ├── fast_lio2.launch.py
│   ├── livox_avia.launch.py
│
├── rviz/                       # RViz配置
│   ├── fast_lio2.rviz
│
└── src/
    └── main.cpp                # 主入口
```

---

## 🔧 编译与安装

### 依赖项

```bash
# ROS2环境 (Humble或Foxy)
# PCL (1.10+)
# Eigen3 (3.4+)
# Sophus (1.x)
# yaml-cpp (可选)
# GTSAM (可选, 用于后端优化)
```

### 编译步骤

```bash
# 创建ROS2工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 复制项目到工作空间
cp -r rosiwit_slam ~/ros2_ws/src/

# 编译
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置环境变量
source install/setup.bash
```

---

## 🚀 使用方法

### 基本启动

```bash
# 使用默认配置启动
ros2 launch rosiwit_slam fast_lio2.launch.py

# 指定配置文件
ros2 launch rosiwit_slam fast_lio2.launch.py config_file:=livox_avia.yaml

# Livox Avia专用启动
ros2 launch rosiwit_slam livox_avia.launch.py
```

### RViz可视化

```bash
# 启动RViz
rviz2 -d $(ros2 pkg prefix rosiwit_slam)/share/rosiwit_slam/rviz/fast_lio2.rviz
```

---

## 📡 ROS2接口

### 输入话题

| 话题              | 消息类型        | 描述             |
| ----------------- | --------------- | ---------------- |
| `/lidar_points` | `PointCloud2` | 3D激光点云       |
| `/imu/data`     | `Imu`         | IMU数据          |
| `/odom`         | `Odometry`    | 里程计数据(可选) |

### 输出话题

| 话题                | 消息类型        | 描述       |
| ------------------- | --------------- | ---------- |
| `/odom_estimated` | `Odometry`    | 估计里程计 |
| `/path_estimated` | `Path`        | 轨迹路径   |
| `/cloud_map`      | `PointCloud2` | 点云地图   |

### 服务接口

| 服务          | 类型        | 描述        |
| ------------- | ----------- | ----------- |
| `/save_map` | `Trigger` | 保存地图    |
| `/save_pcd` | `Trigger` | 保存PCD文件 |

---

## ⚙️ 参数配置

### 关键参数

| 参数                | 默认值 | 描述          |
| ------------------- | ------ | ------------- |
| `lidar_max_range` | 100.0  | 最大距离      |
| `voxel_size`      | 0.2    | 体素滤波大小  |
| `max_iterations`  | 5      | IEKF迭代次数  |
| `acc_noise`       | 0.1    | IMU加速度噪声 |
| `gyro_noise`      | 0.01   | IMU陀螺仪噪声 |

### 外参配置

```yaml
extrinsic:
  translation: [0.040, 0.0, 0.028]  # LiDAR-IMU平移
  rotation: [0.0, 0.0, 0.0]        # LiDAR-IMU旋转(欧拉角)
```

---

## 📊 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      FAST-LIO2 SLAM Node                     │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐     │
│  │ LiDAR In │  │  IMU In  │  │ Odom In  │  │  Config  │     │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘     │
│       │            │            │            │              │
│       ▼            ▼            ▼            ▼              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              DataPreprocessor                         │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  │   │
│  │  │PC Filter    │  │IMU Buffer   │  │Time Sync    │  │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘  │   │
│  └─────────────────────────────────────────────────────┘   │
│                        │                                    │
│                        ▼                                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                FastLio2Core                          │   │
│  │  ┌─────────────┐  ┌─────────────┐                   │   │
│  │  │IEKF Estimator│  │  iKD-Tree   │                   │   │
│  │  └─────────────┘  └─────────────┘                   │   │
│  └─────────────────────────────────────────────────────┘   │
│                        │                                    │
│        ┌───────────────┼───────────────┐                  │
│        ▼               ▼               ▼                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                │
│  │OdomFusion│  │LoopClosure│ │MapManager│                │
│  └──────────┘  └──────────┘  └──────────┘                │
│        │               │               │                  │
│        └───────────────┼───────────────┘                  │
│                        ▼                                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │                 Output Interface                     │   │
│  │  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐                │   │
│  │  │Odom │  │Path │  │Map  │  │ TF  │                │   │
│  │  └─────┘  └─────┘  └─────┘  └─────┘                │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔬 核心算法

### IEKF状态估计 (24维状态向量)

```
状态向量:
├── 位置 (3)      p = [x, y, z]
├── 姿态 (3)      q = [qw, qx, qy, qz]
├── 速度 (3)      v = [vx, vy, vz]
├── IMU偏置 (6)   ba, bg
├── 重力 (3)      g = [0, 0, -9.81]
└── 外参 (6)      ext_R, ext_T
```

### 点云配准流程

```
1. IMU预测 → 状态传播
2. 特征提取 → 平面点/边缘点
3. 最近邻搜索 → iKD-Tree查询
4. IEKF更新 → 点到平面残差
5. 地图更新 → 增量插入
```

---

## 📈 性能指标

| 指标         | 典型值   |
| ------------ | -------- |
| 单帧处理时间 | 8-35ms   |
| 定位精度     | 厘米级   |
| 支持线数     | 6-64线   |
| 实时性       | 实时运行 |

---

## 📝 开发信息

- **作者**: AI Development Team
- **版本**: 1.0.0
- **License**: MIT
- **创建日期**: 2026-04-24

---

## 🔗 参考资料

1. [FAST-LIO2论文](https://arxiv.org/abs/2107.06829)
2. [FAST-LIO2代码库](https://github.com/hku-mars/FAST_LIO)
3. [Scan Context](https://github.com/irapkaist/SC-LeGO-LOAM)
4. [GTSAM](https://gtsam.org/)

---

## 📌 待完善功能

- [ ] 完整的GTSAM后端集成
- [ ] 多线程并行处理优化
- [ ] 地图压缩与存储优化
- [ ] 单元测试完善
- [X] Docker部署支持

---

## 🐳 Docker部署

本项目提供完整的Docker支持，包含生产环境和开发环境镜像。

### 快速开始

```bash
# 构建镜像
cd docker && ./build.sh

# 运行容器
./run.sh --slam
```

### 详细文档

完整的部署指南请参阅：**[DEPLOYMENT.md](DEPLOYMENT.md)**

| 文档                            | 说明         |
| ------------------------------- | ------------ |
| [DEPLOYMENT.md](DEPLOYMENT.md)     | 完整部署指南 |
| [DEPENDENCIES.md](DEPENDENCIES.md) | 依赖清单     |

### Docker镜像

| 镜像                      | 用途     | 说明                 |
| ------------------------- | -------- | -------------------- |
| `rosiwit-slam:humble-2.0` | 生产环境 | 多阶段构建，安全加固 |
| `rosiwit-slam:devel-2.0`  | 开发环境 | 包含调试工具和测试框架 |

### CI/CD

项目配置了完整的CI/CD流水线：

- **Jenkins**: `Jenkinsfile`
- **GitLab CI**: `.gitlab-ci.yml`

流水线阶段：代码检查 → 编译 → 测试 → Docker构建 → 发布

---

## 📄 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件。

---

## 🙏 致谢

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) - 原始FAST-LIO2算法
- [GTSAM](https://gtsam.org/) - 因子图优化库
- [Sophus](https://github.com/strasdat/Sophus) - 李群/李代数库---

## 🗺️ 建图功能增强

### 地图管理系统

FAST-LIO2 SLAM 现已支持完整的建图功能：

#### 核心特性

- **全局地图维护**: 增量式点云添加，体素滤波去重
- **子地图系统**: 自动创建和管理子地图，支持内存优化
- **多会话建图**: 支持多次采集会话的创建、加载和合并
- **地图持久化**: 支持PCD/PLY/BIN格式保存和加载
- **地图质量评估**: 覆盖度、密度、一致性评估
- **ROS2服务接口**: 提供地图保存/加载服务

#### 建图数据流

```
LiDAR数据 → PointCloudFilter → IEKF估计 → MapManager
                                              │
                              ┌───────────────┼───────────────┐
                              │               │               │
                              ▼               ▼               ▼
                         全局地图       子地图系统       ikd-tree
                              │               │               │
                              └───────────────┼───────────────┘
                                              │
                                              ▼
                                        MapServer
                                              │
                              ┌───────────────┼───────────────┐
                              │               │               │
                              ▼               ▼               ▼
                        ROS2发布        ROS2服务          文件存储
                        (可视化)       (保存/加载)      (持久化)
```

### ROS2服务接口

| 服务名         | 功能     | 参数         |
| -------------- | -------- | ------------ |
| `/save_map`  | 保存地图 | path, format |
| `/load_map`  | 加载地图 | path, merge  |
| `/clear_map` | 清空地图 | -            |
| `/get_map`   | 获取地图 | -            |
| `/save_pcd`  | 保存PCD  | path         |

### 配置参数

```yaml
map:
  resolution: 0.2              # 地图分辨率
  submap:
    enable: true               # 启用子地图
    size: 50.0                 # 子地图大小(米)
    max_points: 50000          # 子地图最大点数
  storage:
    map_path: "./map"          # 地图存储路径
    auto_save: true            # 自动保存
    auto_save_interval: 60.0   # 保存间隔(秒)
```

### 地图存储结构

```
map_project/
├── metadata.yaml           # 元数据
├── global_map.pcd          # 全局点云
├── pose_graph.g2o          # 位姿图
├── submaps/
│   ├── submap_000.pcd
│   ├── submap_000.meta
│   └── ...
├── sessions/
│   ├── session_001.yaml
│   └── ...
└── trajectory/
    ├── poses.txt
    └── timestamps.txt
```

### 使用示例

#### 保存地图

```bash
# ROS2服务调用
ros2 service call /save_map std_srvs/srv/Trigger
```

#### 加载地图

```bash
# 加载并继续建图
ros2 service call /load_map std_srvs/srv/Trigger

# 合并现有地图
ros2 service call /merge_map std_srvs/srv/Trigger
```

### 地图质量评估

```cpp
// 评估地图质量
auto evaluator = std::make_shared<MapQualityEvaluator>();
auto report = evaluator->evaluate(map_manager);

// 输出报告
std::cout << report.generateTextReport();

// 检测空洞
auto holes = evaluator->detectHoles(cloud);

// 计算密度分布
auto density = evaluator->computeDensityDistribution(cloud);
```

---

## 📚 文档

详细文档请参阅 `docs/` 目录：

| 文档                                | 说明                                        |
| ----------------------------------- | ------------------------------------------- |
| [API参考文档](docs/API_REFERENCE.md)   | 所有模块的API详细说明，包括类、方法、参数等 |
| [关键函数说明](docs/KEY_FUNCTIONS.md)  | 核心算法函数的详细说明和算法原理            |
| [模块关系图](docs/MODULE_RELATIONS.md) | 系统架构、模块依赖、数据流向图              |
| [架构设计文档](architecture.md)        | 完整的系统架构设计和实施计划                |

### 快速导航

- **API文档**: 查看 [API_REFERENCE.md](docs/API_REFERENCE.md) 了解所有接口详情
- **算法原理**: 查看 [KEY_FUNCTIONS.md](docs/KEY_FUNCTIONS.md) 了解核心算法实现
- **系统架构**: 查看 [MODULE_RELATIONS.md](docs/MODULE_RELATIONS.md) 了解模块关系
- **设计文档**: 查看 [architecture.md](architecture.md) 了解架构设计

---

## 📌 更新日志

### v1.2.0 (2026-05-05)

- ✅ **包名规范化**: 可执行文件从 `fast_lio2_slam` 重命名为 `fast_lio2_node`，包名统一为 `rosiwit_slam`
- ✅ **编译修复**: CMake 目标名冲突修复、C 语言支持、无效库导出删除
- ✅ **头文件修复**: `fast_lio2_node.h` 双重 shared_ptr、`sophus_se3.hpp` 模板歧义、`global_localizer.h` 重复定义
- ✅ **Launch 文件修复**: `fast_lio2.launch.py` 和 `livox_avia.launch.py` 包名/可执行名更新
- ✅ **集成 Launch**: 新增 `simulator_slam_demo.launch.py` (rosiwit_simulator) 集成仿真器与 SLAM
- ✅ **安全加固**: Docker 容器安全加固 (cap_drop, no-new-privileges, 非 root 用户)
- ✅ **测试通过**: 120 用例全部通过 (118 pass + 2 skip, 0 fail)，第 2 轮回归

### v1.1.0 (2026-04-24)

- ✅ 增加建图功能增强模块
- ✅ 添加MapServer ROS2服务接口
- ✅ 添加MapPersistence持久化模块
- ✅ 添加MapQuality质量评估模块
- ✅ 支持多会话建图
- ✅ 支持子地图系统
- ✅ 完善配置参数

### 待完善功能

- [ ] GTSAM后端集成优化
- [ ] 多线程并行处理
- [ ] 地图压缩优化
- [ ] 重定位功能完善
- [ ] 单元测试完善
