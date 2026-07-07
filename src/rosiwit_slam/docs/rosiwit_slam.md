# rosiwit_slam

基于 ROS2 的**分层 SLAM 框架**,采用三层架构(ROS 接口层 / SLAM 接口层 / 算法层),支持运行时切换多种 SLAM 算法。当前内置 **FAST-LIO2**(LiDAR-Inertial 紧耦合里程计,基于 IESKF + iKD-Tree)。

---

## 一、三层架构

```
┌────────────────────── ROS 接口层 (ros_interface/) ──────────────────────┐
│  SlamNode (rclcpp::Node) + RosUtils                                       │
│  · 订阅 imu/lidar → 转 IMUSample/LidarFrame → algo->onImu()/onLidar()     │
│  · 注册 output 回调:算法产出 SlamOutput 时加锁缓存                         │
│  · timer(20ms):发布 odom/path/body_cloud/world_cloud/tf                 │
│  · map timer(2s):algo->getGlobalMap() → 发布 cloud_map                  │
│  · 不感知任何算法内部细节 (无 IESKF/LidarProcessor 依赖)                    │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │ ISlamAlgorithm* (unique_ptr, 工厂创建)
┌──────────────────────────────▼───────────────────────────────────────────┐
│  SLAM 接口层 (slam_core/)                                                  │
│  · ISlamAlgorithm:抽象基类 (init/onImu/onLidar/state/getOutput/callback) │
│  · SlamBase:内置 IMU/LiDAR 缓冲 + 时间同步, 子类只填 onSyncedPackage()    │
│  · SlamFactory:运行时按字符串创建 ("fast_lio2" → FastLio2Algorithm)        │
│  · slam_types.h:IMUSample / LidarFrame / SlamOutput / PoseStamped        │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │ 继承 SlamBase
┌──────────────────────────────▼───────────────────────────────────────────┐
│  算法层 (algorithms/)                                                      │
│  · fast_lio2/FastLio2Algorithm:适配器, 持有 MapBuilder + IESKF           │
│    onSyncedPackage() → m_builder->process() → 组装 SlamOutput             │
│  · fast_lio2/map_builder/:FAST-LIO2 核心 (ieskf/imu/lidar/ikd/commons)  │
└───────────────────────────────────────────────────────────────────────────┘
```

**核心解耦点**:Node 只通过 `SlamOutput`(纯位姿+点云)拿结果,永远看不到 `IESKF`/`Config`/`LidarProcessor`。算法层对 rclcpp/sensor_msgs 零依赖,可脱离 ROS 独立测试。

**数据流**:
```
imu msg  → SlamNode::imuCB   → IMUSample  → algo->onImu()   ┐
lidar msg→ SlamNode::lidarCB → LidarFrame → algo->onLidar() ┴→ SlamBase 同步
    → FastLio2Algorithm::onSyncedPackage() → MapBuilder.process() → SlamOutput
    → emitOutput() → SlamNode::onOutput() (加锁缓存)
    → SlamNode timer: 发 lio_odom/lio_path/body_cloud/world_cloud/tf
    → map timer: 发 cloud_map
```

---

## 二、目录结构

```
rosiwit_slam/
├── src/
│   ├── ros_interface/           SlamNode (极薄 ROS 层) + RosUtils (ROS↔PCL 转换)
│   ├── slam_core/               ISlamAlgorithm + SlamBase (同步) + SlamFactory
│   ├── algorithms/fast_lio2/    FastLio2Algorithm (适配器) + map_builder/ (核心)
│   └── main.cpp                 SlamFactory::create() → SlamNode
├── include/
│   ├── ros_interface/           slam_node.h, ros_utils.h
│   ├── slam_core/               i_slam_algorithm.h, slam_base.h, slam_factory.h, slam_types.h
│   ├── algorithms/fast_lio2/    fast_lio2_algorithm.h
│   └── pch.hpp                  预编译头 (Eigen/PCL/STL/Sophus)
├── config/default.yaml          唯一配置 (扁平格式, FastLio2Algorithm 读取)
├── launch/slam.launch.py        主 launch (slam_algorithm 参数化)
├── test/                        test_slam_factory.cpp + test_slam_base_sync.cpp
├── scripts/                     install_dependencies.sh + fetch_ntu_viral.py + generate_stats_report.py
├── rviz/                        可视化配置
└── CMakeLists.txt, package.xml
```

---

## 三、环境依赖

**系统**:Ubuntu 22.04 + ROS2 Humble(或 Ubuntu 20.04 + Foxy)。一键安装:

```bash
bash scripts/install_dependencies.sh
```

主要依赖:
| 依赖 | 版本 | 说明 |
|---|---|---|
| ROS2 | Humble | ros-base + rclcpp/sensor_msgs/nav_msgs/geometry_msgs/tf2_ros/pcl_conversions |
| PCL | 1.12+ | common/io/filters/kdtree/registration |
| Eigen3 | 3.4 | 线性代数 |
| Sophus | 1.22.10 | 李群 (源码编译安装) |
| yaml-cpp | 0.7+ | 配置读取 |
| OpenMP | — | lidar_processor 并行 (可选) |

构建工具:`colcon`、`cmake`、`ccache`(加速)。Python(可选):`numpy`/`scipy`/`matplotlib`/`evo`(轨迹评估)。

---

## 四、构建与运行

### 构建
```bash
cd ~/rosiwit_ws
colcon build --packages-select rosiwit_slam --symlink-install
source install/setup.bash
```

### 运行
```bash
ros2 launch rosiwit_slam slam.launch.py
```
节点启动后日志:`SlamNode ready: algo=fast_lio2 imu=/imu lidar=/velodyne_points`。

### Launch 参数
| 参数 | 默认值 | 说明 |
|---|---|---|
| `slam_algorithm` | `fast_lio2` | SLAM 算法名 (SlamFactory 解析) |
| `use_sim_time` | `false` | 仿真时钟 |
| `lidar_topic` | `/velodyne_points` | LiDAR 点云话题 |
| `imu_topic` | `/imu` | IMU 话题 |

配置文件固定为 `config/default.yaml`(可通过修改 launch 的 `config_path` 指向其他文件)。

### 话题与 TF
| 类型 | 名称 | 说明 |
|---|---|---|
| 订阅 | `imu_topic`, `lidar_topic` | IMU + PointCloud2 |
| 发布 | `lio_odom` | nav_msgs/Odometry (world→body 位姿+速度) |
| 发布 | `lio_path` | nav_msgs/Path (累计轨迹) |
| 发布 | `body_cloud`, `world_cloud` | 当前帧 body/world 系点云 |
| 发布 | `cloud_map` | 全局地图 (2s 周期) |
| TF | `world_frame`→`body_frame` | 默认 `odom`→`base_link` |

---

## 五、配置 (`config/default.yaml`)

```yaml
slam_algorithm: fast_lio2     # 算法选择 (SlamFactory)

# ROS 接口
imu_topic: /imu
lidar_topic: /velodyne_points
body_frame: base_link
world_frame: odom

# LiDAR 预处理
lidar_filter_num: 3           # 每 N 点取 1 (降采样)
lidar_min_range: 0.5          # 最小量程 (米)
lidar_max_range: 100.0        # 最大量程
scan_resolution: 0.15         # 扫描体素滤波
map_resolution: 0.3           # 地图体素滤波

# iKD-Tree 地图
cube_len: 300                 # 局部地图立方体边长
det_range: 60                 # 检测范围
move_thresh: 1.5              # 里程计移动阈值

# IMU 噪声 (协方差)
na: 0.01                      # 加速度计噪声
ng: 0.01                      # 陀螺仪噪声
nba: 0.0001                   # 加速度计零偏噪声
nbg: 0.0001                   # 陀螺仪零偏噪声
imu_init_num: 20              # IMU 初始化帧数
near_search_num: 5            # 近邻搜索数
ieskf_max_iter: 5             # IESKF 最大迭代

# 外参 (LiDAR → IMU)
gravity_align: true
esti_il: false                # 是否在线估计时间同步
r_il: [1,0,0, 0,1,0, 0,0,1]   # 旋转矩阵 (行优先)
t_il: [0.0, 0.0, 0.0905]      # 平移 (米)
lidar_cov_inv: 1000.0         # LiDAR 测量协方差逆
```

> 不同传感器只需调整 `lidar_topic`/`lidar_max_range`/外参 `r_il`/`t_il` 等,在 yaml 或 launch 参数覆盖。

---

## 六、切换算法

**运行时切换**(无需重编译):修改 `config/default.yaml` 的 `slam_algorithm` 字段,或 launch 参数:
```bash
ros2 launch rosiwit_slam slam.launch.py slam_algorithm:=<name>
```
未知算法会被 `SlamFactory` 拒绝(FATAL 日志 + 抛异常)。

---

## 七、添加新的 SLAM 算法

新增算法**只动算法层和工厂**,ROS 接口层与 SLAM 接口层零改动:

1. **实现 `ISlamAlgorithm`**(继承 `SlamBase` 复用同步逻辑,或直接实现 `ISlamAlgorithm`):
   ```cpp
   // include/algorithms/my_algo/my_algo_algorithm.h
   namespace rosiwit_slam {
   class MyAlgoAlgorithm : public SlamBase {
   public:
       bool init(const std::string& config_path) override;
       std::string name() const override { return "my_algo"; }
   protected:
       bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;
   };
   }
   ```
   `onSyncedPackage` 收到已同步的 IMU 序列 + LiDAR 帧,产出 `SlamOutput`(位姿 + 可选 body/world 点云)。

2. **工厂注册**:`src/slam_core/slam_factory.cpp` 加分支:
   ```cpp
   if (name == "my_algo") return std::make_unique<MyAlgoAlgorithm>();
   ```

3. **配置**:在 `default.yaml` 设 `slam_algorithm: my_algo`(或新增 `<my_algo>.yaml` 并改 launch 的 `config_path`)。

算法层只能依赖 `slam_core/` 头 + PCL/Eigen/Sophus 等,**不得依赖 rclcpp/sensor_msgs**(保持可脱离 ROS 测试)。

---

## 八、测试

```bash
colcon test --packages-select rosiwit_slam --event-handlers console_direct+
```

| 测试 | 覆盖 |
|---|---|
| `test_slam_factory` | 工厂按名创建 + 拒绝未知算法 |
| `test_slam_base_sync` | IMU/LiDAR 时间同步 + 输出回调转发 |

均为 gtest(注册在 `CMakeLists.txt` 的 `BUILD_TESTING` 块),不依赖 ROS 运行时,可快速验证核心逻辑。

---

## 九、设计文档

三层架构的完整设计决策(方案对比、接口契约、迁移步骤)见 git 历史:
`docs/plans/2026-07-07-layered-architecture-design.md`(已并入主分支提交历史)。
