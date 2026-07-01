# rosiwit_slam 架构设计

**项目**: 基于ROS2的3D激光+IMU+里程计融合SLAM节点开发
**包名**: `rosiwit_slam` | **可执行**: `rosiwit_slam`
**角色**: 软件架构师
**日期**: 2026-05-05 (更新)
**输入**: 算法研究员技术选型报告 (FAST-LIO2 + 闭环优化扩展方案)

---

## 一、整体架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ROS2 SLAM Node (rosiwit_slam)                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                    │
│  │   LiDAR     │    │    IMU      │    │   Odometry  │                    │
│  │  Driver     │    │   Driver    │    │   Driver    │                    │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                    │
│         │                  │                  │                            │
│         ▼                  ▼                  ▼                            │
│  ┌─────────────────────────────────────────────────────────────────┐      │
│  │                    Data Preprocessing Module                     │      │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │      │
│  │  │ PointCloud   │  │ IMU Buffer   │  │ Odom Buffer  │          │      │
│  │  │ Filter       │  │ & Undistort  │  │ & Sync       │          │      │
│  │  └──────────────┘  └──────────────┘  └──────────────┘          │      │
│  └─────────────────────────────────────────────────────────────────┘      │
│                                  │                                         │
│                                  ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────┐      │
│  │                   State Estimation Module                        │      │
│  │  ┌───────────────────────────────────────────────────────────┐  │      │
│  │  │              FAST-LIO2 Core (IEKF)                         │  │      │
│  │  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │  │      │
│  │  │  │ Prediction │  │  Feature   │  │  Update    │          │  │      │
│  │  │  │ (IMU)      │  │ Extraction │  │ (LiDAR)    │          │  │      │
│  │  │  └────────────┘  └────────────┘  └────────────┘          │  │      │
│  │  │                          │                                 │  │      │
│  │  │                          ▼                                 │  │      │
│  │  │                ┌────────────────┐                         │  │      │
│  │  │                │  ikd-tree Map  │                         │  │      │
│  │  │                │  Management    │                         │  │      │
│  │  │                └────────────────┘                         │  │      │
│  │  └───────────────────────────────────────────────────────────┘  │      │
│  └─────────────────────────────────────────────────────────────────┘      │
│                                  │                                         │
│                                  ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────┐      │
│  │                    Odom Fusion Module                            │      │
│  │  ┌───────────────────────────────────────────────────────────┐  │      │
│  │  │              ESKF / Factor Graph Fusion                    │  │      │
│  │  │  ┌────────────┐  ┌────────────┐  ┌────────────┐          │  │      │
│  │  │  │ Wheel Odom │  │ Visual Odom│  │ Constraints│          │  │      │
│  │  │  │ Factor     │  │ Factor     │  │ Manager    │          │  │      │
│  │  │  └────────────┘  └────────────┘  └────────────┘          │  │      │
│  │  └───────────────────────────────────────────────────────────┘  │      │
│  └─────────────────────────────────────────────────────────────────┘      │
│                                  │                                         │
│                                  ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────┐      │
│  │                  Global Optimization Module                      │      │
│  │  ┌───────────────────┐  ┌───────────────────────────────────┐  │      │
│  │  │  Loop Closure     │  │  GTSAM Backend                    │  │      │
│  │  │  Detection        │  │  ┌───────────┐  ┌─────────────┐   │  │      │
│  │  │  (Scan Context)   │──▶│  │ iSAM2     │  │ Pose Graph  │   │  │      │
│  │  │                   │  │  │ Optimizer │  │ Manager     │   │  │      │
│  │  └───────────────────┘  │  └───────────┘  └─────────────┘   │  │      │
│  │                          └───────────────────────────────────┘  │      │
│  └─────────────────────────────────────────────────────────────────┘      │
│                                  │                                         │
│                                  ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────┐      │
│  │                    Output & Visualization                        │      │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌──────────┐ │      │
│  │  │ Odometry   │  │ PointCloud │  │ TF         │  │ Diagnostics│ │      │
│  │  │ Publisher  │  │ Map Pub    │  │ Broadcaster│  │ Publisher │ │      │
│  │  └────────────┘  └────────────┘  └────────────┘  └──────────┘ │      │
│  └─────────────────────────────────────────────────────────────────┘      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 二、模块列表与职责

### 2.1 模块划分总览

| 模块名称 | 核心职责 | 优先级 | 依赖关系 |
|---------|---------|--------|---------|
| **DataPreprocessor** | 传感器数据预处理与时间同步 | P0 | 无 |
| **FastLio2Core** | FAST-LIO2核心算法实现 | P0 | DataPreprocessor |
| **OdomFusion** | 里程计数据融合 | P1 | FastLio2Core |
| **LoopClosure** | 闭环检测与优化 | P2 | FastLio2Core |
| **MapManager** | 地图存储与管理 | P1 | FastLio2Core, LoopClosure |
| **RosInterface** | ROS2话题/服务/参数接口 | P0 | 所有模块 |
| **ConfigManager** | 参数配置与动态调整 | P0 | 无 |

### 2.2 模块详细说明

#### 📦 Module 1: DataPreprocessor (数据预处理模块)

**职责**:
- 点云降采样、滤波、运动畸变校正
- IMU数据缓存与积分
- Odom数据缓存与同步
- 多传感器时间戳对齐

**核心类**:
```
DataPreprocessor
├── PointCloudFilter          # 点云预处理
│   ├── voxelFilter()         # 体素滤波
│   ├── motionUndistort()     # 运动畸变校正
│   └── extractFeatures()     # 特征提取
├── ImuBuffer                 # IMU数据管理
│   ├── push()                # 数据入队
│   ├── interpolate()         # 时间插值
│   └── propagate()           # 状态传播
└── OdomBuffer                # 里程计数据管理
    ├── syncWithStamp()       # 时间同步
    └── getConstraints()      # 获取约束
```

**输入数据流**:
| 输入 | 类型 | 频率 | 说明 |
|-----|------|------|-----|
| `/lidar_points` | sensor_msgs/PointCloud2 | 10-20Hz | 原始点云 |
| `/imu/data` | sensor_msgs/Imu | 100-400Hz | IMU数据 |
| `/odom` | nav_msgs/Odometry | 50-100Hz | 里程计数据 |

**输出数据流**:
| 输出 | 类型 | 说明 |
|-----|------|-----|
| `ProcessedCloud` | 自定义结构 | 预处理后点云 |
| `ImuState` | 自定义结构 | IMU积分状态 |
| `OdomConstraint` | 自定义结构 | 里程计约束 |

---

#### 📦 Module 2: FastLio2Core (FAST-LIO2核心模块)

**职责**:
- 迭代扩展卡尔曼滤波(IEKF)状态估计
- ikd-tree点云地图管理
- 点云配准与位姿更新
- 增量式建图

**核心算法流程**:
```
StateEstimation Loop:
    1. IMU Prediction (高频 100-400Hz)
       ├── 状态预测: x_hat = f(x, imu)
       └── 协方差传播: P_hat = F * P * F' + Q

    2. LiDAR Update (低频 10-20Hz)
       ├── 特征提取: plane_features, edge_features
       ├── 最近邻搜索: ikd-tree query
       ├── 残差计算: r = point_to_plane_distance
       └── 卡尔曼更新: K = P*H'*(H*P*H' + R)^-1

    3. Map Update
       ├── 点云下采样
       └── ikd-tree增量更新
```

**核心类**:
```
FastLio2Core
├── IekfEstimator             # IEKF估计器
│   ├── predict()             # IMU预测
│   ├── update()              # LiDAR更新
│   └── getPose()             # 获取位姿
├── IkdtreeManager            # ikd-tree管理
│   ├── initialize()          # 初始化
│   ├── addPoints()           # 增量添加
│   ├── nearestSearch()       # 最近邻搜索
│   └── boxSearch()           # 范围搜索
└── StateManager              # 状态管理
    ├── state_                # 当前状态向量
    ├── covariance_           # 协方差矩阵
    └── imu_bias_             # IMU偏置
```

**状态向量定义**:
```cpp
// 状态维度: 24维
// [position(3), rotation(4), velocity(3),
//  ba(3), bg(3), gravity(3), extrinsic_R(3), extrinsic_T(3)]
struct State {
    Eigen::Vector3d position;      // 位置 (x, y, z)
    Eigen::Quaterniond rotation;   // 姿态四元数
    Eigen::Vector3d velocity;      // 速度
    Eigen::Vector3d acc_bias;      // 加速度计偏置
    Eigen::Vector3d gyro_bias;     // 陀螺仪偏置
    Eigen::Vector3d gravity;       // 重力向量
    Eigen::Vector3d ext_R;         // LiDAR-IMU旋转外参
    Eigen::Vector3d ext_T;         // LiDAR-IMU平移外参
};
```

---

#### 📦 Module 3: OdomFusion (里程计融合模块)

**职责**:
- 轮速计/视觉里程计约束因子
- 多源数据融合策略
- 融合后位姿平滑

**融合策略**:
```
Odom Fusion Strategy:
├── 松耦合模式 (Phase 2)
│   └── 后验融合: pose_lidar + pose_odom → ESKF
└── 紧耦合模式 (Phase 3)
    └── 因子图融合: LiDAR因子 + Odom因子 + IMU因子
```

**核心类**:
```
OdomFusion
├── OdomConstraintFactor      # 里程计约束因子
│   ├── computeError()        # 计算误差
│   └── computeJacobian()     # 计算雅可比
├── FusionFilter              # 融合滤波器
│   ├── eskfFusion()          # ESKF融合
│   └── weightedAverage()     # 加权平均
└── SyncManager               # 同步管理
    ├── alignTimestamp()      # 时间对齐
    └── interpolatePose()     # 位姿插值
```

---

#### 📦 Module 4: LoopClosure (闭环检测与优化模块)

**职责**:
- Scan Context闭环检测
- GTSAM因子图后端优化
- 全局位姿图管理
- 地图全局校正

**核心类**:
```
LoopClosure
├── ScanContext               # Scan Context闭环检测
│   ├── makeScanContext()     # 生成描述子
│   ├── detectLoop()          # 检测闭环
│   └── getLoopConstraints()  # 获取闭环约束
├── GtsamBackend              # GTSAM后端优化
│   ├── addPose()             # 添加位姿
│   ├── addOdomFactor()       # 添加里程因子
│   ├── addLoopFactor()       # 添加闭环因子
│   └── optimize()           # 执行优化
└── PoseGraph                 # 位姿图管理
    ├── saveGraph()           # 保存位姿图
    ├── loadGraph()           # 加载位姿图
    └── correctMap()          # 校正地图
```

---

#### 📦 Module 5: MapManager (地图管理模块)

**职责**:
- 点云地图存储与序列化
- 地图分块管理
- 多会话建图支持
- 地图加载与重定位

**核心类**:
```
MapManager
├── PointCloudMap             # 点云地图
│   ├── save()                # 保存PCD文件
│   ├── load()                # 加载PCD文件
│   └── merge()               # 合并地图
├── SubmapManager             # 子地图管理
│   ├── createSubmap()        # 创建子地图
│   ├── activeSubmap()        # 激活子地图
│   └── mergeSubmaps()        # 合并子地图
└── MapServer                 # 地图服务
    ├── publishMap()          # 发布地图
    └── queryMap()            # 查询地图
```

---

#### 📦 Module 6: RosInterface (ROS2接口模块)

**职责**:
- ROS2话题订阅与发布
- ROS2服务端点提供
- TF树广播
- 参数服务器交互

**核心类**:
```
RosInterface
├── Subscribers               # 订阅管理
│   ├── lidar_sub_            # LiDAR订阅
│   ├── imu_sub_              # IMU订阅
│   └── odom_sub_             # Odom订阅
├── Publishers                # 发布管理
│   ├── odom_pub_             # 里程计发布
│   ├── map_pub_              # 地图发布
│   ├── path_pub_             # 轨迹发布
│   └── diag_pub_             # 诊断发布
├── Services                  # 服务管理
│   ├── save_map_srv_         # 保存地图服务
│   ├── load_map_srv_        # 加载地图服务
│   └── reset_srv_           # 重置服务
└── TfBroadcaster             # TF广播
    ├── broadcastOdom()       # 广播odom->base_link
    └── broadcastMap()        # 广播map->odom
```

---

#### 📦 Module 7: ConfigManager (配置管理模块)

**职责**:
- 参数加载与校验
- 动态参数调整
- 参数热更新

**核心类**:
```
ConfigManager
├── ConfigParams              # 配置参数结构
│   ├── lidar_params_         # LiDAR参数
│   ├── imu_params_           # IMU参数
│   ├── odom_params_          # Odom参数
│   └── algorithm_params_     # 算法参数
├── loadConfig()              # 加载配置
├── validateConfig()          # 校验配置
└── updateConfig()            # 更新配置
```

---

## 三、接口协议定义

### 3.1 ROS2话题接口

#### 订阅话题 (Subscribers)

| 话题名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|-----|
| `/lidar_points` | `sensor_msgs/msg/PointCloud2` | 10-20Hz | 3D激光雷达点云 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 100-400Hz | IMU原始数据 |
| `/odom` | `nav_msgs/msg/Odometry` | 50-100Hz | 外部里程计(可选) |

#### 发布话题 (Publishers)

| 话题名称 | 消息类型 | 频率 | 说明 |
|---------|---------|------|-----|
| `/odom_estimated` | `nav_msgs/msg/Odometry` | 10-20Hz | 融合里程计估计 |
| `/path_estimated` | `nav_msgs/msg/Path` | 10Hz | 估计轨迹 |
| `/cloud_map` | `sensor_msgs/msg/PointCloud2` | 1Hz | 点云地图 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 10-20Hz | 配准后点云 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1Hz | 诊断信息 |

### 3.2 ROS2服务接口

| 服务名称 | 服务类型 | 说明 |
|---------|---------|-----|
| `/save_map` | `rosiwit_slam/srv/SaveMap` | 保存当前地图 |
| `/load_map` | `rosiwit_slam/srv/LoadMap` | 加载已有地图 |
| `/reset_estimation` | `std_srvs/srv/Empty` | 重置状态估计 |
| `/get_pose` | `geometry_msgs/srv/GetPose` | 获取当前位姿 |

#### 自定义服务定义

```
# SaveMap.srv
# Request
string file_path
string format  # "pcd", "ply", "bin"
---
# Response
bool success
string message

# LoadMap.srv
# Request
string file_path
bool merge_with_current  # 是否与当前地图合并
---
# Response
bool success
string message
int64 point_count
```

### 3.3 TF树结构

```
map
 └── odom
      └── base_link
           ├── lidar_link
           └── imu_link
```

| TF变换 | 发布者 | 说明 |
|-------|--------|-----|
| `map → odom` | SLAM节点 (闭环后) | 全局优化后的漂移校正 |
| `odom → base_link` | SLAM节点 | 连续里程计估计 |
| `base_link → lidar_link` | 静态TF/URDF | LiDAR外参 |
| `base_link → imu_link` | 静态TF/URDF | IMU外参 |

### 3.4 参数接口

#### Launch参数

```yaml
# fast_lio2.launch.py
lidar_topic: "/lidar_points"
imu_topic: "/imu/data"
odom_topic: "/odom"
lidar_type: "velodyne"  # velodyne/ouster/hesai/livox
scan_line: 16
point_filter_num: 2
```

#### YAML配置文件

```yaml
# config/algorithm.yaml
fast_lio2:
  # 点云滤波参数
  point_filter_num: 2
  filter_size_map: 0.5
  cube_side_length: 200.0

  # LiDAR参数
  lidar_type: "velodyne"
  scan_line: 16
  blind: 0.5  # 最小距离阈值

  # IMU参数
  imu_type: "9-axis"
  imu_acc_noise: 0.1
  imu_gyr_noise: 0.01
  imu_acc_bias_noise: 0.0001
  imu_gyr_bias_noise: 0.00001
  gravity: 9.81

  # 外参标定
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1.0, 0.0, 0.0,  # 四元数 w, x, y, z
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]

  # IEKF参数
  max_iteration: 4
  esikf_converge_threshold: 0.001

  # 里程计融合
  use_odom: false
  odom_noise: 0.05

  # 闭环检测
  enable_loop_closure: false
  loop_closure_interval: 50  # 每50帧检测一次

  # 输出设置
  publish_odom_tf: true
  publish_map_cloud: true
  map_publish_interval: 2.0  # 秒
```

---

## 四、目录结构树

```
rosiwit_slam/
├── CMakeLists.txt
├── package.xml
├── LICENSE
├── README.md
│
├── config/
│   ├── algorithm.yaml           # 算法参数配置
│   ├── sensors/
│   │   ├── velodyne_16.yaml     # Velodyne VLP-16配置
│   │   ├── ouster_64.yaml       # Ouster OS1-64配置
│   │   ├── hesai_pandar.yaml    # 禾赛Pandar配置
│   │   └── livox_mid360.yaml    # Livox Mid-360配置
│   └── extrinsics/
│       ├── lidar_to_imu.yaml    # LiDAR-IMU外参
│       └── lidar_to_odom.yaml   # LiDAR-Odom外参
│
├── launch/
│   ├── fast_lio2.launch.py      # 主启动文件 (Velodyne)
│   ├── livox_avia.launch.py     # Livox Avia 启动文件
│   └── fast_lio2.rviz           # RViz2配置
│
├── msg/
│   └── LocalizationStatus.msg   # 定位状态消息
│
├── srv/
│   ├── GlobalLocalize.srv       # 全局定位服务
│   ├── SetInitialPose.srv       # 设置初始位姿
│   └── GetLocalizationStatus.srv # 获取定位状态
│
├── include/
│   └── fast_lio2_slam/
│       ├── common/
│       │   ├── types.h          # 类型定义
│       │   ├── macros.h         # 宏定义
│       │   └── utils.h          # 工具函数
│       ├── preprocess/
│       │   ├── point_cloud_filter.hpp
│       │   ├── imu_buffer.hpp
│       │   ├── odom_buffer.hpp
│       │   └── time_sync.hpp
│       ├── core/
│       │   ├── iekf_estimator.hpp
│       │   ├── ikd_tree.hpp
│       │   ├── state_manager.hpp
│       │   └── feature_extraction.hpp
│       ├── fusion/
│       │   ├── odom_fusion.hpp
│       │   ├── odom_factor.hpp
│       │   └── fusion_filter.hpp
│       ├── loop_closure/
│       │   ├── scan_context.hpp
│       │   ├── gtsam_backend.hpp
│       │   └── pose_graph.hpp
│       ├── mapping/
│       │   ├── map_manager.hpp
│       │   ├── submap.hpp
│       │   └── map_server.hpp
│       ├── ros_interface/
│       │   ├── ros_publisher.hpp
│       │   ├── ros_subscriber.hpp
│       │   ├── ros_service.hpp
│       │   └── tf_broadcaster.hpp
│       └── config/
│           ├── config_manager.hpp
│           └── config_params.hpp
│
├── src/
│   ├── main.cpp                 # 节点入口
│   ├── fast_lio2_node.cpp       # 主节点类
│   ├── preprocess/
│   │   ├── point_cloud_filter.cpp
│   │   ├── imu_buffer.cpp
│   │   └── odom_buffer.cpp
│   ├── core/
│   │   ├── iekf_estimator.cpp
│   │   ├── ikd_tree.cpp
│   │   ├── state_manager.cpp
│   │   └── feature_extraction.cpp
│   ├── fusion/
│   │   ├── odom_fusion.cpp
│   │   ├── odom_factor.cpp
│   │   └── fusion_filter.cpp
│   ├── loop_closure/
│   │   ├── scan_context.cpp
│   │   ├── gtsam_backend.cpp
│   │   └── pose_graph.cpp
│   ├── mapping/
│   │   ├── map_manager.cpp
│   │   ├── submap.cpp
│   │   └── map_server.cpp
│   ├── ros_interface/
│   │   ├── ros_publisher.cpp
│   │   ├── ros_subscriber.cpp
│   │   ├── ros_service.cpp
│   │   └── tf_broadcaster.cpp
│   └── config/
│       ├── config_manager.cpp
│       └── config_params.cpp
│
├── third_party/
│   ├── ikd-Tree/                # ikd-tree库
│   ├── scan_context/            # Scan Context库
│   └── README.md
│
├── test/
│   ├── unit/
│   │   ├── test_iekf_estimator.cpp
│   │   ├── test_ikd_tree.cpp
│   │   ├── test_imu_buffer.cpp
│   │   └── test_loop_closure.cpp
│   ├── integration/
│   │   ├── test_data_pipeline.cpp
│   │   └── test_full_system.cpp
│   └── data/
│       └── sample_data.yaml    # 测试数据配置
│
├── scripts/
│   ├── evaluate_evo.py         # EVO轨迹评估
│   ├── calibrate_extrinsics.py  # 外参标定工具
│   └── plot_results.py         # 结果可视化
│
└── docs/
    ├── installation.md         # 安装指南
    ├── configuration.md         # 配置说明
    ├── api_reference.md         # API参考
    └── tutorials/
        ├── quick_start.md       # 快速开始
        ├── calibration.md       # 标定教程
        └── advanced.md          # 高级功能
```

---

## 五、核心类与函数原型

### 5.1 类型定义 (types.h)

```cpp
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

namespace fast_lio2_slam {

// 状态向量 (24维)
struct State {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, 9.81);
    Eigen::Vector3d ext_R = Eigen::Vector3d::Zero();  // LiDAR-IMU旋转(欧拉角)
    Eigen::Vector3d ext_T = Eigen::Vector3d::Zero();  // LiDAR-IMU平移

    double timestamp = 0.0;

    // 转换为SE3
    Sophus::SE3d toSE3() const;
    Eigen::Matrix4d toMatrix() const;
};

// IMU测量数据
struct ImuData {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;

    ImuData() : timestamp(0), acc(Eigen::Vector3d::Zero()),
                gyro(Eigen::Vector3d::Zero()) {}
};

// 点云数据
struct PointCloudData {
    double timestamp;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud;
    int scan_id;

    PointCloudData() : timestamp(0), scan_id(0) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    }
};

// 里程计数据
struct OdomData {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;

    OdomData() : timestamp(0), position(Eigen::Vector3d::Zero()),
                 rotation(Eigen::Quaterniond::Identity()),
                 linear_velocity(Eigen::Vector3d::Zero()),
                 angular_velocity(Eigen::Vector3d::Zero()) {}
};

// 闭环约束
struct LoopConstraint {
    int from_id;
    int to_id;
    Sophus::SE3d relative_pose;
    Eigen::Matrix6d information;
    double score;
};

// 子地图
struct Submap {
    int id;
    Sophus::SE3d pose;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud;
    std::vector<int> frame_ids;

    Submap() : id(-1) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    }
};

// 配置参数
struct ConfigParams {
    // LiDAR参数
    std::string lidar_type;
    int scan_line;
    int point_filter_num;
    double blind_distance;

    // IMU参数
    double acc_noise;
    double gyr_noise;
    double acc_bias_noise;
    double gyr_bias_noise;
    double gravity;

    // 外参
    Eigen::Vector3d extrinsic_T;
    Eigen::Matrix3d extrinsic_R;

    // 算法参数
    int max_iteration;
    double converge_threshold;
    double filter_size_map;
    double cube_side_length;

    // 功能开关
    bool use_odom;
    bool enable_loop_closure;
    bool publish_tf;

    ConfigParams();
    bool loadFromYaml(const std::string& file_path);
    bool validate();
};

} // namespace fast_lio2_slam
```

### 5.2 核心类原型

#### FastLio2Node 主节点类

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/preprocess/imu_buffer.hpp"
#include "fast_lio2_slam/preprocess/point_cloud_filter.hpp"
#include "fast_lio2_slam/core/iekf_estimator.hpp"
#include "fast_lio2_slam/core/ikd_tree.hpp"
#include "fast_lio2_slam/mapping/map_manager.hpp"
#include "fast_lio2_slam/ros_interface/ros_publisher.hpp"
#include "fast_lio2_slam/ros_interface/ros_subscriber.hpp"
#include "fast_lio2_slam/config/config_manager.hpp"

namespace fast_lio2_slam {

class FastLio2Node : public rclcpp::Node {
public:
    explicit FastLio2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FastLio2Node();

    // 初始化
    bool initialize();

private:
    // 回调函数
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 核心处理流程
    void processScan(const PointCloudData& scan_data);
    bool imuPredict(double scan_time);
    bool lidarUpdate(const PointCloudData& scan_data);
    void publishResults();

    // 服务回调
    bool saveMapCallback(const std::shared_ptr<SaveMap::Request> req,
                         std::shared_ptr<SaveMap::Response> res);
    bool loadMapCallback(const std::shared_ptr<LoadMap::Request> req,
                         std::shared_ptr<LoadMap::Response> res);
    bool resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                       std::shared_ptr<std_srvs::srv::Empty::Response> res);

    // 成员变量
    std::unique_ptr<ConfigManager> config_manager_;
    std::unique_ptr<ImuBuffer> imu_buffer_;
    std::unique_ptr<PointCloudFilter> cloud_filter_;
    std::unique_ptr<IekfEstimator> estimator_;
    std::unique_ptr<IkdTree> ikd_tree_;
    std::unique_ptr<MapManager> map_manager_;
    std::unique_ptr<RosPublisher> publisher_;
    std::unique_ptr<RosSubscriber> subscriber_;

    // 状态
    State current_state_;
    bool is_initialized_;
    int frame_count_;
    double last_scan_time_;

    // 线程安全
    std::mutex state_mutex_;
    std::mutex map_mutex_;
};

} // namespace fast_lio2_slam
```

#### IekfEstimator IEKF估计器

```cpp
#pragma once
#include "fast_lio2_slam/common/types.h"
#include <Eigen/Core>

namespace fast_lio2_slam {

class IekfEstimator {
public:
    IekfEstimator();
    ~IekfEstimator();

    // 初始化
    void initialize(const ConfigParams& config);
    void reset();

    // IMU预测 (Prediction Step)
    void predict(const ImuData& imu_data, double dt);

    // LiDAR更新 (Update Step)
    bool update(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud,
                IkdTree* map_tree,
                const ConfigParams& config);

    // 获取状态
    State getState() const { return state_; }
    Eigen::Matrix<double, 24, 24> getCovariance() const { return P_; }

    // 设置状态
    void setState(const State& state) { state_ = state; }

private:
    // 状态转移矩阵
    Eigen::Matrix<double, 24, 24> computeF(const State& state,
                                            const ImuData& imu,
                                            double dt);

    // 观测矩阵
    Eigen::Matrix<double, 1, 24> computeH(const pcl::PointXYZINormal& point,
                                          const State& state);

    // 点到面距离残差
    double computeResidual(const pcl::PointXYZINormal& point,
                           const Eigen::Vector3d& normal,
                           const State& state);

    // 迭代更新
    bool iterateUpdate(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud,
                       IkdTree* map_tree,
                       int max_iter,
                       double threshold);

    // 成员变量
    State state_;                                  // 状态向量
    Eigen::Matrix<double, 24, 24> P_;              // 协方差矩阵
    Eigen::Matrix<double, 12, 12> Q_;              // 过程噪声
    double R_;                                     // 观测噪声

    bool initialized_;
};

} // namespace fast_lio2_slam
```

#### IkdTree ikd-tree点云地图

```cpp
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>

namespace fast_lio2_slam {

struct BoxPointType {
    Eigen::Vector3d vertex_min;
    Eigen::Vector3d vertex_max;
};

class IkdTree {
public:
    IkdTree();
    ~IkdTree();

    // 初始化
    void initialize(int max_points = 100);

    // 点云操作
    void addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud);
    void removePoints(const std::vector<int>& indices);
    void clear();

    // 查询操作
    int nearestSearch(const pcl::PointXYZINormal& point,
                      pcl::PointXYZINormal& nearest,
                      double& distance);

    int radiusSearch(const pcl::PointXYZINormal& point,
                     double radius,
                     std::vector<pcl::PointXYZINormal>& neighbors,
                     std::vector<double>& distances);

    int boxSearch(const BoxPointType& box,
                  std::vector<pcl::PointXYZINormal>& points);

    // 地图管理
    int size() const;
    void setDownsampleSize(double size);
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr getCloud() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace fast_lio2_slam
```

#### ImuBuffer IMU数据缓存

```cpp
#pragma once
#include "fast_lio2_slam/common/types.h"
#include <deque>
#include <mutex>

namespace fast_lio2_slam {

class ImuBuffer {
public:
    ImuBuffer(size_t max_size = 2000);
    ~ImuBuffer();

    // 数据操作
    void push(const ImuData& imu);
    void clear();
    bool empty() const;
    size_t size() const;

    // 查询操作
    bool getImuData(double timestamp, ImuData& imu);
    bool getImuInterval(double t_start, double t_end,
                        std::vector<ImuData>& imu_buffer);

    // 状态传播
    bool propagate(const State& state_init, double t_start, double t_end,
                   State& state_result);

    // IMU积分
    Eigen::Vector3d integrateAcc(double t_start, double t_end);
    Eigen::Vector3d integrateGyro(double t_start, double t_end);

private:
    // 线性插值
    ImuData interpolate(const ImuData& imu1, const ImuData& imu2, double t);

    std::deque<ImuData> buffer_;
    size_t max_size_;
    mutable std::mutex mutex_;
};

} // namespace fast_lio2_slam
```

#### PointCloudFilter 点云预处理

```cpp
#pragma once
#include "fast_lio2_slam/common/types.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fast_lio2_slam {

class PointCloudFilter {
public:
    PointCloudFilter();
    ~PointCloudFilter();

    // 初始化
    void initialize(const ConfigParams& config);

    // 点云滤波
    void voxelFilter(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, double size);
    void removeNearPoints(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                          double min_distance);
    void removeFarPoints(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                         double max_distance);

    // 运动畸变校正
    bool motionUndistort(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
                         const std::vector<ImuData>& imu_buffer,
                         const State& state_begin,
                         double scan_time_start,
                         double scan_time_end);

    // 特征提取
    void extractFeatures(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud,
                         pcl::PointCloud<pcl::PointXYZINormal>::Ptr& features);

    // 点云转换
    void transformCloud(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_in,
                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud_out,
                        const State& state);

private:
    ConfigParams config_;
};

} // namespace fast_lio2_slam
```

#### ScanContext 闭环检测

```cpp
#pragma once
#include "fast_lio2_slam/common/types.h"
#include <vector>
#include <memory>

namespace fast_lio2_slam {

struct ScanContextDescriptor {
    int ring_num;
    int sector_num;
    Eigen::MatrixXd descriptor;
    int scan_id;
    double timestamp;
};

class ScanContext {
public:
    ScanContext();
    ~ScanContext();

    // 初始化
    void initialize(int ring_num = 20, int sector_num = 60, double max_range = 80.0);

    // 生成描述子
    ScanContextDescriptor makeScanContext(
        const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud, int scan_id);

    // 闭环检测
    bool detectLoop(const ScanContextDescriptor& query,
                    int& match_id,
                    double& yaw_diff,
                    double& score);

    // 管理操作
    void addDescriptor(const ScanContextDescriptor& desc);
    void saveDatabase(const std::string& file_path);
    bool loadDatabase(const std::string& file_path);
    void clear();
    int size() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace fast_lio2_slam
```

#### GtsamBackend GTSAM后端优化

```cpp
#pragma once
#include "fast_lio2_slam/common/types.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace fast_lio2_slam {

class GtsamBackend {
public:
    GtsamBackend();
    ~GtsamBackend();

    // 初始化
    void initialize();
    void reset();

    // 添加因子
    void addPriorFactor(int pose_id, const Sophus::SE3d& pose, double noise);
    void addOdomFactor(int from_id, int to_id, const Sophus::SE3d& relative_pose, double noise);
    void addLoopFactor(int from_id, int to_id, const Sophus::SE3d& relative_pose, double noise);
    void addImuFactor(int from_id, int to_id, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    // 优化
    bool optimize();

    // 获取结果
    Sophus::SE3d getPose(int pose_id);
    std::vector<Sophus::SE3d> getAllPoses();
    int poseCount() const;

    // 保存/加载
    void saveGraph(const std::string& file_path);
    bool loadGraph(const std::string& file_path);

private:
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimate_;
    gtsam::ISAM2 isam2_;
    std::map<int, gtsam::Key> pose_keys_;
    int current_pose_id_;
};

} // namespace fast_lio2_slam
```

#### MapManager 地图管理

```cpp
#pragma once
#include "fast_lio2_slam/common/types.h"
#include "fast_lio2_slam/mapping/submap.hpp"
#include <pcl/point_cloud.h>
#include <mutex>

namespace fast_lio2_slam {

class MapManager {
public:
    MapManager();
    ~MapManager();

    // 初始化
    void initialize(double voxel_size, double submap_size);

    // 地图操作
    void addPoints(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud,
                   const Sophus::SE3d& pose);
    void updateCurrentSubmap();

    // 子地图管理
    int createNewSubmap();
    Submap::Ptr getCurrentSubmap();
    std::vector<Submap::Ptr> getAllSubmaps();

    // 全局地图
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr getGlobalMap();
    void applyPoseCorrection(const std::vector<Sophus::SE3d>& corrected_poses);

    // 持久化
    bool saveMap(const std::string& file_path, const std::string& format = "pcd");
    bool loadMap(const std::string& file_path, bool merge = false);

    // 统计
    size_t pointCount() const;
    size_t submapCount() const;

private:
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr global_map_;
    std::vector<Submap::Ptr> submaps_;
    int current_submap_id_;
    double voxel_size_;
    double submap_size_;
    mutable std::mutex mutex_;
};

} // namespace fast_lio2_slam
```

---

## 六、数据流与执行流程

### 6.1 主循环流程图

```
┌─────────────────────────────────────────────────────────────────┐
│                        Main Loop                                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  while (rclcpp::ok()) {                                         │
│                                                                  │
│      ┌──────────────┐                                           │
│      │ IMU Callback │ ──▶ [IMU Buffer] Push                     │
│      └──────────────┘                                           │
│             │                                                    │
│             ▼                                                    │
│      ┌──────────────┐                                           │
│      │LiDAR Callback│ ──▶ [Cloud Filter] Process               │
│      └──────────────┘                                           │
│             │                                                    │
│             ▼                                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │               processScan(scan_data)                    │   │
│  │                                                          │   │
│  │   1. [IMU Buffer] getImuInterval()                      │   │
│  │      └─▶ IMU数据时间同步                                │   │
│  │                                                          │   │
│  │   2. [Cloud Filter] motionUndistort()                    │   │
│  │      └─▶ 运动畸变校正                                    │   │
│  │                                                          │   │
│  │   3. [IEKF] predict()                                    │   │
│  │      └─▶ IMU预测传播                                     │   │
│  │                                                          │   │
│  │   4. [IEKF] update()                                     │   │
│  │      ├─▶ 特征提取                                        │   │
│  │      ├─▶ ikd-tree最近邻搜索                              │   │
│  │      ├─▶ 残差计算                                        │   │
│  │      └─▶ 迭代卡尔曼更新                                  │   │
│  │                                                          │   │
│  │   5. [IkdTree] addPoints()                              │   │
│  │      └─▶ 增量更新地图                                    │   │
│  │                                                          │   │
│  │   6. [Map Manager] addPoints()                          │   │
│  │      └─▶ 更新全局地图                                    │   │
│  │                                                          │   │
│  │   7. [Loop Closure] detectLoop() [可选, Phase 3]        │   │
│  │      └─▶ 闭环检测                                        │   │
│  │                                                          │   │
│  │   8. [Publisher] publishResults()                        │   │
│  │      ├─▶ 发布Odometry                                    │   │
│  │      ├─▶ 发布Path                                        │   │
│  │      ├─▶ 发布Map                                         │   │
│  │      └─▶ 发布TF                                          │   │
│  │                                                          │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                  │
│  } // end while                                                  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 线程模型

```
┌─────────────────────────────────────────────────────────────────┐
│                        Thread Architecture                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌────────────────────────────────────────────────────────┐    │
│  │           Main Thread (ROS2 Executor)                  │    │
│  │  ├─ IMU Callback (异步, 高频)                          │    │
│  │  ├─ LiDAR Callback (异步, 中频)                        │    │
│  │  └─ Odom Callback (异步, 可选)                         │    │
│  └────────────────────────────────────────────────────────┘    │
│                                                                  │
│  ┌────────────────────────────────────────────────────────┐    │
│  │         Processing Thread (可选, Phase 2+)            │    │
│  │  ├─ 闭环检测 (低频)                                    │    │
│  │  ├─ 地图优化 (低频)                                    │    │
│  │  └─ 地图保存/加载 (按需)                               │    │
│  └────────────────────────────────────────────────────────┘    │
│                                                                  │
│  ┌────────────────────────────────────────────────────────┐    │
│  │         Visualization Thread (可选)                    │    │
│  │  ├─ 地图可视化发布 (低频)                              │    │
│  │  └─ 轨迹可视化发布 (中频)                              │    │
│  └────────────────────────────────────────────────────────┘    │
│                                                                  │
│  数据同步:                                                       │
│  ├─ IMU Buffer: 线程安全队列                                   │
│  ├─ State: std::mutex保护                                      │
│  └─ Map: std::mutex保护                                        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 七、实施路线图

### Phase 1: FAST-LIO2基础部署 (优先级: P0)

| 任务 | 描述 | 预估工时 | 依赖 |
|-----|------|---------|------|
| ROS2节点框架搭建 | 创建ROS2包, 配置CMake | 2h | - |
| 数据预处理模块 | 点云滤波、IMU缓存 | 4h | 节点框架 |
| IEKF核心实现 | 状态估计、ikd-tree | 8h | 数据预处理 |
| 地图管理基础 | 增量建图、地图发布 | 4h | IEKF核心 |
| 基础测试验证 | 单元测试、rosbag测试 | 4h | 全部模块 |

**交付物**: 基础FAST-LIO2里程计功能

### Phase 2: Odom融合扩展 (优先级: P1)

| 任务 | 描述 | 预估工时 | 依赖 |
|-----|------|---------|------|
| Odom数据接口 | 订阅、时间同步 | 2h | Phase 1 |
| 融合滤波器 | ESKF/加权融合 | 6h | Odom接口 |
| 外参标定工具 | LiDAR-Odom标定 | 4h | 融合滤波器 |
| 融合测试验证 | 多场景测试 | 4h | 全部模块 |

**交付物**: 支持里程计融合的SLAM节点

### Phase 3: 全局优化模块 (优先级: P2)

| 任务 | 描述 | 预估工时 | 依赖 |
|-----|------|---------|------|
| Scan Context集成 | 闭环检测 | 6h | Phase 2 |
| GTSAM后端优化 | 因子图优化 | 8h | Scan Context |
| 地图校正 | 全局地图更新 | 4h | GTSAM后端 |
| 位姿图管理 | 保存/加载 | 2h | 地图校正 |

**交付物**: 完整闭环SLAM功能

### Phase 4: 工程化完善 (优先级: P3)

| 任务 | 描述 | 预估工时 | 依赖 |
|-----|------|---------|------|
| 参数配置系统 | 动态参数调整 | 4h | Phase 3 |
| 诊断监控接口 | 状态监控、日志 | 3h | 配置系统 |
| 性能优化 | 多线程、内存优化 | 6h | 全部模块 |
| 文档完善 | API文档、教程 | 4h | 全部模块 |

**交付物**: 生产就绪的SLAM节点

---

## 八、风险评估与应对

| 风险项 | 影响等级 | 概率 | 应对措施 |
|-------|---------|------|---------|
| IMU时间戳不同步 | 高 | 高 | 软件时间对齐, 建议硬件PTP同步 |
| LiDAR-IMU外参不准 | 高 | 中 | 提供标定工具, 支持在线标定 |
| 计算资源不足 | 中 | 中 | 点云降采样, 优化算法 |
| 闭环检测误匹配 | 中 | 低 | 增加几何校验, 调整阈值 |
| 地图内存溢出 | 低 | 低 | 子地图管理, 地图压缩 |

---

## 九、参考资料

### 论文
1. FAST-LIO2: Fast Direct LiDAR-Inertial Odometry (TRO 2022)
2. Scan Context: Egocentric Spatial Descriptor (IROS 2018)
3. GTSAM: Factor Graph Optimization (ICRA 2012)

### 开源项目
1. FAST-LIO2: https://github.com/hku-mars/FAST_LIO
2. ikd-Tree: https://github.com/hku-mars/ikd-Tree
3. Scan Context: https://github.com/irapkaist/scancontext

### ROS2文档
1. ROS2 Humble: https://docs.ros.org/en/humble/
2. tf2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html---

## 七、建图功能增强设计

### 7.1 建图功能增强概述

基于现有的FAST-LIO2基础架构，增强建图功能以支持完整的SLAM建图需求：

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          建图功能增强架构                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                      Enhanced Mapping System                         │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐      │   │
│  │  │   Map Manager   │  │  Submap System  │  │  Map Server     │      │   │
│  │  │  (地图管理器)    │  │  (子地图系统)    │  │  (地图服务)     │      │   │
│  │  │                 │  │                 │  │                 │      │   │
│  │  │ - 全局地图维护   │  │ - 子地图创建    │  │ - ROS2服务     │      │   │
│  │  │ - 点云滤波     │  │ - 子地图切换    │  │ - 地图保存     │      │   │
│  │  │ - 地图校正     │  │ - 子地图合并    │  │ - 地图加载     │      │   │
│  │  │ - 内存优化     │  │ - 活跃子地图    │  │ - 地图发布     │      │   │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘      │   │
│  │           │                   │                    │                │   │
│  │           ▼                   ▼                    ▼                │   │
│  │  ┌──────────────────────────────────────────────────────────────┐   │   │
│  │  │                    Persistent Storage                         │   │   │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐          │   │   │
│  │  │  │ PCD Files   │  │ Pose Graph  │  │ Metadata    │          │   │   │
│  │  │  │ (点云文件)   │  │ (位姿图)    │  │ (元数据)    │          │   │   │
│  │  │  └─────────────┘  └─────────────┘  └─────────────┘          │   │   │
│  │  └──────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  │  ┌──────────────────────────────────────────────────────────────┐   │   │
│  │  │                    Multi-Session Support                      │   │   │
│  │  │  - 会话管理 (创建/加载/合并)                                   │   │   │
│  │  │  - 地图拼接 (多轨迹融合)                                       │   │   │
│  │  │  - 重定位支持                                                  │   │   │
│  │  └──────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  └──────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 7.2 建图模块详细设计

#### 📦 Module: MapManager (地图管理器增强)

**核心职责**:
1. 全局点云地图维护
2. 增量式点云添加
3. 体素滤波去重
4. 地图内存优化
5. 闭环后地图校正

**增强接口定义**:

```cpp
/**
 * @file map_manager.h (增强版)
 * @brief 地图管理器 - 支持多会话建图与地图持久化
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <vector>
#include <memory>
#include <mutex>
#include <unordered_map>
#include "fast_lio2_slam/common/types.h"

namespace fast_lio2_slam {

// ============== 子地图配置 ==============
struct SubmapConfig {
    double size = 50.0;              // 子地图大小 (米)
    int max_points = 50000;          // 子地图最大点数
    double overlap_ratio = 0.2;      // 子地图重叠率
    bool enable_compression = true;  // 启用压缩
};

// ============== 子地图结构 ==============
struct Submap {
    int id;                           // 子地图ID
    std::string session_id;           // 会话ID
    SE3d center_pose;                 // 子地图中心位姿
    PointCloudPtr cloud;              // 点云数据
    std::vector<int> frame_ids;       // 包含的帧ID
    std::vector<SE3d> frame_poses;    // 帧位姿
    bool is_active = false;           // 是否活跃
    bool is_modified = false;         // 是否修改过
    double timestamp_start;           // 开始时间戳
    double timestamp_end;             // 结束时间戳

    // 边界信息
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;

    // 描述子 (用于回环检测)
    std::vector<float> descriptor;

    using Ptr = std::shared_ptr<Submap>;
};

// ============== 会话信息 ==============
struct SessionInfo {
    std::string session_id;           // 会话ID
    std::string name;                 // 会话名称
    double start_time;                // 开始时间
    double end_time;                  // 结束时间
    int frame_count;                   // 帧数
    SE3d start_pose;                  // 起始位姿
    SE3d end_pose;                    // 结束位姿
    std::vector<int> submap_ids;      // 包含的子地图ID
    bool is_merged = false;           // 是否已合并
};

// ============== 地图元数据 ==============
struct MapMetadata {
    std::string map_name;             // 地图名称
    std::string version = "1.0";      // 版本号
    double created_time;              // 创建时间
    double modified_time;             // 修改时间
    int total_points;                  // 总点数
    int total_submaps;                 // 总子地图数
    int total_sessions;                // 总会话数
    Eigen::Vector3d map_center;       // 地图中心
    Eigen::Vector3d map_size;         // 地图尺寸

    // 传感器信息
    std::string lidar_type;           // 激光雷达类型
    std::string frame_id;             // 坐标系ID

    // 精度信息
    double avg_point_density;         // 平均点密度
    double map_quality_score;         // 地图质量评分
};

// ============== 地图管理配置 ==============
struct MapManagerConfig {
    // 基础配置
    double resolution = 0.2;          // 地图分辨率 (体素大小)
    double max_range = 100.0;         // 最大距离范围

    // 子地图配置
    SubmapConfig submap_config;

    // 内存配置
    int max_global_points = 5000000;  // 全局地图最大点数
    int max_memory_mb = 2048;         // 最大内存占用 (MB)

    // 文件配置
    std::string map_path = "./map";   // 地图存储路径
    bool auto_save = true;            // 自动保存
    double auto_save_interval = 60.0; // 自动保存间隔 (秒)

    // 质量配置
    double min_point_quality = 0.3;   // 最小点质量
    bool enable_filtering = true;     // 启用滤波
    bool enable_outlier_removal = true; // 启用离群点去除
};

// ============== 地图管理器主类 ==============
class MapManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapManager();
    explicit MapManager(const MapManagerConfig& config);
    ~MapManager();

    // ============ 初始化 ============

    /**
     * @brief 初始化地图管理器
     */
    void initialize(const MapManagerConfig& config);

    /**
     * @brief 重置地图
     */
    void reset();

    /**
     * @brief 设置ikd-tree (用于状态估计)
     */
    void setIkdTree(IkdTree* tree);

    // ============ 点云添加 ============

    /**
     * @brief 添加点云到地图
     * @param cloud 点云数据
     * @param pose 位姿
     * @param frame_id 帧ID
     * @param timestamp 时间戳
     */
    void addPointCloud(const PointCloudPtr& cloud,
                       const SE3d& pose,
                       int frame_id,
                       double timestamp);

    /**
     * @brief 批量添加点云
     */
    void addPointClouds(const std::vector<PointCloudPtr>& clouds,
                        const std::vector<SE3d>& poses,
                        const std::vector<int>& frame_ids);

    // ============ 子地图管理 ============

    /**
     * @brief 更新子地图 (检查是否需要创建新子地图)
     */
    void updateSubmaps(const SE3d& current_pose);

    /**
     * @brief 获取当前活跃子地图
     */
    Submap::Ptr getActiveSubmap();

    /**
     * @brief 获取所有子地图
     */
    std::vector<Submap::Ptr> getAllSubmaps() const;

    /**
     * @brief 获取指定范围内的子地图
     */
    std::vector<Submap::Ptr> getSubmapsInRegion(const Eigen::Vector3d& center,
                                                 double radius);

    /**
     * @brief 合并子地图
     */
    Submap::Ptr mergeSubmaps(const std::vector<int>& submap_ids);

    /**
     * @brief 设置活跃子地图
     */
    void setActiveSubmap(int submap_id);

    // ============ 全局地图 ============

    /**
     * @brief 获取完整全局地图
     */
    PointCloudPtr getGlobalMap() const;

    /**
     * @brief 获取局部地图 (当前位置周围)
     */
    PointCloudPtr getLocalMap(const SE3d& pose, double radius);

    /**
     * @brief 获取滤波后的地图
     */
    PointCloudPtr getFilteredMap(double voxel_size);

    /**
     * @brief 获取地图元数据
     */
    MapMetadata getMetadata() const;

    // ============ 地图校正 ============

    /**
     * @brief 应用位姿校正 (闭环后)
     * @param corrected_poses 校正后的位姿
     */
    void applyPoseCorrection(const std::vector<SE3d>& corrected_poses);

    /**
     * @brief 应用子地图校正
     */
    void applySubmapCorrection(int submap_id, const SE3d& correction);

    // ============ 多会话支持 ============

    /**
     * @brief 开始新会话
     */
    std::string startNewSession(const std::string& session_name = "");

    /**
     * @brief 结束当前会话
     */
    void endCurrentSession();

    /**
     * @brief 获取当前会话ID
     */
    std::string getCurrentSessionId() const;

    /**
     * @brief 获取所有会话
     */
    std::vector<SessionInfo> getAllSessions() const;

    /**
     * @brief 合并会话
     */
    void mergeSessions(const std::vector<std::string>& session_ids);

    // ============ 持久化 ============

    /**
     * @brief 保存地图到文件
     * @param path 保存路径
     * @param format 格式: "pcd", "ply", "bin"
     * @return 是否成功
     */
    bool saveMap(const std::string& path,
                 const std::string& format = "pcd");

    /**
     * @brief 加载地图
     * @param path 地图文件路径
     * @param merge 是否合并到现有地图
     * @return 是否成功
     */
    bool loadMap(const std::string& path, bool merge = false);

    /**
     * @brief 保存子地图
     */
    bool saveSubmap(int submap_id, const std::string& path);

    /**
     * @brief 加载子地图
     */
    bool loadSubmap(const std::string& path);

    /**
     * @brief 导出整个地图项目
     */
    bool exportMapProject(const std::string& directory);

    /**
     * @brief 导入地图项目
     */
    bool importMapProject(const std::string& directory);

    // ============ 质量与统计 ============

    /**
     * @brief 计算地图质量评分
     */
    double computeMapQuality() const;

    /**
     * @brief 获取点密度分布
     */
    std::vector<double> computePointDensity(int grid_size = 10) const;

    /**
     * @brief 检测地图空洞
     */
    std::vector<Eigen::Vector3d> detectHoles(double threshold = 2.0) const;

    /**
     * @brief 获取统计信息
     */
    struct Statistics {
        int total_points;
        int total_frames;
        int total_submaps;
        int active_submaps;
        double memory_usage_mb;
        double map_coverage_area;
    };
    Statistics getStatistics() const;

    // ============ 可视化 ============

    /**
     * @brief 获取用于可视化的点云 (降采样)
     */
    PointCloudPtr getVisualizationCloud(double voxel_size = 0.5);

    /**
     * @brief 获取子地图边界可视化
     */
    std::vector<Eigen::Vector3d> getSubmapBoundaries() const;

private:
    // 私有成员函数
    void createNewSubmap(const SE3d& pose);
    void updateSubmapBounds(Submap::Ptr submap, const PointCloudPtr& cloud);
    PointCloudPtr transformPointCloud(const PointCloudPtr& cloud,
                                      const SE3d& pose);
    PointCloudPtr voxelFilter(const PointCloudPtr& cloud, double size);
    void updateGlobalMap(const PointCloudPtr& cloud);
    void optimizeMemory();
    void saveMetadata(const std::string& path);
    bool loadMetadata(const std::string& path);

    // 私有成员变量
    MapManagerConfig config_;

    // 全局地图
    PointCloudPtr global_map_;
    IkdTree* ikd_tree_;

    // 子地图
    std::vector<Submap::Ptr> submaps_;
    int current_submap_id_;
    std::unordered_map<int, Submap::Ptr> submap_map_;

    // 会话管理
    std::vector<SessionInfo> sessions_;
    std::string current_session_id_;
    std::unordered_map<std::string, SessionInfo> session_map_;

    // 元数据
    MapMetadata metadata_;

    // 统计
    int total_frame_count_;
    int total_point_count_;

    // 线程安全
    mutable std::mutex map_mutex_;
    mutable std::mutex submap_mutex_;
    mutable std::mutex session_mutex_;

    // 自动保存
    std::thread auto_save_thread_;
    std::atomic<bool> running_;
};

} // namespace fast_lio2_slam
```

---

#### 📦 Module: MapServer (地图服务模块)

**职责**:
- ROS2服务端点提供
- 地图保存/加载服务
- 地图拼接服务
- 地图查询服务

**接口定义**:

```cpp
/**
 * @file map_server.h
 * @brief 地图服务 - 提供ROS2服务接口
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "fast_lio2_slam/map_manager/map_manager.h"
#include <memory>
#include <string>

namespace fast_lio2_slam {

// ============== 服务消息类型 ==============
// 自定义服务需要定义在单独的srv文件中

/**
 * @brief 地图服务配置
 */
struct MapServerConfig {
    // 服务名称
    std::string save_map_service = "/save_map";
    std::string load_map_service = "/load_map";
    std::string clear_map_service = "/clear_map";
    std::string get_map_service = "/get_map";
    std::string merge_map_service = "/merge_map";
    std::string save_submap_service = "/save_submap";
    std::string get_metadata_service = "/get_map_metadata";

    // 发布话题
    std::string map_topic = "/global_map";
    std::string local_map_topic = "/local_map";
    std::string submap_marker_topic = "/submap_markers";

    // 发布频率
    double map_publish_rate = 0.5;     // Hz
    double marker_publish_rate = 1.0;  // Hz

    // 地图保存
    std::string default_save_path = "./map";
    std::string default_format = "pcd";
    bool auto_save_on_shutdown = true;
};

/**
 * @brief 地图服务类
 */
class MapServer {
public:
    MapServer(rclcpp::Node::SharedPtr node,
              MapManager::Ptr map_manager);
    ~MapServer();

    /**
     * @brief 初始化服务
     */
    void initialize(const MapServerConfig& config);

    /**
     * @brief 启动服务
     */
    void start();

    /**
     * @brief 停止服务
     */
    void stop();

    /**
     * @brief 发布地图
     */
    void publishMap();

    /**
     * @brief 发布局部地图
     */
    void publishLocalMap(const SE3d& pose, double radius);

    /**
     * @brief 发布子地图边界可视化
     */
    void publishSubmapMarkers();

private:
    // 服务回调
    void handleSaveMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleLoadMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleClearMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleGetMap(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // 发布定时器回调
    void mapPublishTimerCallback();
    void markerPublishTimerCallback();

    // 辅助函数
    void publishPointCloud(const PointCloudPtr& cloud,
                          const std::string& topic);

    // 成员变量
    rclcpp::Node::SharedPtr node_;
    MapManager::Ptr map_manager_;
    MapServerConfig config_;

    // ROS2 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_map_service_;

    // ROS2 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    // 定时器
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    rclcpp::TimerBase::SharedPtr marker_publish_timer_;
};

} // namespace fast_lio2_slam
```

---

### 7.3 地图持久化设计

#### 地图文件格式

```
地图存储结构:
map_project/
├── metadata.yaml           # 元数据文件
├── global_map.pcd          # 全局点云地图
├── global_map.kdtree       # ikd-tree序列化文件
├── pose_graph.g2o          # 位姿图文件
├── submaps/
│   ├── submap_000.pcd      # 子地图点云
│   ├── submap_000.meta     # 子地图元数据
│   ├── submap_001.pcd
│   ├── submap_001.meta
│   └── ...
├── sessions/
│   ├── session_001.yaml    # 会话信息
│   ├── session_002.yaml
│   └── ...
└── trajectory/
    ├── poses.txt           # 轨迹位姿
    ├── timestamps.txt      # 时间戳
    └── keyframes.txt       # 关键帧信息
```

#### metadata.yaml 格式

```yaml
map:
  name: "fast_lio2_map"
  version: "1.0"
  created_time: 1714032000.0    # Unix时间戳
  modified_time: 1714032600.0

  size:
    total_points: 1250000
    total_submaps: 15
    total_sessions: 2
    coverage_area: 1500.0       # 平方米

  bounds:
    min: [-50.0, -30.0, -2.0]
    max: [80.0, 40.0, 5.0]
    center: [15.0, 5.0, 1.5]

  sensor:
    lidar_type: "velodyne_vlp16"
    frame_id: "map"
    resolution: 0.2

  quality:
    avg_density: 833.3          # 点/平方米
    quality_score: 0.85

submaps:
  - id: 0
    file: "submaps/submap_000.pcd"
    center: [10.0, 5.0, 0.0]
    point_count: 80000
    timestamp_start: 1714032000.0
    timestamp_end: 1714032100.0

  - id: 1
    file: "submaps/submap_001.pcd"
    center: [25.0, 10.0, 0.0]
    point_count: 75000
    timestamp_start: 1714032100.0
    timestamp_end: 1714032200.0

sessions:
  - id: "session_001"
    name: "mapping_run_1"
    start_time: 1714032000.0
    end_time: 1714032300.0
    frame_count: 1500
    submap_ids: [0, 1, 2, 3, 4]

  - id: "session_002"
    name: "mapping_run_2"
    start_time: 1714032400.0
    end_time: 1714032600.0
    frame_count: 800
    submap_ids: [5, 6, 7, 8]
```

---

### 7.4 多会话建图流程

```
多会话建图流程:

1. 首次建图 (Session 1)
   ┌─────────────────────────────────────────────────────────┐
   │  开始新会话 → 创建session_001                            │
   │       │                                                 │
   │       ▼                                                 │
   │  采集数据 → IEKF状态估计 → 添加到地图                    │
   │       │                                                 │
   │       ▼                                                 │
   │  创建子地图 (submap_000, submap_001, ...)               │
   │       │                                                 │
   │       ▼                                                 │
   │  闭环检测 → 位姿图优化 → 地图校正                        │
   │       │                                                 │
   │       ▼                                                 │
   │  结束会话 → 保存地图项目                                 │
   └─────────────────────────────────────────────────────────┘

2. 后续建图 (Session 2)
   ┌─────────────────────────────────────────────────────────┐
   │  加载已有地图 → 读取metadata和子地图                      │
   │       │                                                 │
   │       ▼                                                 │
   │  开始新会话 → 创建session_002                            │
   │       │                                                 │
   │       ▼                                                 │
   │  重定位 (可选) → 匹配已有地图                             │
   │       │                                                 │
   │       ▼                                                 │
   │  继续建图 → 扩展地图 → 创建新子地图                       │
   │       │                                                 │
   │       ▼                                                 │
   │  会话间闭环检测 → 全局优化                                │
   │       │                                                 │
   │       ▼                                                 │
   │  保存更新后的地图                                        │
   └─────────────────────────────────────────────────────────┘

3. 地图合并
   ┌─────────────────────────────────────────────────────────┐
   │  加载多个地图项目                                        │
   │       │                                                 │
   │       ▼                                                 │
   │  特征匹配 → 计算相对变换                                 │
   │       │                                                 │
   │       ▼                                                 │
   │  合并位姿图 → 全局优化                                   │
   │       │                                                 │
   │       ▼                                                 │
   │  合并点云地图 → 去重滤波                                 │
   │       │                                                 │
   │       ▼                                                 │
   │  重新划分子地图 → 保存合并结果                           │
   └─────────────────────────────────────────────────────────┘
```

---

### 7.5 地图质量评估

```cpp
/**
 * @brief 地图质量评估器
 */
class MapQualityEvaluator {
public:
    /**
     * @brief 质量评估结果
     */
    struct QualityReport {
        double overall_score;           // 总体评分 [0, 1]
        double coverage_score;          // 覆盖度评分
        double density_score;           // 密度评分
        double consistency_score;       // 一致性评分
        double completeness_score;      // 完整性评分

        std::vector<Eigen::Vector3d> holes;          // 空洞位置
        std::vector<Eigen::Vector3d> low_density_areas;  // 低密度区域
        std::vector<double> density_distribution;    // 密度分布

        std::string summary;            // 摘要描述
    };

    /**
     * @brief 评估地图质量
     */
    QualityReport evaluate(const MapManager::Ptr& map_manager);

    /**
     * @brief 计算点密度分布
     */
    std::vector<double> computeDensityDistribution(
        const PointCloudPtr& cloud,
        double grid_size = 5.0);

    /**
     * @brief 检测地图空洞
     */
    std::vector<Eigen::Vector3d> detectHoles(
        const PointCloudPtr& cloud,
        double threshold = 2.0);

    /**
     * @brief 计算地图一致性
     */
    double computeConsistency(
        const std::vector<Submap::Ptr>& submaps,
        const std::vector<SE3d>& corrected_poses);

    /**
     * @brief 生成质量报告
     */
    std::string generateReport(const QualityReport& report);
};
```

---

### 7.6 ROS2服务接口定义

#### 自定义服务定义

```
# srv/SaveMap.srv
# 请求
string map_path          # 保存路径
string format           # 格式: "pcd", "ply", "bin"
bool save_submaps       # 是否保存子地图
bool save_trajectory    # 是否保存轨迹
---
# 响应
bool success            # 是否成功
string message          # 结果消息
int64 total_points      # 总点数
int64 total_submaps     # 总子地图数
float save_time         # 保存耗时(秒)


# srv/LoadMap.srv
# 请求
string map_path         # 地图路径
bool merge              # 是否合并到现有地图
bool load_submaps       # 是否加载子地图
---
# 响应
bool success            # 是否成功
string message          # 结果消息
int64 total_points      # 总点数
int64 total_submaps     # 总子地图数


# srv/GetMapMetadata.srv
# 请求
---
# 响应
string map_name
string version
float created_time
float modified_time
int64 total_points
int64 total_submaps
float[] map_bounds_min   # [x, y, z]
float[] map_bounds_max
float[] map_center
float quality_score
string[] session_ids


# srv/MergeMaps.srv
# 请求
string[] map_paths      # 要合并的地图路径
bool optimize_after_merge  # 合并后是否优化
---
# 响应
bool success
string message
int64 merged_points
int64 merged_submaps
float merge_time


# srv/GetLocalMap.srv
# 请求
float center_x          # 中心点X
float center_y          # 中心点Y
float center_z          # 中心点Z
float radius           # 半径
double resolution      # 分辨率(可选)
---
# 响应
sensor_msgs/PointCloud2 cloud  # 点云
int64 point_count
float coverage_area
```

---

### 7.7 配置参数扩展

在 `config/default.yaml` 中添加建图相关配置:

```yaml
## ==================== 地图管理参数 ====================
map:
  # 基础配置
  resolution: 0.2              # 地图分辨率 (体素大小, 米)
  max_range: 100.0             # 最大距离范围 (米)

  # 子地图配置
  submap:
    enable: true               # 启用子地图
    size: 50.0                # 子地图大小 (米)
    max_points: 50000          # 子地图最大点数
    overlap_ratio: 0.2         # 子地图重叠率

  # 内存配置
  memory:
    max_global_points: 5000000  # 全局地图最大点数
    max_memory_mb: 2048         # 最大内存占用 (MB)
    enable_optimization: true   # 启用内存优化

  # 文件配置
  storage:
    map_path: "./map"           # 地图存储路径
    auto_save: true             # 自动保存
    auto_save_interval: 60.0    # 自动保存间隔 (秒)
    format: "pcd"               # 默认保存格式
    compression: true           # 启用压缩

  # 多会话配置
  session:
    enable: true                # 启用多会话
    auto_session: true          # 自动创建会话
    merge_on_load: false        # 加载时自动合并

  # 质量配置
  quality:
    min_point_quality: 0.3      # 最小点质量
    enable_filtering: true       # 启用滤波
    enable_outlier_removal: true # 启用离群点去除
    outlier_threshold: 2.0      # 离群点阈值 (标准差)

  # 可视化配置
  visualization:
    publish_rate: 0.5          # 发布频率 (Hz)
    voxel_size: 0.5            # 可视化体素大小
    show_submap_bounds: true    # 显示子地图边界
    show_trajectory: true       # 显示轨迹

## ==================== 地图服务参数 ====================
map_server:
  # 服务名称
  services:
    save_map: "/save_map"
    load_map: "/load_map"
    clear_map: "/clear_map"
    get_map: "/get_map"
    get_metadata: "/get_map_metadata"
    merge_maps: "/merge_maps"

  # 话题名称
  topics:
    global_map: "/global_map"
    local_map: "/local_map"
    submap_markers: "/submap_markers"

  # 保存配置
  save:
    default_path: "./map"
    default_format: "pcd"
    include_trajectory: true
    include_submaps: true
```

---

### 7.8 建图数据流

```
建图数据流:

LiDAR数据 → PointCloudFilter → IEKF估计 → MapManager
                                              │
                              ┌───────────────┼───────────────┐
                              │               │               │
                              ▼               ▼               ▼
                         全局地图       子地图系统       ikd-tree
                              │               │               │
                              │               │               │
                              ▼               ▼               ▼
                         体素滤波       子地图切换       状态估计
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

---

### 7.9 建图性能优化

#### 7.9.1 内存优化策略

```cpp
/**
 * @brief 地图内存优化器
 */
class MapMemoryOptimizer {
public:
    /**
     * @brief 优化策略
     */
    struct Strategy {
        // 点云降采样
        double active_area_radius = 100.0;     // 活跃区域半径
        double downsample_voxel = 0.1;         // 降采样体素大小

        // 子地图卸载
        int max_active_submaps = 5;            // 最大活跃子地图数
        bool auto_unload = true;               // 自动卸载非活跃子地图

        // 点数限制
        int max_points_per_submap = 50000;    // 每子地图最大点数
        int max_total_points = 5000000;       // 总最大点数
    };

    /**
     * @brief 执行内存优化
     */
    void optimize(MapManager::Ptr map_manager, const SE3d& current_pose);

    /**
     * @brief 卸载远距离子地图
     */
    void unloadDistantSubmaps(const SE3d& pose, double distance);

    /**
     * @brief 激活附近子地图
     */
    void activateNearbySubmaps(const SE3d& pose, double distance);

    /**
     * @brief 获取内存使用情况
     */
    size_t getMemoryUsage() const;
};
```

#### 7.9.2 实时性优化

| 操作 | 优化策略 | 目标耗时 |
|-----|---------|---------|
| 点云添加 | 增量式ikd-tree插入 | < 5ms |
| 子地图切换 | 异步加载/卸载 | < 10ms |
| 地图滤波 | 分块并行处理 | < 50ms |
| 地图保存 | 后台线程异步保存 | 不阻塞主线程 |
| 地图发布 | 按需发布，降采样 | < 20ms |

---

### 7.10 建图功能接口汇总

| 模块 | 接口 | 功能 | 线程安全 |
|-----|-----|------|---------|
| MapManager | addPointCloud() | 添加点云 | ✓ |
| MapManager | getGlobalMap() | 获取全局地图 | ✓ |
| MapManager | getLocalMap() | 获取局部地图 | ✓ |
| MapManager | saveMap() | 保存地图 | ✓ |
| MapManager | loadMap() | 加载地图 | ✓ |
| MapManager | applyPoseCorrection() | 位姿校正 | ✓ |
| MapManager | startNewSession() | 开始新会话 | ✓ |
| MapManager | mergeSessions() | 合并会话 | ✓ |
| MapServer | saveMapService() | ROS2保存服务 | ✓ |
| MapServer | loadMapService() | ROS2加载服务 | ✓ |
| MapServer | publishMap() | 发布地图 | ✓ |

---

## 八、建图功能实施计划

### 8.1 Phase 1: 基础建图完善 (1周)

| 任务 | 描述 | 优先级 |
|-----|------|--------|
| 完善MapManager | 实现点云添加、子地图管理 | 高 |
| 体素滤波去重 | 实现增量式体素滤波 | 高 |
| 地图保存/加载 | PCD文件读写 | 高 |
| ROS2服务接口 | SaveMap/LoadMap服务 | 高 |

### 8.2 Phase 2: 子地图系统 (1周)

| 任务 | 描述 | 优先级 |
|-----|------|--------|
| 子地图创建与切换 | 基于位置的子地图管理 | 高 |
| 子地图合并 | 多子地图合并算法 | 中 |
| 内存优化 | 子地图加载/卸载策略 | 中 |
| 子地图可视化 | RViz边界显示 | 低 |

### 8.3 Phase 3: 多会话建图 (1周)

| 任务 | 描述 | 优先级 |
|-----|------|--------|
| 会话管理 | 会话创建、存储、恢复 | 高 |
| 地图拼接 | 多地图特征匹配与拼接 | 中 |
| 重定位支持 | 基于已有地图的初始化 | 中 |
| 地图项目导入导出 | 完整项目序列化 | 中 |

### 8.4 Phase 4: 质量与优化 (1周)

| 任务 | 描述 | 优先级 |
|-----|------|--------|
| 地图质量评估 | 覆盖度、密度、一致性评估 | 中 |
| 空洞检测 | 地图完整性分析 | 低 |
| 性能优化 | 内存优化、异步处理 | 高 |
| 文档完善 | API文档、使用说明 | 中 |

---

**文档版本**: v1.1
**更新日期**: 2026-04-24
**更新内容**: 添加建图功能增强设计，包括地图管理器增强、多会话支持、地图服务、持久化设计等