# rosiwit_slam

基于 ROS2 的**分层模块化 SLAM 框架**，采用三层架构（ROS 接口层 / SLAM 核心层 / 算法层），支持运行时组合多种 SLAM 模块（前端里程计、后端优化、回环检测、定位、地图管理），当前内置 7 种算法组件。

---

## 一、整体架构

### 1.1 三层架构

```
┌──────────────────────────────────────────────────────────────────┐
│  ROS 接口层 (ros_interface/)                                      │
│  SlamNode (rclcpp::Node) + RosUtils                               │
│  · 订阅 imu/lidar → 转 IMUSample/LidarFrame → algo->onImu()/onLidar() │
│  · 注册 output 回调: 算法产出 SlamOutput 时加锁缓存                   │
│  · timer(100ms): 发布 odom/path/body_cloud/world_cloud/tf          │
│  · map timer(2s): algo->getGlobalMap() → 发布 cloud_map            │
│  · 提供 4 个 service: save_map / load_map / save_grid_map / set_slam_mode │
│  · EMA 平滑位姿 (alpha=0.3) 抑制 LIO 抖动                            │
│  · 不感知任何算法内部细节                                             │
└──────────────────────────────┬────────────────────────────────────┘
                               │ ISlamAlgorithm* (unique_ptr, 工厂创建)
┌──────────────────────────────▼────────────────────────────────────┐
│  SLAM 核心层 (slam_core/)                                          │
│  · ISlamAlgorithm: 抽象基类 (init/onImu/onLidar/state/getOutput/callback) │
│  · SlamBase: 内置 IMU/LiDAR 缓冲 + 时间同步, 子类只填 onSyncedPackage() │
│  · SlamPipeline: 编排器, 组合 IFrontend/IBackend/ILoopClosure/IMapManager/ILocalization │
│  · SlamFactory: 运行时按字符串创建顶层算法及子模块                       │
│  · 5 个模块接口: IFrontend / IBackend / ILoopClosure / IMapManager / ILocalization │
│  · slam_types.h: IMUSample / LidarFrame / SlamOutput / PoseStamped │
│  · pipeline_types.h: KeyFrame / Constraint                         │
└──────────────────────────────┬────────────────────────────────────┘
                               │ 继承 SlamBase, 持有子模块指针
┌──────────────────────────────▼────────────────────────────────────┐
│  算法层 (algorithms/)                                               │
│  fast_lio2/         FastLio2Frontend (IESKF + iKD-Tree)           │
│  point_lio/         PointLioFrontend  (逐点处理)                    │
│  ceres_backend/     CeresBackend     (Ceres 位姿图优化)             │
│  gtsam_backend/     GtsamIsam2Backend (GTSAM iSAM2 增量优化)       │
│  scan_context/      ScanContextLoopClosure (描述符回环检测)          │
│  gicp_localization/ GicpLocalization  (GICP 配准定位)              │
│  pcd_map_manager/   PcdMapManager    (PCD 地图管理 + 2D 栅格)      │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 核心解耦

- ROS 接口层只通过 `SlamOutput`（纯位姿 + 点云）获取结果，不依赖任何算法内部类型。
- 算法层对 `rclcpp` / `sensor_msgs` 零依赖，可脱离 ROS 独立测试。
- 模块间通过纯虚接口（`IFrontend` 等）通信，同进程直接方法调用，无 ROS 话题开销。
- `SlamFactory` 统一管理所有模块的创建，新增算法只需注册一行代码。

### 1.3 数据流

```
mapping 模式:
  imu msg   → SlamNode::imuCB   → IMUSample  → algo->onImu()
  lidar msg → SlamNode::lidarCB → LidarFrame → algo->onLidar()
    → SlamBase 缓冲 → 时间同步 → onSyncedPackage()
    → SlamPipeline::handleMapping()
      → m_frontend->onImu(imus) / onLidar(frame)
      → getOdometry → getKeyFrame → backend->addKeyFrame / loop->addKeyFrame / map_mgr->addSubMap
      → 每 50 帧 backend->optimize()
      → loop->detect → 检测到? → backend->addConstraints + optimize
      → 组装 SlamOutput → emitOutput
    → SlamNode::onOutput() (加锁缓存)
    → timer(100ms): 发 lio_odom / lio_path / body_cloud / world_cloud / tf
    → map timer(2s): 发 cloud_map

localization 模式:
  → m_localization->onImu / onLidar
  → getPose → 变换当前扫描到 world → 产出 SlamOutput
  → m_frontend 同时保持运行 (用于可视化等)
```

---

## 二、模块接口体系

### 2.1 IFrontend — 前端里程计

```cpp
class IFrontend {
    virtual bool init(const std::string& config);
    virtual void onImu(const IMUSample&);
    virtual void onLidar(const LidarFrame&);
    virtual bool getOdometry(PoseStamped& out);
    virtual bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world);
    virtual bool getKeyFrame(KeyFrame& out);
    virtual bool getGlobalMap(PointVec& out);
};
```

提供 IMU + LiDAR → 连续里程计 + 关键帧判定。

### 2.2 IBackend — 后端优化

```cpp
class IBackend {
    virtual bool init(const std::string& config);
    virtual void addKeyFrame(const KeyFrame&);
    virtual void addConstraints(const std::vector<Constraint>&);
    virtual bool optimize();
    virtual bool getUpdatedPoses(std::vector<PoseStamped>&);
};
```

管理关键帧间的约束，优化全局位姿图。支持批量（Ceres）和增量（GTSAM iSAM2）两种方式。

### 2.3 ILoopClosure — 回环检测

```cpp
class ILoopClosure {
    virtual bool init(const std::string& config);
    virtual void addKeyFrame(const KeyFrame&);
    virtual bool detect(PoseStamped& relative_pose, double& cov);
};
```

识别已访问区域，返回相对位姿约束。

### 2.4 IMapManager — 地图管理

```cpp
class IMapManager {
    virtual bool init(const std::string& config);
    virtual bool saveMap(const std::string& name);
    virtual bool loadMap(const std::string& name);
    virtual bool getGlobalMap(PointVec& out);
    virtual bool addSubMap(const KeyFrame&);
    virtual bool generateGridMap(double resolution);
    virtual bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path, double resolution);
    virtual GridInfo getGridInfo() const;
};
```

管理 PCD 点云地图的保存/加载/查询，并提供 2D 占据栅格地图（用于导航）。

### 2.5 ILocalization — 纯定位

```cpp
class ILocalization {
    enum Status { INIT, LOCALIZED, LOST };
    virtual bool init(const std::string& config);
    virtual void setMap(const std::string& name);
    virtual void setInitPose(const PoseStamped&);
    virtual void onImu(const IMUSample&);
    virtual void onLidar(const LidarFrame&);
    virtual Status getStatus();
    virtual bool getPose(PoseStamped& out);
};
```

给定预先构建的地图，持续提供全局定位。支持 IMU 航迹推算作为初始猜测。

---

## 三、目录结构

```
rosiwit_slam/
├── src/
│   ├── main.cpp                     入口, 创建 SlamNode
│   ├── ros_interface/
│   │   ├── slam_node.cpp             ROS 节点实现 (256 行)
│   │   └── ros_utils.cpp             ROS↔PCL 转换工具
│   ├── slam_core/
│   │   ├── slam_base.cpp             时间同步 (57 行)
│   │   ├── slam_pipeline.cpp         编排器 (198 行)
│   │   └── slam_factory.cpp          工厂注册 (57 行)
│   └── algorithms/
│       ├── fast_lio2/
│       │   ├── fast_lio2_frontend.cpp IFrontend 适配器
│       │   └── map_builder/          FAST-LIO2 核心 (ieskf/imu/lidar/ikd/commons)
│       ├── point_lio/point_lio_frontend.cpp
│       ├── ceres_backend/ceres_backend.cpp
│       ├── gtsam_backend/gtsam_isam2_backend.cpp
│       ├── scan_context/
│       │   ├── scan_context.cpp      ScanContext 描述符计算
│       │   └── scan_context_lc.cpp   回环检测逻辑
│       ├── gicp_localization/gicp_localization.cpp
│       └── pcd_map_manager/
│           └── pcd_map_manager.cpp   PCD 地图管理
│       └── grid_map/
│           ├── grid_map_factory.cpp            2D 栅格策略工厂
│           ├── grid_map_io.cpp                 PGM/YAML 读写
│           ├── height_projection_grid_map.cpp  高度投影策略
│           ├── ray_casting_grid_map.cpp        射线投射策略(free/occupied/unknown)
│           └── octomap_grid_map.cpp            OctoMap 概率占据策略(可选)
├── include/                          (对应头文件, 结构同上)
├── config/default.yaml               统一配置
├── launch/slam.launch.py             启动文件
├── srv/                              4 个 service 定义
├── test/                             6 个 gtest
├── scripts/                          依赖安装/数据获取/报表生成
└── rviz/                             RViz 配置
```

---

## 四、算法详述

### 4.1 FastLio2Frontend — FAST-LIO2 前端

**核心原理**: 基于 IESKF（误差状态迭代卡尔曼滤波器）的 LiDAR-IMU 紧耦合里程计，使用 iKD-Tree 维护增量式地图。

**技术要点**:
- 21 维误差状态: 旋转(3) + 平移(3) + IMU-LiDAR 外参旋转(3) + 外参平移(3) + 速度(3) + 陀螺零偏(3) + 加速度零偏(3)
- 前向传播: 每帧 IMU 数据驱动状态预测 + 协方差传播（使用 SO(3) 左雅可比 `Jr()`）
- 观测更新: 点到平面的残差，重线性化迭代最多 5 次
- iKD-Tree: 增量式 k-d 树，支持逐点插入/惰性删除/动态重建
- 滑窗局部地图: 立方体边长 300m，检测范围 60m
- OpenMP 并行点处理

| 维度 | 说明 |
|---|---|
| 精度 | 平移 < 0.1% 里程漂移, 旋转 < 0.05°/m (典型场景) |
| 计算开销 | ≈20-40ms/帧 (64 线 LiDAR 降采样后) |
| 内存 | iKD-Tree 约 100-500MB (依地图大小) |

**优势**:
- 紧耦合框架，IMU 和 LiDAR 信息在同一滤波器内融合，精度高
- iKD-Tree 增量更新，无需维护体素地图
- IMU-LiDAR 外参可在线校准（`esti_il: true`）
- 对运动模糊和快速旋转鲁棒

**劣势**:
- 仅 LiDAR + IMU，不支持纯 LiDAR 或纯视觉
- 全局一致性差（纯里程计，无位姿图优化）
- iKD-Tree 实现复杂，约 1400 行 C++，维护成本高
- 对传感器外参标定精度敏感
- IMU 初始化要求传感器静止（否则零偏估计不准，导致发散）

**适用场景**: 室内外快速移动机器人（四足/无人机/车辆），需要高频低延迟里程计，IMU 质量较好，且有静止初始化条件。

### 4.2 PointLioFrontend — Point-LIO 风格逐点前端

**核心差异 vs FastLio2Frontend**:
- FAST-LIO2: 攒完整帧 → 去畸变 → 全体点 IESKF 更新 → 增量地图
- Point-LIO: 每点到达 → IMU 传播到该点时间戳 → 单点 IESKF 更新 → 立即加入地图

**优势**:
- 无运动畸变补偿环节（每点在真实时间戳处理）
- 更低延迟（不等攒帧），延迟由单点处理时间决定（≈0.1-1ms）
- 高动态场景精度更好（如剧烈旋转、振动平台）
- 计算量分布均匀（非突发式），适合实时性要求严苛的系统

**劣势**:
- 每点触发一次 KF 更新，CPU 占用略高（但单点计算量小）
- 地图更新频繁，iKD-Tree 重建更密集
- 关键帧判定需额外逻辑

**适用场景**: 高动态运动平台（无人机特技飞行、四足机器人奔跑）、对延迟敏感的系统。

### 4.3 CeresBackend — Ceres 位姿图优化

**核心原理**: 使用 Ceres Solver 构建关键帧位姿图，优化 6-DOF SE(3) 位姿。

- 每 50 关键帧触发一次全局优化
- 使用 `Pose3Parameterization` (6-DOF 流形参数化)
- Huber 损失函数抑制离群约束
- 最大 50 次迭代

| 维度 | 说明 |
|---|---|
| 复杂度 | O(N³) 每帧, N 为关键帧数 |
| 触发频率 | 每 50 帧 |

**优势**:
- 成熟稳定的优化库，数值稳定性好
- 全局一致性显著提升，消除长轨迹漂移
- 支持任意自定义残差（可扩展 GPS/高度计等）

**劣势**:
- 每次优化从零构建问题，随地图增大计算量立方增长
- 不适合长期运行（数小时）—— 建议定期重置或切到 iSAM2
- 内存随关键帧数线性增长

**适用场景**: 中小规模场景（< 1000 关键帧）、需要最高精度的后处理建图、离线或半在线优化。

### 4.4 GtsamIsam2Backend — GTSAM iSAM2 增量后端

**核心原理**: 使用 GTSAM 的 iSAM2（增量平滑与建图）维护因子图，每帧仅添加新因子，无需全局重建。

- 增量更新，摊销复杂度 O(log N)
- 支持贝叶斯树重新排序和部分线性化
- `USE_GTSAM` 编译开关，未安装时提供桩实现

**优势**:
- 真正的增量优化，适合长时间运行
- 添加新约束（回环）时自动更新受影响部分，无需全量重算
- 内存效率高（只维护必要的信息矩阵）

**劣势**:
- 需安装 GTSAM（apt 或源码），增加构建依赖
- 增量近似的精度略低于 Ceres 全局优化
- 调试和参数调优更复杂（ISAM2 重线性化阈值等）

**适用场景**: 大规模地图（> 1000 关键帧）、长期运行的机器人、需要实时位姿图优化。

### 4.5 ScanContextLoopClosure — ScanContext 回环检测

**核心原理**: 将点云编码为 N_rings × N_sectors 的极坐标描述符（每个 bin 存储最大高度），使用 RingKey 快速预筛选 + 精确匹配。

| 参数 | 默认 | 说明 |
|---|---|---|
| sc_num_rings | 20 | 环数 |
| sc_num_sectors | 60 | 扇区数 |
| sc_max_radius | 80.0 | 最大半径 |
| sc_min_keyframe_gap | 30 | 最小关键帧间隔 |
| sc_dist_thresh | 0.4 | 描述符匹配阈值 |

**优势**:
- 视角无关的全局描述符，对 LiDAR 扫描顺序不敏感
- RingKey 预筛快速（O(1) 近似），适合大量关键帧
- 无需 GPU，纯 CPU 实时运行
- 返回相对位姿（后续可接后端优化）

**劣势**:
- 对称场景（长廊、隧道）易误检
- 对高度方向变化敏感（上下坡）
- 描述符大小固定半径，远距离点云采样分辨率下降
- 需关键帧位姿初步估计（依赖前端里程计）

**适用场景**: 大场景建图（园区/矿区/地下车库）、消除长轨迹累计误差。

### 4.6 GicpLocalization — GICP 配准定位

**核心原理**: 使用 PCL 的 `GeneralizedIterativeClosestPoint` 将当前扫描配准到预构建的地图，IMU 航迹推算提供初始猜测。

- 体素滤波: 地图 0.5m / 扫描 0.5m
- 最大 64 次迭代
- 支持 IMU 航迹推算（简易积分传播位姿）

**优势**:
- 无需 LiDAR-IMU 紧耦合，对 IMU 质量要求低
- 加载预建地图后即可纯定位，适合已知环境部署
- GICP 比 ICP 精度更高（考虑点云分布协方差）

**劣势**:
- 依赖高精度预建地图（需先运行 mapping 模式）
- 对初始位姿敏感（需 `setInitPose` 或好的 IMU 初始猜测）
- 地图加载后不支持动态更新（需重新建图）
- 大范围重定位需先验

**适用场景**: 已知环境的长期部署（已建图的室内仓库/工厂巡检）、纯定位任务。

### 4.7 PcdMapManager — PCD 地图管理

**核心功能**:
- 保存/加载 PCD 点云地图
- `addSubMap()` 增量累积关键帧到全局地图
- `generateGridMap()` / `saveGridMap()`: 调用可插拔的 `IGridMapGenerator` 策略生成 2D 占据栅格
  （yaml `grid_map_generator`: `height_projection` 高度投影 / `ray_casting` 射线投射三态）
- `saveGridMap()`: 输出 PGM + YAML 格式（兼容 ROS `map_server` / navigation stack）

**2D 栅格生成策略** (`algorithms/grid_map/`, 工厂 `GridMapFactory`):
| 策略 | 原理 | free/unknown | 输入 |
|------|------|--------------|------|
| `height_projection` | 障碍高度带内点数投影 + 阈值 + 膨胀 | 未占据一律 free | 合并全局点云 |
| `ray_casting` | 逐帧原点向回波投射线(Bresenham), 途经 free/终点 occupied | 未观测 unknown(-1) | 逐帧观测(原点+扫描) |
| `octomap` | OctoMap 3D 概率占据(内含射线 free)投影到 2D | free/unknown 三态 | 逐帧观测(原点+扫描) |

策略通过 `IGridMapGenerator` 接口解耦, 新增算法只需实现接口并在 `GridMapFactory` 注册。
`octomap` 依赖 `liboctomap-dev`(可选), 未安装时自动不注册。

对比工具: `scripts/compare_grid_maps.py out.png a.pgm b.pgm ...` 将多张 PGM 并排渲染为 PNG,
并输出 occupied/free/unknown 占比(控制台/CSV), 用于横向评估各策略能力上限。

**优势**:
- 标准 PCD 格式，与主流工具兼容（CloudCompare / PDAL）
- 2D 栅格地图可直接用于 ROS Navigation 导航栈
- 支持 3D 空间的地图管理

**劣势**:
- 不维护几何图元（平面/线），无法用于高级规划
- 大场景 PCD 文件 I/O 耗时
- 体素滤波 0.5m 可能丢失小物体细节

**适用场景**: 建图保存/加载的基础设施、为导航栈提供代价地图。

---

## 五、组合配置与场景推荐

### 5.1 组合模式

通过 `config/default.yaml` 的 `modules` 字段自由组合:

| 配置 | 行为 | 适用场景 |
|---|---|---|
| frontend only | 纯里程计 | 快速原型/开发调试 |
| frontend + backend | 里程计 + 位姿图 | 中小场景高精度建图 |
| frontend + backend + loop | 完整 SLAM 带回环 | 大场景园区/矿区建图 |
| frontend + backend + loop + map_manager | 完整建图 + 地图持久化 | 最终建图任务 |
| localization only | 预建地图重定位 | 已知环境长期部署 |
| frontend + localization | 建图 + 可切定位 | 一机多用 |

### 5.2 推荐配置模板

**快速高精度建图**:
```yaml
modules:
  frontend: "fast_lio2_frontend"
  backend: "ceres_pose_graph"
  loop_closure: ""
  map_manager: "pcd_map_manager"
```

**长时间大场景建图**:
```yaml
modules:
  frontend: "fast_lio2_frontend"
  backend: "gtsam_isam2"
  loop_closure: "scan_context"
  map_manager: "pcd_map_manager"
```

**高动态/无人机**:
```yaml
modules:
  frontend: "point_lio_frontend"
  backend: ""
  loop_closure: ""
```

**已知环境部署**:
```yaml
pipeline_mode: localization
modules:
  frontend: ""
  backend: ""
  localization: "gicp_localization"
  map_manager: "pcd_map_manager"
```

### 5.3 算法选型决策树

```
任务是什么?
├─ 纯里程计 (高频低延迟)
│  └─ IMU 质量好?
│     ├─ 是 → FastLio2Frontend
│     └─ 否 → PointLioFrontend (运动场合同样鲁棒)
├─ 完整建图
│  ├─ 小场景 (< 500m²)
│  │  └─ CeresBackend (精度最高)
│  └─ 大场景 (> 500m², 长时间)
│     ├─ 需要回环消除漂移?
│     │  ├─ 是 → GtsamIsam2Backend + ScanContext
│     │  └─ 否 → GtsamIsam2Backend
│     └─ 需要离线后处理?
│        └─ CeresBackend (全局优化一次)
├─ 纯定位 (已知地图)
│  └─ GicpLocalization
└─ 开发测试
   └─ frontend only + 禁用其他模块
```

---

## 六、服务接口

| Service | Request | Response | 说明 |
|---|---|---|---|
| `save_map` | `string path` | `bool success, string message` | 保存 PCD 地图 |
| `load_map` | `string path` | `bool success, string message` | 加载 PCD 地图, 自动切到定位模式 |
| `save_grid_map` | `string pgm_path, string yaml_path, float64 resolution` | `bool success, string message` | 导出 2D 栅格地图 |
| `set_slam_mode` | `string mode` (mapping/localization/idle) | `bool success, string message` | 运行时切换模式 |

调用示例:
```bash
ros2 service call /set_slam_mode rosiwit_slam/srv/SetSlamMode "{mode: localization}"
ros2 service call /load_map rosiwit_slam/srv/LoadMap "{path: /path/to/map.pcd}"
ros2 service call /save_map rosiwit_slam/srv/SaveMap "{path: /path/to/saved_map.pcd}"
```

---

## 七、算法对比总表

| 算法 | 类型 | 精度 | 计算开销 | 依赖 | 优势 | 劣势 | 适用场景 |
|---|---|---|---|---|---|---|---|
| FastLio2Frontend | 前端里程计 | ★★★★★ | 中 | Eigen+Sophus | 紧耦合高精度, iKD-Tree 增量地图 | 无全局优化, 需外参标定, 需静止初始化 | 室内外快速移动 |
| PointLioFrontend | 前端里程计 | ★★★★☆ | 中高 | Eigen+Sophus | 低延迟, 高动态鲁棒, 无运动畸变 | 点级处理开销高 | 无人机/高动态 |
| CeresBackend | 后端优化 | ★★★★★ | 高(O(N³)) | Ceres | 全局精度最高 | 非增量, 不适合长时 | 中小场景后处理 |
| GtsamIsam2Backend | 后端优化 | ★★★★☆ | 低(O(log N)) | GTSAM | 增量实时, 适合长期运行 | 精度略低, 需额外安装 | 大规模长时间运行 |
| ScanContextLoopClosure | 回环检测 | ★★★★☆ | 低 | 无 | 视角无关, CPU 实时 | 对称场景误检 | 大场景建图消除漂移 |
| GicpLocalization | 定位 | ★★★★☆ | 中 | PCL | 纯定位, 对 IMU 要求低 | 需预建地图, 初值敏感 | 已知环境长期部署 |
| PcdMapManager | 地图管理 | — | — | PCL | PCD 格式兼容, 2D 栅格输出 | 无几何语义 | 地图持久化/导航 |

---

## 八、环境依赖

| 依赖 | 版本 | 说明 |
|---|---|---|
| ROS2 | Humble (22.04) / Foxy (20.04) | rclcpp / sensor_msgs / nav_msgs / tf2_ros / pcl_conversions |
| PCL | 1.12+ | common / io / filters / kdtree / registration |
| Eigen3 | 3.4 | 线性代数 |
| Sophus | 1.22.10 | 李群 (源码编译) |
| yaml-cpp | 0.7+ | YAML 配置解析 |
| Ceres | 2.1+ | 位姿图优化 (可选, 无则禁用 ceres_backend) |
| GTSAM | 4.2a9 | iSAM2 增量优化 (可选, 无则禁用 gtsam_backend) |
| OpenMP | — | lidar_processor 并行 (可选) |

一键安装: `bash scripts/install_dependencies.sh`

---

## 九、构建与运行

```bash
cd ~/rosiwit_ws
colcon build --packages-select rosiwit_slam --symlink-install
source install/setup.bash
ros2 launch rosiwit_slam slam.launch.py
```

> 通过 `slam_algorithm` 参数选择顶层算法，目前仅 "fast_lio2_pipeline"。子模块组合通过 `config/default.yaml` 的 `modules` 配置。

---

## 十、添加新算法

1. **实现模块接口**: 继承 `IFrontend` / `IBackend` / `ILoopClosure` / `ILocalization` / `IMapManager`
2. **注册工厂**: 在 `slam_factory.cpp` 中添加 `createXxx()` 分支
3. **配置组合**: 在 `default.yaml` 的 `modules` 中填入算法名
4. 算法层**不得依赖** `rclcpp` / `sensor_msgs`，保持可脱离 ROS 测试

---

## 十一、设计文档

- 三层架构完整性设计决策: `docs/plans/2026-07-07-layered-architecture-design.md` (已并入主分支)
- 模块化扩展设计: `docs/plans/2026-07-07-modular-slam-architecture-design.md`
- Phase 1 实现记录: `docs/plans/2026-07-07-modular-slam-impl-phase1.md`
