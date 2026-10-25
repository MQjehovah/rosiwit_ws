# rosiwit_slam_ros1 架构与代码走向

> 代码位置：`ros1_ws/src/rosiwit_slam_ros1`（ROS1 薄接口）+ `src/rosiwit_slam`（算法核心）
> 本文只描述**现状**；缺口分析见 `docs/dev` 的缺口报告与
> `docs/superpowers/plans/2026-10-26-nav-safety-recovery-hardening.md`。

---

## 1. 一句话结构

**一个进程、一个算法核心实例、一个对外接口**：
`main` 里只有 `SlamEngine`（核心持有者/输入泵）+ `SlamRos1Interface`（旧 xz_3d_slam 兼容层）。
接口层不做算法，只做「形状复刻 + 状态粘合 + 定时发布」；算法全部在 `src/rosiwit_slam`。

---

## 2. 分层架构

```mermaid
flowchart TB
  subgraph EXT["传感器 / 外部"]
    LIDAR["/vanjee_points722  3D LiDAR"]
    IMU["/imu"]
    ODOM["/odom  底盘轮速"]
    INIT["/initialpose"]
    SCAN["/scan  2D LiDAR"]
  end

  subgraph ENG["SlamEngine   src/engine/slam_engine"]
    CFG["装配置: 4 份旧 conf + 核心 yaml"]
    FACT["SlamFactory::create → 唯一核心实例"]
    Q["输入队列 ≤4096 + input_worker 线程"]
    PUMP["processCB  50Hz"]
    SNAP["输出快照 latestOutput  拉模型"]
  end

  subgraph CORE["算法核心   src/rosiwit_slam"]
    MAP["SlamPipeline.handleMapping<br/>前端 去畸变/EKF/关键帧<br/>后端 Ceres 位姿图<br/>回环 ScanContext<br/>地图 submaps/observations/global<br/>栅格 地面处理 + raycast"]
    LOC["LocalizationPipeline<br/>预测 IMotionPrior<br/>匹配 pp_icp/gicp<br/>门控 acceptMatch<br/>有界恢复 + 全局重定位<br/>2D 约束"]
    IO["io: xzmap_writer/reader<br/>map_adapter · grid_map_io"]
  end

  subgraph IFACE["Ros1SlamInterface   src/interface"]
    ASM["assembly  构造/模式同步/TF"]
    OUT["outputs  24 话题注册"]
    POSEP["pose_publisher  位姿/健康/2D 约束"]
    CLOUD["cloud_publisher  点云类话题"]
    SRV["services  17 个服务"]
    WARN["warning  告警码/频率窗口"]
  end

  LIDAR --> Q
  IMU --> Q
  ODOM --> Q
  INIT --> CFG
  CFG --> FACT
  FACT --> Q
  Q --> CORE
  SCAN --> LOC
  PUMP --> MAP
  CORE --> SNAP
  SNAP --> IFACE
  ASM --> OUT
  ASM --> SRV
  POSEP --> OUT
  CLOUD --> OUT
  IFACE --> PUBOUT["24 话题 · 17 服务 · TF map to odom"]
```

---

## 3. 线程与频率

```mermaid
flowchart LR
  SPIN["ros::spin 主线程<br/>imu/lidar 回调只入队<br/>17 个服务"]
  IW["input_worker 线程<br/>抽稀/去NaN/裁距/自过滤"]
  P50["processCB 50Hz<br/>建图前端 + tick hooks"]
  LW["定位内部 worker<br/>预测→匹配→门控→快照"]
  T01["接口定时器 0.1s<br/>点云/子图/当前帧"]
  T001["接口定时器 0.01s<br/>位姿 100Hz 上限"]
  TRT["接口定时器 1Hz<br/>实时 /map + 告警码"]
  BG["后台线程<br/>全局重定位候选<br/>fpfh 建库<br/>切图第二阶段 async"]

  SPIN --> IW
  IW --> LW
  IW --> P50
  P50 --> T01
  LW --> T001
  BG --> T001
  SPIN -.-> T01
  SPIN -.-> T001
  SPIN -.-> TRT
```

要点：**输入只入队**、**建图前端在 50 Hz 定时器里同步跑**、**定位在自己线程跑**、
**接口层靠定时器拉快照**（pull，不是回调推送）。

---

## 4. 数据流

### 4.1 建图（MAPPING）

```mermaid
sequenceDiagram
  autonumber
  participant S as 3D LiDAR
  participant E as SlamEngine
  participant W as input_worker
  participant C as SlamPipeline
  participant I as Interface 0.1s 定时器

  S->>E: lidarCB 只入队
  E->>W: 出队
  W->>W: ros2Pcl 抽稀/去NaN/裁距/逐点时间
  W->>W: applySelfFilter 车身包围盒
  W->>C: onLidar
  loop 每 20ms
    E->>C: tryPopAndProcess
    C->>C: 去畸变 + EKF + 关键帧
    C->>C: Ceres 位姿图 + 回环约束
    C->>C: submaps/observations/global + 栅格
    C-->>E: SlamOutput 写入快照
  end
  loop 每 100ms
    I->>E: latestOutput
    I-->>S: /lio_odom /lio_path /cloud_map /body_cloud /map
  end
```

### 4.2 定位（LOCALIZATION）

```mermaid
sequenceDiagram
  autonumber
  participant S as 3D LiDAR
  participant W as input_worker
  participant P as LocalizationPipeline
  participant I as Interface 0.01s 定时器
  participant N as nav / App

  S->>W: 点云
  W->>P: onLidar
  P->>P: 预测 IMotionPrior IMU + 轮速软门控
  P->>P: 匹配 pp_icp / gicp
  alt acceptMatch 通过
    P->>P: 位姿 + 协方差 + prior sigma
  else 门控失败
    P->>P: coast 计数 → tryBoundedRecovery → LOST
  end
  P-->>I: 快照
  I->>N: /lio_odom /curr_localization_pose 可 100Hz 重发
  I->>N: /slam/info 10Hz 节流
  I->>N: /localization/state 位姿+6x6+健康+map_id
  I->>N: TF map to odom LOST 时冻结最后有效值
```

---

## 5. 启动时序

```mermaid
sequenceDiagram
  autonumber
  participant M as main
  participant E as SlamEngine
  participant C as 核心
  participant I as Ros1SlamInterface

  M->>E: 构造
  E->>E: defaultXz3dPaths + 装 Xz3dConfig
  E->>C: SlamFactory::create + init
  E->>E: 订阅 imu/lidar/vo/absolute/initialpose
  E->>E: 启动 input_worker + 50Hz timer
  M->>I: 构造
  I->>I: initPub 24 话题
  I->>I: initTimerPub 4 个定时器
  I->>I: initSub scan/odom/lift* + Engine 钩子
  I->>I: initClients + initServer 17 服务
  I->>I: TF buffer/listener/broadcaster
  I->>C: setPipelineMode 按 slam_init_type
  I->>C: realtime_grid 数据源 lambda
  I-->>M: node up
```

---

## 6. 关键路径（函数级）

### 6.1 模式切换 `SlamMode ↔ core pipelineMode`

```mermaid
flowchart LR
  A["/slam/mode_switch 或 /set_slam_mode"] --> B["setModeFromConfig"]
  B --> C["applyModeToCore<br/>mapping / localization / idle"]
  C --> D{"reset_on_mode_switch?"}
  D -->|是 默认| E["resetCoreAndCaches<br/>重建核心 + 清实时栅格/常驻 /map"]
  D -->|否| F["只切模式"]
```

### 6.2 切图 `/switch_map`（两阶段，服务立即返回）

```mermaid
sequenceDiagram
  autonumber
  participant A as App
  participant I as Interface
  participant C as 核心

  A->>I: /switch_map
  I->>I: 同名图 + 已在定位 + mtime 未变 → 只 applyInitPose 返回
  I->>C: beginMapSwitch 停核心 / 清旧输出
  I->>I: generation++ 发 PGM 预览
  I-->>A: success = detect_ok or pgm_ok 立即返回
  I->>C: startMapLoadJob async
  C->>C: loadMapData xzmap
  C->>C: loadLocalizationMapData
  C->>C: applyInitPose 全零四元数 → 当作 0,0,0
  C-->>I: 之后 /lio_odom 从新图输出
  Note over C,I: generation 过期则丢弃；加载失败只打日志
```

### 6.3 存图 `/stop_map_operation`

```mermaid
flowchart TB
  A["/stop_map_operation"] --> Q{"质量门限<br/>free ≥ 1m² 且 unknown ≤ 60%"}
  Q -->|不达标| R["success=false + error=MAP_QUALITY_REJECTED"]
  Q -->|通过| X["exportMapData → saveMapData .xzmap"]
  X --> G["saveGridMap 写 .tmp.pgm/.tmp.yaml"]
  G --> RN["rename 成 .pgm/.yaml 原子写"]
  RN --> TH["缩略图 .jpg / thumb.png"]
  TH --> TG["callTagDetect → charging_pile_json"]
```

### 6.4 重定位与冷启动

```mermaid
flowchart LR
  A["/relocation"] --> B["applyInitPose + requestRelocalization"]
  B --> C["globalRelocWorker 后台候选"]
  C --> D["acceptMatch"]
  D -->|通过| E["位姿跳变 relocalize_count++"]
  D -->|失败| F["保持 LOST / 有界恢复"]
```

### 6.5 健康等级 `/localization/state`

```mermaid
stateDiagram-v2
  [*] --> INIT
  INIT --> TRACKING: 匹配通过
  TRACKING --> DEGRADED: matching 低于阈值 / 2D 一致性差
  DEGRADED --> TRACKING: 连续通过
  TRACKING --> LOST: coast 超限
  DEGRADED --> LOST: 有界恢复失败
  LOST --> TRACKING: 全局重定位成功
```

---

## 7. 对外接口（现状）

| 类别 | 内容 |
|---|---|
| 话题（24） | `/map` `/global_grid_map` `/grid_map` `/lio_odom` `/lio_path` `/curr_localization_pose` `/cloud_map` `/body_cloud` `/world_cloud` `/scan_f_map` `/curr_scan_point` `/curr_submap_point` `/origin_point` `/or_curr_scan_point` `/global_map` `/points_2d` `/mapping_path` `/slam/info` `/slam_calibration` `/slam_warning_code` `/loop_constraint` `/localization/state` |
| 服务（17） | `/switch_map` `/stop_map_operation` `/relocation` `/global_localization` `/slam/mode_switch` `/slam/dump_recorder` `/finish_task` `/start_task` `/localization/{relocalize,set_initial_pose,switch_map}` `/save_map` `/load_map` `/merge_map` `/save_grid_map` `/set_slam_mode` `/update_map_operation` |
| TF | `map→odom`（唯一发布者；LOST 时冻结最后有效值） |
| 订阅 | 核心：imu/lidar(+vo/absolute)/initialpose；接口：`scan`（频率统计）/`odom`（→核心）/`lift/state`/`lift_type` |

---

## 8. 设计上值得注意的 6 点

1. **pull 模型**：接口层用 `latestOutput()` 拉快照，引擎从不主动回调接口 →
   发布频率完全由 4 个定时器决定，与核心输出频率解耦（核心 10Hz、位姿可 100Hz 重发）。
2. **两种模式两条执行路径**：建图在 50 Hz 定时器里同步跑（会占用 spinner），
   定位在自己线程跑（避免重定位阻塞）。
3. **兼容层只做形状**：`SlamMode`(FREE/MAPPING/LOCALIZATION)、话题名、告警码、
   `map->odom` 都是照抄旧 `xz_3d_slam`，新能力（`/localization/*`）在同层旁挂。
4. **切图是两阶段 + generation**：服务立即返回、加载在后台；但 `success` 只表示
   "探测/PGM 可读"，xzmap 加载失败只能看日志。
5. **状态靠 `resetCoreAndCaches()` 清**：切建图会重建核心并清实时栅格，
   默认开启是"与旧实现一致"的选择，代价是切换瞬间建图停摆。
6. **协方差/健康是两套来源**：核心给 `localizationStatus()` + prior sigma，
   ROS 层又用 `loc_degrade_matching_` 二次分档，阈值不一致时会出现
   "核心认为不健康、对外还是 TRACKING"。

---

## 9. 文件索引

| 关注点 | 文件 |
|---|---|
| 入口 | `src/main.cpp` |
| 核心持有/输入泵/50Hz | `src/engine/slam_engine.{h,cpp}` |
| 构造顺序/模式同步/TF | `src/interface/assembly.cpp` |
| 话题注册/订阅/定时器 | `src/interface/outputs.cpp` |
| 位姿/健康/TF/2D 约束 | `src/interface/pose_publisher.cpp` |
| 点云类话题 | `src/interface/cloud_publisher.cpp` |
| 17 个服务 + 切图/存图 | `src/interface/services.cpp` |
| 告警/频率窗口 | `src/interface/warning.cpp` |
| 配置（4 份旧 conf + yaml） | `src/config/config.{h,cpp}`, `config/default.yaml`, `config/simulation.yaml` |
| 建图流水线 | `src/rosiwit_slam/src/slam_core/slam_pipeline.cpp` |
| 定位流水线 | `src/rosiwit_slam/src/algorithms/localization/localization_pipeline.cpp` |
| 匹配器 | `src/rosiwit_slam/src/algorithms/localization/matcher/*.cpp` |
| 2D 平面匹配 | `.../localization/planar_scan_matcher.cpp` |
| 地图 IO | `src/interface/io/{xzmap_writer,xzmap_reader,map_adapter}.cpp`, `.../grid_map/grid_map_io.cpp` |
| 核心单测（含未注册的 886 行） | `test/*.cpp` |
