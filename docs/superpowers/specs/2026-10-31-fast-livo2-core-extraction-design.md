# FAST-LIVO2 并入 SLAM 核心（完整 LIVO）设计

> 日期：2026-10-31
> 状态：待评审
> 关联：`docs/dev/fast-livo2-ros2.md`、`ros2_ws/src/fast_livo2`（独立包，作为参考实现与验证基准）

## 1. 意图（为什么做）

`rosiwit_slam` 现在是一个 ROS 无关的分层核心：`ISlamAlgorithm` / `SlamPipeline` 编排，
前端（`IFrontend`）负责 LIO 位姿与地图。当前只有 LiDAR-惯性前端（`fast_lio2_frontend`、
`point_lio_frontend`），视觉只有"松耦合"入口（外部 VO 结果注入）。

目标是把 **FAST-LIVO2（紧耦合 LiDAR-惯性-视觉）** 作为**核心内的一个前端**接进来，
让 `rosiwit_slam` 在不依赖任何 ROS 包的情况下，直接吃 `IMU + 点云 + 图像`，输出位姿、
关键帧、地图（含彩色点云），并保持现有 ROS1/ROS2 接口层仍然很薄。

## 2. 目标 / 非目标

### 目标

1. 新增核心前端 `fast_livo2_frontend`（`SlamFactory::createFrontend` 可选），
   实现完整 LIVO（LIO + VIO 紧耦合），不依赖 ROS、不依赖 `livox_ros_driver2`。
2. 核心新增图像输入通道：`ImageFrame`（`cv::Mat` + 时间戳）与相机内参配置，
   `IFrontend::onImage()` 默认空实现，不影响现有前端。
3. 保持现有 `fast_lio2_frontend` / `point_lio_frontend` 行为不变，可由 yaml 切换。
4. ROS2 接口层支持订阅图像与相机参数并把数据喂给核心；验证精度与独立包一致。
5. 彩色点云地图可通过 `SlamOutput` 输出（可选字段，旧接口忽略即可）。

### 非目标

- 不改后端（Ceres/GTSAM）、回环（ScanContext）、定位（GICP 等）模块。
- 不支持多相机、卷帘快门、相机-IMU 在线标定（沿用配置外参）。
- 不在核心内做 ROS 消息类型（`sensor_msgs/*`、`livox_ros_driver2/*`）的解析。
- 不改变 `fast_lio2_frontend` 的精度/行为，不做性能重构。
- 本轮不做 ROS1 接口的图像接入（见 6.6，作为后续阶段，接口预留）。

### 反目标（技术上跑通但算失败）

- 核心库出现任何 `rclcpp`/`ros/ros.h` 依赖，或必须链接 ROS 才能 standalone 构建。
- 为了接 LIVO 而复制一份 `fast_lio2` 的管线逻辑，导致两套模式切换/关键帧语义分叉。
- 精度明显退化：NTU VIRAL `eee_03` 的 ATE 比独立包（0.257 m）差 > 0.05 m。
- 强制所有用户安装 OpenCV 之外的视觉依赖，或强制 Livox 驱动。

## 3. 现状与约束

### 现状（核心）

- 输入类型：`IMUSample{acc,gyro,time}`、`LidarFrame{CloudType::Ptr, start_time, end_time}`；
  `CloudType = pcl::PointCloud<pcl::PointXYZINormal>`，逐点时间偏移放在 `curvature`（ms）。
- `IFrontend`：`init(config_path)` / `onImu` / `onLidar` / `getOdometry` / `getClouds` /
  `getKeyFrame` / `getGlobalMap`。
- `SlamPipeline` 负责 IMU/点云打包、模式（mapping/localization/idle）、关键帧、后端/回环/地图。
- 配置：`config/default.yaml`，`global.modules.frontend` 选择前端；
  `mapping.frontend.*` 是该前端的参数段。
- 核心 CMake 明确"ROS 无关"，`file(GLOB_RECURSE src/*.cpp)` + standalone 测试（GTest）。
- 已依赖：Eigen、PCL、OpenCV、Sophus（可选）、Ceres/GTSAM/octomap（可选）。

### 现状（FAST-LIVO2 独立包）

- `LIVMapper`（约 1.5k 行）里混着 ROS：`rclcpp::Node` 参数、发布器、`image_transport`、
  `cv_bridge`、TF；`Preprocess` 支持 `CustomMsg` 与多种 `PointCloud2`；
  `VIOManager`（~1.9k 行）与 `voxel_map`（~1k 行）基本是纯算法；
  `vikit`（GPLv3）提供相机模型与视觉工具。
- 已在 NTU VIRAL `eee_03`（LIVO）上验证：ATE 0.257 m；Retail_Street 彩色建图正常。

### 约束

- 核心必须继续 **ROS 无关**，能在 Ubuntu 18.04（ROS1 Melodic，GCC 7）+ 22.04（Humble）编译。
- C++17、OpenCV（Melodic 可能是 3.2，需避免仅 4.x 的 API）、PCL 1.8+、Sophus。
- `vikit` 为 GPLv3；并入后核心二进制对外分发按 GPLv3 处理（内部使用不受影响）。
- 机器人（aarch64）算力有限：保留 OpenMP，避免新增重依赖。
- 图像与点云是异步到达的，前端内部必须自己缓存/对齐（沿用 FAST-LIVO2 的 `sync_packages` 思路）。

## 4. 设计

### 4.1 核心类型扩展（`slam_core/slam_types.h`）

```cpp
struct ImageFrame {
    cv::Mat image;        // BGR8 或 MONO8，已是可处理格式
    double time = -1.0;   // 与 IMU/LiDAR 同一时间基准（秒）
    std::string frame_id;
};

struct CameraIntrinsics {
    std::string model = "Pinhole";  // Pinhole | EquidistantCamera | ...
    int width = 0, height = 0;
    double scale = 1.0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double d0 = 0, d1 = 0, d2 = 0, d3 = 0;  // 依模型解释
};
```

- `SlamOutput` 增加可选彩色地图：
  ```cpp
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_world_cloud;  // 可为空
  ```
- `IFrontend` 增加：
  ```cpp
  virtual void onImage(const ImageFrame& /*f*/) {}
  virtual bool getRGBCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& /*out*/) { return false; }
  ```
  默认空实现 → `fast_lio2` / `point_lio` 零改动。
- `ISlamAlgorithm` 增加 `onImage()`（默认空实现），`SlamPipeline` 转发给前端；
  ROS 接口层统一通过 `m_algo->onImage(...)` 注入。

### 4.2 新前端 `FastLivo2Frontend`

位置：

```
src/rosiwit_slam/include/algorithms/fast_livo2/
    fast_livo2_frontend.h          # 对外（实现 IFrontend）
    fast_livo2_config.h            # 纯数据结构，无 ROS
src/rosiwit_slam/src/algorithms/fast_livo2/
    fast_livo2_frontend.cpp
    livo_mapper.{h,cpp}            # 由 LIVMapper 改造（去 ROS）
    imu_processing.{h,cpp}
    preprocess.{h,cpp}
    vio.{h,cpp}
    voxel_map.{h,cpp}
    utils/...
    vikit/                         # 相机模型（include/src），内部使用
```

要点：

- `FastLivo2Frontend::init(config_path)`：用核心现有的 yaml 读取方式（与 `fast_lio2` 一致）
  解析 `mapping.frontend.*` + `mapping.camera.*` + `mapping.extrin_calib.*`，填充
  `FastLivo2Config`；不再声明 ROS 参数。
- `onImu` / `onLidar` / `onImage`：只入队（IMU 队列、LiDAR 队列、图像队列，加锁）；
  `onLidar` 触发一次处理（内部沿用 FAST-LIVO2 的 `sync_packages` 配对：
  LiDAR 帧 + 最近的图像 + IMU 区间）。
- 处理输出缓存：最新位姿、body/world 点云、彩色地图、关键帧计数。
- `getKeyFrame`：按距离/角度阈值（与 `FastLio2Frontend` 相同语义）产出关键帧，
  避免后端两套关键帧逻辑。
- 点云输入：核心只认 `LidarFrame`（`curvature` 已是 ms）。`CustomMsg` / 各种
  `PointCloud2` 的解析放在**接口层**（复用独立包 `generic_pcl_handler` 的字段识别逻辑，
  抽成接口层工具）；核心 `Preprocess` 只保留"从 PCL 点云 + curvature 出发"的处理
  （AVIA/OUST64/VELO16 分支按需保留，只按 `lidar_type` 选择参数）。
- 图像：`onImage` 收到 `cv::Mat`（BGR8/MONO8），不再使用 `cv_bridge`/`image_transport`。
- 相机模型：`vikit` 直接编译进核心（静态库/对象库），不引入 ament。
- 所有 `RCLCPP_*` 日志改为 `std::cerr`/可选 `Diagnostics`；所有发布器改为输出缓存。

### 4.3 配置（yaml）

在 `config/default.yaml` 增加（示意）：

```yaml
global:
  modules:
    frontend: "fast_livo2_frontend"   # 切回 fast_lio2_frontend 即恢复原行为

mapping:
  camera:
    model: Pinhole
    width: 1280
    height: 1024
    scale: 0.5
    fx: 1293.56944
    fy: 1293.3155
    cx: 626.91359
    cy: 522.799224
    d0: -0.076160
    d1: 0.123001
    d2: -0.00113
    d3: 0.000251
  extrin_calib:
    lidar_imu_t: [0.0, 0.0, 0.0]
    lidar_imu_r: [1,0,0, 0,1,0, 0,0,1]
    camera_lidar_r: [...]
    camera_lidar_t: [...]
  fast_livo2:
    img_en: true
    lidar_type: 8          # 8=GENERIC；1=AVIA 由接口层标记
    point_filter_num: 3
    blind: 0.5
    ...                    # vio/lio/imu 参数按独立包默认值迁移
```

- 兼容性：`request` 未配置 `mapping.camera` 时，`fast_livo2_frontend` 初始化失败并给出
  明确错误；其它前端不受影响。
- `frontend: fast_livo2_frontend` 与 `global.modules.frontend` 保持一致。

### 4.4 数据流

```
ROS2 SlamNode ── onImu  ─┐
               ├ onLidar ─┼─> SlamPipeline ──> FastLivo2Frontend
               └ onImage ─┘        │                 │
                                    │                 ├─ LIO（IMU 预积分 + 体素地图）
                                    │                 ├─ VIO（视觉地图 + 光度误差）
                                    │                 └─ 输出缓存（pose/clouds/keyframe/rgb map）
                                    └─> SlamOutput ─> ROS 发布 (/aft_mapped_to_init, /path, ...)
```

- 图像与 LiDAR 的配对在**前端内部**完成（时间最近 + 偏移可配置），
  不改 `SlamPipeline` 的打包/模式逻辑。
- `SlamPipeline::process` 在 `onLidar` 后照常取 `getOdometry/getClouds/getKeyFrame`。

### 4.5 接口层改动（ROS2 优先）

- `rosiwit_slam_ros2`：
  - 新增 `image_topic`、`camera_info_topic`（可选，优先用 yaml 内参）；
  - `sensor_msgs::Image` → `cv::Mat`（`cv_bridge` 已依赖；或直接按 encoding 转换）；
  - 若为 `CompressedImage`，用 `cv::imdecode`（避免额外插件依赖）；
  - 通过 `m_algo->onImage(ImageFrame{...})` 注入；
  - 发布彩色地图 `/world_cloud_rgb`（若前端提供）。
- `rosiwit_slam_ros1`：本轮只加配置项与 `onImage` 接口占位，不改 `input_pipeline`
  （后续阶段接入，避免一次改两条链路）。
- 独立包 `ros2_ws/src/fast_livo2` **保留**，作为参考实现与对比基准；
  核心版本验证通过后再决定是否删除（不在本轮）。

### 4.6 关键取舍

| 取舍 | 选择 | 理由 |
|---|---|---|
| 图像同步位置 | 前端内部缓存 | 不改管线打包逻辑，贴近 FAST-LIVO2 原设计 |
| 点云类型 | 核心只用 `LidarFrame`（curvature） | 保持核心与 ROS 消息解耦；CustomMsg 留在接口层 |
| 相机模型 | vikit 编入核心 | 独立包已验证；避免 ament/ROS 依赖 |
| 彩色地图 | `SlamOutput` 可选字段 | 不改变旧输出类型，接口按需发布 |
| 前端选择 | 工厂 + yaml | 与 fast_lio2/point_lio 一致，可回退 |
| ROS1 图像接入 | 后续阶段 | 控制单次改动范围，先打通 ROS2 + 核心测试 |

## 5. 分阶段实施

1. **骨架**：类型扩展、`IFrontend::onImage`、工厂注册、CMake 接入、空实现；
   保证现有测试与 `fast_lio2` 构建不变。
2. **预处理 + LIO**：迁移 `preprocess`/`imu_processing`/`voxel_map`/`livo_mapper`，
   去掉 ROS；先用 NTU 数据做 **LiDAR-惯性** 验证（`img_en=false`）。
3. **VIO**：迁移 `vio` + `vikit`，打通图像；验证 LIVO 精度。
4. **ROS2 接口**：订阅图像 + 配置 + 彩色地图发布；用 NTU `eee_03` 复测。
5. **ROS1 接口**（可选/后续）：`input_pipeline` 图像接入 + 配置。
6. **回归与文档**：`fast_lio2` 回归、standalone 测试、更新
   `docs/dev/fast-livo2-ros2.md` 与核心 README。

## 6. 验证标准

- **构建**：核心 standalone（Ubuntu 22.04）通过；ROS2 包内联构建通过；
  ROS1 Melodic（若可达）编译通过。
- **单元测试**：新增 `test_fast_livo2_frontend.cpp`（配置解析、IMU/图像入队、
  关键帧阈值、空输入不崩），加入现有 ctest。
- **精度**：NTU VIRAL `eee_03`（ROS2 接口回放）：
  - ATE（SE3，Umeyama）≤ **0.30 m**（独立包 0.257 m）；
  - 轨迹长度 ~137 m；
  - 无异常退出，VIO 持续更新。
- **回归**：`fast_lio2_frontend` 在相同数据/配置下输出与改动前一致（对比轨迹）。
- **性能**：x86 上 ≥ 实时（1x 回放不掉帧）；记录单帧耗时。

## 7. 风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| Melodic/OpenCV 3.2 兼容 | ROS1 编译失败 | 避免 4.x-only API；必要时加版本宏；先只保证 ROS2 |
| GPLv3（vikit）并入核心 | 分发许可 | 文档标注；内部使用不受影响 |
| 视觉线程与管线线程竞争 | 数据竞争/崩溃 | 前端内部统一 mutex；输出加锁拷贝 |
| 精度退化 | 验收失败 | 分阶段对照独立包；保留独立包做基准 |
| 核心体积/编译时间 | 开发体验 | vikit 只编必要源文件；算法模块独立 target |
| ARM 算力 | 实时性 | 保留 OpenMP/参数可调；不做额外开销 |

## 8. 留给实现者的决定

- 是否把 `Preprocess` 的多雷达分支全部保留，还是核心只保留 GENERAL + OUSTER/VELODYNE；
- `FastLivo2Config` 的 yaml 键名（与独立包同名优先，减少迁移成本）；
- 彩色地图默认是否发布（建议默认关闭，按需开启）；
- 关键帧阈值默认值（建议对齐 `FastLio2Frontend`：1.0 m / 0.17 rad）。

## 9. 评审问题与决策（2026-10-31）

以下是独立评审提出的 6 个会影响方案的问题，以及本设计的决定（标注为"我的判断"）：

### 9.1 调度：保留 FAST-LIVO2 的"图像时间驱动"

**决定**：前端内部完整保留 `sync_packages` 的 `WAIT → LIO → VIO` 状态机与
`pcl_proc_cur/next` 切分；`onLidar` 只负责入队并尝试推进一次处理。
若该 LiDAR 帧没有产生新的更新：

- `getOdometry()` 仍返回**最近一次**位姿（`true`），时间戳保持该位姿真实时间（图像时间），
  与独立包一致；管线照常发布，不要求每帧都有新状态。
- `getKeyFrame()` 返回 `false`。
- 位姿时间戳 = `last_lio_update_time`（图像时间），与独立包 0.257 m 的轨迹同源。

**为什么**：用户要的是完整 LIVO；"每帧 LiDAR 都更新一次"是另一种算法，精度不可比。
**风险**：`SlamPipeline` 目前按 `onLidar` 后立即取位姿，需确认它能接受"位姿时间早于帧尾"；
实现阶段先加一个集成测试覆盖该情况。

### 9.2 彩色点云：只做"当前帧世界系彩色点云"

**决定**：

- `IFrontend::getRGBCloud()` 与 `SlamOutput::rgb_world_cloud` 表示**当前帧**（`cloud_registered`
  语义）的彩色世界系点云，按位姿频率输出，可应用回环锚点变换；
- 累计彩色地图不通过 `SlamOutput` 流式输出（NTU 2.5M / Retail 18.9M 点，ARM 上不现实）；
  地图导出仍走 PCD（后续把颜色写入 `MapData` 是独立工作项）。

**为什么**：与 FAST-LIVO2 实际发布的 `/cloud_registered` 对齐，开销可控；累计地图已有离线产物。

### 9.3 外参：沿用现有的 "TF 优先" 规则

**决定**：

- LiDAR↔IMU：继续以 ROS 接口层写入的根级 `r_il/t_il` 为**唯一权威**（与 `fast_lio2` 一致），
  `mapping.extrin_calib` 仅在无根级键时兜底；
- 相机←LiDAR：yaml 的 `camera_lidar_r/t`（即独立包的 `Rcl/Pcl`），**只在初始化时**与当时
  生效的 `r_il/t_il` 组合成 `Rci/Pci`；
- 运行中 TF 变化（重标定）时，按现有机制重建算法实例（不新增在线重标定）。

**为什么**：标定以 TF 为准是现有工作流的约定，不能让 yaml 静默覆盖；相机外参没有 TF 来源。

### 9.4 时间偏移：进前端配置，避免与接口层自动标定叠加

**决定**：

- 新增 `mapping.fast_livo2.time_offset.{lidar,img,imu}`（秒），默认 0；
  NTU 配置用 `lidar = -0.1`（与独立包一致）；
- ROS2 接口层现有的 **IMU 自动偏移**（≤0.05 s）保持不动，它作用在 `IMUSample.time` 上；
  因此前端 `imu` 偏移默认 0，文档明确"不要同时配两处"；
- 图像时间偏移只在前端生效；LiDAR 逐点时间已在接口层写入 `curvature`，
  前端的 `lidar_time_offset` 作为整帧时间修正。

**为什么**：0.1 s 的 LiDAR–IMU 失配会直接毁掉 LIO/VIO；默认 0 保证不改变现有前端行为。

### 9.5 验收口径：原始前端里程计 + 可选全管线

**决定**：

- **主验收**：`SlamOutput::odom_pose`（前端原始里程计，未加回环锚点）对
  `ros2_ws/src/fast_livo2/Log/result/ntu_viral/eee_03_gt.txt` 做 SE(3) Umeyama，
  ATE ≤ 0.30 m；测量时关闭后端/回环（`backend: none` 或等价配置），与 0.257 m 同口径；
- **副验收**：默认全管线（Ceres + ScanContext）跑同一段，轨迹不崩、无跳变；
- **回归**：改动前用同一 bag/配置记录一份 `fast_lio2_frontend` 的 `odom_pose` 基线
  （新脚本 `scripts/record_odom_baseline.py` 或现有录制方式），改动后对比最大漂移 < 1 cm。

**为什么**：只有原始前端 odom 与独立包可比；全管线数字受后端/回环影响，不能当回归基线。

### 9.6 滤波归属：接口层为 fast_livo2 关闭预滤波

**决定**：

- ROS 接口层的 `RosUtils::ros2PCL`（`lidar_filter_num`、距离裁剪、curvature 填充）继续为
  `fast_lio2`/`point_lio` 服务，默认不变；
- 新增配置 `input.disable_prefilter`（默认 `false`）；使用 `fast_livo2_frontend` 时置 `true`，
  由前端的 `Preprocess` 按独立包语义做 `point_filter_num`/`blind`，避免双重抽样；
- 核心 `Preprocess` 只保留 GENERAL（按 curvature）+ OUSTER/VELODYNE 参数分支，
  `livox CustomMsg` 分支留在接口层，不进入核心。

**为什么**：双重滤波会让点数变成 1/9、blind 语义也变，直接改变 LIO 残差与 ATE。

## 10. 需要你确认的最小问题

1. 9.5 的验收口径（原始前端 odom ≤ 0.30 m）是否接受？还是必须看"全管线 + 回环"的数字？
2. 9.2 只输出"当前帧彩色点云"、累计彩色地图走 PCD 导出，是否够用？
3. 9.1 允许某些 LiDAR 帧复用上一帧位姿（时间戳为图像时间），是否接受？

（其余 9.3/9.4/9.6 按现有工程约定决定，不再单独确认。）

