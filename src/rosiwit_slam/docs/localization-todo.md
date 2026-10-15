# 定位（Localization）优化 TODO — 追踪

> 用途：记录定位子系统的优化任务与完成状态，配合「选择性执行 → 完成后勾选更新」的工作方式。
> 图例：`[ ]` 待办 · `[~]` 进行中 · `[x]` 已完成
> 更新日期：2026-10-15

---

## 一、已完成（本轮定位可编排流水线 + 变化场景鲁棒性）

- [x] 可编排定位流水线（5 阶段可插拔）：`relocalizer/motion_prior/local_map/matcher/monitor` + 工厂 → `617a9b8`
- [x] BBS 分支定界全局重定位 → `8db0c49`
- [x] p2p_icp 鲁棒点到点配准（KISS-ICP 风） → `67bbfac`
- [x] keyframe_submap + `io::readXzMap` 读取器 → `243e4c2`
- [x] 近批：OpenMP 加速 / 可切换鲁棒核 / 多候选重定位 / 协方差输出 → `7aad264`
- [x] 中批 M1+M2：vgicp（体素化点到面）+ fpfh_ransac（6DoF） → `c262f6f`
- [x] 中批 M3：非破坏性叠加地图维护（`map_maintainer: overlay`） → `2e118e6`
- [x] 视觉松耦合（route b）：VO between 因子 → GTSAM → `465de9d`
- [x] lifelong：叠加载持久化过滤 + 自动落盘 → `22c514b`
- [x] 视觉里程计 ROS 接线（开关 + 可配置话题） → `5e41238`
- [x] 局部地图缓存 + lifelong 刷新重定位器；去掉视觉回环外部话题 → `8c2f6ea`
- [x] 变化场景鲁棒性：动态物剔除 + 叠加载衰减 + 原图压制 → `b41cc37`
- [x] 过桥与半静态堆货：coasting + 置信门控 + 变化增量 + 自由空间雕刻 → `9eecf21`
- [x] 过桥增强：打滑门控轮式里程计 + 出口有界局部重锁 → `108c3c7`

---

## 二、待办（按优先级）

### 高优先级

- [x] **L11 IMU 零偏在线估计（陀螺 + 加速度/重力，静止标定，未静止跳过）** ✅ `62b67d9`
  - 目标：改善过桥期 IMU 外推的朝向/姿态漂移（洗地机打滑更依赖 IMU）。
  - 设计：`ImuMotionPrior` 增加 `gyro_bias/acc_bias` 状态；静止检测(gyro/acc 标准差+‖a‖≈g)成立才标定(均值)，未静止则跳过并继续尝试；`onMatch` 用几何观测器在线更新两零偏(夹紧)；`use_gravity` 用重力方向做 roll/pitch 校正。
  - 关键文件：`motion_prior/imu_motion_prior.{h,cpp}`；`localization_stage_types.h`(MotionPriorConfig)；config `localization.motion_prior.*`
  - 验收：合成(静止+常值零偏→收敛；未静止→跳过标定但在线仍收敛)；真机过桥 yaw 漂移下降。

- [x] **L12 高频位姿输出（方案一高率传播 + 方案三输出滤波）** ✅ `62b67d9`
  - 目标：LiDAR 10Hz 校正，但输出 ~100Hz 平滑位姿（≤ IMU 有效频率）。
  - 设计：`ILocalization/SlamPipeline` 暴露 `predictAt/getHighRatePose(t)`（基于最后校正位姿 + 运动先验/IMU 传播）；node 加高频输出定时器(默认 100Hz)，odom/TF 发传播位姿；**输出用 alpha-beta/常速 Kalman 滤波替代现有 EMA**（低延迟、抑 IMU 抖动）；扫描帧校正锚点（可用 L4 平滑器）。
  - 关键文件：`slam_core/i_localization.h`、`localization_pipeline.{h,cpp}`、`slam_pipeline.{h,cpp}`、`ros_interface/slam_node.{h,cpp}`、新增 `slam_core/pose_filter.h`；config `output_rate_hz`/`output_filter_*`
  - 验收：单测(alpha-beta 外推命中解析值)；真机 TF/odom 平滑无阶梯；频率可配。
  - 备注：常速 Kalman(因果低延迟) 用于 100Hz 输出流；L4 固定滞后平滑器用于扫描帧锚点(更准)，两者互补。

- [x] **L1 地面/平面约束（开关 + 自适应）** ✅ `89b8c8f`
  - 目标：平地上锁 `z/roll/pitch` 抗退化；坡道自动跟随地面平面。
  - 设计：配置 `planar_constraint: none | lock_zrp | ground_plane`（默认 `none`）。
    - `lock_zrp`：硬锁 `z/roll/pitch=0`（仅全平区域）。
    - `ground_plane`：每帧估计地面平面（法向 n + 高度，复用 `grid_map/ground_segmentation`），
      约束 roll/pitch 到平面法向、z 到平面高度；平面拟合不可靠或法向与竖直夹角超阈值时回退 3D。
  - 关键文件：`include/slam_core/localization_stage_types.h`、`localization_pipeline.cpp`、新增平面估计工具。
  - 验收：平地模式 `z/roll/pitch` 稳定不漂；坡道模式位姿能跟随坡度、坡底/坡顶不跳变；契约测试。
  - 备注：洗地机会过坡，**不可**硬 2D，须开关 + 自适应。
  - 实现说明：从底图构建"地面模型"(每格高度+法向) → `ground_plane` 相对地图地面约束(平地锁平/坡道跟随); 只用实时扫描无约束(恒等式), 故必须相对地图。文件 `constraints/planar_constraint.{h,cpp}`; config `localization.planar`.

- [x] **L2 协方差门控 + NEES 一致性** ✅ `89b8c8f`
  - 目标：用 `MatchResult.cov` + 运动协方差做 Mahalanobis 门控，替代/补充纯 fitness 阈值；NEES 判可信/丢失。
  - 关键文件：`localization_pipeline.cpp`（accept/monitor），`MatchResult.cov`（已具备）。
  - 验收：门控在动态外点帧更稳；NEES 在丢帧/漂移时报警；单测。
  - 实现说明：`poseInnovation()` + `acceptMatch` Mahalanobis 门控(chi2=16.81) + `MonitorInput.nees`/`WindowedMonitor` NEES 滑窗; config `accept.mahalanobis_*` / `monitor.nees_threshold`.

- [x] **L3 静态底图清理（在线 + 离线两层）** ✅ `1436231`
  - **L3a 在线（建图阶段，已有雏形，增强）** ✅
    - 现状：`fast_lio2/map_builder/lidar_processor.cpp` 的 `dynamic_removal`（`persist_thresh`/`persist_window`/`dynamic_residual_thresh`）：逐体素持久性过滤，只写"多帧持续观测"的点，超窗未见即剔除。
    - 增强：补**逐帧射线自由空间雕刻**（对 ikd-tree 负信息：射线穿过的体素判自由并剔除先前占据点），弥补纯持久性过滤对滞留物/局部视角的不足；亦利于 LIO 配准/回环。
    - 关键文件：`src/algorithms/fast_lio2/map_builder/lidar_processor.cpp`、`commons.*`、`config/default.yaml`。
    - 验收：移动人/叉车不入图；保存的图当场更干净；对 LIO 配准无负作用。
    - 实现说明：开关 `free_space_carving`(默认关) + `carve_free_thresh`/`carve_stride`；`carveFreeSpace()` 沿射线对已占据体素累计"看到空"次数，达阈值经 `Radius_Search`+`Delete_Points` 删除。**需真机验证。**
  - **L3b 离线（后处理，ERASOR/Removert 式全局清洗）** ✅
    - 目标：对导出的种子底图做全局可见性/伪占据清理，产出定位用干净底图（一次性精修）。
    - 数据：用 `io::readXzMap` 的关键帧位姿+点云做多视角可见性推理。
    - 关键文件：新增离线工具 `tools/map_cleanup.cpp`（复用 occupancy/可见性逻辑）。
    - 验收：清理后底图不含动态/慢速滞留物；同场地再定位更稳；工具可重复运行。
    - 实现说明：关键帧体素投票(occ/free)，`free > free_ratio*occ` 判动态并移除输入点；`map_cleanup <in.pcd> <out.pcd> [--xzmap …]`。

- [x] **L9 真实数据回归 + 预设 + 量化** ✅（预设部分） `1436231`
  - 目标：合成脚本（进狭道堆满货 → 过桥 → 发走 → 重开）量化“不丢率”；真实工厂 bag 回归；产出 `config/factory.yaml`。
  - 验收：给出不丢率/漂移指标；预设可直接用。
  - 实现说明：工厂调参已直接写入 `config/default.yaml`（pipeline/matcher/maintainer/planar/monitor）。**真实数据回归仍待做。**

### 中优先级

- [ ] **L13 地图维护升级为位姿图（GTSAM + 可见性 replace/extend）**
  - 目标：overlay 从"点级非破坏"升级为"变化关键帧+锚点进位姿图"，可见性/凸包判替换或扩展，周期优化后重建 store/底图。
  - 关键文件：`localization/maintainer/*`、复用 `gtsam_backend`；config `maintainer.*`
  - 验收：长期大范围变化下地图不退化；单测/合成。

- [ ] **L14 2.5D 平面 NDT 定位（matcher: ndt2d）**
  - 目标：货架/狭道等特征匮乏区，在平面 (x,y,yaw) 求解 + 地面约束 z/roll/pitch（复用 L1）。
  - 关键文件：新增 `matcher/ndt2d_matcher.{h,cpp}`；config `matcher.*`
  - 验收：狭道/重复货架区比 3D 更稳；契约测试。

- [x] **L4 固定滞后平滑器（Fixed-lag smoother）** ✅ `d4f9618`
  - 目标：用 GTSAM 滑窗融合 IMU + 扫描匹配 + VO/轮速因子，替代 α 混合，输出平滑位姿 + 传播协方差。
  - 关键文件：`localization_pipeline.cpp` + 复用 `gtsam_backend`；新增滑窗因子图。
  - 验收：位姿平滑、异常帧被优化器抑制；协方差合理；单测。
  - 实现说明：`smoother/fixed_lag_smoother.{h,cpp}`（`gtsam::IncrementalFixedLagSmoother`：运动先验 between + 扫描匹配 unary 带 `MatchResult.cov`）；`smoother.enabled` 默认关；无 GTSAM 直通降级。

- [x] **L5 GNC 渐进非凸鲁棒核** ✅ `d4f9618`
  - 目标：`matcher` 新鲁棒核选项，高外点比（人/叉车多）下逼近全局最优（Black-Rangarajan 对偶，逐步收紧）。
  - 关键文件：`ScanMatcherConfig.robust_kernel` 扩展 `GNC`；`pp_icp`/`p2p_icp` 权重更新。
  - 验收：高外点合成数据下位姿误差显著低于 Huber/Cauchy；单测。
  - 实现说明：`RobustKernel::GNC` + `gncWeight()`（Geman-McClure，mu=mu0·decay^iter）；config `matcher.gnc_mu0/gnc_decay`；鲁棒核遍历测试含 GNC。

- [x] **L6 多假设 / 粒子滤波** ✅ `d4f9618`
  - 目标：应对工厂重复货架/通道的错配（aliasing），保留多个位姿假设按证据淘汰。
  - 关键文件：`localization_pipeline.cpp` 或新 `hypothesis_manager`。
  - 验收：重复场景下不发生错误锁定；单测/合成。
  - 实现说明：`hypothesis/hypothesis_manager.{h,cpp}`：重定位 Top-K 初始化假设，逐假设局部配准评分，落后连续 `stale` 帧淘汰；config `localization.hypothesis.{max,margin,stale}`。

### 低优先级

- [x] ~~**L7 学习式 place recognition（OverlapTransformer/MinkLoc3D/BTC）**~~ **已取消（不做）**
  - 决定：不引入学习式召回（需训练数据/GPU），重定位保留 SC/BBS/FPFH。

- [x] **L8 统一地图存储底座（IMapStore 抽象 + Pcd/Voxel 实现 + 算法解耦）** ✅ P1–P5
  - 目标：让算法只依赖 `IMapStore`，换存储(PCD/KD 树 ↔ 体素哈希)不影响算法；各算法按需取"能力视图"(点/近邻/统计/地面/维护)。研究型项目便于切换方案 A/B。
  - 决策：底座 = **体素哈希 + 格内存点(iVox 思路)**（纯 GMM 不能当通用底座，pp_icp 需精确近邻）。
  - 分期：
    - [x] **P1** `IMapStore` 接口（`slam_core/i_map_store.h`）✅ `217910c`
    - [x] **P2** `PcdMapStore` / `VoxelMapStore` + `MapStoreFactory` + 等价契约测试 ✅ `217910c`
    - [x] **P3** 配准器消费 store：`IScanMatcher::setStore/usesStore`；pp_icp/p2p 支持 store 路径 ✅ `26bb6f0`
    - [x] **P4** 管道接入：`map_store: none|pcd|voxel`；matcher 用 store 时跳过局部地图构建；叠加层刷新重建 store ✅ `127f2d0`
    - [x] **P5** `.vxmap` 持久化；`VoxelMapStore::groundAt/decay`；`PlanarConstraint` 地面查询走 store ✅ `5bc5662`
  - 验收：契约/等价测试通过（KD 树 vs 体素最近邻距离等价 <1e-3）；`map_store` 可 A/B；默认 none 零行为变化。
  - 备注：非"绝对最优"，是研究可插拔底座；GMM/NDT 视图可在 VoxelMapStore 上按需扩展。

- [x] ~~**L10 配置分层（base / lidar / scene 深合并）**~~ **暂缓（不做）**
  - 决定：暂缓。方案已设计（内存深合并 `ConfigLoader::load`，顺序 base→scene→lidar，lidar 最高；无落盘），但先不做。当前单文件配置继续可用。

---

## 三、约定

- 每完成一项：勾选 `[x]`，补上 commit 短哈希与日期，必要时附验收数据。
- 进行中：标 `[~]`。
- 拆分/新增：在该项下加子项。
