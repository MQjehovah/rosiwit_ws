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

- [ ] **L7 学习式 place recognition（OverlapTransformer/MinkLoc3D/BTC）**
  - 目标：大范围/跨时段召回，作为 `relocalizer` 新策略（需训练数据/GPU）。
  - 验收：跨时段召回率优于 ScanContext；集成到工厂 → relocalizer。

- [ ] **L8 统一 voxel/GMM 地图**
  - 目标：合并「静态 PCD + overlay cells + 雕刻」为单体素哈希地图（每体素 mean+cov+计数），原生增量/雕刻/衰减。
  - 验收：行为等价且内存/查询更优；为 L5/L7 打基础。

---

## 三、约定

- 每完成一项：勾选 `[x]`，补上 commit 短哈希与日期，必要时附验收数据。
- 进行中：标 `[~]`。
- 拆分/新增：在该项下加子项。
