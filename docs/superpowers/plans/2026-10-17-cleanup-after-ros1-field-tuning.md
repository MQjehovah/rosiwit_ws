# 建图/定位链路 收尾整理计划

日期: 2026-10-17
分支: `feature/ros1-ros2-compat`
背景: 实机联调完成（建图 + 定位可用）。本计划只做**整理与收敛**，不引入新功能。

---

## 0. 目标（按优先级）

1. **调试代码归位**：探针/诊断日志要么删除，要么收进统一开关；
2. **同一功能只有一处实现**（本轮出现过"两处各写一遍、只改了一处"导致的问题）；
3. **核心层 / ROS 薄层边界清晰**；
4. **坐标变换统一**（只有一处做 base_link ↔ 雷达系换算）；
5. **可调参数集中标注**（当前值 + 实测依据 + 调大调小的后果）；
6. **不破坏 ROS2 节点**（核心层的改动 ROS2 也吃）。

---

## 1. 本轮遗留的调试代码（需处理）

| 位置 | 内容 | 处置建议 |
|---|---|---|
| `src/algorithms/localization/localization_pipeline.cpp` | `[LocProbe] local_map=…` / `[LocProbe] scan/target/ok/converged/fitness` / `[LocProbe] REJECT …` | 收进 `~debug_probe`（默认 false）；REJECT 那条建议**保留为 WARN 但限流**，它是排查"匹配成功却不输出"的关键 |
| `ros1_ws/src/rosiwit_slam_ros1/src/legacy_xz3dslam_interface.cpp` | `[PubProbe] tick=… have_new_pose=…` | 同上，收进 `~debug_probe` |
| `ros1_ws/.../slam_engine.cpp` | `[slam_engine] point-time check: curvature_ms=[…]`（一次性） | 保留（一次性、成本极低）或收进 debug_probe；**它证明逐点时间去畸变生效**，建议保留 |
| `src/algorithms/fast_lio2/fast_lio2_frontend.cpp` | `[FLIO2] extrinsic(used) esti_il=… rpy_deg=…`（每 100 帧） | 保留但降频到每 600 帧（1 分钟），它是"外参是否被改错"的唯一现场证据 |
| `ros1_ws/.../slam_engine.cpp` | `[compat] …` 系列（acc_scale / r_il / ignored legacy keys） | 保留（启动一次性，核对用） |
| `src/algorithms/pcd_map_manager/pcd_map_manager.cpp` | `Generated grid map (…)` 每次实时栅格重算都打 → 1Hz 刷屏 | **改成节流**（例如每 10 次一条）或降为 debug 级 |

> 原则：**启动类**日志（一次性、核对用）保留；**每帧/每秒**日志必须可关且默认安静。

---

## 2. 重复实现（本轮的病根，必须收口）

| 功能 | 现状 | 目标 |
|---|---|---|
| 车身 ↔ 雷达系换算 | 之前 `SlamEngine` 与 `LegacyXz3dSlamInterface` **各查一次 TF**；现已统一到 `SlamEngine::lidarToBase()` | ✅ 已完成，但**要检查 `rosiwit_interface`（新接口）是否也需要**，避免第三次重复 |
| 初始位姿入口 | 之前 `/initialpose` 与 `/switch_map` 各自实现 | ✅ 已统一到 `SlamEngine::applyInitPose()` |
| 栅格生成 | 之前薄层 `realtime_grid.cpp::buildGridFromPoints`（合并点云）与核心 `PcdMapManager::generateGridMap`（逐帧观测）两套 | 已改为统一走核心；**删除薄层的 `buildGridFromPoints` 死代码**，避免以后又被误用 |
| `global_grid_map` | `setGlobalGridMap()` **从未被调用**（死代码/半成品） | 二选一：接上（载图后发布一次）或删除 |
| 位姿-点云一致性 | 输出位姿做了 base 换算，`world_cloud` 用核心位姿 → 语义需写清 | 在 `publishCurrScan()` 顶部加注释说明"点云在 map 系，与发布位姿的分辨率无关" |

---

## 3. 坐标系约定（写成一份唯一的说明）

必须在 `docs/` 留一份**唯一权威说明**，明确：

| 量 | 坐标系 |
|---|---|
| 核心内部位姿（`SlamOutput::pose`） | **雷达/IMU 刚体系**（`r_il`/`t_il` 是它与雷达之间的关系） |
| 核心输出点云 `world_cloud` | map 系（已含位姿） |
| 对外发布位姿（`curr_localization_pose` / `slam_info.robot_pose` / map→odom TF） | **base_link 系**（经 `r_lb_` 换算） |
| 上层下发的初始位姿 | **base_link 系**（进入核心前换算成雷达系） |
| 地图文件（`.xzmap` / `pcd`）| 核心内部（雷达系） |
| 2D 栅格 `map.pgm/.yaml` | map 系，`origin` = 栅格左下角 |
| 2D 栅格高度带 | **相对地面**（`z - ground_z ∈ [min_height, max_height]`），`ground_z ≈ -t_lb.z()` |

**规范**：任何坐标系转换只允许出现在 `SlamEngine`（输入侧）与 `LegacyXz3dSlamInterface::publishCurrPose`（输出侧）两个点。

---

## 4. 可调参数集中标注

目标：**同一个参数只在一处定义**，配置里写清"实测依据 + 后果"。
建议在 `config/default.yaml` 顶部维护一张"关键参数表"，并给每个关键项加 3 行注释（当前值/为什么/调大会怎样）。

本轮已确认必须标注的参数：

| 参数 | 当前值 | 实测依据 |
|---|---|---|
| `planar.mode` | `none` | 雷达俯装 41.67°，约束会拉平 → 覆盖率 0.43；关掉后 0.99 |
| `localization.accept.min_coverage` | `0.35` | 实测覆盖率 0.43~0.99；旧值 0.5 会拒掉有效匹配 |
| `matcher.max_iterations` | `12` | 32 迭代单帧 ~0.7s（输出 1.4Hz）；12 迭代 → 10Hz，精度 0.99 |
| `matcher.voxel_resolution` | `0.4` | 与迭代数配套 |
| `matcher.max_correspondence_dist` | `1.0` | 收紧后精度由 `inlier_dist=0.3` 兜住 |
| `localization.pipeline.relocalizer` | `none` | 手动重定位不跑 BBS（用户要求）；`/global_localization` 仍可显式调用 |
| `reset_on_mode_switch` | `true` | 进建图清残留（旧实现语义）|
| `mapper`/`backend` | `ceres_pose_graph` | 机器人无 gtsam |
| `esti_il` | `false` | 实测会发散到 ~180° |
| `r_il/t_il` | identity / 0 | 两话题 frame_id 同名（雷达内置 IMU）|
| `map_resolution` | `0.05` | 与现网 map.yaml 一致 |

---

## 5. ROS2 不受影响 —— 核心层改动的回归清单

本轮改到 **`src/rosiwit_slam`（核心，ROS2 也编译）** 的文件：

| 文件 | 改动 | ROS2 风险 | 验证方式 |
|---|---|---|---|
| `slam_pipeline.cpp` | detach 代替 join；定位模式不喂前端；`getRealtimeGrid` | 低（接口新增默认实现）| ROS2 编译 + 单测 |
| `pcd_map_manager.cpp/.h` | 后端位姿按时间戳对齐；修正量门限；`getGeneratedGrid` | 低 | 单测（已有 54 个）|
| `ceres_backend.cpp` | 相对位姿残差参考系修正 | **中**（若 ROS2 也用 ceres 后端，结果会变——这是修 bug，但要重跑定位/建图精度验证）| ROS2 端到端建图 |
| `grid_map_io.cpp` | `loadGridMapPGM` 读 origin | 低（纯修复）| 单测 |
| `localization_pipeline.cpp` | 逸出保护；接受门限；探针 | 低 | ROS2 定位冒烟 |
| `fast_lio2_frontend.cpp` | 外参打印 | 低 | 编译 |

**验证步骤**（本机只装了 melodic，ROS2 需在 ROS2 环境执行）：
1. `colcon build`（`ros2_ws`）；
2. `colcon test` + 现有单测；
3. ROS2 端跑一次建图（确认 `Rebuilt map … matched k/k` 且 `anchor trans` 为 cm~m 级）；
4. ROS2 端跑一次定位（确认 `matching_degree` 正常、输出频率 ≥10Hz）。

---

## 6. 执行顺序（建议）

1. 建 `docs/superpowers/specs/…-coordinate-and-param-conventions.md`（第 3、4 节内容）
2. 删除死代码（`buildGridFromPoints`、`setGlobalGridMap` 或接上它）
3. 探针收进 `~debug_probe`，日志分级
4. 参数注释补全（第 4 节）
5. ROS2 编译 + 单测 + 端到端冒烟
6. 提交（每步独立提交，便于回滚）
