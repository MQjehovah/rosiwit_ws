# TEB / DWA 控制器完成与四控制器对比 — 设计

状态：待评审（brainstorming 已完成，等确认后写计划）
日期：2026-11-01

## 1. 意图（为什么做）

导航"总觉得有问题"，但当前只用过 `rpp`（仿真）/ `mppi`（实车）。要判断问题出在
**控制层**还是**规划/代价地图/安全层**，需要一个可信的对照组：同一套接口、同一批场景、
同一套指标下，至少 4 个控制器的横向对比。

而 `teb` 现在只是"跟线器"：优化只在 `setPath()` 里跑 50 次，运行中不再优化；障碍项只用
`setObstacles()` 的圆，完全没用局部代价地图；没有足迹校验、没有可行性约束、没有同伦备选。
拿它对比没有意义。

## 2. 目标 / 非目标

**目标**
1. 把 `teb` 做成"贴近 `teb_local_planner`"的**定时弹性带**：稀疏图上的 Gauss-Newton、
   多目标权重、同伦候选并行、足迹感知的障碍项、速度/加速度约束、端点固定、每拍增量优化。
2. 把 `dwa` 补到"能公平参赛"：无可行采样时的兜底、倒车支持、末段速度收敛。
3. 提供四控制器 A/B：`rpp` / `mppi` / `dwa` / `teb`，同场景、同指标、可重复。

**非目标**
- 不改全局规划器、代价地图分层、安全层、恢复状态机（本轮只做控制层；发现的问题单独记）。
- 不追求与 `teb_local_planner` 参数**逐项**兼容；`teb_*` 参数名尽量对齐，语义按本仓库实现。
- 不做多机器人 / 非完整约束以外的动力学（不做全向轮、不做阿克曼）。

**反目标（即使"能跑"也算失败）**
- 为了通过对比而放宽安全：TEB 输出任何一条足迹压到 lethal 格的轨迹都算失败。
- 控制器内部偷偷改写全局路径几何（TEB 允许偏离，但必须限制 `max_deviation`）。
- 对比数字来自不同场景/不同起点（不可比）。

## 3. 约束

- ROS1 melodic + C++17，**不引入新第三方依赖**（无 Eigen、无 g2o、无 Ceres）。
- 必须实现现有 `core::IControllerStrategy`（`initialize/setPath/computeVelocityCommand/
  isGoalReached/getProgress/reset/setVelocityLimit/setObstacles/setLocalCostmap/
  supportsFootprint/supportsMultiDirection`），由 `NavigationFactory` 按名字 `"teb"` 创建。
- 每拍预算：`computeVelocityCommand()` **≤ 8 ms**（控制 10 Hz，同拍还要跑安全层）。
- 与现有 footrpint/EDT 复用：`core::sampleFootprint` / `core::footprintCost` /
  `core::euclideanDistanceCells`。
- 单测必须能在无 ROS 环境跑（core 的 `test_*.cpp` 风格，catkin_add_gtest 已就绪）。

## 4. 结构（新增/修改）

```
src/rosiwit_navigation/
  include/rosiwit_navigation/nav_core/
    teb_types.hpp        # 带顶点/边/权重参数 (PoseSE2 + dT)
    banded_solver.hpp    # 块三对角线上 Gauss-Newton + LM 阻尼 (headless, 可单测)
    obstacle_distance.hpp# lethal mask -> EDT 缓存 + 最近障碍距离查询(含足迹采样)
  src/algorithms/local_planner/
    teb_controller.cpp   # 重写: 构图/每拍优化/候选同伦/输出首段速度
    dwa_controller.cpp   # 局部增强: 无解兜底/倒车/末段
  test/
    test_banded_solver.cpp
    test_teb_controller.cpp
    test_dwa_controller.cpp   # 扩充
scripts/sim/
    compare_controllers.sh    # 四控制器 A/B
    nav_sim_goal.py           # 增加"最小间隙"指标
```

## 4.1 已定的口径（评审后确认）

- **对比矩阵 = 两套 profile**：`clean`（四个控制器都拿单向路径 + 末端原地转，比纯跟踪）与
  `free`（TEB/RPP 拿带倒车的双向 Hybrid A*，MPPI/DWA 单向，比整套能力）。两套数字都出。
- **场景固定**：固定 spawn + 固定 goal + 固定世界（新增 0.9 m 门洞世界、堵路场景用禁行区注入），
  同一 (控制器, 场景) 跑 3 次，路线完全一致。
- **可行性只校验窗口内顶点**：弹性带可比 6 m 局部窗口长，越界顶点不参与碰撞判定。
- **`setPath()` 保持轻量**：只存路径并预热，带子在 `computeVelocityCommand()` 里增量构建，
  避免 1 Hz 重规划那一拍叠加构图开销。
- `getCurrentWaypointIndex()` 仍返回**路径索引**（节点把它当 `current_path_.poses` 的下标用），
  带的裁剪另存一套索引。
- EDT 每次 `setLocalCostmap` 都重算（窗口 6×6 m ≈ 1.44 万格，≈1 ms），不做"掩码是否变化"判断
  （窗口每拍都随机器人平移，判断只会永远为真）。
- 倒车速度符号以 `Path::points[].velocity_x` 为准（cusp 处顶点位移方向可能与标记不一致）。
- DWA 倒车复用已有 `ControllerConfig::allow_reverse`，不新增字段。

## 5. 关键设计决定（我的判断，可推翻）

1. **求解器自研**：图是链式（pose 0..n + ΔT 0..n-1），Hessian 是块三对角 → 用带状
   LDLᵀ 直接解，`O(n·b²)`，n=40~80 时 <1 ms。比引入 g2o 更可控，也更容易单测。
2. **障碍距离用 EDT 场**，不用逐点遍历障碍：每收到一次 `setLocalCostmap` 且 lethal 掩码
   变化时重算 `euclideanDistanceCells()`（O(N)，10 万格 ~1 ms），之后按格双线性插值查询。
   足迹用 `sampleFootprint(footprint, 0.05)` 的点集，取"各点到最近 lethal 的最小距离"作为约束量。
3. **同伦候选**：默认 3 条初始带 = 全局路径 + 左右各偏移 `teb_homotopy_offset`(0.35 m)，
   各自优化后取代价最低且可行的；**上一拍获胜者作为首选初值**（warm start + 迟滞，避免
   两族来回跳）。不做真正的同伦类判定（用偏移初值近似，文档里写明）。
4. **每拍增量优化**：`max_iterations=3`（内层 1），warm start；机器人前进后裁掉身后顶点、
   在末端补点；`Δt` 初值由弧长/`max_vel_x` 估计。
5. **可行性 = 硬约束后验校验**：优化完用 `footprintCost >= 100` 检查整条带；不过就用次优候选；
   全不过 → 输出零速 + `getDiagnostics()` 标 `teb_infeasible`，交给上层安全/恢复。
6. **倒车**：顶点位移方向决定 v 的符号（与 `Path::points[].velocity_x` 的方向标记一致），
   速度约束作用于 |v|。
7. **DWA 兜底**：所有采样被拒时，按"最小代价 + v 限幅 0.3×max"放行一条最不坏的轨迹（保持
   有解可退），并在 `getDiagnostics()` 里标 `dwa_all_colliding`；倒车靠放开 `min_vx`。

## 6. 参数（新增 `teb_*`，默认值 = teb_local_planner 量级）

| 参数 | 默认 | 说明 |
|---|---|---|
| `teb_max_vel_x` | 沿用 `max_velocity_x` | m/s |
| `teb_max_vel_theta` | 沿用 `max_velocity_theta` | rad/s |
| `teb_max_acc_x` | `max_accel_x` | m/s² |
| `teb_min_obstacle_dist` | 0.12 | m，软约束阈值（车身外） |
| `teb_inflation_dist` | 0.30 | m，软约束外带 |
| `teb_weight_obstacle` | 8.0 | 无量纲 |
| `teb_weight_kinematics` | 6.0 | |
| `teb_weight_velocity` | 4.0 | |
| `teb_weight_acceleration` | 3.0 | |
| `teb_weight_time` | 1.0 | |
| `teb_weight_shortest` | 1.0 | |
| `teb_weight_path` | 1.0 | 偏离全局路径 |
| `teb_max_deviation` | 0.6 | m，偏离上限（硬拒） |
| `teb_homotopy_classes` | 3 | 候选条数（1=关） |
| `teb_homotopy_offset` | 0.35 | m |
| `teb_max_iterations` | 3 | 每拍外层次数 |
| `teb_dt_ref` | 0.3 | s，Δt 初值尺度 |
| `teb_max_samples` | 80 | 带顶点上限 |

## 7. 对比方案（验收就靠它）

场景（仿真，`scripts/sim/run_nav_sim.sh`）：全部**固定起点 + 固定终点**，新增两个世界。

| 场景 | 世界 / 起点 | 终点 | 看什么 |
|---|---|---|---|
| `corridor` | large.world，spawn(-18,-18,1.57) | 沿走廊 +10 m | 速度利用、直线跟踪 |
| `house_short` | house.world，(0,0,0) | (1.0, 2.0) | 短距到点精度、末段收敛 |
| `house_long` | house.world，(0,0,0) | (2.4, 4.8) | 长距/拐角（当前会 `FAILED_PATH_FOOTPRINT`） |
| `doorway` | 新增 `doorway.world`（0.9 m 门洞） | 门后 3 m | 窄通道：能不能过、最小间隙 |
| `blocked` | house.world | (1.0, 2.0)，行驶 1.5 s 后横插禁行区 | 运行中被堵：换路还是恢复 |

每个 (控制器 × profile × 场景) 跑 3 次，输出 CSV + markdown 表：

| 指标 | 说明 |
|---|---|
| `success_rate` | SUCCEEDED / 3 |
| `xy_err_m` / `yaw_err_rad` | 到点精度 |
| `wall_s` | 用时 |
| `cmd_mean_v` / `cmd_max_v` | 速度利用 |
| `cmd_max_dv` / `cmd_max_dw` | 抖动（单拍跳变） |
| `xtrack_rms_m` / `max` | 横向跟踪 |
| `min_clearance_m` | 行驶轨迹到 lethal 格的最近距离（新增） |
| `safety_scale_min` | 安全层是否被触发 |
| `plan_ms_max` / `replan_fail` | 上层代价（确认不是规划拖累） |

判读方式：`teb`/`dwa` 若在同一场景下 `min_clearance` 更小但 `success_rate` 更高，
就说明"控制层能救规划"；若 `xtrack` 明显更差而 `safety_scale` 频繁掉，则是安全层在替
控制器兜底——这正是要区分的东西。

## 8. 留给实现者的判断

- 带状求解器的阻尼策略（LM 的 λ 增减规则）——按"失败×10、成功÷3"的常规做法即可。
- 多候选是否并行（开线程 vs 顺序）——先顺序，实测 >8 ms 再考虑。
- `teb` 是否声明 `supportsMultiDirection()`——初版声明 `true`（支持 cusp），若实测 cusp 处
  抖动则退回 `false`，由规划器只生成单向路径 + 原地旋转。
- `supportsInPlaceRotation()`：初版 `false`（交给上层执行器），与 `rpp` 一致。
