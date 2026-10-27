# SLAM 回环：把地图改差的三个原因（2026-10-27）

> 背景：反馈"回环有问题，会把地图搞差"。本文记录定位过程、三个缺陷、修复与验证，
> 以及为什么仿真只能验证一部分。

## 1. 后端优化被"每帧"触发

`slam_pipeline.cpp` 里这段写在**每帧**都会执行的位置：

```cpp
if (m_backend && m_frame_count > 0 && m_frame_count % 50 == 0) {
    pushBackendTask(BackendTask::OPTIMIZE);
}
```

`m_frame_count` 只在**产生新关键帧**时递增。一旦它停在 50/100/…（车被安全层停住、
或关键帧间隔变大），这个条件**每一帧都成立** → 10 Hz 请求一次 Ceres 优化 + 全图重建。
实测日志（静止段）：

```
[CeresBackend] optimize N=50 loop=0 new=0 cost=1.05e-27->1.05e-27 iter=1 ...
[PcdMapManager] Rebuilt map from backend poses: 50 kfs ...
[CeresBackend] optimize N=50 loop=0 new=0 ...        <- 同一 N 连刷
[PcdMapManager] Rebuilt map from backend poses: 50 kfs ...
```

修法：加 `m_last_optimize_kf_` 守卫，只在**关键帧计数变化**时触发一次。

## 2. 重建时把两个坐标系混在一张图里（"回环越修越差"的直接原因）

增量累积（`addSubMap`）用的是**修正坐标系**：

```cpp
const PoseStamped eff_pose = poseMul(m_frontier_anchor_, kf.pose);   // A ∘ P_raw
```

但后端优化后的全图重建，未匹配到后端的帧用的是**原始 odom 位姿**：

```cpp
for (size_t j = 0; j < submaps_.size(); ++j) used[j] = submaps_[j].pose;   // 原始 odom！
for (...) if (kf_of_corr[i] >= 0) used[...] = corrected[i];               // 修正系
```

只要后端比地图慢一点（很常见：ICP/Ceres 期间前端还在出关键帧），重建出来的图里
"已修正的帧"在一个坐标系、"未修正的帧"在另一个坐标系，差一个 `A`。
表现就是**回环之后墙变成双线 / 整段错位**。

修法：未匹配的帧也按 `A_new ∘ P_raw` 变换（`A_new = G_last · P_last⁻¹`），
重建时所有帧都在同一坐标系；日志增加 `anchor-filled` 计数。

## 3. 对"不自洽的回环"没有事后检查

原来的保护只有一层：**如果本次优化把位姿移动超过 20 m / 60°，就回滚全部新约束**。
但假回环通常不会造成 20 m 的跳变 —— 它通过里程计边把误差**均匀分摊**到整条轨迹上，
产生 0.3~2 m 的"温和"形变，正好绕过这个门限，然后把地图拉歪。

修法：优化后按残差检查每一条回环约束（`dropInconsistentLoops`）：

* 用解出的位姿重算该约束的预测相对位姿，与测量值比较；
* 残差 > `loop_residual_trans_m`（默认 **0.40 m**）或
  > `loop_residual_rot_deg`（默认 **5°**）→ 判定为"与图里其它约束矛盾"，剔除；
* 剔除后**重解一次**（Ceres 1.x 没有 `RemoveResidualBlock`，所以重建 Problem）；
* 日志：`[CeresBackend] drop inconsistent loop kf_x -> kf_y (residual …)`，
  `optimize … dropped=N`。

倾向是"宁可少修正，也不要把图拉歪"：被剔除的回环不会再生效，但地图保持自洽。

配置（`mapping.backend`）：

```yaml
loop_residual_trans_m: 0.40
loop_residual_rot_deg: 5.0
```

## 验证与局限

* 仿真（两圈闭环、~100 关键帧）：`optimize` 次数恢复为"每 50 关键帧一次"；
  重建日志出现 `anchor-filled` 计数；`dropped=0`（这次没有不自洽回环）；
  地图保存正常；标准 3 目标路线回归 3/3。
* **仿真无法复现"大修正"场景**：Gazebo 的里程计几乎无漂移，真回环的修正量只有
  0.005~0.016 m，所以第 2 条的错位在仿真里看不出来、第 3 条也不会触发。
  这两条是按代码路径 + 实机症状修的，需要实机日志确认。
* 实机排查看这几行：
  * `[ScanContextLC] Loop detected: kf_a <-> kf_b (fitness=… overlap=… cov=…)`
  * `[CeresBackend] optimize … new=… dropped=… max_dt=… max_dr=…`
  * `[CeresBackend] drop inconsistent loop …`（假回环被剔除）
  * `[PcdMapManager] Rebuilt map … (matched X/Y, Z anchor-filled), anchor trans=…`
  * `[PcdMapManager] WARN reject correction: d_trans=… d_rot=…`（兜底拒绝，锚点不动）

## 待确认（需要产品/现场输入）

`map_manager.max_correction_trans_m: 15` / `max_correction_rot_deg: 150` 与
`backend.max_optimization_trans_m: 20` / `max_optimization_rot_deg: 60` 对室内地图来说
非常宽松（15 m 的锚点跳变仍会被接受）。这组值历史上是为了"让真回环能过"而放开的，
收紧需要现场数据支撑，本次没有改。


## 4. 本轮引入的改进（2026-10-27 第二轮）

分析回环链路后，有 4 处值得改，已全部实现并验证。

### 4.1 配置缺口：仿真一直在用"代码默认值"跑回环

`simulation.yaml` 里**没有 `mapping.loop_closure` 段**，于是回环参数全部退回代码默认：

| 参数 | 代码默认 | `default.yaml` | 影响 |
|---|---|---|---|
| `icp_fitness_thresh` | **1.0** | 0.5 | MSE 1.0 m² ≈ 1 m RMS 也放行 |
| `min_keyframe_gap` | 30 | 30 | — |
| `min_overlap` | 0.35 | 0.35 | — |

启动日志里能直接看到：`Initialized (... icp_fitness<1, odom_gate=10m)`。
**实机如果用的配置也没有这一段，就是同一个问题**。已在 `simulation.yaml` 补上与
`default.yaml` 对齐的整段（并加 `odom_gate_m`）。

### 4.2 ICP 初值：加入里程计先验

原来只用 ScanContext 的列偏移给 yaw、平移给单位阵。里程计漂移一旦超过
`icp_max_corr_dist=2 m`，GICP 从单位阵出发根本收敛不到真值，很容易停在错误的局部极小
（这是假回环最常见的来源）。

现在每个候选按优先级尝试 4 个初值，取 fitness 最优：

1. **里程计相对位姿** `T_cand⁻¹ · T_latest`（离真值通常只差漂移量级）
2. ScanContext `+yaw`
3. ScanContext `-yaw`
4. 单位阵（仅前 20 个"起点候选"）

收敛足够好（fitness ≤ 0.35×阈值 且 overlap ≥ 0.6）就早停，不再试其它初值。

**实测（两圈闭环仿真）**：

```
改前: icp_fitness<1.0, 只能用 sc_yaw/identity 初值, fitness 0.39~0.7
改后: icp_fitness<0.5, odom_gate=10m
      ICP passed: fitness=0.1069 overlap=0.9995 init=odom odom_gap=1.39m
      ICP passed: fitness=0.1433 overlap=0.9977 init=odom odom_gap=1.65m
```

fitness 降了 3~5 倍，重叠率 0.997~0.9995。**同样的阈值下，假回环更难混进来。**

### 4.3 里程计门控：不可能的回环不做 ICP

前端里程计说两帧相距超过 `odom_gate_m`（默认 10 m）时直接跳过候选。
以前所有候选都要跑 GICP（每个还带 2~3 个初值），既慢又给假回环机会。
门控值取 `max_relative_trans_m(8) + 余量`：即便真回环，里程计漂移超过 8 m 也会被
相对位姿门限拒绝，所以 10 m 不会漏掉"还能救"的回环。

### 4.4 复用降采样

`icpVerify` 以前每次都对自己的 src/tgt 重新做 VoxelGrid —— 而同一个 `latest` 子图
会被**每个候选 × 每个初值**反复用。现在：

* `latest` 子图只在检测开始时降采样一次；
* 候选子图每个候选降采样一次；
* 新增 `icpVerifyDownsampled()` 只做 GICP + 重叠率。

aarch64 上一次回环检测的体素滤波次数从"候选数 × 初值数"降到"候选数 + 1"。

## 5. 还值得做（需要你决定或后续排期）

| # | 改进 | 收益 | 代价/风险 |
|---|---|---|---|
| A | `getUpdatedPoses` 按 **id** 对应后端位姿（现在按时间戳 ±0.2 s 猜） | 去掉一个启发式；时间戳重复/抖动时不会错配 | 改 `IBackend`/`IMapManager`/`Submap` 三个接口，中等改动 |
| B | `addSubMap` 的增量体素滤波（现在每个关键帧对**整张**累积云重滤，O(N²)） | 建图时 CPU 明显下降，关键帧更不容易被拖慢 | 需要保证 `global_map_` 语义不变 |
| C | `m_detected_loops` 加上限 + 记录"已拒绝"的候选对 | 长会话内存/重复 ICP 都收敛 | 小 |
| D | `optimize_period_` 配置项目前**未使用**（读了但没人用） | 去掉误导 | 小 |
| E | 阈值组收紧（`icp_fitness_thresh 0.5`→0.3、`max_correction_trans_m 15`→2~3、`max_optimization_trans_m 20`→3~5） | 室内地图不该接受 15 m 锚点跳变 | **需要现场日志数据支撑**，本次只补了配置缺口，没有动数值 |
| F | 里程计边按运动量/匹配质量加权（现在统一 Huber(1.0) + 单位信息） | 回环和里程计的相对可信度更合理 | 工程量大，先有实机数据再谈 |
