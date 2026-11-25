# 子图锚偏 / 方向可观测性 实验报告（2026-11-25）

> 目标：修 `rosiwit_slam_ros1` 现场"子图锚偏 → 整体定位丢失"。
> 约束：**不改 fitness 判定、不对点数打折**（对接工位 ~200 点但位姿是对的），只动算法本身。
> 现场配置：`local_map: keyframe_submap`、`max_keyframes: 24`、`map_maintainer: none`、
> `session_map.enabled: false`、`min_coverage: 0.6`、`lost_fitness: 0.60`、
> `min_evidence_points: 150`、`low_evidence_limit: 300`。

## 1. 结论（先给结论）

1. **给 gicp 补方向可观测性本身是可行的，但不能直接接上门控。**
   新估计器在正常帧算出的 `λmin/λmax` 只有 ~4e-3（`dirs` 恒为 0），而现有
   `accept.degeneracy_ratio_soft` 是 0.05 —— 两者量纲/加权口径不一致。
   直接置 `has_hessian` 会让 `degeneracy_partial_update` 把**每一帧**的修正压到 ~8%，
   实测 SUCCESS 从 99.8% 掉到 85.6%（见 Run C2）。所以本次只"上报诊断值"，不置位。
2. **"子图窗口滞后"（方案 ③-2）没有证据显示更好，默认关闭。**
   同一份包对照：关闭 98.5%~99.8%（Run A/E/Joff 里关闭侧）；打开 lag=1m 99.4%；
   打开 lag=8m + 可信门限 85.6%（一次，末尾 LOST 32 s）。
   值得注意的是：**D（7% SUCCESS）用同一配置 `Joff` 复跑得到 99.4%** —— 说明 D 是
   台架/初始锁定的偶发失败，不能归因于 anchor 滞后（本台架的偶发失败率约 1/5，
   见 §5 坑）。既然没有收益、又确实可能让窗口滞后，保持默认关闭。
3. 因此本次提交只保留：**③-1 只读诊断** + **③-2 配置开关（默认 false）** +
   **冻结逃逸（§6，默认开）**。
   `anchor_hold=false` 时目标建图路径与改动前完全相同（Run A vs Run E：99.8% vs 98.5%，
   p50 0.790 vs 0.789，均无 LOST）。

## 2. 实验矩阵（同一 `loc_sessionmap.bag`、设备风格配置）

| Run | 代码/配置 | SUCCESS | matching p50 | 帧率 | 说明 |
|-----|-----------|---------|--------------|------|------|
| A | 基线（无本次改动） | 2427/2431 = **99.8%** | 0.790 | 9.84 Hz | 无 LOST |
| B | 基线 + `radius_submap` | 80.6% | 0.854 | — | 初始锁定 37.7 s，不采用 |
| C | ③-1 门控打开（J 用了绝对世界点，量纲错） | 45/1927 = **2.3%** | 0.500 | 3.5 Hz | `eig=1.3e-5`、`dirs=3`，修正被清零 |
| C2 | ③-1 门控开（J 修好）+ ③-2 lag=8m/可信 0.85 | 2021/2360 = **85.6%** | 0.777 | 9.06 Hz | 末尾 LOST 32.4 s（161.9–194.4 s） |
| D | ③-1 只诊断 + ③-2 lag=1m/min_fit=0 | **LOST 1812** | — | 1.36 Hz | 开局锁死；**Joff 同配置复跑 99.4% → 偶发，不是 anchor 的锅** |
| E | ③-1 只诊断 + ③-2 **关闭** | 2420/2456 = **98.5%** | 0.789 | 9.89 Hz | 无 LOST；与基线一致 |
| Joff | ③-2 lag=1m 打开，escape 关 | 2441/2455 = **99.4%** | 0.791 | 9.9 Hz | 无 LOST（同 D 的配置） |
| G2 | E 同配置 + `max_keyframes: 40` | 2454/2462 = **99.7%** | **0.814** | 9.9 Hz | 无 LOST；匹配质量小幅提升 |
| E2 | E 同配置，但改了测试台"初始位姿"取法 | 496 RECOVERING / **1459 LOST** | — | — | 台架自身引入的偏差，**不算算法结果**（见 §5 坑） |

### Run C 的教训（量纲）
第一版用 `J = [p_w × n, n]`（`p_w` 是世界点，`|p_w| ≈ 100 m`），旋转块被 `|p_w|²` 放大，
`λmin/λmax` 恒 ~1e-5 → 所有帧都判"严重退化" → `gain` 压到 2e-4 → 位姿不更新 → 98% LOST。
改成 `J = [(p_w - t) × n, n]`（与 `pp_icp` 的 `R*bp` 同量纲）后，正常帧回到 ~4e-3。

### Run C2/D 的教训（窗口滞后）
- 正常帧 `eig` 分布：min 1.8e-3 / p50 4.0e-3 / max 9.2e-3，`dirs` 恒 0。
  → 用 1e-3 做 anchor 门限等于"永不放行"，用 0.05 做 partial_update 门限等于"永远压制"。
- C2 里 74/81 帧 `hold`，窗口滞后到 6.7 m，目标建在旧位置 → SUCCESS 85.6%。
- D 里 `hold` 期间匹配被旧窗口拽住，创新量冲到 0.97 m → 被 `max_trans_innov(0.6)` 拒绝 →
  `coast` → 速度先验衰减、odom 被判 slip 拒绝 → **位姿冻在原地**，匹配明明找到了正确位姿
  （`rx=15.37` vs `gx=13.53`）也进不去；`relocalizer: none` 下永不恢复。
- 这条"冻结"链条是现场"整体定位丢失"的可复现机制，但它是**结果**，不是可以靠 lag 修复的原因。

## 3. 本次改动

| 文件 | 改动 |
|------|------|
| `registration_metrics.{h,cpp}` | 新增 `evaluateObservability()`：内点 5-NN 局部平面法向 → `J=[(p_w-t)×n, n]` → 6×6 `H` → 特征分解，返回 `λ、λmin/λmax、退化方向数`（有效点 <30 时返回 false） |
| `matcher/gicp_matcher.cpp` | 匹配后调用它，填 `hess_eigvals/hess_eigvecs/smallest_eig_ratio/degenerate_dirs`；**故意不置 `has_hessian`**（原因见结论 1） |
| `localization_stage_types.h` | `LocalMapConfig` 加 `anchor_hold(false)/anchor_max_lag(1.0)/anchor_min_fitness(0.0)/anchor_min_eig_ratio(1e-3)` |
| `localization_pipeline.{h,cpp}` | 窗口中心可选走 `m_submap_anchor`；`[ObsProbe]` 每 20 帧打印 eig/dirs/innov/anchor_lag；换图/重置/coast 重锚时一起失效 |
| `config/default.yaml` | 显式写出 `anchor_hold: false` 等 4 项 + 实测依据 |

### 观测方式（现场）
```bash
export ROSIWIT_LOC_PROBE=1        # 每 20 帧一条
# [ObsProbe] t=... eig=0.0042 dirs=0 hess=0 fit=0.86 innov=0.03 anchor_lag=-1 anchor_hold=0
```
有了这个 `eig/dirs`，才能按现场数据重标定 `degeneracy_ratio_soft`（建议先统计正常帧与
贴墙/走廊帧的分布，再定 soft 门限），这是后续打开 ③-1 门控的前提。

## 4. 下一步建议（按性价比）

1. **窗口不要变窄，可以适当变宽**：现场 `max_keyframes: 24`。本轮把窗口加到 40
   （Run G2）：SUCCESS 99.7%（与基线持平）、`matching p50` 0.790 → **0.814**、无 LOST，
   输出仍 9.9 Hz。方向正确但收益不大，建议现场 A/B（24 vs 32~40）后再定；
   若"锚偏"来自重复几何（走廊/货架），宽窗口能让局部地图包含唯一结构，比"让窗口滞后"安全得多。
2. **冻结逃逸（已实现，见 §6）**：给"连续 K 帧匹配自洽但创新量超限"加一条受控跳变
   （K 帧互相一致 + fitness 正常才放行）。它直接针对 §2 里复现的"位姿冻死"，
   且不动 fitness 判据、不看点数。
3. 重标定 `degeneracy_ratio_soft` 后再打开 ③-1 门控（C2 已证明"能算出来"，只是阈值不对）。

## 6. 冻结逃逸（2026-11-25 实现）

### 规则
`accept.escape_*`（默认开）。一帧算"候选"需同时满足：

- 本帧**没被接受**（所以不改变正常路径）、`converged`、不在 seed 待校验阶段；
- `fitness >= accept.min_coverage`（就是模式 0 的接受门限，**不新增阈值**）；
- 与当前预测的位移 `<= escape_max_jump`（默认 8 m，防跨图乱跳）。

候选**彼此一致**（这才是关键保护：错误匹配在相似结构间通常帧间乱跳）：

- 相邻帧位移 `<= escape_consist_trans`（0.20 m）、姿态差 `<= escape_consist_rot_deg`（5°）；
- 相对本轮首个候选的累计离散 `<= escape_max_spread`（0.45 m）。

连续 `escape_frames`（6 帧）+ 已连续拒绝 `escape_min_coast`（8 帧）后**放行一次**：

```
out_pose = 匹配位姿   accepted = true   reject_reason = "escape"
prior->reset / fusion->reset 到该位姿；maha_bypass=5；coast=0；[Escape] 日志
冷却 escape_cooldown_s(3s)
```

### 判据为什么这么定
- 只看 `min_coverage`：现场对接工位 200 点也能满分，不能按点数打折；
- "彼此一致"：门控本来就是在防"单帧被拽歪"，这里只放行**多帧都指向同一处**的解；
- 冷却 + 最大跳距：避免在两个相似解之间来回跳。

### 验证（同一 `loc_sessionmap.bag`）
| Run | 配置 | 结果 |
|-----|------|------|
| H2o | 初始位姿故意偏 +2.5 m，`escape_enabled: false` | LOST 1920/1943（永久锁死） |
| H2e | 同上，`escape_enabled: true` | LOST 1881/1904（**没救回来**） |
| Joff | `anchor_hold: true`（escape 关） | 99.4%（台架没复现出漂移） |
| Jon | 同上，escape 开 | 见 §7 |

**H2 的负结果很有价值**：初始位姿一次偏 2.5 m 时，局部目标窗口是**按这个错位姿建的**，
gicp 根本找不到真值（`fitness` 一路掉到 0.000，`inliers` 掉到 0）。
这时"连续一致的候选"根本不存在，逃逸无候选可用 —— **大跳变这类故障只能靠全局重定位，
不是逃逸能解决的**。逃逸的适用场景是**窗口仍然覆盖真值**（即"锚偏/慢漂"）：匹配能找到
真值、只是被单帧创新量门限挡在门外。

### 用真实失败日志回放判据（最硬的证据）
把每帧日志里的 `accepted / fitness / 匹配位姿 / 预测位姿` 取出来，按 C++ 里同一套规则
离线跑一遍（`DropProbe` 每帧都带 `accepted=`，所以不需要近似）：

| 日志 | 帧数 | 触发逃逸 | 说明 |
|------|------|----------|------|
| A / E / G2 / Joff | 2038 / 2048 / 2067 / 2046 | **0 / 0 / 0 / 0** | 正常行驶 8000+ 帧，**零误触发** |
| D（真实失败） | 282 | **1** | t=…3947.0，jump **1.92 m**、fit 0.85、inl 2856 —— 正是"位姿冻在 13.53、匹配找到 15.37"那一帧 |
| H1（台架偶发） | 1268 | 0 | 错锁是**被接受**的（fit 0.85），不属于"被拒但稳定"，逃逸不该管 —— 对症 |

结论：判据在真实失败点上**正好**触发，在 4 次正常跑里一次都不触发。


## 5. 复现方式与坑

```bash
# 设备风格配置 + 指定 tag 跑一轮（驱动脚本在临时目录，不进仓库）
bash run_anchor_tagged.sh E2      # 基线（anchor_hold=false）
MAXKF=40 bash run_anchor_tagged.sh G2
```

**坑（别踩）**：回放是"先起 `rosbag play`、后 switch_map / 对齐位姿"，
所以时间轴上 switch_map 用 bag 第一帧位姿、`[5]` 再用"当前 sim 时刻"位姿补一次，
两次之间节点会有一段积压帧。**不要**把 switch_map 的 initial_pose 改成"当前 sim 时刻"：
那会让两次位姿差从 ~5 m 变成 ~8 m，节点积压帧按新位姿匹配直接崩（实测 SUCCESS 99% → 25%，
Run E2）。要彻底对齐，得让回放先暂停、等地图就绪+位姿注入后再放（未实现）。
