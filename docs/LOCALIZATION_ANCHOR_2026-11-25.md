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
2. **"子图窗口滞后"（方案 ③-2）实测是错的方向，默认关闭。**
   窗口一旦滞后于当前估计，gicp 会把扫描往旧窗口拽，位姿被拉回去，然后
   anchor 又跟着这个被拉偏的位姿更新 —— **自己制造"自洽锁歪"**。
   实测：lag=8m + 可信门限 → 85.6%（末尾 LOST 32 s）；lag=1m → 开局 4.7 s 就 LOST
   且 191 s 内无法恢复（见 Run C2 / D）。关掉后完全复现基线（Run E）。
3. 因此本次提交只保留：**③-1 只读诊断** + **③-2 配置开关（默认 false）**。
   `anchor_hold=false` 时目标建图路径与改动前完全相同（Run A 与 Run E 统计一致：
   99.8% vs 98.5%、p50 0.790 vs 0.789、均无 LOST）。

## 2. 实验矩阵（同一 `loc_sessionmap.bag`、设备风格配置）

| Run | 代码/配置 | SUCCESS | matching p50 | 帧率 | 说明 |
|-----|-----------|---------|--------------|------|------|
| A | 基线（无本次改动） | 2427/2431 = **99.8%** | 0.790 | 9.84 Hz | 无 LOST |
| B | 基线 + `radius_submap` | 80.6% | 0.854 | — | 初始锁定 37.7 s，不采用 |
| C | ③-1 门控打开（J 用了绝对世界点，量纲错） | 45/1927 = **2.3%** | 0.500 | 3.5 Hz | `eig=1.3e-5`、`dirs=3`，修正被清零 |
| C2 | ③-1 门控开（J 修好）+ ③-2 lag=8m/可信 0.85 | 2021/2360 = **85.6%** | 0.777 | 9.06 Hz | 末尾 LOST 32.4 s（161.9–194.4 s） |
| D | ③-1 只诊断 + ③-2 lag=1m/min_fit=0 | **LOST 1812** | — | 1.36 Hz | 4.7 s 起一直 LOST，重定位循环拖慢 |
| E | ③-1 只诊断 + ③-2 **关闭** | 2420/2456 = **98.5%** | 0.789 | 9.89 Hz | 无 LOST；与基线一致 |
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
2. **冻结逃逸**：给"连续 K 帧匹配自洽但创新量超限"加一条受控跳变（K 帧互相一致 + fitness
   正常才放行）。它直接针对 §2 里复现的"位姿冻死"，且不动 fitness 判据、不看点数。
3. 重标定 `degeneracy_ratio_soft` 后再打开 ③-1 门控（C2 已证明"能算出来"，只是阈值不对）。

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
