# 控制器对比（第一轮）— 2026-11-01

> 状态：**脚手架跑通，样本量 1 次/控制器**。这张表用来证明对比链路可用、并给出第一批数字；
> 结论性结论需要 Task 7 的 4×2×5×3 = 120 次矩阵。

## 方法

- 场景：`house_short`（house.world，起点 (0,0,0)，固定终点 **(1.0, 2.0)**），`--goal 1.0,2.0` 跳过 BFS 选点
- profile：`clean`（四个控制器都拿单向路径 + 末端原地转）
- 每个控制器 1 次：`CONTROLLERS="rpp dwa teb" SCENES="house_short" RUNS=1 ./scripts/sim/compare_controllers.sh`
- 指标来源：`/nav_diagnostics` + `/cmd_vel` + `/global_path` + `/map`
  （`min_clearance_m` = 机器人到最近 lethal 格的距离，20 格窗口内搜索）

## 结果（第一轮）

| 控制器 | 结果 | 到点 xy | 到点 yaw | 用时 | 均速 | 峰值 v | 单拍 Δv | 单拍 Δω | xtrack RMS | 最小间隙 | safety_scale 最小 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `rpp` | SUCCEEDED | 0.183 m | −0.198 rad | 15.6 s | 0.341 | 0.744 | 0.112 | 0.104 | **0.030** | **1.02 m** | 1.00 |
| `dwa` | SUCCEEDED | 0.191 m | −0.017 rad | **145.7 s** | 0.046 | 0.777 | 0.15 | 0.121 | 0.217 | 0.90 m | 1.00 |
| `teb` | SUCCEEDED | **0.045 m** | 0.158 rad | **12.0 s** | 0.506 | 0.800 | 0.600 | **0.063** | 0.107 | 0.949 m | 0.75 |

## 读法（暂定，样本 1 次）

- **三者都能到点**（xy ≤ 0.19 m），说明这条短距场景下"控制层"不是主要瓶颈 —— 与之前
  `rpp` 的多次结果一致。真正的差异在速度/平滑/稳定性。
- `teb` 本轮最快、到点最准，但 `Δv 0.60` 明显大于 `rpp`（0.11）：带每拍重解会让首段速度跳。
  需要把速度/加速度项真正放进 GN，而不是只在代价里算。
- `dwa` 在本场景退化到 0.046 m/s（145 s）：它的速度项仍是"越快代价越高"的写法，
  这一轮没来得及修（补丁未命中，见 `progress.md`）。
- `teb` 的 `safety_scale 0.75` 说明安全层减速过一次；`min_clearance 0.95 m` 说明不是贴墙 ——
  多半是减速区（1.15 m）内的点触发了 slow-down，属于预期行为但要和 `rpp`（1.02 m 未触发）对比确认。

## 未完成

1. `mppi` 还没进矩阵（同一脚本加 `CONTROLLERS="... mppi"` 即可）
2. 每控制器只跑了 1 次 → 无法给出方差；`teb` 此前有过 1 次 `FAILED_PATH_FOOTPRINT`，必须多次重复
3. 场景只跑了 `house_short`；`corridor` / `house_long` / `doorway` / `blocked` 未跑
4. `doorway.world` 还没建（0.9 m 门洞）
5. `free` profile 未跑（TEB/RPP 会拿到带倒车的双向路径）

## 复现

```bash
CONTROLLERS="rpp mppi dwa teb" SCENES="house_short house_long corridor" RUNS=3 \
  ./scripts/sim/compare_controllers.sh
```

结果写 `~/sim_verify/results_compare.csv`（列：scene, profile, controller, run, status, xy/yaw 误差,
用时, 速度统计, Δv/Δω, xtrack, min_clearance, safety_scale, replan 计数）。
