# 控制器对比（第一轮）— 2026-11-01

> 状态：脚手架跑通；`house_short` 四个控制器各 1 次；`doorway` 场景**未跑通**（见第 3 节）。
> 结论性数字仍需多次重复（每档 3 次）与 `free` profile。

## 1. 方法

- `house_short`：house.world，起点 (0,0,0)，固定终点 **(1.0, 2.0)**，`clean` profile
- `doorway`：新增 `world/doorway.world`（0.9 m 门洞，门在 y=3，起点 (0,0)，终点 (0,6)），`clean`
- 每次：`./scripts/sim/compare_controllers.sh`（控制器名/profile 用临时 yaml 覆盖，
  指标来自 `/nav_diagnostics` + `/cmd_vel` + `/global_path` + `/map`）

## 2. house_short 结果（每控制器 1 次）

| 控制器 | 结果 | 到点 xy | 到点 yaw | 用时 | 均速 | 峰值 v | 单拍 Δv | 单拍 Δω | xtrack RMS | 最小间隙 | safety_scale |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `rpp` | SUCCEEDED | 0.177 m | −0.193 rad | 15.5 s | 0.338 | 0.741 | 0.107 | 0.094 | **0.036** | **1.005 m** | 1.00 |
| `mppi` | SUCCEEDED | 0.172 m | −0.040 rad | 36.1 s | 0.181 | 0.476 | **0.085** | **0.083** | 0.060 | 0.906 m | 1.00 |
| `teb` | SUCCEEDED | **0.091 m** | 0.159 rad | **13.1 s** | 0.504 | 0.800 | 0.560 | 0.067 | 0.134 | 0.854 m | 0.70 |
| `dwa` | SUCCEEDED | 0.201 m | −0.042 rad | **149.8 s** | 0.050 | 0.769 | 0.150 | 0.086 | 0.161 | 0.906 m | 1.00 |

读法：

- 四个控制器都能到点（xy ≤ 0.20 m），**短距场景下控制层不是瓶颈**。
- `teb` 最快（13.1 s）且到点最准（0.091 m），但 `Δv 0.56` 是 `rpp` 的 5 倍 —— 带每拍重解导致首段速度跳；
  同时 `safety_scale 0.70` 说明安全层减速过一次（`min_clearance 0.854 m`，是缓降区触发，不是贴墙）。
- `mppi` 最平滑（Δv 0.085）但只有 0.181 m/s 均速；`rpp` 速度/精度/平滑三者最均衡。
- `dwa` 仍然是 0.05 m/s（149.8 s）。**速度项翻转的修复没有解决它** —— 说明真正的限速不在代价函数，
  而在 `dynamicWindow()`（`b.max_vx = min(max_vx_, vel.linear_x + max_accel_*dt_)`）：若 `dt_` 很小，
  单拍可达速度增量就小，速度永远爬不起来。下一步先打印 `dt_/max_accel_` 实参。

## 3. doorway：场景未跑通（重要）

`rpp` 与 `teb` 都在 doorway 场景 **TIMEOUT，均值速度 0.000**（210 s 完全不动，`xy_err 5.96 m`）。
两个完全不同的控制器表现完全一致 → 问题不在控制器，而在场景/上游（安全层 STOP、SLAM 没地图、
或 spawn/世界配置）。下一轮先单独诊断 doorway（看 `/nav_diagnostics` 的 `safety`/`map_w`）。

顺带：`min_clearance_m` 在这两个 run 里输出 `1e9` —— 我的实现 `return None if best > 1e9 else best`
在"20 格内没有任何 lethal 格"时返回 1e9 而不是 None，属于指标 bug，要改成 `>= 1e9`。

## 4. 复现

```bash
CONTROLLERS="rpp mppi dwa teb" SCENES="house_short" RUNS=3 ./scripts/sim/compare_controllers.sh
CONTROLLERS="rpp teb" SCENES="doorway" RUNS=1 ./scripts/sim/compare_controllers.sh
```

CSV：`~/sim_verify/results_compare.csv`。
