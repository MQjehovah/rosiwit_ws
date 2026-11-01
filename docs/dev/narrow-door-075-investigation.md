# 0.75 m 窄门通行排查记录 (2026-11-01)

硬指标：车体宽 0.66 m，必须通过 **0.75 m** 门洞。

## 结论速览

| 层 | 结论 | 证据 |
|---|---|---|
| 安全层几何 | **原实现下 0.75 m 门洞数学上不可通过** | `sweep_margin = max(stop_margin 0.05, side_margin 0.03)` 各向同性外扩 → 侧向半宽 0.373 m ≈ 门洞半宽 0.375 m |
| 安全层几何 (修后) | 侧向用 `side_margin`，前/后用 `stop_margin` | `test_safety_monitor` 新增 3 条门洞契约测试 |
| 路径居中 | 逐点梯度上升会把路径变成锯齿 (±8 cm/4 cm 间距) | 改为"法向扫描取中轴中点 + 沿路径平滑"，`test_path_centering` 8 条 |
| 控制器跟踪 | 起步带 28° 航向差直接开走 → 切出 0.2 m 弧 → 到门平面仍剩 6~7° 斜置 | `test_rpp_alignment` 闭环复现 + 修复 |
| 完整链路 | 干净接近时**可以过** 0.75 m | `test_door075_passage` 4 条：控制器 + 安全层 + 门框点云 + 运动学积分 |
| 实仿真 | 仍未干净通过 (见下) | 首轮停在 y≈2.3 (门前 0.6 m) |

## 关键几何 (算出来的，不是估的)

车体 0.80 m 长 × 0.66 m 宽，门洞半宽 0.375 m。斜置 θ 时车体横向占用：

```
half_extent = 0.55·sin(θ) + 0.33·cos(θ)
θ = 2.3°  → 0.375   (边界)
θ = 4°    → 0.388   (塞不进)
θ = 6.3°  → 0.403   (塞不进)
```

**所以门带内车体斜置必须 < 2.3°**（含安全层 side_margin 时 < 1.5°）。这不是调参能绕过的物理约束。

## 本轮修掉的 4 个确定缺陷

1. **扫掠区余量各向同性**（`safety_monitor.cpp`）
   `margin = max(stop_margin, side_margin)` 沿径向均匀外扩：5 cm 的"刹车余量"被同时加到车身两侧 →
   `0.33 + 0.05 = 0.38 > 0.375`，任何跟踪精度都不可能通过。改为前/后用 `stop_margin`、
   侧向用 `side_margin`（包围盒中心各向异性缩放，保持车体形状比例）。

2. **`side_margin` 在扫掠分支从未被使用**
   只有 `buildDynamicStopPolygon` 用它，而 `dynamic_geometry` 下那个函数的结果被扫掠结果覆盖 → 死参数。

3. **`safety_slowdown_extra` 写进没人读的字段**
   扫掠减速实际用硬编码的 `slowdown_lookahead = 0.8`。新增 `safety_slowdown_lookahead` 并接线。

4. **`min_lookahead_distance` 永远不生效**
   rpp 里 `L = clamp(0.35 + 0.8|v|, la_min, la_max)` 的 0.35 硬底 > 任何合理 min 值。
   改为 `L = clamp(lookahead_offset + lookahead_gain·|v|, la_min, la_max)`，并把 gain/offset 接到参数。

## 本轮新增的行为

- **`core/path_centering.hpp` 重写**：法向一维扫描找"间隙最大的横向偏移"（取平台区中点，避免偏向
  进入侧），再沿路径做 [0.25,0.5,0.25] 平滑 + 相邻点斜率限幅。开阔处（间隙充足）不动。
- **起步先对准**（rpp `prealign_heading_error`，默认 15°）：航向差过大且车没动时先原地转，
  不要用"边转边走"的弧去消化几十度。
- **调速 (regulated)**：`v *= clamp(1 - k_y|e_y| - k_h|e_θ|, floor, 1)`，偏了先减速。
- **窄口不打舵**：`tight_arc_straighten`（弧线会蹭时按弧线安全比例压 ω）+ `tight_lateral_slack`
  （车身左右余量 < 6 cm 时按比例压 ω）。原因：安全层把"制动期间继续以当前 ω 旋转"也算进横向占用，
  ω=0.24 rad/s × 0.41 s = 5.6° → 横向 0.399 m > 0.375 m → 必停。
- **初始旋转改用目标方位角 + 转完重规划**：原来用"路径第一个方向"，而规划器从当前朝向起步，
  两者几乎相等 → 该分支形同虚设。

## 离线门洞回归台 (秒级，替代 4 分钟仿真)

`src/rosiwit_navigation/test/test_door075_passage.cpp`：rpp 控制器 + `core::SafetyMonitor`
（扫掠安全区）+ 门框点云 + 单极模型积分，断言：

1. 居中直行：不 STOP、不越界、通过；
2. 起点朝 +x（需转 90°）：先转后走，仍通过；
3. 起点横偏 5 cm：在门口前收敛，通过；
4. 起点 4 cm 横偏 + 2° 航向差：通过（锁 ω 抑制）。

## 已知遗留

- **实仿真首轮仍停在 y≈2.3**：此时 x≈0.03、斜置 ≈1°，但控制器仍在打 ω 收敛；
  安全层扫掠把 ω 的旋转也计入 → 判撞。ω 抑制已加，最近一轮已能到 `SUCCEEDED`，
  但那一次是 recovery 期间轮子打滑导致 odom 漂移（`xtrack_max 0.85`、`min_clearance 0`），
  **不是干净的通过**，不能作为验收依据。
- 判据需要升级：以"门带内无 STOP + 车体角点始终在门框内 + 轨迹是直线"为准，
  不能只看 `goal_status == SUCCEEDED`（odom 漂移会造成假成功）。
- 规划器起步段仍会给出 S 形路径（最大折角 70~136°）；门带内路径本身已居中
  (`path_x_at_door ≈ 0.000`)，但门前 0.8~1.5 m 的收敛需要更多行程。

## 复现命令

```bash
# 离线门洞回归 (秒级)
cd ~/rosiwit_ws/ros1_ws && catkin_make -DCATKIN_ENABLE_TESTING=1
./devel/lib/rosiwit_navigation_ros1/test_door075_passage

# 实仿真 (约 3 分钟): Gazebo doorway.world + maps/doorway + cfg_door_test.yaml
bash scripts/sim/door_trace.sh   # 录像 -> /tmp 分析
```
