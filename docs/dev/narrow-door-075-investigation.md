# 0.75 m 窄门通行排查记录 (2026-11-01)

硬指标：车体宽 0.66 m，必须通过 **0.75 m** 门洞。

> 重要前提（仿真 vs 需求）：Gazebo 里 `gt_robot.xacro` 的**碰撞体是 0.87 × 0.70**
> (`base_collision_box2`)，即碰撞体宽 **0.70 m**，而导航声明 footprint 是 0.66 m
> (±0.33)。所以这套仿真实际考的是"**0.70 m 车过 0.75 m 门**"——单边只剩
> **2.5 cm**，要求门内斜置 ≤1°。比用户口径（0.66 m → 单边 4.5 cm）严格得多。

## 结论速览

| 层 | 结论 | 证据 |
|---|---|---|
| 安全层 (旧) | **0.75 m 门洞数学上不可通过** | 侧向余量误用 stop_margin 0.05 → 0.33+0.05 = 0.38 > 0.375 |
| 安全层 (新) | 改回 Nav2 `velocity_polygon` 语义：按速度选多边形，前/后用制动距离 + stop_margin，侧向只加 side_margin | `test_safety_monitor` 22 项 |
| 自车回波 | 排除盒 0.01 太小 → 停在自己的轮子/挡泥板回波上 | `[safety] STOP hits b(0.35,-0.35)`，改 `safety_body_margin 0.05` 后命中点变成真门框 |
| 路径居中 | 旧版逐点梯度上升产生 ±8 cm 锯齿；新版法向扫描 + 平滑 | `test_path_centering` 6 项，门处 `path_x = -0.001` |
| 跟踪 | 按 Nav2 RPP 参数重写后 `xtrack_rms` 3.1~4.9 cm → **1.6 cm** | 仿真 4 轮 |
| 门内斜置 | **仍未达标**：最好一次 x=1 cm、斜置 3.3°，要求 ≤1° | 见下 |
| 端到端 | 干净接近可以过 (离线断言 4/4)，实仿真尚未干净通过 | `test_door075_passage` |

## 关键几何（算出来的）

车体长 0.87（碰撞体），门洞半宽 0.375，碰撞半宽 0.35：

```
斜置 θ 时横向占用 = 0.554·sin(θ) + 0.35·cos(θ)     (前 0.554 / 半宽 0.35)
θ = 0°   → 0.350   (余量 2.5 cm)
θ = 1°   → 0.360   (余量 1.5 cm)
θ = 3.3° → 0.382   (已超 0.375 → 必碰)
```

**门带内斜置必须 ≤1°**。这解释了为什么"跟踪误差只有 1 cm"仍然过不去。

## 本轮按行业标准修掉的 5 个缺陷

1. **扫掠区余量各向同性**（`safety_monitor.cpp`）
   `margin = max(stop_margin 0.05, side_margin 0.03)` 沿径向均匀外扩，5 cm 的"刹车余量"
   被加到车身两侧 → 侧向半宽 0.38 > 门槛 0.375 → 任何跟踪精度都不可能通过。

2. **安全层语义对齐 `nav2_collision_monitor`**
   默认改为按速度选多边形（`buildDynamicStopPolygon`：前/后 = 制动距离 + stop_margin，
   侧向 = 半宽 + side_margin + 转向余量）；旧的"把 footprint 沿预测轨迹扫掠、并把
   制动期间继续以 ω 旋转也算进去"保留为可选 `safety_swept_geometry`（**默认关**）。
   Nav2 不做这个外推 —— 它会与窄通道直接冲突（0.4 s 内 ω 转 5° 就把扫掠区撑宽）。

3. **`side_margin` 在扫掠分支从未被使用** → 死参数（只有被覆盖的 `buildDynamicStopPolygon` 用）。

4. **`safety_slowdown_extra` / `min_lookahead_distance` / `lookahead_offset` 三类死参数**
   - 扫掠减速实际用硬编码 `slowdown_lookahead = 0.8` → 新增 `safety_slowdown_lookahead` 并接线
   - rpp 的 `L = clamp(0.35 + 0.8|v|, min, max)` 里 0.35 硬底让 `min_lookahead_distance` 失效

5. **自车回波被当成障碍**（致命且隐蔽）
   `safety_body_margin = 0.01` 时，命中点是 base 系 `(0.35, -0.35)` 这类"贴着车身外沿 1 cm"
   的点 —— 是自己的轮子/挡泥板。Gazebo 碰撞体 0.70 > 声明 0.66，所以排除盒必须按真实
   车体外扩。改回 0.05 后命中点变成真门框 `(0.649, -0.345)` / world `(0.388, 2.938)`。

## 按行业参数重写的控制器（rpp → nav2 RPP 语义）

| 参数 | 值 (Nav2 默认) | 作用 |
|---|---|---|
| `use_velocity_scaled_lookahead_dist` / `lookahead_time` | true / 1.5 | `L = clamp(v·1.5, min, max)` |
| `min/max_lookahead_dist` | 0.3 / 0.9 | 前瞻夹紧 |
| `use_rotate_to_heading` / `rotate_to_heading_min_angle` | true / 0.3925 | 等价 rotation_shim 的 `angular_disengage_threshold` |
| `use_regulated_linear_velocity_scaling` / `_min_radius` | true / 0.9 | 曲率调速 |
| `use_collision_detection` | true | 沿指令弧会撞就降速（v,ω 同比例 → 曲率不变） |
| `regulated_linear_scaling_min_speed` | 0.2 | 调速下限，防止压死 |
| `approach_velocity_scaling_dist` / `min_approach_linear_velocity` | 0.6 / 0.05 | 末段调速 |

同时删掉上一轮我自创的 `cross_track_speed_gain` / `heading_speed_gain` /
`tight_arc_straighten` / `tight_lateral_slack` / `prealign_heading_error`（非行业做法）。

## 离线门洞回归台（秒级，替代 4 分钟仿真）

`src/rosiwit_navigation/test/test_door075_passage.cpp`：rpp + 真实 `SafetyMonitor` +
门框点云 + 单极模型积分。4 条全过：
居中直行 / 起点需转 90° / 起点横偏 5 cm / 起点 4 cm 横偏 + 2° 航向差。

`test_rpp_alignment.cpp`：3 条（先转后走存在、直线跟踪 ≤3 cm、需转 90° 起步仍能在门平面 ≤2.3°）。

## 遗留（唯一一条，已量化）

门带内**斜置 3.3°**（要求 ≤1°）。原因是横向偏差在门前方 0.3 m 才收敛完，
收敛过程本身就带 8° 左右的航向；而 Nav2 RPP 的 `la_min = 0.3 m` 决定了
"每米衰减 e^-3.3"，起点瞬态的 ~10 cm 误差到门前只收到 1 cm、航向还没settle。

三条标准解法（下一步选一条，都能在离线台秒级验证）：
1. **门区 lane 直穿**：用代价地图的 inflation 势场（Nav2 tuning guide 推荐的
   "gentle potential field"）+ 规划器平滑，让门前的路径在 1 m 内就是直线，
   而不是靠控制器收敛；
2. **门区降速**：`max_velocity_x 0.2` + `la_min 0.2`（窄口专用），把收敛距离缩到 0.4 m；
3. **前置对准点**：在门前 1 m 插入一个"对准航向"的原地旋转（Nav2 rotation_shim 的
   正常用法），转完再直穿 —— 但要同步把 rotation 安全区从"最远顶点 + 0.10"收到 +0.02。

## 复现命令

```bash
cd ~/rosiwit_ws/ros1_ws && catkin_make -DCATKIN_ENABLE_TESTING=1
./devel/lib/rosiwit_navigation_ros1/test_door075_passage   # 秒级端到端
./devel/lib/rosiwit_navigation_ros1/test_rpp_alignment
./devel/lib/rosiwit_navigation_ros1/test_safety_monitor

# 实仿真 (~3 分钟)
bash scripts/sim/door_trace.sh
```
