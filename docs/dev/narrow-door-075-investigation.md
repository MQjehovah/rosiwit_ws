## 最终结果 (2026-11-02): 0.80 m 窄通道 3/3 干净通过

口径: footprint = 真实碰撞体 0.87 x 0.70 (x 偏移 0.119); 窄通道 = 车宽 + 10 cm = 0.80 m。
靶场: `world/doorway.world` 门洞 0.80 m, `maps/doorway/map.pgm` 由生成器重建
(`scripts/sim/gen_doorway_map.py`, 可复现)。

| run | status | xtrack_rms | 门平面 x | 斜置 | 门带内安全状态 | 耗时 |
|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 4 mm | +0.001 | 0.6° | 全程 PASS (无减速) | 23.2 s |
| 2 | SUCCEEDED | 2 mm | -0.005 | 0.2° | 全程 PASS | 23.0 s |
| 3 | SUCCEEDED | 1 mm | +0.002 | 0.1° | 全程 PASS | 22.1 s |

几何判定 (`scripts/sim/analyze_trace.py`, 车体角点 vs 门框半宽 0.40):

```
gate_verdict[collision_0.87x0.70]: worst_corner_x=0.3525 -> PASS  (余量 4.8 cm)
gate_verdict[visual_0.87x0.718]:    worst_corner_x=0.3615 -> PASS  (余量 3.9 cm)
```

`min_clearance_m = 0.400` = 门洞半宽, 即车体中心与门洞中心线重合。

## 让 3/3 通过的最后一块: 定位精度, 不是控制

之前 5 次里有 4 次是"跟踪 1~3 mm、斜置 0.1~1° 却被安全层 STOP"。根因通过两条独立测量锁定:

1. **odom vs Gazebo 真值**: 2.4 m 行程累积横向偏差 1.8 → 9.0 cm (dx 随里程线性增长,
   约 2° 航向偏置), 来自轮式编码器积分 (`odometrySource: encoder`)。
2. **安全层命中点**: 世界坐标 (0.341, 2.950), 而门框在 x >= 0.40 —— 相差 6.6 cm,
   正好等于当时的 odom 漂移量。雷达看到的是**真实**门框, 是位姿错了 →
   安全层"以为"门框进门洞 6.6 cm → 正确停车。

0.80 m 门洞单边只剩 5 cm, 9 cm 的定位误差下这个任务**无解**。改用 Gazebo 真值
(`<odometrySource>world</odometrySource>`, 仿真里做控制器/安全层验收的标准做法) 后
一次通过 3/3。

### 排障期间修掉的仿真坑 (都会伪装成"导航不行")

- **xacro 注释里的 `--`** → XML 非法 → `xacro` 失败 → 没有 `robot_description`
- **xacro 里的中文注释** → `spawn_model` (python2, `xml.etree`) 抛 `UnicodeEncodeError`
  → 模型静默不生成 → 没有 `/odom`、没有 TF → `FAILED_POSE`
  (两个坑合起来表现为"启动抖动 / no map / FAILED_POSE", 排查了很久)
- `odometrySource: encoder` 的 9 cm 漂移 (见上)

## 仍然要注意

- 上述结论是在"位姿真值"下取得的。实车用 LIO, 定位精度约 2~5 cm; 0.80 m 门洞
  (单边 5 cm) 对定位的要求是 <=2 cm, 所以**实车必须验证定位精度**, 否则同样会
  表现为"跟踪很好但被安全层停"。
- `odometrySource: world` 只用于仿真验收; 量产/回放不要依赖。

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

## 最新一轮（指数膨胀 + 规划器只用位置）

| 指标 | 值 |
|---|---|
| `goal_status` | **SUCCEEDED**（终点 `(0.012, 3.613)`，`yaw_err 0.001`） |
| `xtrack_rms` | **0.028 m**（历史最好） |
| 门平面处 (y=2.90) | `x=-0.025`、`yaw=85.1°` → 斜置 **4.9°** |
| 门带内 | 有一次 STOP（60 拍 ≈ 1 s），随后继续通过 |
| 结论 | **目标送达，但不是"干净直穿"**：4.9° 斜置下，0.70 m 碰撞体的横向占用 `0.554·sin(4.9°)+0.35 = 0.397 > 0.375`，实际大概率擦着门框过（Gazebo 接触容许了少量穿透） |

按几何，**干净通过要求斜置 ≤1°**。当前残差来自起步瞬态（rotate→drive 后
仍有 8~9° 航向差，收敛到门前只剩 4.9°）。

### 下一步（按优先级）

1. **先统一验收口径**：把 Gazebo 碰撞体从 0.70 改到 0.66（`gt_robot.xacro`
   的 `base_collision_box2` 宽），否则我们在测一个比需求难 80% 的题；
2. **起步瞬态**：`rotate_to_heading` 目前用 rotation_shim 的
   `angular_disengage_threshold 0.3925`（22.5°）就交接，导致起步带 20° 航向差
   走弧。窄口场景应转到 ≤0.10 rad 再起步（这也是很多 AGV 的"door mode"做法）；
3. **终末旋转**：目标 yaw 与进入方向差 90° 时，目标位置要离门 ≥1 m。


**唯一剩下的问题：门带内的斜置**（要求 ≤1°，实测最好 3°），以及门后 0.5 m 处
再做 90° 原地转时空间不够（目标 yaw=0 时）。

实测两轮（目标 (0,3.8)，vmax 0.30）：

| 目标 yaw | 结果 | 门平面处 | 备注 |
|---|---|---|---|
| 0（+x） | `FAILED_SAFETY_STOP` | y=2.90 时 x=-0.036、斜置 3.0°；最终到达 (0.081, 3.653)，xy_err 0.168 | **已穿门**，失败在门后 0.5 m 的 90° 终末旋转 |
| 1.571（+y） | `FAILED_RECOVERY` | 停在 y=2.46、x=0.059 | 该轮起步瞬态更差（xtrack_max 0.28） |

两轮差异说明：**剩余不确定性来自起步瞬态**（rotate→drive 后的横向摆动），
而不是门洞本身。离线台（直线路径 + 理想点云）任何参数下都能过（vmax 0.45 时
门内斜置 0.25°），所以差距在"规划路径 + 起步瞬态"这一环。

### 下一步（按标准做法，都能秒级验证）

1. **门区 lane 直穿 + 代价势场**：把 inflation 从线性 `1-d/r` 换成 nav2 的
   `253·exp(-cost_scaling_factor·(d - inscribed))`（tuning guide 推荐的
   "gentle potential field"），让规划器自己在门口就走中轴，门前方 1 m 内是直线。
2. **起步瞬态**：`rotate_to_heading` 之后用 `angular_disengage_threshold` 的一半
   再确认一次（或直接转到 ≤0.1 rad 再起步），把起步摆动压到 <3 cm。
3. **终末旋转**：目标 yaw 与进入方向差 90° 时，先用 `plan_goal_yaw_from_bearing`
   规划（已加，默认开），到点后再 final_rotate；门后 0.5 m 不够转，需要把目标
   放到门后 ≥1 m（或末端允许小幅倒车让出空间）。

## 复现命令

```bash
cd ~/rosiwit_ws/ros1_ws && catkin_make -DCATKIN_ENABLE_TESTING=1
./devel/lib/rosiwit_navigation_ros1/test_door075_passage   # 秒级端到端
./devel/lib/rosiwit_navigation_ros1/test_rpp_alignment
./devel/lib/rosiwit_navigation_ros1/test_safety_monitor

# 实仿真 (~3 分钟)
bash scripts/sim/door_trace.sh
```


# ============================================================================
# 实车部署记录 (2026-11-02)
# ============================================================================

## 三方几何对照 (footprint vs URDF vs 实车)

| 口径 | x 范围 | 宽 | 来源 |
|---|---|---|---|
| **实机模型 (真值)** | −0.327 … +0.565 | **0.717** | `urdf/meshes/gt/gt.dae` 顶点包围盒 (x ±0.4459, y ±0.3585) 经 `base_mesh_xyz=(0.119,0,−0.11)` 摆放 |
| Gazebo 碰撞体 | −0.316 … +0.554 | 0.700 | `gt_robot.xacro: base_collision_box2 = 0.87 0.70 0.66` |
| **导航 footprint (部署后)** | −0.327 … +0.565 | **0.717** | `default.yaml` / `safety_footprint` / `movebase_robot_footprint` |
| 旧 footprint (历史) | −0.25 … +0.55 | 0.660 | 比实车窄 **5.7 cm** → 安全层每边多算 2.85 cm |

结论: 旧声明比实车窄 5.7 cm, 属于真实安全风险; 现统一按实机模型包络。
窄通道口径 = 车宽 + 10 cm = **0.82 m**; 仿真靶场用的是 0.80 m (更严, 已 3/3 通过)。

## 部署内容与路径

| 项 | 路径 | 说明 |
|---|---|---|
| 节点二进制 | `/opt/ros/melodic/lib/rosiwit_navigation_ros1/rosiwit_navigation_ros1_node` | aarch64, 在车端 `catkin_make` 生成后 `install` |
| 算法核心 .so | `/opt/ros/melodic/lib/librosiwit_navigation.so` | **必须一起装** (见下) |
| 头文件 | `/opt/ros/melodic/include/rosiwit_navigation/` | 同步, 防止以后编译拿到旧头 |
| 参数 | `/opt/ros/melodic/share/rosiwit_navigation_ros1/config/default.yaml` | launch 用 `$(find rosiwit_navigation_ros1)/config/default.yaml` → **安装副本**, 不是工作区那份 |
| 兼容层参数 | 同目录 `movebase_compat.yaml` | overlay, 会覆盖 `movebase_robot_footprint` |
| 源码 | `~/rosiwit_ws/src/rosiwit_navigation`, `~/rosiwit_ws/ros1_ws/src/rosiwit_navigation_ros1` | 无 git, 用 tarball + 备份 |
| 备份 | `~/rosiwit_ws/backup_navfix_<ts>.tgz`, `/opt/.../*.bak_<ts>` | 回滚用 |

## 坑 1: 只装二进制 -> 启动即 SIGSEGV

现象: `[nav] FATAL signal 11`, 在 `[INFO] Hybrid A* initialized` 之后, roslaunch 反复重启。

原因: core 编成**动态库** `librosiwit_navigation.so`; 运行时 `LD_LIBRARY_PATH=/opt/ros/melodic/lib`
优先, 加载的是 10 月 30 日的**旧 .so**(没有本次新增的 `setCostScalingFactor` /
`setInscribedRadius` 虚函数), 而新节点调用它们 -> **虚表错位** -> 段错误。

判据: `nm -D --defined-only 旧.so | grep -c setCostScalingFactor` = 0, 新 .so = 2。
修正: **二进制 + .so + 头文件 + 配置四件套一起装**, 停服务后再覆盖 (否则 `Text file busy`)。

## 坑 2: 参数改了不生效

launch 里是 `$(find rosiwit_navigation_ros1)/config/default.yaml` → 解析到 **`/opt/ros/melodic/share`**,
工作区里的 yaml 改了不影响运行。必须同步安装副本; 且 `movebase_compat.yaml` 会覆盖
`movebase_robot_footprint`(旧梯形)。

## 部署后核对 (实测)

```
服务            active, node pid 稳定 (5s 内不变), 近 5 分钟 FATAL 0 次
footprint       [-0.327,-0.359, 0.565,-0.359, 0.565,0.359, -0.327,0.359]
safety_footprint        同上
movebase_robot_footprint 同上
safety_body_margin 0.02 / side 0.03 / turn 0.05
door_lane_enabled true, lane_width 0.85, lane_max_speed 0.18, lane_max_omega 0.03
rotate_to_heading_min_angle 0.10, inflation_cost_scaling_factor 0.0 (保持旧线性势场)
/robot_footprint 发布 4 点 (新矩形)
/nav_diagnostics 带 path_turn_deg (证明跑的是新二进制), state=IDLE, safety=PASS
/scan 15 Hz
```

## 真机待验证 (下一步)

1. **定位精度**: 0.82 m 门洞要求横向定位误差 <=2 cm。先直行 3~5 m 量 LIO 与真值/回环偏差。
2. **窄道 lane 行为**: `lane_width 0.85` 是否合适 (车宽 0.717 + 13 cm)。过宽会在普通走廊里冻结转向。
3. 安全层 STALE/STOP 频率: `safety_body_margin` 从 0.05 降到 0.02 后, 自车回波是否仍被正确排除。


# ============================================================================
# 图纸定稿 (2026-11-02, 官方工程图, 单位 mm)
# ============================================================================

## 车体外形 (取代之前按 mesh 反推的值)

| 项 | 图纸 | 说明 |
|---|---|---|
| 总长 | **906.23** | 含前后突出件 |
| 车宽 | **676** | 侧面本体宽度 (顶部视图另一处标 696.67 为含侧刷/护边的最大包络) |
| 底盘长 | 846.74 | 视图 A (底面) |
| 驱动轮轴 → 后端 | **261.74** | |
| 驱动轮轴 → 前端(底盘) | 585 | 261.74+585=846.74 |
| 驱动轮轴 → 前端(总长) | 644.49 | 906.23-261.74 |
| 轮距 | 405 | URDF `wheel_joint_y=0.19` → 380, 差 25mm |
| 后轮中心距 | 476 | |
| 2D 激光 → 驱动轮 | **494** | URDF 是 **470** → 差 **24mm** (扫描点会整体前后偏 2.4cm) |
| 3D 激光 → 驱动轮 | 517.54 | URDF 0.4796 ✔ |
| 前相机 → 驱动轮 | 533.85 | |
| 后相机 → 驱动轮 | 194.8 | |

**footprint (base_footprint = 驱动轮轴)**:
```
x [-0.262, +0.644]   y [-0.338, +0.338]
= [-0.262, -0.338, 0.644, -0.338, 0.644, 0.338, -0.262, 0.338]
```
窄通道口径 = 676 + 100 = **776 mm** → 仿真靶场用 **0.78 m** (世界几何 0.78,
地图 0.05 栅格下显示 0.80; 规划看地图、安全层看真实雷达)。

### 与旧值的差异 (为什么之前"costmap 上贴墙、实车还离得远")

| | 旧 (mesh 反推) | 图纸 | 差 |
|---|---|---|---|
| 宽 | 717 | 676 | 每边 **+20.5mm** |
| 后端 | -327 | -262 | **+65mm 过长** |
| 前端 | +565 | +644 | -79mm 过短 |

旧的径向顶点半径 = hypot(0.565,0.359) = 0.669; 图纸 = hypot(0.644,0.338) = 0.727。
"看起来贴墙"还有两个叠加因素:
1. `inflation_radius: 0.62` —— RViz 里膨胀色块从墙面铺开 0.62m, 车体在 0.5m 处
   也会视觉重叠 (那是软代价, 不是碰撞)。
2. 车在充电桩上时, 静态地图里有桩体(低于 2D 雷达平面, 雷达看不到), footprint 会
   与**地图里的**格重叠 —— 实测 map/scan 对齐中位数 **0.023m**, 说明地图是对的。

## 0.78m 靶场验收 (车宽 676)

run1: SUCCEEDED, xtrack_rms **2mm**, 门平面 x=+0.002 / 斜置 **0.0°**,
      全程无 SLOWDOWN; 最外角点 **0.3406 vs 门框 0.39 -> 余量 4.9cm**
run2: 起步瞬态失败 (与门无关, 见"遗留")
