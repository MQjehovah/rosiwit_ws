# 导航/定位加固 —— 按成熟项目机制补齐（2026-10-26）

> 输入：`docs/dev` 两轮缺口调研（代码通读 + nav2 / ipa_coverage_planning / move_base 机制对照）。
> 目标：把"已经确认的致命项"和"成熟项目都有而我们没有的机制"一次性补齐，并留可回归的证据。
> 约束：ROS1 Melodic，不改 App 侧协议（`NaviCmdAction` 字段不变），只加参数与内部机制。

---

## 0. 验收总原则

1. 每条改动都要有**能失败的测试**（单测或仿真脚本），不是"改完看起来对"。
2. 安全/指令/恢复三条链路上，**所有** `cmd_vel` 出口收敛到一个函数（`publishCommand`）。
3. 配置漂移必须在**启动 3 秒内**打出来（生效值 vs yaml 值）。
4. 不引入新依赖（不装 nav2/twist_mux，照机制自己实现）。

---

## 1. 安全层（P0，nav2_collision_monitor 机制）

### 1.1 现状问题
- `scan_self_*` 大矩形（`x -0.45..0.75, y ±0.55`）把 `safety_stop_polygon`（`-0.37..0.67, ±0.45`）**整个吞掉** → STOP 永不触发；连带局部代价地图看不到 0.75 m 内的点。
- `safety_self_filter_radius=0.25` 再吃一层 → 0.25 m 内彻底盲。
- 无滞回：`count > stop_max_points` 单帧判定，噪声一抖就 stop/slow 翻转。
- 只有 `/scan` 一个数据源；无"按运动状态切安全区"（原地旋转时车体扫掠区无保护）。
- 覆盖旋转、覆盖出界分支绕过 `applySafety`。

### 1.2 设计（照 nav2 的四件事）
| nav2 机制 | 我们的实现 |
|---|---|
| `exclusion_zones: robot_body` | 新增 `body_polygon`（默认 footprint 外扩 5 cm），**只排除车身轮廓内**的点，不再用大矩形 |
| `stop/slowdown` 多边形 + `min_points` | 保留；多边形起点改到车身外 0.05 m |
| `trigger_consecutive_points` / `release_consecutive_points` | 新增 `stop_trigger_frames/stop_release_frames`（默认 2/3）、`slowdown_trigger_frames/release_frames`（默认 1/2） |
| `VelocityPolygon`（rotation/forward/backward） | 新增 `stop_polygon_rotation`、`stop_polygon_backward`；`\|vx\|<0.05` 用旋转区，`vx<0` 用后退区（默认镜像），否则前进区 |
| `source_timeout` → 停车 | 已有（scan 超时返回零）；补：`source_timeout` 同时覆盖 3D 点云/代价地图源（后续阶段） |

### 1.3 代码改动
- `nav_core/safety_monitor.hpp`：`SafetyMonitorConfig` 加 `body_polygon`、`body_margin`、三个 stop 区、两组 trigger/release；`filter()` 改为**有状态**（`stop_active_`、`slowdown_active_`、计数器），新增 `reset()`。
- `nav_core/safety_monitor.cpp`：先在 `body_polygon` 内做排除，再按运动状态选区、按滞回更新状态。
- `ros1/.../node.cpp`：
  - `scan_self_*` 改为 `scan_body_polygon`（默认取 `configured_footprint_` 外扩 5 cm），保留 `scan_self_*` 作为兼容回退；
  - `safety_self_filter_radius` 默认 0.25 → **0.10**（只做传感器盲区），注释写清"车身轮廓由 body_polygon 排除"；
  - 覆盖旋转 `publishCommand(applySafety(rot))`；OUT_OF_AREA 分支改写后同样过 `applySafety`；
  - 新增参数：`safety_stop_polygon_rotation`、`safety_stop_polygon_backward`、`safety_stop_trigger_frames`、`safety_stop_release_frames`、`safety_slowdown_trigger_frames`、`safety_slowdown_release_frames`、`safety_body_margin`；
  - 诊断里输出 `safety_state/scale` 与三个计数（已在 diagnostics，补滞回状态）。

### 1.4 验收
- 单测（`test_safety_monitor.cpp` 扩展）：
  - 车身内点（0.1, 0.0）**不**触发 STOP；
  - 0.3 m 正前方点 → 连续 2 帧后进入 STOP，1 帧不进入；
  - 进入 STOP 后点数掉到 0，3 帧内保持 STOP，第 3 帧后释放；
  - 原地旋转指令（vx=0）用旋转区判定：把点放在车侧 0.3 m → STOP；
  - 前进指令 + 同样的侧向点 → 不 STOP（旋转区不生效）。
- 仿真：0.3 m 处放一个假点，覆盖率任务必须 `safety=STOP` 停车（脚本 `scripts/safety_zone_check.py`，新增）。

---

## 2. 指令通路（P0，nav2_velocity_smoother + twist_mux 机制）

### 2.1 现状
- 加速度限制在**控制器内部**、`dt` 硬编码 0.1 s → 改 `controller_frequency` 实际加速度翻倍。
- 无 deadband、无 deadman（节点卡死时底盘继续执行最后一条指令）、无多源仲裁。
- `applySpeedLimit` 只钳 `linear.x`，不钳 `angular.z`。

### 2.2 设计
```
nav 控制器 ─┐
teleop ─────┼→ 仲裁（优先级 + 每源 timeout）→ VelocityShaper（vel/accel/deadband）→ /cmd_vel
恢复动作 ───┘                                        ↑
                                     20 Hz watchdog：no cmd > timeout → 零
```
- `VelocityShaper`（`nav_core/velocity_shaper.hpp`，纯逻辑）：
  - 输入：命令、当前时间；
  - 输出：按 `max_accel_x/theta` 与 `dt` 限幅后的命令；
  - `velocity_timeout`：超时后由 watchdog 调 `shape(zero)` 平滑回零；
  - `deadband_velocity`：小于阈值置零。
- 仲裁：`/cmd_vel_teleop`（可配，默认空=关闭）+ `teleop_timeout`；激活时覆盖 nav 命令。
- `applySpeedLimit`：`angular.z` 按同一比例缩放。

### 2.3 验收
- 单测（`test_velocity_shaper.cpp`）：阶跃 0→0.8 在 `max_accel=0.8` 下需要 ≥0.4 s 才到 0.32；`dt` 改 0.05/0.1 结果一致；deadband 生效；超时回零。
- 仿真：`/cmd_vel` 相邻两帧的 `Δv/Δt` 不超过配置（脚本检查）。

---

## 3. 配置自检（P1）

### 3.1 设计
- 启动时打印**生效配置指纹**：
  `profile=clean planner=astar controller=dwa odom=/odom frames=map/base_link keepout=1 safety_stop_pts=...`
- 校验并 `ROS_ERROR`（`strict_config=true` 时直接退出）：
  1. `planner_name` / `controller_name` 能在 factory 里创建；
  2. `motion_profile ∈ {clean, free}`；
  3. 若 `profile=clean` 且控制器为 `rpp`/`mppi`（消费轨迹）而规划器是 `astar` → 提示"轨迹速度将全 0"（根因已修，见 3.2，仍保留提示）；
  4. `map_frame/robot_frame` 与 TF 是否在 3 s 内可达；
  5. `odom_topic` 若为 `/odom` 而 SLAM 发 `/lio_odom` → 明确告警（真机掉线检测会失效）。
- `default.yaml` / `movebase_compat.yaml` 补 `motion_profile`、`clean_planner`、`free_planner`，并统一 planner/controller。

### 3.2 根因修复
- `astar_planner` 输出 `velocity_x=+1.0`（原来恒 0 被轨迹参数化当成旋转段 → 整条轨迹 v=0）；`navfn` 同样处理。

### 3.3 验收
- 启动日志必须打出生效指纹；故意把 `clean_planner` 写成不存在的名字，必须 ERROR。
- 单测：`trajectory_parameterizer` 对 `velocity_x=+1` 的 A* 路径产出非零 `v`。

---

## 4. 恢复（P1，nav2_behaviors + BT 恢复链机制）

### 4.1 设计
- 恢复动作做成 4 个可组合步骤（照 move_base 的 `RecoveryBehavior` + nav2 的 RoundRobin）：
  1. `ClearCostmaps`（新增：重建 planner 与 local 代价地图，清掉假障碍）；
  2. `Spin`（原地转 60°~180°，带安全层）；
  3. `Wait`（5 s，等动态障碍离开）；
  4. `BackOff`（现有盲倒车，改为**带前方/后方实时点云预演**）。
- 上下文恢复：规划失败先清**全局**图，控制失败先清**局部**图，再进 RoundRobin。
- 预算：每个动作有 `time_allowance`，整轮有 `max_recovery_cycles`（默认 3），用完才 `failRecovery`。
- 抢占：新 goal / 取消立即中止恢复（复用现有 generation）。

### 4.2 验收
- `scripts/recovery_check.py` 扩展：
  - 假障碍落在目标上 → 先 `ClearCostmaps`（日志出现），再成功；
  - 后方被墙堵死 → 走 Spin/Wait 而不是直接 `-16`；
  - 恢复动作全部失败 → `-16` 且 `recovery_cycles=3`。
- 单测：恢复状态机的顺序与预算（纯逻辑）。

---

## 5. 覆盖：多房间（P2，ipa_coverage_planning 机制）

### 5.1 现状
- 只接受一个多边形；房间之间的分割/排序靠调用方。
- 好消息：`buildRoute` 的单元分解 + 单元贪心 + 转场 BFS 已经等价于"分割→排序→逐房间"的雏形。

### 5.2 设计（不改 App 协议）
- 新增参数 `coverage_split_doorways`（默认 true）：切片时若区间端点跳变 **或** 区间宽度连续 3 个切片都小于 `2×robot_inset`（门洞），强制切单元 → 保证每个房间是独立单元。
- 单元排序改为 **按区域面积降序 + 最近邻**（避免小房间被留到最后绕远）。
- 离线验证：用 house_sim 整层多边形跑 `coverage_route_dump` + `plan_coverage.py`，要求 ≥95% 可覆盖面积、无跨房间来回跳。
- 后续（不在本次）：形态学/Voronoi 自动房间分割 + 房间级语义（跳过厨房）。

### 5.3 验收
- 离线：整层 20~30 m² 多边形，单元数 ≥2、覆盖率 ≥95%、路线中"跨单元往返"次数 ≤ 单元数。

---

## 6. 地图保存（P2，SLAM 侧）

### 6.1 设计
- `MapMeta` 加 `version`（内容 hash 前 8 位）与 `created_at`；`xzmap` 头写入；`LocalizationState` **不改**（用 `map_id`+`map_valid`），但在 `/localization/state` 的 `map_id` 里带版本后缀 `name@abcdef12`（消费端可据此判断换图）。
- 保存前质量门限（复用 `scripts/map_quality_report.py` 的指标，内联到 C++）：
  - free 面积 > 1 m²；unknown 比例 < 60%；被 3 面墙围住的 free 面积占比 > 50%；否则**拒绝保存**并返回 error（`/stop_map_operation` 不再无条件成功）。
- 原子写：所有地图文件先写 `*.tmp` 再 `rename`，最后写 `manifest.json`（文件名+hash+时间）。
- 同名换图：`/switch_map` 检测 `map_dir` 相同但 mtime/hash 变化 → 强制重载。
- `.scdb` 跟随地图一起保存/加载（修 `saveMap` 死代码）。

### 6.2 验收
- 单测：空图/全 unknown 图保存必须失败；正常图 version 稳定（同内容两次 hash 相同）。
- 脚本：保存过程中 kill -9，重启后地图目录里不能出现"新 xzmap + 旧 pgm"的混合（manifest 校验）。

---

## 7. 回归与交付

| 层 | 命令 |
|---|---|
| 核心单测 | `catkin_make run_tests_rosiwit_navigation`（含新增 safety/shaper/recovery 用例） |
| 安全 | `scripts/safety_zone_check.py`（新增，3 个造点场景） |
| 指令 | `/cmd_vel` 加速度审计脚本（新增） |
| 恢复 | `scripts/recovery_check.py`（扩展） |
| 覆盖 | `coverage_check.py` + 离线 `plan_coverage.py` |
| 定位 | `localization_kidnap_check.py` + 地图保存失败用例 |
| 全链路 | 3×2 覆盖 + 单点导航 + 窄道（0.8 m） |

**交付顺序**：1 → 2 → 3 → 4 → 5 → 6，每步一个 commit，commit message 里附该步的验收输出。


---

# 实施状态（2026-10-26）

## 已确认的决策
- **A = 2**：`safety` 与局部 `costmap` 都用**车身轮廓**排除自身点（不再是 [-0.45..0.75, ±0.55] 大矩形）。
- **B = catkin 自带 gtest**（`catkin_add_gtest`，不依赖系统 `libgtest-dev`）。

## 各阶段落地情况

| 阶段 | 状态 | 证据 |
|---|---|---|
| 1 安全层 | ✅ | `safety_monitor.hpp/cpp`：body_polygon 排除、前进/后退/旋转三套 STOP 区、trigger/release 滞回；`node.cpp` 用 footprint 包围盒生成 body_polygon（`scan_self_*` 只作回退）；覆盖旋转/出界分支已并入 `publishCommand` 单一出口 |
| 2 指令通路 | ✅ | `velocity_shaper.hpp`：绝对限幅 + 按真实 dt 的加速度限幅 + deadband；`publishCommand` = 限速 → 遥控仲裁 → 整形 → 安全层；20 Hz `commandWatchdog`（`velocity_timeout` 0.6 s 平滑回零）；`applySpeedLimit` 只缩 `linear`（角速度按比例缩待下一轮） |
| 3 配置自检 | ✅ | `astar_planner`/`navfn_planner` 输出 `velocity_x=+1`（原来恒 0 → 轨迹 v≡0，rpp 爬行）；`validateConfig()` 启动打印生效指纹 + planner 可用性 + `velocity_timeout<=0`/`odom=/odom` 告警；三份 yaml 补 `motion_profile`/`clean_planner`/`free_planner` 与全部新参数 |
| 4 恢复 | ✅ | 恢复链改为 `CLEAR → SPIN → WAIT → 反向逃逸(BACKOFF)`；`clearRecoveryCostmaps()`（清 voxel/tracker/scan + 重建规划视图）；`rearClearForBackoff()` 用实时点云校验倒车走廊 |
| 5 覆盖多房间 | ⚠️ 部分 | 门洞强制切分已实现（`CoverageConfig::doorway_width`），但**实测无收益**：两室一门洞 26 m² 上覆盖 +0.04%、路径 +9%、原地转 16→29 次 → **默认关闭**。多房间本身已验证：现有「单元分解 + 单元贪心 + 转场 BFS」在两室地图上 94.2%（95.2% of coverable），会依次扫两个房间。自动房间分割（形态学/Voronoi）仍留待下一轮 |
| 6 地图保存 | ⚠️ 部分 | 已做：保存前质量门限（free ≥ 1 m²、unknown ≤ 60%，用 `getRealtimeGrid` 同路径体检）、失败返回 `success=false + error`（不再无条件成功）、PGM/YAML 原子写（tmp→rename）、同名地图 mtime 变化强制重载。未做：`manifest.json`/内容 hash、`.scdb` 随图保存、`map_id` 带版本后缀（怕影响 App，先不碰契约） |

## 测试
```
catkin_make run_tests_rosiwit_navigation_ros1
  test_movebase_protocol   34 tests
  test_safety_monitor      22 tests   (新增: 车身排除/三区切换/2-3 帧滞回/单帧噪声不翻转)
  test_velocity_shaper     10 tests   (新增: 加速度/绝对限幅/dt 无关/deadband/回零)
  Summary: 66 tests, 0 errors, 0 failures
```

离线覆盖验证（`tools/coverage_route_dump` + `plan_coverage.py`）：
- 两室一门洞 26.4 m²：doorway=0 → 63.4 m / 94.2%；doorway=0.72 → 69.3 m / 94.3%（故默认关）
- 真实地图 roomA / 4×3 / 3×2：99.9%~100%（见前一轮记录）

## 待办（下一轮）
1. 速度限速区同时缩 `angular.z`（进入限速区不应保持满角速度）。
2. 恢复：新目标抢占恢复动作、每个动作独立 `time_allowance`（当前用整轮 20 s）。
3. 覆盖：形态学/Voronoi 自动房间分割 + 房间级跳过/顺序配置。
4. 地图：`manifest.json`（内容 hash + 时间戳）与 `.scdb` 随图保存。
5. 真机回归：0.3 m 造点必进 STOP、窄道 0.8 m、切图后继续导航。


---

## 补记：安全层上线后暴露的两个真问题（2026-10-26 晚）

3×2 覆盖回归一开始从 95.7% 掉到 4%：车在起点原地转不动，`/cmd_vel` 前 17 帧
从 −0.06 平滑爬到 −0.80（整形器正常），随后变成**硬 0** 并保持 609/626 帧。
硬 0 只能是 `applySafety` 的 STOP 分支。

### 问题 1：旋转安全区用了矩形，对角方向多判 0.3~0.5 m（真设计缺陷）

诊断日志（`[safety] STOP hits ...`）打出了触发点的两套坐标：

```
pose=(-0.50,-0.16,-53.4deg)
hit base(0.62,-0.55)  world(-0.57,-0.98)   -> 距车心 0.83 m，位于地图自由区
```

- 该点在**世界系固定**、base 系也固定 → 真回波（低于建图高度的小物体，2D 地图里没有）；
- 原地转时车体扫掠圆半径 = `hypot(0.55, 0.33) = 0.64 m`，0.83 m 的物体扫不到；
- 但旋转区当时是**矩形** `x[-0.50..0.80] y[±0.55]`，对角到 `0.97 m` → 误判。

**修法**：`SafetyMonitorConfig::stop_radius_rotation` —— 原地转（`|vx|<0.05`）用
**扫掠圆**，半径 = `max|footprint 顶点| + safety_rotation_radius_margin`（0.64+0.10=0.74）。
新增两个单测钉住：0.80 m 对角点旋转不再停、0.61 m 近点仍然停。

顺带把 `safety_stop_max_points` 从试探性的 8 退回 nav2 默认的 **4**（slowdown 6）——
点数阈值不该用来掩盖区域几何错误：0.8 m 处 3 cm 的椅子腿只有 ~3 个点，设 8 会漏。

### 问题 2：2D 地图与 2D 雷达的"高度不一致"会被安全层放大

那个 0.83 m 的物体在 `/map` 里是自由区（建图时被地面/高度过滤掉了），但 `/scan`
在 z≈0.115 m 看得见。旧实现因为大矩形自过滤 + STOP 区被吞掉而"看不见"，所以历史测试
从没暴露；现在安全层真的生效了，它就会停下来。

这不是 bug，是**传感器/地图分工问题**：
- 清洁机器人应该把低矮物体当障碍（压不过去），所以停是对的；
- 但覆盖区域（用户画的多边形）若包含这类物体，就只有"标记 blocked / 让它停"两种结果。
- 下一步：给安全层加**第二数据源**（`/world_cloud` + `min_height`，nav2 的 pointcloud source），
  用高度带把地面回波和真实低矮障碍分开；同时把"低于 0.15m 的障碍"纳入建图高度带。

### 待查（低优先级）

- `applySafety` 用 `current_pose_` + yaw 把 map 系点转到 base 系，而 `scanCallback` 用 TF。
  两者若有固定偏移（map→odom 或时间戳差），会把点整体搬移。建议加一条自检：同一帧用
  TF 算的 base 点 vs `applySafety` 算的，误差应 < 5 cm，超了打 WARN。


### 问题 3：旋转区判据漏了 ω，导致"蠕行/停住"也被套用旋转圆（真死锁）

修完扫掠圆后仍然卡在行末（19%，`state=COVERING`、`pose=(-1.28,0.81)` 十秒不动），
诊断日志出现 `STOP hits` **为空**但 `stop_active_=true`：

- 我的选区判据写成了 `|vx| < 0.05 → 用旋转圆`；
- 而覆盖在行末蠕行时 `vx≈0`，ω 也≈0 → 被当成"旋转"，套用 0.74 m 的圆 →
  把 0.9 m 外、任何方向的物体都算进安全区 → STOP 锁存 → 车再也动不了；
- 诊断里按"前进区"打印命中点自然是空的（两边选的区不同）。

nav2 `VelocityPolygon` 的语义是 **`|v| < 0.05 且 |ω| > 阈值`** 才算 rotation，
另外还有独立的 `stopped` 区。修正为：

```cpp
const bool rotating = std::fabs(cmd.linear_x) < 0.05 && std::fabs(cmd.angular_z) > 0.05;
```

停住/蠕行时回退到前进区（更宽松，符合"没在转就不需要旋转保护区"）。
新增单测：同一个侧后方 0.70 m 的点，`(0,0,0)` 时 PASS、`(0,0,0.6)` 时 STOP。
