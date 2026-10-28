# 2026-10-29 第二轮审查：23 条 + 修复状态

方法：4 个只读审查并行（规划/控制几何、代价地图/安全层、恢复/接口契约、SLAM 核心/ROS1 接口），
每条都在改代码前**自己复核过**（读调用链/算数值/实测 topic），虚警单列在文末。

## P0：确定性 bug（已修，不需要现场数据）

| # | 问题 | 修法 |
|---|---|---|
| 1 | `toCorePath` 产出 `velocity_x=0`，而 rpp 把 `|v|<0.1` 当"原地旋转段"（只返回角速度）→ FOLLOW_PATH / 切控制器后 `linear.x` 恒 0，车只转不走 | `toCorePath` 标 `velocity_x=1.0`；换控制器/FOLLOW_PATH 入口优先复用 `current_core_path_` |
| 2 | 恢复逃逸规划跨轮复用：`resetRecovery()` 不递增 `recovery_plan_gen_`，`launchReverseEscape` 见 future 未 ready 直接复用 → 可能执行旧位姿/旧地图下的倒车路径 | `resetRecovery` 递增 generation + `recovery_plan_graveyard_`（async future 析构会阻塞，只能顺带回收）；不复用 |
| 3 | 两点导航 `has_start` 100% 无效：`startPlanning` 先写 override，`if (restart) stopNavigation()` 又清掉 | override 赋值移到 restart 分支之后 |
| 4 | `/update_map_operation` 必然失败：传无扩展名 base 给只吃 PCD 的核心 | 传 `base + ".pcd"`，失败时打印需要的路径 |

## P1：几何口径（已修，需要一次回归确认路径质量）

| # | 问题 | 修法 |
|---|---|---|
| 5 | 距离场是 **曼哈顿**（4 邻域 BFS）却当欧氏用：外接圆 0.585m 的快速放行在 45° 等效 0.41m（角部最多 13cm 侵入被放行），膨胀场呈菱形 | 新增 `nav_core/distance_field`（Felzenszwalb 精确 EDT），替换 hybrid 膨胀场 + `inflatedCostmap`；单测与暴力解逐格比对 |
| 6 | 起点清除盒用 `configured+0.05`，规划器按 `configured+0.06` 判碰撞 → 保险杠距墙 <6cm 必现 `Start in obstacle` | 清除盒改用 `planner_footprint_` + 半格余量 |
| 7 | 原地旋转基元只查终点姿态，扫掠不查（门洞 30° 旋转中途会扫到门框） | `checkArc` 旋转分支按 10° 采样中间姿态 |
| 8 | 回退 A* 的路径不做节点层平滑（条件看 `planner_name_`，回退时它仍是 hybrid_astar）→ `smoothed=0` 折线切角 | 规划线程记 `last_plan_used_fallback_`，回退路径也平滑 |
| 9 | `/scan` 按 3×3 **格**膨胀：res=0.05 → 0.15m，res=0.10 → 0.30m | 按米（5cm 半径）换算格数 |

## P2：已修（结构性）

| # | 问题 | 修法 |
|---|---|---|
| 10 | 障碍聚类"整面墙 = 一个 r≈3m 的圆"，DWA 清距为负、候选全丢 | 每连通域按 0.5m 分块成小圆（半径=块实际外接半径），保留最近 128 个 |
| 11 | 聚类 `vector<vector<bool>>` 每次 100MB+ 分配（5Hz，单线程 spinner 下饿死 watchdog） | 扁平 `vector<uint8_t>`（1 字节/格） |
| 12 | 安全层用"最新位姿"反算扫描点，无时间对齐（100ms × 0.8rad/s ≈ 7.6cm 错位 vs 1.5cm 余量） | 位姿历史 3s + 按 `/scan` 的 `header.stamp` 取位姿 |
| 13 | rpp 不消费 `min/max_lookahead_distance`（L 达 0.99m > 配置 0.9）；`slow_down_distance` 无消费点 | L 夹在 [min,max]；接上 `slow_down_distance`（末段按距离限速） |
| 14 | 实车 `footprint_aware_control: false` → 控制器无车体检查且吃 0.62m 膨胀层 | 置 true（rpp 现在声明 supportsFootprint，拿未膨胀层） |
| 15 | `/goal_status` 与取消/拒绝不一致（cancel 不写、CANCEL 报 SUCCEEDED、空闲 /nav_stop 覆盖 FAILED_*） | CANCELED / REJECTED / 仅在活动时写 STOPPED |
| 16 | 恢复预算：早退也烧预算；进度阈值写死 0.5m 而仿真逃逸 0.4m → 预算永不清零 | 早退退还；阈值 = 1.25×`recovery_back_distance_` |
| 17 | 取消只发一拍**经整形器**的零速，之后 0.6s 不发 → ~1.3s / 0.75m 才停 | `publishHardStop()`：复位整形器、直接发零 |
| 18 | `dynamic_min_views` 统计点数而非帧数（同一帧一个行人就能过阈值） | 每帧先去重再 +1 |
| 19 | 回环修正位姿不回写 loop closure → xzmap 存回环前位姿，与 map.pcd 不一致 | `ILoopClosure::updatePoses()`，后端修正后同步回写 |
| 20 | 模式切换失败被吞（`resetCore()` 返回值丢弃，服务仍回 result=1） | `resetCoreAndCaches()` 返回 bool，mode=0/1 都传播 |
| 21 | IMU/LiDAR 缓冲不随切模式清空 → 新一轮用上一轮旧 IMU 估重力/零偏 | `SlamBase::clearBuffers()`，`resetForNewMapping` 调用 |
| 22 | 接口读路径绕过核心锁（`resetCore` 换实例时可能 UAF） | 状态类 getter 持 `core_mu_`；`getRealtimeGrid` 用 `core_excl_mu_` try_lock，忙则跳过 |
| 23 | `mapping_path_msg_` 跨线程 push/publish 无锁；`curr_pose_` 写侧不加锁 | `path_mu_` + 发布前锁内拷贝；写侧持 `pose_mu_` |

## 仍需现场数据 / 未做

1. **单线程 spinner 与 commandWatchdog 的本质矛盾**：watchdog 与代价地图同一条回调队列，
   一旦某次 `updateCostmap` 超过 `velocity_timeout_`(0.6s)，正是它最该响的时候不响。
   本轮只降了单次开销（扁平 visited、EDT、窗口化）。彻底解法是 watchdog 用独立
   `AsyncSpinner(1)` + 给 `publishCommand` 的共享状态加锁，属架构改动，需专门评审。
2. **STOP 点数阈值**仍是绝对点数（`>4`）。本轮把"雷达角分辨率 vs 5cm 障碍能采到多少点"
   做成启动自检（不可达时 WARN），但阈值是否要改成几何判据，取决于现场雷达与目标最小障碍。
3. **门洞切内角**的正解是前视点几何（`arcClearanceScale` 已把度量留好，只观测不动作）。
   需要真实门洞段的侧向偏差数据再决定改前视还是改路径。
4. `observations_` 无界（长时间建图内存线性增长）。
5. 核心另外 11 个 `test_*.cpp` 仍未接 catkin。
6. `SnapshotSlot` 的 `{数据, version}` 非一致对（当前无消费者读 version）。
7. **构建陷阱**：核心包用 `file(GLOB_RECURSE ... CONFIGURE_DEPENDS)`，但 `cmake_minimum_required(3.10)`
   不支持 `CONFIGURE_DEPENDS`（3.12+ 才有）——新增 .cpp 不会自动进构建，必须手动重跑 cmake
   （本轮新增 `distance_field.cpp` 就踩到了；已在本文件记录，改源码清单需一并处理）。

## 本轮撤销的虚警（核实后不改）

| 审查说法 | 核实结论 |
|---|---|
| `is_navigating_` 在 `startPlanning(restart=true)` 后永久 false → 导航挂死 | `applyPlanningResult` 会置 true（node:3345），`pollPlanning` 在早退之前跑，能自愈 |
| `/map` 占据值用 `>=100` 判障碍过严（概率地图 60~99 会整体变自由） | 实测 `/map` 取值只有 `0/100/-1` |
| `scan_pts:989 / scan_cap:1000` 说明点云被截断 | `scan_cap` 是 `vector::capacity()`；真截断在 4000 点且按角度等步长，不丢近处 |
| `location_status=1 / health=0` 是矛盾值 | 两套枚举：`SlamInfo.location_status`(0=SUCCESS,1=RECOVERING,2=FAILED) 与 `LocalizationState.health`(0=INIT,1=TRACKING,2=DEGRADED,3=LOST)，0 的含义相反。这正是 `last_slam_status_` 被两个回调互相覆盖的根因 |
| A* 的 safety 环写 99（可通行）与注释不符，应改 100 | 注释/代码是有意为之：写 100 会让贴墙起步的 A* 直接 `Start in obstacle`，而 A* 本来就是"贴墙也得出路"的兜底；本轮改为修"回退路径不平滑" |
| `lookahead_distance` 在 rpp 里没被消费 = 死键 | 该键由 pure_pursuit/mppi 消费；rpp 用 min/max + 速度项，本轮已夹紧 |

## 回归结果与"定位质量"的相关性（本轮最后一条修复的依据）

同一路线（3 目标，180s/目标）多批次回归后，失败与定位质量强相关：

| 批次 | run1 | run2 | run3 | 说明 |
|---|---|---|---|---|
| P0+P1+P2 后（第 1 次） | 0.776 → 3/3 | **0.635 → 2/3** | 0.940 → 3/3 | run2 目标 3 `FAILED_SAFETY_STOP` |
| 加"退化保持(matching>0)" | **匹配 0/RECOVERING → 2/3** | 0.935 → 3/3 | 0.952 → 3/3 | run1 结束时 `location_status=1 / matching=0`，条件没覆盖 |
| 改为"模式感知保持" | 0.940 → 3/3 | **mean 0.644 → 3/3（hold 触发 4 次）** | 0.941 → 3/3 | run2 定位最差却通过 |

**机制（已排除"地图过期/真障碍"）**

- 世界里目标区 `(-2.3..-1.0, -0.8..1.8)` **没有任何 model pose**（283 个 pose 全查过）；
- 保存的地图在该区域也是全 free（最近墙 2.35 m）；
- 但安全层 STOP 命中点全落在这片 free 区域，且聚类后集中在
  `(-1.6,1.4) / (-1.6,-0.4) / (-2.0,-0.2)`。
  → 位姿偏 20~30 cm 时，用"当前位姿"把 `/scan` 换到 map 系，真墙会落到地图空白处，
  安全层看到幻影障碍 → 停 → 恢复动作改不了位姿 → 预算烧完 → `FAILED_SAFETY_STOP`。

**修法**：记录 `SlamInfo.slam_state`（0=MAPPING / 1=LOCALIZATION / 2=ERROR）；安全停超过
`safety_stop_timeout` 时，若处于 LOCALIZATION 且位姿不可信（`location_status != SUCCESS`
或 `matching < slam_degrade_matching`），则保持停车 + `WAITING_LOC`，**不进恢复、不烧预算**；
定位恢复（`relocalize_count` 变化触发 `onPoseJump` 作废旧规划）后自动继续。
只在 LOCALIZATION 判 —— MAPPING 下 `health` 恒 INIT / `matching` 恒 0，按状态停车会让
建图模式永久停住（本会话早前踩过 3/3 `TIMEOUT:WAITING_LOC`）。

**验证**：最终 3 次回归，9/9 目标全部 SUCCEEDED；门洞段最小 footprint 间隙
0.05 / 0.05 / 0.10 m（改前失败态是 0.000）；safety stop 0/4/0，恢复 0/1/0。
其中定位最差的那次（mean matching 0.644，多次掉到 0）靠"退化保持"扛过去，
而不是失败或空转到超时。
