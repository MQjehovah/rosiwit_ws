# 导航/定位架构改造计划（2026-10-25）

> **进度**：
> - A1 轨迹类型 + 参数化器：✅ `cea5f11`（编译通过）
> - A2 轨迹接入（setTrajectory + RPP 速度前馈）：✅ `41471c6`
>   - clean+mppi 3/3；clean+rpp 3/3（footprint 余量 0.350m）
> - A3 MPPI 消费轨迹（speed critic 用 v_ref）：✅ `ad36545`
>   - clean+mppi 3/3（余量 0.250m）
> - A4 `free,rpp` 多方向（含倒车段）回归：✅
>   - 之前从未验证过；这次实测通过：路径 `reverse=27`，RPP 输出
>     `dir=-1 vx=-1 v=-0.66`，14 帧倒车，三腿全部到达
>   - 脚本固化到 `ros1_ws/src/rosiwit_navigation_ros1/scripts/free_rpp_check.py`
>   - **边界**：末段 0.6m 以内由节点 precision-approach 接管（只前进），
>     所以"距离 <0.6m 的倒车目标"会掉头前进，不是 bug；测试目标必须 >0.6m
>   - 顺带发现：`RppController::setPath` 在切换控制器时用的是
>     `toCorePath(ROS Path)`（丢方向字段），只影响"运行中切控制器且已有路径"的
>     瞬间；新规划会立刻用核心路径覆盖，因此实测无影响
> - A3b 删节点"末段两段式精调"：**取消（不需要）**
>   实测确认：Hybrid 路径末点航向 = 目标航向（goal yaw），节点末段精调就是"最终
>   航向对齐 + 微调"的执行体；轨迹末段只保证 v=0，不包含对齐逻辑。保留它比在
>   两个控制器里各写一份对齐更简单可靠。**不做 A3b，直接进 B。**
> - B 分层代价地图：✅ **B1+B2+B3 完成**
>   - `nav_core/costmap_layer.hpp`  层接口 (LayerConfig / ICostmapLayer / inflatedCostmap)
>   - `nav_core/layered_costmap.hpp` LayeredCostmap (层列表 + max 合成 + combinedOf 子集视图)
>   - `nav_core/costmap_layers.hpp`  StaticLayer / PointsLayer / CirclesLayer /
>     KeepoutLayer / SpeedLimitLayer
>   - 两个视图全部改为分层合成, 行为与旧实现等价 (未配禁行区时):
>     planner = static + keepout (自身 footprint 膨胀, 254 unknown 仍阻断且不产 halo)
>     controller = static + scan + tracked + voxel + keepout (无膨胀, footprint 负责车体)
>     safety = 原始激光点 (不下采样, 保持现状)
>   - 新增功能: 禁行区 `/virtual_obstacles` (xzrobot_ros_dependency/VirtualObstacles,
>     每个 form 是多边形) 与分区限速 `/speed_limit_zones` ("x,y,w,h,v;..." 或 yaml
>     `speed_limit_zones`), 限速在 `publishCommand()` 唯一出口生效
>   - 回归: clean+mppi 3/3 (余量 0.250m), clean+rpp 3/3 (余量 0.350m)
>   - 功能验收: 限速 0.25 -> 区内 max|v|=0.251, 撤销后 0.643;
>     禁行区盖住目标 -> costmap=100, planner "Goal in obstacle" 失败;
>     撤销 -> costmap=0 且同目标 SUCCEEDED
>   - **取值约定（全链路一套，不做任何转换）**：`nav_msgs/OccupancyGrid` 语义
>     `-1` unknown / `0` free / `1..98` 膨胀梯度 / `99` inscribed / `100` lethal。
>     层栅格、`CostmapGrid`、`core::Costmap`、发布消息、planner 内部
>     `costmap_data_` 都是这一套；不再有 254/255 的私有刻度。
>   - **坑**：以前 `core::Costmap` 用 `unsigned char` + 254/255 私有约定，而
>     `CostmapGrid`/消息是 `int8_t` + -1/0..100，中间靠转换桥接；层内又一度用
>     `int16_t` 绕过 254 溢出。现在统一为 8 位 `-1/0..100`，`int8_t` 足够：
>     199/254 那种"大数值"在 `int8_t` 里存不下，本身就是语义没对齐的信号。
> - C LocalizationServer：
>   - **C1 + C3 完成**
>     - `xzrobot_ros_dependency/LocalizationState.msg`：pose(含 6x6 协方差) + health
>       (INIT/TRACKING/DEGRADED/LOST) + matching + map_id + map_valid + relocalize_count
>     - SLAM 节点发布 `/localization/state`（10 Hz，与 `/slam/info` 同源同节奏）
>     - 服务别名：`/localization/relocalize`、`/localization/set_initial_pose`、
>       `/localization/switch_map`（复用既有 handler，relocalize_count 单调递增）
>     - nav 改为消费 `/localization/state`（`loc_state:1` 表示已切到新契约），
>       `/slam/info` 降级为迁移期 shim（一旦收到新契约就不再采信）
>     - 验收：health=TRACKING matching=0.954 map_id=house_sim；
>       拉偏 0.4m 后 relocalize -> count+1、health 经 INIT 回 TRACKING、
>       位姿偏差 0.000/-0.001m；nav 3/3（余量 0.250m）
>   - **C2 进行中**：
>     - C2a ✅ `planar_scan_matcher`（核心库，仅依赖 Eigen）
>       - 多分辨率穷举 0.30m/0.15rad -> 三级细化，亚栅格步长
>       - 打分 = 全体点的截断距离均方（外点也惩罚，否则搜索会主动丢点）
>       - 信息矩阵 H = Σ j·jᵀ（j = [法向x, 法向y, r×n]），
>         协方差 = (H/σ² + P_prior⁻¹)⁻¹ —— 未观测方向由先验兜底（0.5m/0.2rad）
>       - 离线验证 `test/planar_scan_matcher_check.cpp`（g++ 直接编译，54ms）：
>         房间场景 0.15m/0.06rad 偏差 -> 误差 8mm/0mm，σ=(14,18,7)mm；
>         长走廊沿墙偏 0.3m -> 横向 2.8cm 准确，σ_x=0.5m、σ_y=4mm、比值 126、
>         条件数 1e9（退化方向被正确放大，而不是"假装精确"）
>     - C2b ✅ **已打通（`planar_constraint.enabled: true`）**
>       - `/scan` 按**完整四元数**投影到 base_link（倾斜安装的雷达不能用
>         `2*atan2(z,w)` 取 yaw，会把俯仰混进航向，实测 ~12° 误差）
>       - 距离场按地图几何变化重建并缓存；TRACKING 时 1Hz 校验、DEGRADED/LOST 时
>         4Hz + 更大窗口；无效就按 0.6→1.08→1.94m 逐级放大重试
>       - 纠偏要连续两帧一致 + 条件数<1e5 才执行；同一处纠偏重复 4 次即停止
>       - **为此修掉两个核心 bug**（不修则一定失败）：
>         1. 注入位姿后没清 `m_last_accepted/m_coast_count/m_has_cached_target`
>            → 逸出保护与 coast 重锚把位姿硬拉回旧锚点（表现为"注入了又被推回"）
>         2. 注入位姿后 `acceptMatch` 的 Mahalanobis 门限每帧都拒（`d2≈18`，
>            创新主要由 3D 匹配与 IMU 先验在 roll/pitch 上约 12° 的**固有**偏差构成）。
>            被拒的帧不进 `onMatch` → 偏置永远学不回来 → 死锁在 INIT
>            （`accepted=0, coast` 一路涨）。修法：注入后放行 5 帧
>            （`m_maha_bypass_frames`），让先验偏置重新收敛
>       - seed 门限做成可配 `global.seed_max_drift`（仿真 0.35m/8°）：
>         1.0m 太松，ICP 能从种子滑 0.84m 并以 fitness 0.96 假锁定
>       - 验收：拉偏 1m → **0.8~1.0s 恢复**，偏差 ≤0.08m，health 回 TRACKING
>         （matcher 同时给出 dx=-0.94/内点 0.64/残差 0.026 的独立确认）
>     - C2c ✅ **轮速从"硬门控"改成"带方差的观测"**
>       - `ImuMotionPrior` 不再开关式丢弃 odom：
>         * 原来 `onOdom` 在 `v_odom > max_odom_speed` 时 **return 丢弃整包**
>           —— 连 `m_odom_time` 都不前进，下一个样本的 dt 被拉长、角速度估计连带失真
>         * 原来 `predict` 在 `|odom_w - gyro_w| > slip_yaw_thresh` 时 **整段不用**
>           （0/1 悬崖）
>       - 现在：`w = 1/(1+(slip/thresh)^4)`（0→1.0，阈值→0.5，2×阈值→0.06），
>         平移速度超上限再 ×0.1；样本始终入账（odom 时钟连续）
>       - 位姿 = `(1-w)·先验 + w·odom_mapped`，朝向仍由 IMU 积分
>       - 输出等效不确定度 `transSigma()/yawSigma()`（0.02..0.62 m / 0.01..0.31 rad），
>         供下游按方差加权
>       - 离线验证 `test/odom_soft_gate_check.cpp`（g++ 直编）：
>         直行 w=1.000/σ=0.02（与旧行为一致）；slip=阈值 w=0.500（旧为 0）；
>         2×阈值 w=0.059；毛刺 w=0.100 且 dt 仍为 0.020（时钟未被冻结），
>         下一帧恢复 w=1.000
>       - 回归：clean+mppi 3/3（余量 0.200）、clean+rpp 3/3（0.350）、
>         禁行区/限速 PASS、拉偏 1m 恢复 0.8s/0.199m
>     - C2d ✅ **先验不确定度写进对外契约**
>       - 访问器链：`IMotionPrior::priorTransSigma/priorYawSigma` →
>         `ILocalization::priorUncertainty` → `ISlamAlgorithm` → `SlamPipeline`
>         → `SlamEngine` → ROS 层 `publishLocalizationState`
>       - `/localization/state` 协方差优先级：
>         1. DEGRADED 且有新鲜 2D 匹配 → 用 2D 匹配的真实协方差（含退化方向）
>         2. 否则 → `max(分档值, 先验 sigma)`，INIT/LOST 仍置 1e3（不可用）
>       - 日志验证：`[locstate] ... prior=(0.020,0.010)`，正常跟踪时
>         cov_xy=0.050（分档值占优，行为不变）；打滑时先验 sigma 升到 0.62 m，
>         协方差随之膨胀
>       - 回归：拉偏 1m 1.0s/0.127m、mppi 3/3（0.250）、rpp 3/3（0.350）、
>         禁行区/限速 PASS
>
> **C 段到此全部完成。**
>
> ### 本轮补充验证（语义统一后的回归缺口）
> | 项 | 结果 |
> |---|---|
> | 恢复流程（反向逃逸） | ✅ 目标盖在禁行区 -> `recovery 1/2 planned reverse escape (11 pts)` -> 实际倒车 `cmd_v=-0.15`（4 帧）-> 清除后同一目标 SUCCEEDED |
> | 覆盖清洁（navi_cmd=20） | ⚠️ **能跑通但完成率不达标**（详见下） |
>
> **覆盖清洁的既有问题（与代价地图语义无关）**：
> - 现象：`tracking_path_result=1`、行能规划、面积能累计（0.84~1.14 m²），
>   但覆盖率只到 30~80%，最后 `markAllUncoveredBlocked` 收尾；日志显示
>   **同一行被反复规划**（`row=(1.04,-0.34)->(-0.61,-0.34)` 连续 3~7 次，
>   每次间隔 3~4 s）
> - 成因 1（已修）：`updateDynamicObstacles()` 原来喂的是**全部激光点**，
>   等于把静态墙标成动态障碍；而 `markDisk()` 用 `skip_dynamic=true` 记录覆盖，
>   导致靠墙格子永远标记不成"已覆盖"。现在只喂 `tracker_.tracked(true)`
>   的真实动态目标。
> - 成因 2（已缓解）：行端点贴着多边形边界时，approach 目标会被 A* 判成
>   `Goal in obstacle`，于是退回 "direct row"（从 0.8m 外的边界开始跟）。
>   现在把行两端沿行方向内缩 0.15m（sweep_width/2 的清扫半径本来能盖住）。
> - 成因 3（**未修**）：行端点的**可达性**没有用 coverage grid 的 `passable`
>   来校验。区域比可通行走廊宽时（如走廊净宽 0.85m、机器半宽 0.33m，中心只能
>   在 ±0.1m 内），行会落到走不到的格子上 → 永远规划同一行 → 直到全局重试超时。
>   下一步：行选择按 `passable` 过滤 + 单行超时只 `markUnreachable(该行)` 而不是
>   整块放弃（core 已有 `markUnreachable` 接口）。
> - 另：原 `[coverage] done` 日志打印的 `covered` 是另一个变量（恒为 0），
>   已改成 `stats.covered * res²`。
> - 回归脚本：`scripts/coverage_check.py`（可用参数指定多边形）与
>   `scripts/recovery_check.py`。
>
> ### 覆盖完成率的继续修复（第二轮）
> 用可视化定位（`docs/artifacts/coverage_before_rowfix.png`：区域上方整片橙色
> = 被一次性标记 blocked）后修了三处：
> 1. **行缺少"离墙车体半径"校验**：新增 `CoverageConfig::robot_inset`
>    （= footprint 半宽 0.31m）+ `halo_` 距离场；`passable()` 同时要求
>    `inset_`(区域边界) 和 `halo_`(障碍半径) 都为 0。之前行会选到 y=-0.29
>    这种车体站不下的位置，同一行无限重试。
> 2. **单行卡死只放弃这一条带**：同一行(0.1m 量化)反复选中且 `covered` 不增
>    超 `coverage_retry_timeout_` -> `markBlockedAround(该行, sweep_width/2)`
>    后继续下一行（`CoverageGrid::markBlockedAround` 新增）。
> 3. **全局放弃不再清空整块**：原来是 `markAllUncoveredBlocked()`（一次把
>    6.49/7.5 m² 全标 blocked，报表失真）；现在用 `markUnreachable(pose)`
>    只标真正到不了的格子，可达的继续。
> - 效果：行不再无限重试、blocked 面积是真实值；但**完成率仍不达标**——
>   剩下的问题是行选择本身：BFS 逐格找种子会在区域边缘"啃"出 3 点长的碎行
>   （`row=(1.49,0.66)->(1.39,0.66)`），approach 又判 `Goal in obstacle`。
>   下一步应改成**固定行距扫描**（按 `sweep_width` 生成平行行，端点用
>   `passable` 裁剪），而不是从最近的未覆盖格 BFS 起行。
> - 可视化工具（纯 python，不依赖 numpy/matplotlib）：
>   `scripts/render_coverage.py`（地图+覆盖状态+轨迹）、
>   `scripts/render_route.py`（地图+轨迹按速度着色+目标圈）。
>   产物示例：`docs/artifacts/coverage_before_rowfix.png`、
>   `docs/artifacts/route_clean_mppi.png`。
> - 定位拉偏回归也固化：`scripts/localization_kidnap_check.py`
>   （实测 0.8s / 0.131m 恢复，health 回 TRACKING，accepted=1 coast=0）。
>
> ### 覆盖行选择重写（第三轮，完成率 13% -> 95%）
> `CoverageGrid::nextCoverageSegment` 从"BFS 找最近未覆盖格再沿轴扩展"改为
> **固定行距平行行 + 只在机器人可达集合里选行 + 端点 passable 裁剪**：
> 1. 一次性 BFS 出机器人可达集合，只在可达格里选行（避免"可行但到不了"）
> 2. 行位置 = 区域 bbox + `sweep_width` 步长，跨帧稳定；**行线再内缩半个
>    sweep**（否则机器人一路压着区域边界，频繁触发出界限速，实测平均
>    速度只有 0.107 m/s）
> 3. 优先选长度 >= 0.3m 的行（避免碎行），否则取最近的
> 4. 行端点外扩到 sweep_width/2 内且必须 `passable`
> 5. 兜底：固定行线之间残留条带（实测剩 0.23m²/3%）时，按最近的
>    可达未覆盖格临时定一行，保证收尾
>
> 参数（simulation.yaml）：`sweep_width 0.5`、`max_speed 0.45`、
> `horizon 6.0`（2.5m 会把每行切段，反复停下重规划）。
>
> **实测结果（同一套代码，取决于机器人是否冲出区域）**：
> | 区域 | 结果 |
> |---|---|
> | 1.5 x 1.2 m (1.8 m²) | **99.4%，`done: rate=1.000`** |
> | 3.0 x 2.0 m (6.0 m²) | **99.5%，`done: rate=0.995` blocked=0.03 m²** |
> | 3.0 x 2.5 m (7.5 m²) | 95.3%（仍在上升时被测试超时 480s 切断） |
> | 5.0 x 3.0 m (15 m²) | 69.7%（卡在界外/碎行） |
> | 若干次重跑 | 22~64%（机器人冲出区域后再也回不来） |
>
> **结论：完成率能到 99.5%，但不稳定**。根因是"区域围栏 + 恢复"没做扎实：
> - `pure_pursuit` 在行末会冲过刹车距离（0.45 m/s 时约 0.2m），直接压到
>   区域边界外；出去的格子在 `cells_` 里是 OUTSIDE，`markDisk` 会跳过，
>   覆盖统计不会变脏，但**机器人回不来**（approach 从界外规划不出来）。
> - 我试过三种"界外恢复"：① 停车（死锁，卡在界外）② 允许任意移动
>   （跑偏 1.9m）③ 朝区域质心开（跑偏 4.4m）。都没解决，因为
>   `pure_pursuit` 的 `w ∝ v`，v 清零时转向也归零，机器人贴着边界转不动。
> - 另外 coverage 速度只有 ~0.107 m/s（配置 0.45），慢 4 倍，原因未查。
>
> ### 覆盖路线化（已实现，97.1% / 弓字形）
> 按 `fields2cover`(SwathGenerator+Route) / `opennav_coverage`(一次出 route) 的
> 架构落地：
> 1. **`CoverageGrid::buildRoute(pose, route)`** —— 一次生成剩余全部路线：
>    行线（bbox + `sweep_width` 步长 + headland 内缩 `sweep/2`）→ 每行取最长
>    可达可通行段（`passable && reach`）→ 只保留还有 UNCOVERED 的行 →
>    弓字形排序（从离机器人最近的行开始，奇偶行反向）→ 逐段串成
>    `原地转点(位置不变) + 直线到行首 + 行段`
> 2. **`coveragePlanTick`** 改为：整条路线规划**一次**（不再每行一次、不再用
>    A* 拼 approach）；`velocity_x=0` 标记原地转点（位置与前一点相同）
> 3. **活动路线缓存**：`coverage_route_` 非空且机器人仍在 0.8 m 内、且没走到
>    路线终点时**不重建**（之前每 10 s 周期重建 → 机器人反复重跑同一段，
>    轨迹 162 m）
> 4. **短行兜底**：所有行段都短于 0.30 m 时取最长的一条（否则 buildRoute 永远
>    false，覆盖率卡在 98% 空转）；无解时 `markUnreachable` + `finishCoverage`，
>    不再 `WAITING_OBSTACLE` 空转
>
> 实测（3×2 m，sweep 0.5，speed 0.25）：**97.1%（5.83/6.00 m²），
> `done: rate=0.971` blocked=0.18 m²，轨迹 27.5 m**，图上能看到 4 行平行
> 往返 + 端点掉头的弓字形（`docs/artifacts/coverage_boustrophedon_97.png`）。
> 仍未到 100%：左上/左中还有 0.18 m²（区域边缘 + 一个左端小环），
> 下一步是把行端点的 headland 与 `xy_goal_tolerance` 对齐、以及消除左端小环。
>
> **下一步（按优先级）**：
> 1. 覆盖用独立的"行段执行器"：到行末前按 `v²/2a` 提前减速、原地转向下一行，
>    不把行末交给 pure_pursuit 的渐近跟踪；顺带解决 `w ∝ v` 的转向死锁
> 2. 区域围栏做成"硬约束"：界外时用**独立的转向指令**（不乘 v）朝质心转，
>    转正后限速 0.15 回区（`CoverageGrid::centroid()` 已加好）
> 3. 查覆盖速度为什么只有 0.107 m/s（怀疑 `coverage_replan_period`/`path_done`
>    频繁重规划 + 每次 `reset()` 让控制器从 0 速起步）
> 4. 太短的行（<0.4m）不要跟，也不要用 disk 标 blocked（会吃掉 3.65 m² 区域），
>    改成按行线段标记
>
> 可视化：`docs/artifacts/coverage_before_rowfix.png`（修前行线死循环）、
> `coverage_after_rowfix.png`（95%）、`coverage_99p5_6m2.png`（6 m² / 99.5%）。
>
> ### 覆盖路线化（下一轮的唯一方向，已与用户确认）
> **问题定性**：缺陷不在控制器，而在覆盖层的路径契约——每行重规划 + approach
> 拼接，导致 nav2 那套"跟一条稳定路径"的跟踪律失效（实测照搬后只剩 22.6%、
> 轨迹 162m）。参考实现摘录见 `docs/dev/nav2-rpp-reference.md`。
>
> **做法**（参考 `fields2cover` 的 SwathGenerator+Route、`opennav_coverage`
> 的一次出 route）：
> 1. `CoverageGrid::buildRoute(pose, max_length, std::vector<Pose2D>& route)`：
>    一次生成**剩余全部路线**——headland 行线（现有 `row_margin`）+ `passable`
>    裁剪 + 弓字形排序 + 行间连接点（标记 `velocity_x=0` 表示原地转）
> 2. `coveragePlanTick` 改为：整条路线规划**一次**，只在整条路线末尾做
>    approach 收缩；行间转向交给 `rotate_to_heading`；不再每行 replan
> 3. 路线失效条件（新增障碍 / 偏离 > 0.8m / 走到末尾）才重建
> 4. 然后重新启用 nav2 跟踪律（弧长前视 + 曲率限速 + approach 收缩 +
>    `rotate_to_heading_min_angle=0.785`），参数取 nav2 默认量级
>
> **验收**：3×2 m 与 5×3 m 区域覆盖率 ≥99%，`render_coverage.py` 出的轨迹呈
> 明显弓字形；`clean+hybrid_astar+mppi` 3/3 不回归（pure_pursuit 只用于覆盖）。
>
> ### A3b 的前提（重要）
> 节点的末段精调不只做"减速逼近"，还负责**最终航向对齐**（要求
> `|yaw - goal_yaw| < 0.2`）。轨迹目前只保证末段 v=0，没有"对齐目标航向"这一步。
> 直接删末段精调会让 goal3 的最终 yaw 卡住。正确顺序：
> 1. 给 `TimedPoint` 增加 `yaw_ref`（末点 = goal yaw），RPP/MPPI 在 s 接近终点时
>    用 `yaw_ref` 做对齐（RPP 已有 ALIGN，MPPI 用 heading critic 的目标）；
> 2. 再删节点末段精调，跑 clean×{mppi,rpp} 各 3/3。

三个工作流，互相独立，按 A → B → C 顺序落地。**迁移期硬约束**：每一步都必须保持
`clean + hybrid_astar + mppi` 三目标 3/3（回归脚本 route_test2.py），出问题立即用
feature flag 回退。

---

## A. 轨迹化（优先级 1，最小改动，收益立竿见影）

### 目标
路径（几何）→ 轨迹（带时间/速度），把 cusp、末段、窄口从"控制器特例"变成"轨迹属性"。

### 数据
```cpp
// include/rosiwit_navigation/nav_core/trajectory.hpp
struct TrajectoryPoint {
  core::Pose2D pose;
  double s = 0.0;      // 弧长
  double t = 0.0;      // 到达时刻
  double v = 0.0;      // 参考线速度（含方向符号）
  double w = 0.0;      // 参考角速度
};
struct Trajectory {
  std::vector<TrajectoryPoint> points;
  double duration = 0.0;
  bool valid = false;
};
```

### 参数化器
`src/algorithms/support/trajectory_parameterizer.{hpp,cpp}`
- 输入：`core::Path`（`velocity_x`：+1/-1/0=旋转段）+ 限幅（v_max, a_max, w_max, alpha_max, a_lat_max）
- 规则：
  1. `v_curve = sqrt(a_lat_max / |κ|)`，取 min(v_max)
  2. 前向/后向加速度限幅（一次前向 + 一次后向扫描）
  3. `velocity_x == 0` 的旋转段：`v=0`，按 `w_max/alpha_max` 计算原地转时长
  4. cusp（方向切换）：切点 `v=0`，重新加速
  5. 末段：按 `a_max` 减速到 0
- 输出 `Trajectory`；路径无方向标记（A*）时按 +1 处理

### 控制消费（可选，缺省回退）
- `IControllerStrategy::setTrajectory(const Trajectory&)` 默认 no-op
- RPP：参考点取 `t_now` 对应点（或 `s`），`v_ref` 作前馈，`ω` 前馈 + 反馈
- MPPI：speed critic 目标从"v_max"改为轨迹 `v_ref`；时间索引取最近 `t`
- 节点：`applyPlanningResult` 里参数化，`setTrajectory`；轨迹无效则沿用现有 path 逻辑

### 验收
- clean + mppi / clean + rpp 各 3 目标 3/3
- 用 `/odom` 验证：实际 |v| ≤ 轨迹 v_max、加速度 ≤ a_max（抽 10 s 窗口）
- 去掉 `末段两段式精调` 后仍 3/3（说明轨迹把末段接管了）

---

## B. 分层代价地图（优先级 2，同时补禁行区/分区限速）

### 目标
规划器/控制器/安全层使用**同一障碍层**，只是膨胀参数不同；功能上补禁行区、限速区。

### 接口
```cpp
class ICostmapLayer {
 public:
  virtual ~ICostmapLayer() = default;
  virtual bool initialize(const LayerConfig&) = 0;
  virtual void update(double now) = 0;                 // 各自频率
  virtual void contribute(core::CostmapGrid& master) = 0;
  virtual std::string name() const = 0;
};
```

### 层
| 层 | 数据源 | 频率 | 说明 |
|---|---|---|---|
| StaticLayer | PGM `/map` | 1 Hz | 底图 |
| ObstacleLayer | `/scan` | 15 Hz | 带 raytrace 清除（自由空间回写） |
| VoxelLayer | `/world_cloud` | 10 Hz | 3D 投影 + decay（现有 voxel_layer_ 复用） |
| InflationLayer | 上层输出 | 5 Hz | 按消费者不同参数（planner 0.62 / controller 0.0+footprint / safety 0） |
| KeepoutLayer | `VirtualObstacles` 消息 | 事件 | 禁行区（功能缺口） |
| SpeedLimitLayer | 配置/消息 | 事件 | 分区限速（功能缺口） |

`LayeredCostmap`：持有层列表 + 合并策略（max）+ 三种视图：
- `viewForPlanner()`：static + keepout + 膨胀（0.62）
- `viewForController()`：obstacle + voxel + 小膨胀（供 footprintCost）
- `viewForSafety()`：obstacle 原始层（不膨胀）

### 迁移
1. 新增 `LayeredCostmap`，先把现有 `updateCostmap()`/`publishLocalCostmap()` 包成 `LegacyLayer`，行为不变；
2. 逐层替换（先 Static，再 Obstacle/Voxel，最后 Inflation）；
3. 规划器/控制器/安全层分别接对应视图；
4. 打开 Keepout/SpeedLimit。

### 验收
- 三个视图的障碍**同帧一致**（日志打印同一格三视图 cost）
- 禁行区：区域内设 10 m² 禁行，路径绕开；撤销后恢复
- clean 3/3 不回归

---

## C. LocalizationServer（优先级 3，把 2D/轮速从"开关"升级为"约束"）

### 目标
一个对外只暴露"状态 + 地图生命周期 + 重定位 + 恢复策略"的组件；内部把 2D 雷达、轮速作为**观测约束**接入。

### 对外契约
```
topic:   /localization/state  (自定义 msg)
         pose, covariance[36], health(0=INIT,1=TRACKING,2=DEGRADED,3=LOST),
         matching, map_id, stamp
service: /localization/relocalize        (PoseWithCovarianceStamped)
         /localization/set_initial_pose
         /localization/switch_map
TF:      map -> odom（唯一发布者）
```

### 内部
- 状态机：INIT → TRACKING ⇄ DEGRADED → LOST（bounded recovery / global relocalize）
- 地图生命周期：load/switch/version + **保存前质量门限**（map_quality_report 的指标内联）
- 传感器约束：
  - 现有 3D + IMU（定位流水线）
  - **2D `/scan`**：加入平面匹配约束（yaw/平移），退化方向由 2D 补齐
  - **轮速 `/odom`**：作为带协方差的观测因子（打滑时协方差膨胀），替代现在的 slip 硬门控
- 输出协方差来自估计器（协方差传播 + 退化方向膨胀），不再是按状态分档的启发值

### 消费者迁移
- nav：`/lio_odom` + `/slam/info` → `/localization/state`（健康等级驱动限速/降级/停车）
- App/监控：同上
- 旧接口保留一个版本（shim）后删除

### 验收
- 人为拉偏 1 m：2~3 s 内自动回到正确位置，健康等级可见（TRACKING→DEGRADED→TRACKING）
- 原地 0.5 rad/s × 20 s：位置误差 < 0.15 m（当前 0.25 m）
- 窄走廊（0.8 m）通过：2D 约束介入，退化方向不再漂
- nav 不再直接订阅 `/slam/info`

---

## 覆盖路径规划（地图上的规划结果 + 三个结构性问题）

离线工具（不跑仿真/控制）：
- `src/rosiwit_navigation/tools/coverage_route_dump.cpp`：读 PGM + 多边形 → `CoverageGrid::buildRoute` → CSV
  （新增：任意凹/凸多边形文件、`turn_radius`、起点可指定）
- `plan_coverage.py`（临时脚本）：按"车心 + sweep/2 圆盘"离线算规划路线的覆盖率
- `render_route_plan.py`：把路线画在地图上（蓝=直行行段，橙=圆弧，红圈=原地转，绿=进场）

### 转弯方式：半圆弧 vs 原地转（实测，离线）

| 区域 | 方式 | 路径 | 原地转次数 | 规划覆盖率 |
|---|---|---|---|---|
| 5.55×2.85 m 真实房间（`reg_roomA`） | 原地转 90°+横移+90° | 32.36 m | 5 | 89.5% |
| 同上 | 半圆弧 R=0.25 | **31.04 m** | **1** | 89.1% |
| 3×2 m | 原地转 | 14.93 m | 4 | 100% |
| 3×2 m | 半圆弧 | 15.56 m | 2 | 98.8% |

- 半圆半径由几何唯一确定：`R = 行距/2 = sweep/2 = 0.25 m`，弧长 `πR = 0.785 m`。
- 角速度上限决定转弯速度：`ω = v/R ≤ 0.8` → 转弯段 `v ≤ 0.20 m/s`。
- 单次掉头耗时：圆弧 `πR/v = 3.9 s`；原地转 `2×90°/0.8 = 3.9 s` + 横移 `0.45/0.25 = 1.8 s ≈ 5.7 s`，
  且原地转有两次零速尖点（控制器要停车再起步）。
- 圆弧必须吃行端外侧 R 的余量（进 headland），因此圆弧版两侧各内缩 R，这正是 roomA 覆盖率
  比原地转低 0.4% 的原因 —— **圆弧要配 headland 环**才不吃亏。
- 用法：`CoverageConfig::turn_radius = 0`（默认，行为不变）/ `0.25`（半圆弧）。仿真 yaml 尚未接线。

### 顺带修掉的规划 bug（这就是"路径很丑"的根因）

1. **行方向与交替逻辑打架**：第 0 行按"离机器人近的端点"翻转了方向，第 1 行仍按 `k%2` 从另一端进 →
   每两行就要横穿整个区域一次（roomA 实测 `5.07 m` 对角线 + `174°` 原地转）。
   改为**永远从离上一段终点最近的端点进**（第 0 行用机器人位置），天然交替且能容忍行长不一致。
2. **排序用"到行端点的距离"**：5 m 长的行两个端点都远，会被误判成"远行"，起点跑到区域另一头。
   改为**到行线的垂距**（`|pose.y - row_y|`）。

### 结构性问题 —— 已按 F2C 2.x / opennav_coverage 的做法修掉

参考实现：
- **Fields2Cover 2.x**：`BoustrophedonDecomp`（Choset 单元分解，2.0 才支持非凸区域与障碍）、
  `generate_headland`、`genSwaths`（swath angle 取"行数最少"）、`genRoute`
  （BOUSTROPHEDON/SNAKE/SPIRAL/TSP）、`genPath`（Dubins / Reeds-Shepp），
  以及 `default_allow_overlap`（收尾允许重叠补满）。
- **`path_coverage_ros2`**：单元分解 + `border_drive`（先绕单元边界一圈）+ 碎多边形剔除。

落地到 `CoverageGrid::buildRoute`：

1. **距离场**：每格到区域外的距离（两遍 3-4 chamfer）。headland / 内部裁剪都靠它。
2. **headland 环（可选 `headland_width`）**：取跨过 `headland_width/2` 等值线的那一层格子，
   贪心最近邻走一圈（断口另起一段）。与 `border_drive` 同义。
3. **单元分解（x-monotone / Choset）**：沿扫掠方向切片，可通行区间 **1:1 延续**归同一单元；
   出现**分叉 / 合并 / 端点台阶 (>2 格)** 就切单元。这是"每行只取最长一段"那个 bug 的正解。
4. **每单元按自己的长边扫**（F2C swath angle 取行数最少）：行线首条放在
   `边界 + 车心留边`，末条贴边（重叠 ≤ sweep/2）。
5. **转场**：直线能走就直走，不能走用栅格 BFS + 视线捷径（不再斜穿家具）；
   转场折点不插原地转标记（否则每个折点都要停一下）。
6. **车心留边 0.15 m**：行线/端点离区域边界留 0.15（清扫盘半径 0.25 > 0.15，不吃覆盖）。
   没有它行线直接顶到边界，控制超调一步就出界（实测 4×3 区域冲到 x=2.62/边界 2.0，
   每次行末 `OUT_OF_AREA` 卡死）。

离线实测（sweep 0.5，robot_inset 0.31，同一台车同一张图；分母含"物理上扫不到的
家具口袋"时另给 coverable）：

| 区域 | 旧实现（bbox 行扫） | 新实现（分解 + 留边） | 路径 |
|---|---|---|---|
| roomA 5.55×2.85（真实墙） | 89.5% | **99.9%（headland 环 100%）** | 36.6 m |
| L 形 32 m² | 89.5% | **92.5%（headland 环 96.3%）** | 78 m |
| 有家具 4×2.8 | 49.9% | **81.8%（= 95.5% of coverable）** | 38 m |
| 3×2 | 100% | 99.9% | 16.5 m |
| 4×3（仿真里跑的那块） | — | **99.9%** | 29.6 m |

headland 环的结论：**规则矩形不需要**（行线本来就能贴到边界，开环只多 8~10 m 空跑）；
非凸/边带漏的时候才开（L 形 +3.6%）。

### 执行侧两个必须记住的坑

1. `coverage_horizon`（纯跟踪预瞄距离）：**0.5 m 行距下 6.0 m 预瞄必然切角**。
   行末的预瞄点已经跳到下一行，车直接斜切出区域。实测 6.0 → 81%/27% 乱走，
   改成 1.2 才跟得住（行距的 2~3 倍是合理量级）。
2. 旋转点执行器**不能取全局最近点**：后面的转场折点/别的行端点离当前行很近时会被
   劫持，目标朝向在两个旋转点之间来回跳，车原地左右转不停
   （实测卡在 (1.10,0.97)，`angular.z` 在 +0.36 / −0.70 之间反复）。
   改成**按路径顺序取第一个够得着的旋转点 + 完成后游标前移**。



| 问题 | 现象（离线规划覆盖率） | 正确做法 |
|---|---|---|
| **没有 headland 环** | roomA 89.5%：顶部 0.34 m 条带（1.67 m²）一条行线都不经过 | fields2cover 的 headland swath / opennav 的 perimeter pass：先沿边界内缩 `sweep/2` 走一圈，再走内部行线 |
| **每行只取最长的一段** | 有家具的房间只有 **49.9%**：障碍另一侧的整块被丢掉（一行被切成两段，只留长的那段） | 行级 swath splitting：一行的每个 `passable` run 都成段，再按蛇形串起来；或直接做单元分解 |
| **bbox 扫描遇到凹区域** | L 形 92.1%：长短行交替（8.35 m 臂 + 2.55 m 房间），行间来回跳 | boustrophedon 单元分解：沿障碍/凹角把区域切成若干矩形单元，每个单元按自己长边扫，单元间用最近端点连 |

结论：**圆弧是对的**（同样覆盖、路径更短、少 4 次原地转、无零速尖点），但要把覆盖率做上去，
必须按"headland 环 → 单元分解 → 行段切分"三步补齐，圆弧只是行间连接的一种。

复现：
```bash
# 规划 (离线, 不需要 ROS/sim)
g++ -O2 -std=c++17 -I include -I src -I /usr/include/eigen3 \
    tools/coverage_route_dump.cpp src/nav_core/coverage_grid.cpp -o /tmp/covdump
/tmp/covdump docs/maps/house_sim/map.pgm /tmp/r.csv /tmp/reg_roomA.txt 0.5 0.31 0.25
python3 plan_coverage.py /tmp/r.csv /tmp/reg_roomA.txt 0.5 0.31
```

证据：`docs/artifacts/plan_map_roomA_{rot,arc}.png`、`plan_map_roomA_{rot,arc}_cov.png`、
`plan_map_L_arc[_cov].png`、`plan_map_clutter_arc[_cov].png`、`plan_3x2_{rot,arc}.png`。

---

## 排期建议

| 步骤 | 预估 | 产出 | 回滚 |
|---|---|---|---|
| A 轨迹化 | 1 轮 | Trajectory + parameterizer + 控制器消费 | 关闭 `use_trajectory` flag |
| B 分层代价地图 | 2 轮 | LayeredCostmap + 6 层 + 三视图 + 禁行区 | LegacyLayer 兜底 |
| C LocalizationServer | 2~3 轮 | server + state msg + 2D/odom 约束 + nav 迁移 | 保留旧话题 shim |

每个工作流单独提交，附对应的回归证据（route_test2 + 专项测试）。
