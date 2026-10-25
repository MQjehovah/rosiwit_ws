# 导航/定位架构改造计划（2026-10-25）

> **进度**：
> - A1 轨迹类型 + 参数化器：✅ `cea5f11`（编译通过）
> - A2 轨迹接入（setTrajectory + RPP 速度前馈）：✅ `41471c6`
>   - clean+mppi 3/3；clean+rpp 3/3（footprint 余量 0.350m）
> - A3 MPPI 消费轨迹（speed critic 用 v_ref）：✅ `ad36545`
>   - clean+mppi 3/3（余量 0.250m）
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

## 排期建议

| 步骤 | 预估 | 产出 | 回滚 |
|---|---|---|---|
| A 轨迹化 | 1 轮 | Trajectory + parameterizer + 控制器消费 | 关闭 `use_trajectory` flag |
| B 分层代价地图 | 2 轮 | LayeredCostmap + 6 层 + 三视图 + 禁行区 | LegacyLayer 兜底 |
| C LocalizationServer | 2~3 轮 | server + state msg + 2D/odom 约束 + nav 迁移 | 保留旧话题 shim |

每个工作流单独提交，附对应的回归证据（route_test2 + 专项测试）。
