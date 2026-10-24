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
>   - **坑**: 层内部栅格必须用 `int16_t`. 用 `int8_t` 时 254 (unknown) 溢出成 -2,
>     整个 unknown 语义失效且没有任何编译告警 (planner 仍能从 `grid->data` 兜底工作,
>     所以表面上"不漏", 实际全部代价地图都是空的).
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
>     - C2b ⏳ 接入：DEGRADED/LOST 时用 2D 匹配拉回 + 把匹配协方差写进
>       `/localization/state`；轮速从硬门控改成协方差膨胀；拉偏 1m 回归
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
