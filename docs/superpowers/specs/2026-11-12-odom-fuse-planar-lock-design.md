# Odom 先验重构：fuse 吃满 odom + 平面约束独立 + 后端轮速因子

日期：2026-11-12
状态：待评审

## 1. 背景与证据

原始录包 `remap_20260908_odom_ros2` 的 `/odom` 是**轮速里程计**，不是地图系绝对位姿：

- 首帧 `pose.position = (-13.11, 743.07, 0.0)`，坐标在它自己的系里，与地图原点相差约 743 m；
- `pose.covariance` 对角：x = 0.237 m²，y = 0.237 m²，z = 1e6，roll = 1e6，pitch = 1e6，yaw = 0.0237 rad²；
- `twist.covariance` 全 0；`child_frame_id = base_footprint`（≠ `body_frame = base_link`）。

现有三种用法（`odom_mode`，互斥）：

| 模式 | 实现 | 量测 |
|---|---|---|
| `gate` | `setOdomPrior(speed, yaw_rate)` → `kMaxDv/kMaxDt/kMaxDr` | 不更新状态，只收紧 LiDAR 更新门限 |
| `pose` | `updatePose(rot, trans, cov, lock_z)` | 绝对位姿（3-DoF 或 6-DoF） |
| `fuse` | `updateOdom(speed, yaw_rate, gyro, 0.2, 0.05)` | 水平速度模长 + 世界系 yaw 率 |

`pose` 是为「`/odom` = 2D SLAM 输出的地图系位姿」那套准备的。用在原始包上时，一次性对齐（`pose align: t_lio_map = 55.97 739.62 ...`）把 743 m 偏移记死，之后长期信任轮速；轮速自身漂移表现为日志里的

```
[FLIO2] large 2D pose correction 45.2388m / 45.8261m / 22.778m / 21.8284m / 11.2682m
```

600 s 实测（`remap_600s_srv_20261112`）：`[Map] skip divergent keyframe` 累计 350 次，地图在约 240 s 后停止增长（轨迹到 596 s，但 244 s 之后位姿附近 2 m 内已无地图点），处理帧率 4867/5985 = 81%。

对照 `odom_mode: none`（`remap_600s_srv_noodom_20261112`）：帧率 5922/5995 = 98.8%，全程覆盖，但 z 自由积分到 −36 m。

结论：`pose` 不适用于本包；`none` 丢掉了可用的水平信息；`fuse` 只用了 odom 的两个标量。

## 2. 目标

- **G1** 删除 `pose` 模式（配置选项、接口、代码路径）。
- **G2** `fuse` 吃满 odom：带符号前进速度、侧向速度≈0（非完整约束）、yaw 率、协方差驱动权重。
- **G3** 平面对称约束（z 锁）与 odom 模式解耦：`pose_lock_z`（平地硬锁）、`pose_z_hold`（判坡解锁 + 坡度感知 `v.z` 先验）。
- **G4** 后端新增轮速 3-DoF（Δx, Δy, Δyaw）相对位姿因子。

## 3. 非目标

- 不改回环检测/ICP；不改 2D 栅格生成；不做 IMU/轮速外参在线标定；不改 `map->odom` 的 TF 归属（仍由 `odom_tf_publisher` 决定）。

## 4. 反目标（即使“能跑”也算失败）

- 坡道上把 z 压平（平面约束不得在坡度段生效）。
- 引入任何以轮速 odom 为基准的绝对坐标。
- 让 600 s 帧处理率低于当前 98.8%。
- 破坏使用 `/slam_pose` 提供绝对位姿的链路（见 D4）。

## 5. 约束

- **C1** 所有状态更新必须走 IEKF（协方差一致），平面约束以「状态写入 + 协方差收紧」实现，不允许绕过更新的硬赋值。
- **C2** 轮速 odom 不含 z / roll / pitch 信息，平面约束不能从 odom 推导。
- **C3** 关键帧发散过滤（`slam_pipeline.cpp:806-819`：`|z| > 50 m`、相邻平移 > 5 m、旋转 > 1.2 rad）保留，但其触发不应来自先验跳变。
- **C4** `odom_mode: pose` 不再接受：解析时给出错误信息并提示改用 `fuse`（绝对位姿入口见 D4）。

## 6. 设计决策

**D1（`fuse` 扩为 3 维量测）**
`updateOdom` 改为 3 维残差：

- `r0 = vx_odom_signed − (Rᵀv)_x`：带符号前进速度（现在 `std::abs` 丢符号，倒车不可辨）；
- `r1 = 0 − (Rᵀv)_y`：侧向速度≈0，非完整约束（可通过开关关闭，供全向底盘）；
- `r2 = wz_odom − (R·(ω − bg))_z`：保持现状。

H 相应地作用于 `v`（3 维）与 `bg`（yaw 率项）。权重：优先读 `twist.covariance`（`linear.x`、`linear.y`、`angular.z` 的对应对角），全 0 时回退默认 σ（`vx=0.2 m/s`、`vy=0.05 m/s`、`wz=0.05 rad/s`）。低速（`|v_h| < 0.2 m/s`）时 `r0/r1` 的权重退化为不约束，避免静止时把速度钉在 0。

**D2（平面约束独立）**
新增 `IESKF::updatePlanar(z_ref, r_flat, cov)`：写入 `t_wi.z`、把 `r_wi` 的 roll/pitch 拉向 `r_flat`（保留当前 yaw），置 `v.z=0`，并收紧 `P(2,2)`（yaw 不动）、`P(5,5)`、roll/pitch 对应块。

- `pose_lock_z: true`：每帧 LiDAR 更新后调用，`z_ref` = 首次进入 `MAPPING` 时的 `t_wi.z`，`r_flat` = 该时刻的重力对齐姿态。
- `pose_z_hold: true`：同上，但用**地面法向**判坡（`estimateGroundNormal`），超过 `pose_z_hold_slope_deg` 时不调用 `updatePlanar`，并启用坡度感知软先验 `v.z_target = R(2,0) · speed_signed`（`updateVerticalVelocity`，现被 `!m_pose_lock_z` 卡住）。
- 两者生效与 odom 模式无关（`none / gate / fuse` 均可）。

**D3（后端轮速 3-DoF 相对因子）**
- `KeyFrame` 增加 `PoseStamped wheel_pose; bool has_wheel_pose;`，由前端在产出关键帧时用最近一帧 `/odom`（按时间插值）填入；
- `CeresBackend::KeyFrameNode` 增加 `wheel_relative / has_wheel_relative`，在 `addKeyFrame` 时由相邻关键帧的 `wheel_pose` 计算，**冻结**（与现有 `odom_relative` 同样理由：不能每次优化重新推导）；
- 新增 `RelativePose2DError`（3 残差：Δx、Δy、Δyaw），权重取 `1/σ²`，σ 来自 `/odom` 协方差（x/y 0.49 m、yaw 0.15 rad 的等效值），并使用 Huber 核；
- 现有从 `kf.pose` 推导的 6-DoF `addOdometryConstraints` 保留为轨迹基础边。

**D4（绝对位姿唯一入口改为 `/slam_pose`）**
`absoluteCB`（`PoseWithCovarianceStamped`）已实现并带协方差重排（ROS `[x,y,z,rx,ry,rz]` → 内部 `[rot(3),trans(3)]`）。删除 `pose` 后，需要绝对位姿的链路（2.5D 部署）改为向 `/slam_pose` 发布，由 `localization.absolute.enabled: true` 打开。`/odom` 只承载轮速。

## 7. 改动清单

| 文件 | 改动 |
|---|---|
| `include/slam_core/pipeline_types.h` | `KeyFrame` 加 `wheel_pose` / `has_wheel_pose` |
| `include/slam_core/i_frontend.h` | 删 `onAbsolutePose` 的 pose 语义注释；保留接口给 `/slam_pose` |
| `include/algorithms/fast_lio2/fast_lio2_frontend.h` | 删 `AbsPose` / `m_abs_buffer` / `m_abs_aligned` / `m_R_lio_map` / `m_t_lio_map`；保留 z-lock 成员并改注释 |
| `src/algorithms/fast_lio2/fast_lio2_frontend.cpp` | 删 `onWheelOdom` 的 `odom_pose` 分支、`onLidar` 的 abs 对齐与 `setPosePrior` 调用；z-lock/z-hold 条件去掉 `odom_pose`；fuse 取 `vel.x` 不再 `abs`；关键帧产出时填 `wheel_pose` |
| `src/algorithms/fast_lio2/map_builder/lidar_processor.{h,cpp}` | `updateOdom` → 3 维；`v.z` 先验条件改为「坡度解锁时」；`updatePlanar` 调用点 |
| `src/algorithms/fast_lio2/map_builder/ieskf.{h,cpp}` | 新增 `updatePlanar`；`updateOdom` 改 3 维；删 `updatePose` |
| `src/slam_core/slam_pipeline.cpp` | `handleMapping` 把 `wheel_pose` 透传给后端（`KeyFrame` 已带） |
| `src/algorithms/ceres_backend/ceres_backend.{h,cpp}` | 新增 `RelativePose2DError`、`addWheelConstraints`，`KeyFrameNode` 加 `wheel_relative` |
| `ros2_ws/src/rosiwit_slam_ros2/src/ros_interface/slam_node.cpp` | `odomCB` 读 `twist.covariance`；`odom_mode` 拒绝 `pose`；`absoluteCB` 保持 |
| `ros2_ws/.../config/vanjee722.yaml` | `odom_mode: fuse`；新增 `use_lateral_constraint` 开关；删除 pose 注释 |

## 8. 验收标准

用 `remap_20260908_odom_ros2` 前 600 s，服务器 `rosiwit/slam-mapping:humble-map`：

1. 帧处理 ≥ 95%（当前 fuse 基线 98.8%，不得回退到 81%）；
2. 平地段 `|z|` 的 p95 < 0.15 m；坡道段 z 单调跟随地面法向，不被压平（与 `traj_lio.csv` 的 z 曲线对照）；
3. `map.pcd` 覆盖 0–600 s 全程（按轨迹每 10 s 采样，位姿附近 2 m 内地图点数 > 0）；
4. PCD 无 `|xyz| > 200` 离群点；
5. `[Map] skip divergent keyframe` 计数 < 前者（350），且不出现 `large 2D pose correction`；
6. `/slam_pose` 打开时，绝对位姿链路功能不回退（用 `remap_3d_localization` 数据集冒烟）。

## 9. 风险

- **R1** 侧向约束对差速平台成立、对全向/滑移转向不成立 → 配置开关默认开，全向平台关闭。
- **R2** 直接拉 roll/pitch 与 IMU 重力对齐冲突 → 只在平地生效，且单次修正限幅（≤0.5°），避免与 IMU 打架。
- **R3** 后端同时有 LIO odometry 边和轮速边，信息被重复计入 → 轮速边权重按协方差归一，必要时降权。
- **R4** `twist.covariance` 全 0 的包会走默认 σ；默认值需在 600 s 上校准，不能把 yaw 拉偏。

## 10. 留给 builder 自行决定

- 具体默认 σ、插值方式、`use_lateral_constraint` 的命名与放置位置；
- `updatePlanar` 中 roll/pitch 修正的收敛方式（直接置位 vs 限幅逼近）；
- 是否需要给后端轮速边加「仅当相邻关键帧间隔 < N s」的守卫。
