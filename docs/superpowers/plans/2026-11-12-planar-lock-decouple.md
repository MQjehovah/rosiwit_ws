# 平面约束解耦 + 删除 pose 模式 实施计划（Increment 1）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让「平地锁 z/roll/pitch、坡道解锁并真实爬升」成为与 odom 模式无关的独立状态约束，并删除 `pose` 模式；用 `odom_mode: none` 跑 600 s 验证 z 能稳住。

**Architecture:** 把坡度判定抽成带迟滞的纯逻辑类（可单测），把平面约束做成 `IESKF::updatePlanar()` 的一次状态写入 + 协方差收紧，在 `LidarProcessor::update()` 的 LiDAR 更新之后调用；前端只负责提供「地面法向/IMU 倾角」和「是否在坡上」，不再与 `odom_pose` 耦合。

**Tech Stack:** C++17 / Eigen / Sophus / PCL / ROS2 Humble / colcon；核心库可用 `cmake -S . -B build && ctest` 单独构建测试。

**Spec:** `docs/superpowers/specs/2026-11-12-odom-fuse-planar-lock-design.md`

## Global Constraints

- 状态更新必须走 IEKF（保持协方差一致）；平面约束以「状态写入 + 协方差收紧」实现，禁止绕过更新的硬赋值（spec C1）。
- 轮速 odom 不含 z / roll / pitch，平面约束不得从 odom 推导（spec C2）。
- `odom_mode: pose` 不再接受，解析时打印错误并提示改用 `fuse`（spec C4）。
- 坡道上不得把 z 压平（spec 反目标）。
- 现有 `slam_pipeline.cpp:806-819` 的关键帧发散过滤保持不变。

## Review Focus

- 地面法向估计失败（点云太少 / 全是墙）时：必须回退到「保持上一状态」，不能默认解锁把 z 放跑。
- IMU 倾角在斜装（实测 42°/82°）下不可信：只在 `ground_ok == false` 且倾角 < 10° 时才用 IMU。
- 解锁瞬间 `v.z` 仍带旧值：解锁第一帧必须清零 `v.z`，避免带着平地约束残留的速度冲上坡。
- 回锁瞬间的 z 阶跃：回锁应把 `z_ref` 取当前 `t_wi.z`（不是建图起点），否则会出现台阶。
- `ba/bg` 在解锁期间被打到 ±1（实测）：解锁期间对 `ba/bg` 加限幅，不允许饱和。

---

### Task 1: 带迟滞的坡度判定 `SlopeDetector`

**Files:**
- Create: `src/rosiwit_slam/include/algorithms/fast_lio2/slope_detector.h`
- Create: `src/rosiwit_slam/src/algorithms/fast_lio2/slope_detector.cpp`
- Test: `src/rosiwit_slam/test/test_slope_detector.cpp`

**Interfaces:**
- Produces:
  - `struct SlopeGateConfig { double enter_deg = 10.0; double exit_deg = 6.0; int enter_frames = 5; int exit_frames = 20; };`
  - `class SlopeDetector { public: explicit SlopeDetector(const SlopeGateConfig&); bool onSlope() const; bool update(bool ground_ok, double ground_tilt_rad, double imu_tilt_rad); double lastSlopeDeg() const; void reset(); };`

- [ ] **Step 1: 写失败测试** `test/test_slope_detector.cpp`，断言：
  1. 平地连续输入（0°）→ `onSlope()==false`；
  2. 12° 只给 `enter_frames-1` 帧 → 仍 `false`（迟滞生效）；
  3. 给满 `enter_frames` 帧 12° → `true`；
  4. 之后回 8°（介于 exit 与 enter 之间）`exit_frames-1` 帧 → 仍 `true`；
  5. 给满 `exit_frames` 帧 8° → `false`；
  6. `ground_ok=false` 且 `imu_tilt=80°` → 不改变当前状态、`onSlope()` 保持原值。
- [ ] **Step 2: 运行确认失败**：`cmake -S src/rosiwit_slam -B src/rosiwit_slam/build && cmake --build src/rosiwit_slam/build --target test_slope_detector`，预期编译失败（头文件不存在）。
- [ ] **Step 3: 实现**：`update()` 用 `enter_frames/exit_frames` 计数器做双阈值迟滞；`ground_ok==false` 时若 `imu_tilt_rad < 10°` 用 IMU，否则**保持**当前状态（不计数）。
- [ ] **Step 4: 运行测试通过**：`ctest --test-dir src/rosiwit_slam/build -R slope_detector --output-on-failure`。
- [ ] **Step 5: Commit**：`git add ...; git commit -m "feat(slam): 带迟滞的坡度判定 SlopeDetector"`

### Task 2: `IESKF::updatePlanar()`

**Files:**
- Modify: `src/rosiwit_slam/include/algorithms/fast_lio2/map_builder/ieskf.h`
- Modify: `src/rosiwit_slam/src/algorithms/fast_lio2/map_builder/ieskf.cpp`
- Test: `src/rosiwit_slam/test/test_ieskf_planar.cpp`

**Interfaces:**
- Consumes: 无（纯状态操作）。
- Produces: `void IESKF::updatePlanar(double z_ref, const M3D& r_flat, bool lock_roll_pitch);`

- [ ] **Step 1: 写失败测试**，断言：
  1. 任意 `t_wi.z` 与 `r_wi`，调用后 `t_wi.z()==z_ref`，`v.z()==0`；
  2. `lock_roll_pitch=true` 时，结果的 roll/pitch 与 `r_flat` 一致（用 `r_flat.transpose()*r_wi` 的 roll/pitch 分量 < 1e-6），且 yaw 保持不变（与调用前差 < 1e-9）；
  3. `lock_roll_pitch=false` 时姿态完全不变；
  4. 调用后 `P(5,5) <= 1e-3`。
- [ ] **Step 2: 运行确认失败**（`updatePlanar` 未定义）。
- [ ] **Step 3: 实现**：保留当前 yaw，构造 `R_new = Rz(yaw) * R_flat_rollpitch`；写 `t_wi.z`、`v.z=0`；收紧 `P(2,2)` 不动的块（`P(5,5)`、roll/pitch 对角）。
- [ ] **Step 4: 运行测试通过**。
- [ ] **Step 5: Commit**。

### Task 3: `LidarProcessor` 调用平面约束（与 odom 模式无关）

**Files:**
- Modify: `src/rosiwit_slam/src/algorithms/fast_lio2/map_builder/lidar_processor.cpp:372-420`
- Modify: `src/rosiwit_slam/include/algorithms/fast_lio2/map_builder/lidar_processor.h`
- Modify: `src/rosiwit_slam/src/algorithms/fast_lio2/map_builder/commons.h`（新增 `planar_lock_enter_deg/exit_deg/enter_frames/exit_frames`、`planar_bias_max`）

**Interfaces:**
- Consumes: `IESKF::updatePlanar`（Task 2）、`SlopeDetector`（Task 1）。
- Produces: `void LidarProcessor::setPlanarConstraint(bool enabled, bool on_slope, double z_ref);`

- [ ] **Step 1: 写失败测试** `test/test_lidar_planar.cpp`：构造 plane 状态，`enabled=true,on_slope=false` 调一次 `update()` 后 `t_wi.z` 被拉到 `z_ref`；`on_slope=true` 时不拉，且 `v.z` 目标为 `R(2,0)*speed`（用 `updateVerticalVelocity` 后的 `v.z` 与期望差 < 0.05）。
- [ ] **Step 2: 运行确认失败**。
- [ ] **Step 3: 实现**：在现有 `m_z_lock_valid` 分支处替换为 `setPlanarConstraint`；解锁第一帧 `v.z=0`；解锁期间 `ba/bg` 限幅到 `planar_bias_max`（默认 `ba=0.3`、`bg=0.05`）。
- [ ] **Step 4: 测试通过**。
- [ ] **Step 5: Commit**。

### Task 4: 前端解耦 + 删除 pose 模式

**Files:**
- Modify: `src/rosiwit_slam/include/algorithms/fast_lio2/fast_lio2_frontend.h`（删 `AbsPose`、`m_abs_buffer`、`m_abs_aligned`、`m_R_lio_map`、`m_t_lio_map`）
- Modify: `src/rosiwit_slam/src/algorithms/fast_lio2/fast_lio2_frontend.cpp`（删 `onWheelOdom` 的 `odom_pose` 分支、`onLidar` 的 abs 对齐块与 `setPosePrior` 调用；z-lock/z-hold 条件去掉 `odom_pose`，改为调用 `SlopeDetector` + `setPlanarConstraint`）
- Modify: `ros2_ws/src/rosiwit_slam_ros2/src/ros_interface/slam_node.cpp:277-301`（`odom_mode: pose` → 打印错误并退出）
- Modify: `ros2_ws/src/rosiwit_slam_ros2/config/vanjee722.yaml`

**Interfaces:**
- Consumes: `SlopeDetector`、`LidarProcessor::setPlanarConstraint`。
- Produces: `odom_mode` 只接受 `none | gate | fuse`；`pose_lock_z` / `pose_z_hold` 在三种模式下均生效。

- [ ] **Step 1: 写失败测试** `test/test_odom_mode_parse.cpp`：`odom_mode: pose` 解析失败；`fuse/none/gate` 解析成功且 `pose_z_hold` 保留为 true。
- [ ] **Step 2: 运行确认失败**。
- [ ] **Step 3: 实现**：删 pose 分支；`slam_node.cpp` 对 `pose` 报 `"odom_mode: pose 已移除, 绝对位姿请走 /slam_pose"` 并 `return false`。
- [ ] **Step 4: 测试通过 + 全量构建** `colcon build --packages-select rosiwit_slam_ros2`。
- [ ] **Step 5: Commit**。

### Task 5: 服务器 600 s 验证（`odom_mode: none`）

**Files:**
- Modify/Create: `config/vanjee722_none_planar.yaml`（`odom_mode: none`、`pose_z_hold: true`、`pose_z_hold_slope_deg: 10`、`loop_closure: ""`）

- [ ] **Step 1:** 同步源码到服务器并 `colcon build`（容器 `rosiwit/slam-mapping:humble-map`，40 核，约 5 min）。
- [ ] **Step 2:** 跑 600 s（`timeout 600 ros2 bag play ...`），保存 `map.pcd`。
- [ ] **Step 3: 验收断言**（脚本检查）：
  - `z` 的 p95 `|z| < 0.15 m`（平地段，用 `traj_lio.csv` 中 `|slope|<6°` 的样本）；
  - 轨迹每 10 s 采样，位姿 2 m 内地图点数 > 0（覆盖全程）；
  - `map.pcd` 无 `|xyz| > 200`；
  - `[Map] skip divergent keyframe` < 50；
  - 帧处理 ≥ 95%。
- [ ] **Step 4:** 结果写入 `docs/superpowers/specs/` 的实验记录并 commit。

## Self-Review

- 覆盖 spec：G1（Task 4）、G3（Task 1-3）、G0 验证（Task 5）；G2（fuse 吃满 odom）与 G4（后端轮速因子）留给 Increment 2/3，不在本计划内。
- 类型一致性：`SlopeDetector::update(bool, double, double)`、`IESKF::updatePlanar(double, const M3D&, bool)`、`LidarProcessor::setPlanarConstraint(bool, bool, double)` 在三处引用一致。
- Review Focus 的五条各自已落到 Task 1 Step 1 的断言、Task 3 Step 1/3。
