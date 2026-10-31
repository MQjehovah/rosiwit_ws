# 定位架构（LIO + S2M + ESKF）实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让定位模式同时使用 LIO 里程计、轮式 odom、S2M 绝对位姿与 IMU，由一个 ESKF 输出位姿；并在地图插入处加门控，避免低置信位姿污染先验地图。

**Architecture:** `SlamPipeline` 在 LOCALIZATION 模式下把 IMU 同时喂给前端与定位核心、把雷达帧先喂 LIO 前端再喂定位核心；`LocalizationPipeline` 通过新增的 `onRelative()` 把 LIO 相对位姿转给 `IPoseFusion`；`EkfPoseFusion` 内做四路融合（IMU 过程、odom 速度、LIO 相对、S2M 绝对）与单帧限幅；`OverlayMapMaintainer` 仅在 `health == TRACKING` 且本帧 S2M 被接受且 LIO/S2M 分歧达标时写入叠加层。

**Tech Stack:** C++17、ROS1 Melodic、catkin、gtest（`catkin_add_gtest`）、PCL、Eigen、yaml-cpp

**Spec:** `docs/superpowers/specs/2026-10-31-localization-architecture-lio-eskf.md`

## Global Constraints

- 只在 `LOCALIZATION` 模式生效；`MAPPING` / `IDLE` 行为不得改变。
- 2D 雷达（`/scan`）不得进入定位判据（临时件）。
- 所有新增参数必须有默认值，且默认值 = 改造前行为（可零配置回退）。
- 新增参数同时写入包内 `config/default.yaml` 与设备 `~/.ros/rosiwit_slam_ros1/compat_legacy.yaml`。
- 文档与注释中的图一律 Mermaid。
- 单帧端到端预算 100 ms；LIO 接入后若超预算，允许前端降频到 5 Hz（由参数控制）。
- 编译验证命令：`cd /home/xzrobot/rosiwit_ws/ros1_ws && catkin_make -j3`；测试：`catkin_make run_tests_rosiwit_slam_ros1`。
- 部署：`catkin_make` → 拷 `librosiwit_slam*.so` 与节点到 `/opt/ros/melodic/lib` → `systemctl restart xzrobot`。

## Review Focus

1. **LIO 前端的 IMU 缓冲无界增长**：定位模式若不喂雷达帧而喂 IMU，`FastLio2Frontend` 的缓冲会持续增长；必须保证每帧雷达都消费一次。测试见 Task 4。
2. **时间基准不一致**：相对位姿必须用雷达帧时间对齐，不能用"最新 LIO 位姿"。测试见 Task 3。
3. **打滑检测除零**：`|Δodom| ≈ 0` 时比值发散，需退化处理。测试见 Task 3。
4. **插入门控的组合条件**：同一帧可能被多次回调触发，重复写入会放大叠加层。测试见 Task 6。
5. **模式切换残留**：`LOCALIZATION → MAPPING` 后前端缓冲与 `T_map_lio` 必须清空。测试见 Task 4。

---

### Task 1: 融合层新增相对位姿观测接口

**Files:**
- Modify: `src/rosiwit_slam/include/slam_core/i_pose_fusion.h`
- Modify: `src/rosiwit_slam/include/algorithms/localization/fusion/ekf_pose_fusion.h`
- Modify: `src/rosiwit_slam/src/algorithms/localization/fusion/ekf_pose_fusion.cpp`
- Test: `ros1_ws/src/rosiwit_slam_ros1/test/test_pose_fusion.cpp`（新建）
- Modify: `ros1_ws/src/rosiwit_slam_ros1/CMakeLists.txt`（注册 gtest）

**Interfaces:**
- Produces: `virtual void IPoseFusion::onRelative(const PoseStamped& pose, const Eigen::Matrix<double,6,6>& cov)`；`FusionDiag` 新增 `int relative_accepted; double last_rel_residual;`
- Consumes: 现有 `PoseStamped`（`trans/rot/time`）、`Eigen::Matrix<double,6,6>`（顺序 rot3→trans3，与 `MatchResult::cov` 一致）

- [ ] **Step 1: 写失败测试**

```cpp
// 相对观测：LIO 报告前进 0.10 m，滤波预测前进 0.02 m -> 位姿应朝 0.10 m 方向移动
TEST(PoseFusion, RelativeMeasurementPullsTowardsLio) {
    EkfPoseFusion f;
    PoseFusionConfig cfg;             // 默认值
    cfg.chi2_gate = 0.0;              // 本测试只验证方向
    f.configure(cfg);
    PoseStamped p0; p0.time = 1.0; p0.trans = {0,0,0}; p0.rot = I;
    f.reset(p0);
    PoseStamped lio0 = p0;
    f.onRelative(lio0, Identity6());
    PoseStamped lio1 = p0; lio1.time = 1.1; lio1.trans.x() = 0.10;
    f.onRelative(lio1, Identity6());
    PoseStamped out = f.predict(1.1);
    EXPECT_GT(out.trans.x(), 0.01);
    EXPECT_LE(out.trans.x(), 0.10);   // 限幅内，不会一步到位
}
```

- [ ] **Step 2: 运行确认失败**

Run: `catkin_make run_tests_rosiwit_slam_ros1 --pkg rosiwit_slam_ros1`
Expected: 编译失败，`onRelative` 不是 `IPoseFusion` 成员

- [ ] **Step 3: 实现接口与最小逻辑**

`i_pose_fusion.h` 增加默认空实现（保持 `direct` 不受影响）：

```cpp
virtual void onRelative(const PoseStamped&, const Eigen::Matrix<double,6,6>&) {}
```

`EkfPoseFusion::onRelative` 的语义（固定决策，避免实现者自行发挥）：
1. 首次调用只记录 `m_rel_prev_lio = pose` 与 `m_rel_prev_state = m_p/m_R`，直接返回；
2. 之后计算 `d_lio = m_rel_prev_lio.rot⁻¹·(pose.trans − m_rel_prev_lio.trans)`（体系平移增量）与 `d_state = m_R⁻¹·(m_p − m_rel_prev_state)`；
3. 残差 `e = d_lio − d_state`（3 维），协方差取 `cov` 的平移块 + `trans_cov_floor`；
4. 按 `gain_trans` 上限做一次标量增益修正 `m_p += m_R · (K⊙e)`，并按 `max_correction_trans` 限幅；
5. 更新 `m_diag.relative_accepted` 与 `last_rel_residual = ‖e‖`；
6. 更新 `m_rel_prev_lio` / `m_rel_prev_state`。

- [ ] **Step 4: 运行确认通过**

Run: `catkin_make run_tests_rosiwit_slam_ros1 --pkg rosiwit_slam_ros1`
Expected: `test_pose_fusion` PASS

- [ ] **Step 5: 提交**

```bash
git add src/rosiwit_slam/include/slam_core/i_pose_fusion.h \
        src/rosiwit_slam/include/algorithms/localization/fusion/ekf_pose_fusion.h \
        src/rosiwit_slam/src/algorithms/localization/fusion/ekf_pose_fusion.cpp \
        ros1_ws/src/rosiwit_slam_ros1/test/test_pose_fusion.cpp \
        ros1_ws/src/rosiwit_slam_ros1/CMakeLists.txt
git commit -m "feat(slam): pose_fusion 支持相对位姿观测(LIO)"
```

---

### Task 2: 定位流水线转发相对观测

**Files:**
- Modify: `src/rosiwit_slam/include/slam_core/i_localization.h`
- Modify: `src/rosiwit_slam/include/algorithms/localization/localization_pipeline.h`
- Modify: `src/rosiwit_slam/src/algorithms/localization/localization_pipeline.cpp`
- Modify: `src/rosiwit_slam/include/slam_core/slam_pipeline.h`
- Modify: `src/rosiwit_slam/src/slam_core/slam_pipeline.cpp`
- Test: `ros1_ws/src/rosiwit_slam_ros1/test/test_pose_fusion.cpp`（追加用例）

**Interfaces:**
- Consumes: Task 1 的 `IPoseFusion::onRelative`
- Produces: `ILocalization::onRelative(const PoseStamped&, const Eigen::Matrix<double,6,6>&)`；`SlamPipeline::onRelative(const PoseStamped&, const Eigen::Matrix<double,6,6>&)`；`ISlamAlgorithm::onRelative(...)` 默认空实现

- [ ] **Step 1: 写失败测试（direct 模式下不得改变输出）**

```cpp
// direct 模式：相对观测不得影响位姿（保持回退语义）
TEST(PoseFusion, DirectIgnoresRelative) {
    DirectPoseFusion f;
    PoseStamped p; p.time = 1.0; p.trans = {1,2,0}; p.rot = I;
    f.reset(p);
    PoseStamped lio = p; lio.trans.x() = 9.0;
    f.onRelative(lio, Identity6());
    EXPECT_NEAR(f.predict(1.0).trans.x(), 1.0, 1e-9);
}
```

- [ ] **Step 2: 运行确认失败**

Expected: 编译失败（`DirectPoseFusion` 无 `onRelative`，需继承默认实现后仍然通过——若已通过则说明继承生效，直接进入 Step 3 并在报告中注明）

- [ ] **Step 3: 接通调用链**

按顺序加四层转发，全部只在 `LOCALIZATION` 模式调用：
1. `ILocalization::onRelative` 纯虚 + `LocalizationPipeline` 实现：加锁 `m_fusion_mu` 后 `m_fusion->onRelative(...)`；
2. `SlamPipeline::onRelative`：`if (m_mode==LOCALIZATION && m_localization) m_localization->onRelative(...)`；
3. `ISlamAlgorithm::onRelative` 默认空实现 + `SlamPipeline` override；
4. `SlamAlgorithm`（ROS1 引擎）透传方法，供 ROS 层调用（Task 4 使用）。

- [ ] **Step 4: 运行确认通过并提交**

Run: `catkin_make run_tests_rosiwit_slam_ros1 --pkg rosiwit_slam_ros1`

```bash
git commit -am "feat(slam): 相对位姿观测贯通到 pose_fusion"
```

---

### Task 3: odom 打滑门控

**Files:**
- Modify: `src/rosiwit_slam/include/slam_core/localization_stage_types.h`（`PoseFusionConfig` 增字段）
- Modify: `src/rosiwit_slam/src/algorithms/localization/fusion/ekf_pose_fusion.cpp`
- Test: `ros1_ws/src/rosiwit_slam_ros1/test/test_pose_fusion.cpp`（追加用例）

**Interfaces:**
- Produces: `PoseFusionConfig::slip_ratio_lo/hi`、`slip_yaw_thresh_deg`、`odom_weight_min`；`FusionDiag::odom_weight`
- Consumes: 现有 `onOdom(const PoseStamped&)`（体体系速度观测）

- [ ] **Step 1: 写失败测试**

```cpp
// 打滑：位姿只前进 0.02 m，而 odom 报告 0.20 m -> odom 权重应降到下限
TEST(PoseFusion, OdomSlipDownweights) {
    EkfPoseFusion f; PoseFusionConfig cfg;
    cfg.slip_ratio_lo = 0.6; cfg.slip_ratio_hi = 1.6; cfg.odom_weight_min = 0.0;
    f.configure(cfg);
    PoseStamped p; p.time = 0.0; p.rot = I; f.reset(p);
    // 用匹配位移锚定"真实"位移 0.02 m
    PoseStamped m = p; m.time = 0.1; m.trans.x() = 0.02; f.onMatch(m, Identity6(), {});
    // odom 报 0.20 m
    PoseStamped o0 = p; o0.time = 0.1; f.onOdom(o0);
    PoseStamped o1 = p; o1.time = 0.2; o1.trans.x() = 0.20; f.onOdom(o1);
    EXPECT_LT(f.diag().odom_weight, 0.2);
}
```

- [ ] **Step 2: 运行确认失败**（`diag().odom_weight` 不存在）

- [ ] **Step 3: 实现门控**

在 `onOdom` 内：
1. 用最近一次被接受的匹配位姿与时间，算出 `d_pose`；用同区间的 `d_odom` 算 `ratio = |d_pose| / max(|d_odom|, 1e-3)`；
2. `weight = 1` 若 `ratio ∈ [lo, hi]`，否则线性衰减到 `odom_weight_min`；
3. `|odom yaw rate − gyro yaw rate| > slip_yaw_thresh_deg` 时再乘 0.2；
4. 用 `weight²` 放大速度观测噪声（等价于降权），并写入 `m_diag.odom_weight`；
5. `|d_odom| < 1e-3` 时 `ratio` 取 1.0（除零保护）。

- [ ] **Step 4: 运行确认通过并提交**

---

### Task 4: 定位模式启用 LIO 前端

**Files:**
- Modify: `src/rosiwit_slam/src/slam_core/slam_pipeline.cpp`（`onImu` / `onLidar` / `init` / 模式切换清空）
- Modify: `src/rosiwit_slam/include/slam_core/slam_pipeline.h`
- Test: `ros1_ws/src/rosiwit_slam_ros1/test/test_pose_fusion.cpp`（追加用例）

**Interfaces:**
- Consumes: `IFrontend::onImu`、`IFrontend::onLidar`、`IFrontend::getOdometry(PoseStamped&)`
- Produces: `SlamPipeline::onRelative` 的调用方（本任务实现）
- 配置：`localization.use_lio_relative: bool`（默认 `false`）、`localization.lio_rel_cov_trans: 0.02`、`lio_rel_cov_rot: 0.002`、`lio_frontend_hz: 10`

- [ ] **Step 1: 写失败测试（模式切换后缓冲清零）**

```cpp
// 模式切出 LOCALIZATION 时必须清空 LIO 相对观测的锚点
TEST(PoseFusion, RelativeStateClearedOnReset) {
    EkfPoseFusion f; PoseFusionConfig cfg; f.configure(cfg);
    PoseStamped p; p.time = 1.0; f.reset(p);
    f.onRelative(p, Identity6());
    f.reset(p);                       // 模拟模式切换/重定位
    PoseStamped q = p; q.time = 1.1; q.trans.x() = 5.0;
    f.onRelative(q, Identity6());     // 首帧只记录锚点
    EXPECT_LT(f.diag().relative_accepted, 2);
}
```

- [ ] **Step 2: 运行确认失败**（`relative_accepted` 未清零或计数行为不符）

- [ ] **Step 3: 实现前端接入**

1. `init()`：`LOCALIZATION` 模式下也创建 `m_frontend`（仅当 `use_lio_relative=true`）；缺前端时打 WARN 并退回纯 odom/S2M 融合；
2. `onImu()` 的 `LOCALIZATION` 分支：`if (m_frontend) m_frontend->onImu(s);`（必须与下一行同时存在，保证每帧被消费）；
3. `handleLocalization(f)` 开头：`if (m_frontend) { m_frontend->onLidar(f); PoseStamped odom; if (m_frontend->getOdometry(odom)) m_localization->onRelative(odom, CovFromConfig()); }`；
4. `setMode()` 切出 `LOCALIZATION` 时：`m_frontend.reset()` 或 `m_frontend->onLidar` 缓冲清空（按前端实现暴露的 reset，若无则重建实例）。

- [ ] **Step 4: 运行确认通过 + 现场验证**

Run（设备）：`catkin_make && systemctl restart xzrobot`
检查：`/slam/fusion_diag` 的 `relative_accepted` 每 2 s 递增；`top` 中前端 CPU 增量 < 25%。

- [ ] **Step 5: 提交**

---

### Task 5: 修好融合诊断输出

**Files:**
- Modify: `ros1_ws/src/rosiwit_slam_ros1/src/interface/pose_publisher.cpp`
- Modify: `ros1_ws/src/rosiwit_slam_ros1/src/interface/assembly.cpp`
- Test: 无单测（ROS 集成），用 `rostopic hz` 验证

**Interfaces:**
- Consumes: `SlamAlgorithm::poseFusionDiagnostics()`
- Produces: `/slam/fusion_diag`（`std_msgs/String`，JSON，1 Hz）

- [ ] **Step 1: 复现问题**（当前该话题无消息）

Run（设备）：`timeout 5 rostopic hz /slam/fusion_diag`
Expected: `no new messages`

- [ ] **Step 2: 定位原因**

检查 `publishLocationInfo()` 是否被调用（`[locstate]` 日志是否存在）与 `fusion_diag_puber_` 是否为空；若话题未 advertise，检查 `initPub()` 是否在 `sys` 初始化之后调用。

- [ ] **Step 3: 修复并验证**

Run：`timeout 8 rostopic echo -n2 /slam/fusion_diag`
Expected: 两帧 JSON，含 `mode/accepted/relative_accepted/rejected_*/applied_trans`

- [ ] **Step 4: 提交**

---

### Task 6: 地图插入门控

**Files:**
- Modify: `ros1_ws/src/rosiwit_slam_ros1/src/interface/pose_publisher.cpp`（把 health 与"本帧匹配是否被接受"传给 maintainer）
- Modify: `src/rosiwit_slam/src/algorithms/localization/localization_pipeline.cpp`（`onLocalized` 调用处加条件）
- Test: `ros1_ws/src/rosiwit_slam_ros1/test/test_pose_fusion.cpp`（追加门控判定函数用例）

**Interfaces:**
- Produces: `bool allowMapInsert(bool tracking, bool match_accepted, double lio_s2m_divergence_m, const MaintainerConfig&)`
- Consumes: `MatchResult`（本帧接受状态）、`FusionDiag::last_rel_residual`（作为 LIO/S2M 分歧代理）

- [ ] **Step 1: 写失败测试**

```cpp
TEST(MapGate, RejectsWhenNotTrackingOrDiverged) {
    MaintainerConfig cfg; cfg.max_insert_divergence = 0.2;
    EXPECT_FALSE(allowMapInsert(false, true, 0.01, cfg));  // 非 TRACKING
    EXPECT_FALSE(allowMapInsert(true, false, 0.01, cfg));  // 匹配未被接受
    EXPECT_FALSE(allowMapInsert(true, true, 0.50, cfg));   // LIO/S2M 分歧过大
    EXPECT_TRUE (allowMapInsert(true, true, 0.05, cfg));   // 全部满足
}
```

- [ ] **Step 2: 运行确认失败**（`allowMapInsert` 未定义）

- [ ] **Step 3: 实现并接入**

1. 在 `localization_stage_types.h` 的 `MaintainerConfig` 增加 `double max_insert_divergence = 0.2;`；
2. 实现 `allowMapInsert`（纯函数，便于测试）；
3. `localization_pipeline.cpp` 里把 `m_maintainer->onLocalized(...)` 包在 `if (allowMapInsert(...))` 内，参数取：`m_status == LOCALIZED`、本帧 `accepted`、`m_fusion->diag().last_rel_residual`。

- [ ] **Step 4: 运行确认通过 + 现场验证**

现场：在"变化区域"跑一趟，检查叠加层体素数增长 < 5%，且下一趟该区域匹配分数上升。

- [ ] **Step 5: 提交**

---

## 自检

- **Spec 覆盖**：架构文档第 2 节（四路输入）→ Task 1/2/3/4；第 4 节（五条插入规则）→ Task 6 覆盖规则 1~3，规则 4/5（单趟增量上限、多趟确认）留待后续计划；第 6 节（三个待测数）→ Task 3/4 的现场验证。
- **类型一致性**：`PoseStamped`、`Eigen::Matrix<double,6,6>`（rot3→trans3）贯穿全计划；`FusionDiag` 字段名在 Task 1/3/6 一致。
- **未覆盖项（需后续计划）**：多趟确认与三层地图（规则 4/5）、LIO CPU 降频策略、退化部分更新。
