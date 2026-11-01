# 导航速度/规划/重规划 三项修复 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修掉"建图模式下导航被砍半速"，并让"路径被轮廓检查拒绝"和"运行中被堵"两种情况能自愈，而不是直接失败或等 30 s。

**Architecture:** 把"定位退化 → 限速"的判断从节点里抽成 core 的纯函数（可单测），节点只做取值与发布；规划被拒时增加一次"放宽规划余量重试"，再退化为失败；重规划策略用仿真量化后调参，不再靠猜。

**Tech Stack:** C++17 / catkin (ROS1 melodic) / gtest / WSL2 + Gazebo 仿真（`scripts/sim/run_nav_sim.sh`）

**Spec:** 证据与本轮定位过程见 `docs/dev/nav-ros1-thread-model.md`（线程模型 + 仿真 A/B 方法）与下方 Background。

## Background（本轮实测数据，WSL Gazebo）

| 场景 | 命令峰值 v | 命令峰值 ω | 用时(墙钟) |
|---|---|---|---|
| large.world 10 m 直走廊，现状 | 0.400 | 0.400 | 42.0 s |
| 同上，仅把 `slam_degrade_scale` 改 1.0 | 0.800 | 0.779 | 31.2 s |

同一时刻控制器内部 `[RPPProbe] v_ref=0.799`，安全层关闭后峰值仍 0.385 → 损失点在
`slamHealthScale()`（节点第 4657 行）：mapping 模式下 `/slam/info.matching_degree == 0`，
`0 < 0.65` 被判成"定位退化"，整条指令乘 `slam_degrade_scale=0.5`。同文件 4845 行附近的注释
已经写明"matching==0 是纯建图，不能算退化"，该判断漏了同样的守卫。

## Global Constraints

- ROS1 melodic + C++17；不要引入新依赖。
- 所有中文注释/日志保持 UTF-8（禁止 PowerShell `Get-Content/Set-Content` 改源文件）。
- core 库改动必须 `catkin_make` 通过且 `test_*` 全绿。
- 实车默认值不许被仿真调参污染：仿真参数只改 `config/simulation.yaml`。
- 每个任务单独 commit。

## Review Focus

- `matching_degree` 缺失/为 0（建图模式、假 SLAM、bag 回放）→ 不能限速，也不能被当成"定位丢失"。
- `matching_degree` 在阈值附近抖动 → 限速要能恢复，不能锁死。
- 沿路障碍导致规划被拒 / 中途被堵 → 任务应自愈，不能无提示地失败。
- 放宽规划余量后仍要走安全层与安全区，不能为了通过检查牺牲碰撞安全。

---

### Task 1: core 纯函数 `localizationSpeedScale` + 单测

**Files:**
- Create: `src/rosiwit_navigation/include/rosiwit_navigation/nav_core/localization_health.hpp`
- Test: `src/rosiwit_navigation/test/test_localization_health.cpp`
- Modify: `ros1_ws/src/rosiwit_navigation_ros1/CMakeLists.txt`（在 `test_velocity_shaper` 之后加一个 `catkin_add_gtest`）

**Interfaces:**
- Produces: `double core::localizationSpeedScale(bool info_fresh, double matching, double degrade_matching, double degrade_scale);`
  - `info_fresh == false` → `1.0`（没有健康信息不额外限速）
  - `matching <= 0.0` → `1.0`（无匹配分：建图 / 未提供分数）
  - `0 < matching < degrade_matching` → `degrade_scale`
  - 其余 → `1.0`

- [ ] **Step 1: 写失败测试**

```cpp
// test_localization_health.cpp
#include <gtest/gtest.h>
#include "rosiwit_navigation/nav_core/localization_health.hpp"

TEST(LocalizationHealth, NoInfoMeansFullSpeed) {
  EXPECT_DOUBLE_EQ(core::localizationSpeedScale(false, 0.1, 0.65, 0.5), 1.0);
}
TEST(LocalizationHealth, MappingZeroMatchingIsNotDegraded) {
  EXPECT_DOUBLE_EQ(core::localizationSpeedScale(true, 0.0, 0.65, 0.5), 1.0);
}
TEST(LocalizationHealth, LowMatchingDegrades) {
  EXPECT_DOUBLE_EQ(core::localizationSpeedScale(true, 0.40, 0.65, 0.5), 0.5);
}
TEST(LocalizationHealth, AtThresholdIsHealthy) {
  EXPECT_DOUBLE_EQ(core::localizationSpeedScale(true, 0.65, 0.65, 0.5), 1.0);
}
TEST(LocalizationHealth, GoodMatchingFullSpeed) {
  EXPECT_DOUBLE_EQ(core::localizationSpeedScale(true, 0.97, 0.65, 0.5), 1.0);
}
```

- [ ] **Step 2: 跑测试确认失败**

Run: `cd ~/rosiwit_ws/ros1_ws && catkin_make -j8 && ./devel/lib/rosiwit_navigation_ros1/test_localization_health`
Expected: 编译失败（头文件不存在）

- [ ] **Step 3: 实现头文件**

header-only，`namespace rosiwit_navigation { namespace core {`，函数体按 Interfaces 的四条规则，注释写明"matching==0 表示没有定位分（建图），不是退化"。

- [ ] **Step 4: 跑测试确认通过**

Run: 同上
Expected: `[  PASSED  ] 5 tests.`

- [ ] **Step 5: 提交**

```bash
git add src/rosiwit_navigation/include/rosiwit_navigation/nav_core/localization_health.hpp \
        src/rosiwit_navigation/test/test_localization_health.cpp \
        ros1_ws/src/rosiwit_navigation_ros1/CMakeLists.txt
git commit -m "feat(nav_core): localizationSpeedScale 纯函数 + 单测 (matching=0 不限速)"
```

---

### Task 2: 节点接入 + 诊断字段 + 关掉 RPP 调试输出

**Files:**
- Modify: `ros1_ws/src/rosiwit_navigation_ros1/src/rosiwit_navigation_ros1_node.cpp`
- Modify: `src/rosiwit_navigation/include/rosiwit_navigation/nav_core/types.hpp`（`ControllerConfig` 加 `bool debug_probe = false;`）
- Modify: `src/rosiwit_navigation/src/algorithms/local_planner/rpp_controller.cpp`（两处 `std::cout` 用 `cfg_.debug_probe` 门控）
- Test: 单测（Task 1）+ 仿真 A/B

**Interfaces:**
- Consumes: `core::localizationSpeedScale(...)`（Task 1）
- Produces: `/nav_diagnostics` JSON 新增 `"health_scale": <double>`；新参数 `controller_debug_probe`（bool，默认 false）

- [ ] **Step 1: 节点 `slamHealthScale()` 换成纯函数**

```cpp
double slamHealthScale() const
{
  const bool fresh = !last_slam_info_stamp_.isZero() &&
    (ros::Time::now() - last_slam_info_stamp_).toSec() <= slam_info_timeout_;
  return core::localizationSpeedScale(fresh, last_slam_matching_,
                                      slam_degrade_matching_, slam_degrade_scale_);
}
```

- [ ] **Step 2: 诊断加 `health_scale`**

在 diagnostics JSON 里 `slamHealthScale()` 结果旁边加 `,"health_scale":<v>`（与 `"safety_scale"` 同级），保证现场一眼能看出"速度被谁砍了"。

- [ ] **Step 3: RPP 调试输出收进参数**

`ControllerConfig` 加 `bool debug_probe = false;`；节点 `initController` 里 `cfg.debug_probe = controller_debug_probe_;`，参数默认 `false`；
`rpp_controller.cpp` 的 `[RPPProbe]`/`[RPPClr]` 两处 `std::cout` 前置 `if (cfg_.debug_probe)`。

- [ ] **Step 4: 编译 + 单测**

Run: `cd ~/rosiwit_ws/ros1_ws && catkin_make -j8 && for t in test_safety_monitor test_action_task test_velocity_shaper test_distance_field test_localization_health; do ./devel/lib/rosiwit_navigation_ros1/$t | tail -1; done`
Expected: 全部 `[  PASSED  ]`

- [ ] **Step 5: 仿真 A/B（红→绿）**

Run: `cd ~/rosiwit_ws && ./scripts/sim/run_nav_sim.sh --driver goal --label A_after --world $HOME/rosiwit_ws/ros1_ws/src/rosiwit_simulation_ros1/world/large.world --spawn -18,-18,1.57 --band 8,12`
Expected: `cmd_max_v ≥ 0.75`、`cmd_max_w ≥ 0.7`、`status=SUCCEEDED`、`xtrack_rms_m ≤ 0.05`、用时比 42 s 明显短
（现状基线 0.400 / 42.0 s）

- [ ] **Step 6: 提交**

```bash
git commit -am "fix(nav): matching=0(建图)不再触发 0.5 限速; 诊断加 health_scale; RPP 调试输出收进参数"
```

---

### Task 3: 轮廓检查被拒时的自愈（替代"直接失败"）

**Files:**
- Modify: `ros1_ws/src/rosiwit_navigation_ros1/src/rosiwit_navigation_ros1_node.cpp`（`applyPlanningResult` 拒绝分支、`launchPlan`）
- Test: 仿真 `--band 5,7`（当前该场景曾出现 `FAILED_PATH_FOOTPRINT`）

**Interfaces:**
- Consumes: 现有 `pathFootprintClear()` / `startRecovery()`
- Produces: 新成员 `int footprint_retry_ = 0;` 与新参数 `footprint_retry_margin`（默认 0.04 m）

- [ ] **Step 1: 复现并记录证据**

Run: 3 次 `--driver goal --label B_before --band 5,7`
记录：`status`、`plan_ms_max`、日志里 `[plan] 拒绝执行: 路径让车体轮廓压到障碍` 的次数

- [ ] **Step 2: 实现"放宽余量重试一次"**

拒绝分支里：若 `footprint_retry_ == 0`，`footprint_retry_ = 1`，把 `planner_footprint_` 临时按 `footprint_retry_margin` 内缩（`insetFootprint(configured_footprint_, -(planner_footprint_margin_ - footprint_retry_margin))`），`launchPlan()` 重来；成功应用后把 `footprint_retry_` 归零并恢复 `planner_footprint_`。仍被拒才走现有的 recovery → fail。
注释写清"只放宽到车体轮廓，不放宽安全层"。

- [ ] **Step 3: 复测**

Run: 同 Step 1 的 3 次
Expected: `FAILED_PATH_FOOTPRINT` 次数下降（目标 0/3），且至少 2/3 到达目标；日志出现"放宽余量重规划"1 次

- [ ] **Step 4: 提交**

```bash
git commit -am "fix(nav): 路径轮廓检查被拒时先放宽规划余量重试一次, 不再立刻失败"
```

---

### Task 4: 运行中被堵的重规划/恢复策略量化

**Files:**
- Modify: `ros1_ws/src/rosiwit_navigation_ros1/config/simulation.yaml`（候选参数）
- Create/Modify: `scripts/sim/nav_sim_blocked.py`（走一半后把禁行区横在路径上，量 replan 延迟与恢复结果）

**Interfaces:**
- Consumes: `/virtual_obstacles`、`/nav_diagnostics`
- Produces: 报告字段 `replan_after_block_s`（从堵住到新路径的点数变化）、`recovery_used`、`final_status`

- [ ] **Step 1: 写复现脚本**（先不改参数，量基线：`replan_on_blocked=false`、`replan_max_interval=20`）
- [ ] **Step 2: 量基线 3 次**，记录：被堵后多久换路、是否进 recovery、最终状态
- [ ] **Step 3: 调参复测**（`replan_on_blocked=true`、`replan_max_interval` 8、`replan_min_interval` 1.5）同场景 3 次
- [ ] **Step 4: 把有数据支撑的参数写进 `simulation.yaml`，并把结论写进 `docs/dev/nav-ros1-thread-model.md`**
- [ ] **Step 5: 提交**

```bash
git commit -am "tune(nav): 被堵场景重规划策略按仿真数据调整 + 复现脚本"
```
