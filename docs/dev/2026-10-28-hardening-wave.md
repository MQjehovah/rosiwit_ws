# 2026-10-28 加固轮：审查 → 修（P0 + 覆盖率 + 核心 + 基础层）

6 份独立只读审查（SLAM 算法 / SLAM 核心基础设施 / SLAM-ROS1 层 / 导航核心 / 导航节点+覆盖率 /
构建与配置一致性）产出的问题，逐条核实后修了 22 处；**撤销了 7 条虚警**（见文末）。

## 一、导航 P0（安全 / 正确性）

| # | 问题 | 修法 |
|---|---|---|
| 1 | `/nav_stop` 后旧规划被重新启动：`stopNavigation` 只 `++generation`，`pollPlanning` 的过期分支看到 `pending_goal_` 非空仍 `launchPlan()` | `stopNavigation` 清 `pending_goal_`；重启条件加 `is_navigating_`；`applyPlanningResult` 再校验 generation |
| 2 | 换 profile 会把正在 `plan()` 的 planner 析构（UAF） | planner 改 `shared_ptr` + 异步 lambda 按值捕获；有规划在飞时拒绝换 profile |
| 3 | `cancel()`(spinner) 与 `plan()`(worker) 竞争 `planning_active_` | 改 `std::atomic<bool>` |
| 4 | 恢复复用上一轮规划结果（`recovery_plan_gen_` 只 `++` 从不比较） | 结果带 `gen`，不匹配就丢弃重规划 |
| 5 | 恢复预算不重置：两次成功后第三次 safety stop 必然 `FAILED_SAFETY_STOP` | `finishRecovery` 归零 `recovery_attempts_` |
| 6 | 命令为 0（停车/等待/规划中）时仍按"旋转区"判定，侧后方墙锁存 STOP → 2 s 后误判 blocked | STOP 区选择与"是否在转"统一；主动零速不 arm 看门狗 |
| 7 | 安全硬停后 `VelocityShaper` 仍从"停前速度"斜坡 | 硬停时 `reset()` |
| 8 | DWA 接受 `cost=99`（内切圆带）轨迹 | `clearance < 0.01` 直接丢弃 |
| 9 | **窄通道**：前向停止区侧向 ±0.40，0.8 m 门洞的墙正好在 ±0.40，`pointInPolygon` 用严格 `>` → 横向误差 >1 mm 就停 | 侧向半宽改 **0.345**（车体半宽 0.33 + 15 mm），三份 YAML + 代码默认值同步 |
| 10 | `safety_slowdown_polygon` 重复定义（后一处覆盖前一处） | 删旧行，实车与仿真统一为前方 0.55–1.15 m |
| 11 | `planner_heuristic_weight` / `safety_stop_max_points` / `safety_slowdown_max_points` 只有代码默认 | 三个 `pnh_.param` 补上 |

## 二、覆盖率

| # | 问题 | 修法 |
|---|---|---|
| 12 | `robot_inset=0.33 > sweep_width/2=0.25` → 墙边 8 cm 永远标记不到，天花板 ≈93% | `sweep_width >= 2*robot_inset`（自动抬升 + 告警） |
| 13 | 地图刷新把 COVERED 降级成 OBSTACLE → 覆盖率倒退、同一条带反复清扫 | 不降级已覆盖格 |
| 14 | 覆盖率控制器完全"障碍盲"（无 footprint、无代价地图） | 喂 footprint + 同一份局部代价地图 |

## 三、核心（保留的模块修成可用）

| # | 问题 | 修法 |
|---|---|---|
| 15 | `point_lio` 单点更新 `+K*pd2`（与 IESKF 的 `delta=-H⁻¹b` 反号） | 改 `-K*pd2` |
| 16 | `point_lio` `const auto& head = front()` 后 `pop_front()` → 悬垂 | 按值拷贝 |
| 17 | `gicp_localization` IMU 回放：`onImu` 自己 `push_back`，`size>=2` 永远成立 → 死循环 | 先出队再 `onImu` |
| 18 | IESKF 无有效约束时仍重写协方差（H=I → P 变单位阵） | 无有效迭代直接 return；`b` 显式清零 |

## 四、基础层

| # | 问题 | 修法 |
|---|---|---|
| 19 | `ThreadPool` `kBlock` 关停路径漏 `sources_.erase`/`done(false)` | 补清理与回调 |
| 20 | `done()` 回调抛异常 → `std::terminate` | try/catch |
| 21 | `shutdown()` 并发调用重复 `join`（UB） | 静态互斥 + `shutdown_done_` |
| 22 | `PeriodicThread` tick 抛一次异常 → 50 Hz 发布线程永久停摆 | tick 包 try/catch |

## 验证

```
catkin_make (-j6): 0 error;  catkin_test_results: 210 tests, 0 failures
同路线 3 次（90 s 预算）: 目标 1/2 全过; 目标 3 = 2/3
  改前: 目标 3 稳定 FAILED_SAFETY_STOP（安全层误停）
  改后: 失败模式变为 TIMEOUT（还在跑, 不是被误停）
```

## 撤销的虚警（核实后不改）

| 审查说法 | 核实 |
|---|---|
| `planner_heuristic_weight` / `safety_*_max_points` 没读 → 用错值 | 键确实没读，但成员默认值 = YAML 值（2.0 / 4 / 6）→ 无行为差异；本轮仍补上了读取 |
| IESKF 无效帧重置 P | 无有效点时在 `lidar_processor.cpp:403` 就 return；本轮仍加防御 |
| `scan_context_lc::detect` 持锁阻塞建图 | 同一线程调用，锁不竞争 |
| `ThreadPool::shutdown` 重复 join 崩 | 有 `joinable()` 保护；本轮仍串行化 |
| 关键帧两半被 drop 拆散 → 整图错位 | 机制真，但地图按**时间戳**匹配，不是下标 → 影响被夸大 |
| `point_lio`/`gicp` 的 bug | 代码为真但未启用（配置是 fast_lio2 + pp_icp）；按"保留"决定修成可用 |

## 仍未做（下一轮）

1. `addSubMap` 每关键帧对整张累积图重做 VoxelGrid（O(N²)）；`observations_` 无界。
2. `VoxelMapStore::groundAt` 每次遍历全部 voxel（planar 每帧调用）。
3. `saveGridMap` 持 `m_mutex_` 跨 RANSAC + 文件 IO。
4. 核心 20 个 `test_*.cpp` 未接 catkin；`test_localization_pipeline.cpp` 孤儿。
5. `SnapshotSlot` 的 `{数据, version}` 非一致对（当前无消费者读 version）。
6. `HEALTH_INIT` 仍按"可继续行驶"处理。
7. 覆盖率 `stats()`/`hasUncoveredReachable` 仍在 20 Hz 控制环里做全图 BFS。
