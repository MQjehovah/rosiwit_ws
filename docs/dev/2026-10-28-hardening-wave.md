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

## 五、性能与测试接入（同日晚间第二批）

| # | 问题 | 修法 |
|---|---|---|
| 23 | `addSubMap` 每个关键帧对**整张累积图**重跑 `pcl::VoxelGrid`（第 N 帧处理 N 帧的点，O(N²)），建图越久单帧越慢 | 增量体素累积 `voxelKey -> (下标, 计数)`，同体素滑动平均（= centroid）；`loadMap/appendMap/onBackendOptimized/saveMap 动态剔除` 后 `rebuildVoxels()` |
| 24 | `groundAt` 每次遍历**全部**体素找 3x3 列（planar 约束每帧调用） | 增加 `(cx,cy) -> cell keys` 列索引，insert/erase/carve/decay/load 全程维护，`groundAt` 变 O(9×列内体素) |
| 25 | `saveGridMap` 全程持 `m_mutex_`（栅格化 + 全图 RANSAC + 写 PGM/YAML），期间 `addSubMap` 被阻塞 → SLAM 位姿/TF 停发 | 改快照模式（与 `generateGridMap` 一致）：锁内拷贝、锁外构建、短锁写回 |
| 26 | 覆盖率 `hasUncoveredReachable`（全图 BFS）在 20 Hz 控制环里跑 | 缓存 0.5 s 或车位移动 >0.15 m 才重算（"不可达"处理本身有秒级超时兜底） |
| 27 | `HEALTH_INIT` 被当成"可继续行驶"（与 TRACKING 等价） | 初始化期间发零速 + `WAITING_LOC`，初始化完成自动继续；SLAM 侧转 LOST 仍走 `FAILED_LOC_LOST` |
| 28 | 核心 13 个 `test_*.cpp` 只在 STANDALONE 构建里跑，`catkin_make run_tests` 看不到；且因 PCL 1.8/gtest 1.8 差异**从未编译过** | 先接入与本次改动直接相关的 2 个（`test_map_store` / `test_pcd_map_manager`），修 `PointXYZINormal(x,y,z)`、`INSTANTIATE_TEST_CASE_P`、`TempDir`、补 `main()`，并新增 3 个针对性用例 |

## 验证

```
catkin_make (-j6): 0 error;  catkin_test_results: 210 tests, 0 failures
同路线 3 次（90 s 预算）: 目标 1/2 全过; 目标 3 = 2/3
  改前: 目标 3 稳定 FAILED_SAFETY_STOP（安全层误停）
  改后: 失败模式变为 TIMEOUT（还在跑, 不是被误停）
放宽到 180 s 复跑 3 次: 目标 3 仍是 2/3 -> 不是测试预算问题（见下"已知残留"）
catkin_test_results（含新接入的两个核心测试）: 242 tests, 0 failures
smoke_legacy.sh: 与改动前完全一致 (PASS=5 FAIL=4 SKIP=3)
```

### 已知残留：目标 3 的 1/3 失败（门洞回程）

失败那次的证据链（`~/.ros/log/.../rosout.log`）：

- `STOP hits b(0.628, -0.30...)`、`pts=999`：障碍在车体系 (0.63 m 前, 0.30 m 侧)，
  即**门框**在车身半宽 (0.33) 之内、但已在前保险杠 (0.55) 之外 8 cm。
- 停止区是"预测扫描区"（0.55~0.95 m，±0.345），所以它会提前 0.4 m 判定；
  0.8 m 门洞 + 0.66 m 车体 → 允许的横向偏差只有 `0.40 - 0.345 = 5.5 cm`。
  本次运行的最小 footprint 间隙是 **0.000 m**（另两次 0.05 m）→ 确实是"要撞了"，停是对的。
- 卡死原因是恢复手段改不了横向偏差：清代价地图 → 原地转 60° → 等 2 s → 倒车 0.4 m，
  转完再规划（125 点）仍把门框留在停止区里，循环到测试超时。
- 根因在**控制器**：`controller_name: "rpp"`，而 `rpp_controller.cpp` 里
  **完全没有** footprint / costmap / clearance 相关代码（`grep` 零命中），
  即 `footprint_aware_control: true` / `footprint_shrink` 对 rpp 不生效 ——
  它只按 0.5 m 前视点跟踪，拐弯时会切内角（这次横向偏差 >5.5 cm 就是这么来的）。

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

1. `rpp_controller` 没有 footprint/代价地图感知（配置里的 `footprint_aware_control` 对它无效）。
   门洞回程那次失败就是它切内角导致横向偏差 >5.5 cm。修法两选一（需对比验证）：
   (a) 给 rpp 加"沿指令弧的 footprint 扫掠检查"（必须用 raw 障碍层，不能用 99 膨胀环）；
   (b) 门洞/狭窄段切到 DWA（已有 footprint 采样 + `cost>=98` 拒绝）。
2. `observations_` 无界（保存时按帧统计动态点，长时间建图内存线性增长）。
3. 核心另外 11 个 `test_*.cpp` 仍未接 catkin（同样有 PCL/gtest 版本差异，需逐个过一遍）。
4. `SnapshotSlot` 的 `{数据, version}` 非一致对（当前无消费者读 version）。
5. 覆盖率 `stats()` 仍是 O(栅格) 扫描（每控制周期 2~3 次）；实测约 0.1 ms，暂不动。
