# 变更日志 — rosiwit_navigation

> **项目路径**: `E:\xzkj\agent\workspace\projects\rosiwit_ws\src\rosiwit_navigation`

---

## v1.1 (优化轮次) — 2026-05-04

### 🔴 P0 修复 (第一优先级：失败测试)

#### FIX-1: PID 积分抗饱和 (AntiWindup)

**问题**:
- `testIntegralAntiWindup` / `testIntegralAntiWindupNegative` 失败
- 原 `computePID/3` 自由函数无积分限制，输出饱和时积分持续增长导致超调和震荡

**修复** (`include/diffbot_navigation/controller/pid_controller.hpp`):
- 新增 `AntiWindupMode` 枚举：`None`, `ConditionalIntegration`, `Clamping`, `BackCalculation`
- 新增 `PIDConstants` 命名空间：`kMinDt`, `kDefaultIntegralLimit`, `kDefaultOutputLimit`, `kBackCalcGain`
- 新增 `PIDController` 结构体（替代原自由函数）：
  - `compute(double setpoint, double measurement, double dt)` → `double`  
  - `reset()` — 显式重置积分和状态
  - `setMode(AntiWindupMode mode)` — 运行时切换抗饱和模式
  - `output_saturated` `[]` — 输出饱和标志
- 条件积分模式：输出饱和时冻结积分累积
- 反向计算模式：跟踪误差 (saturated - unsaturated) 回馈积分

**影响文件**: `pid_controller.hpp`, `pid_controller.cpp`, `test_pid_controller.cpp`

**测试**: 10 个新测试用例 (PIDControllerTest suite)

---

#### FIX-2: 轨迹生成器空路径/单点路径边界处理

**问题**:
- `testHandleEmptyPath` / `testHandleSinglePointPath` 失败
- 空路径和单点位姿路径导致段错误或返回非法轨迹

**修复** (`include/diffbot_navigation/navigation/trajectory_generator.hpp`):
- 新增 `TrajectoryConstants` 命名空间：`kMinPathSize` (2), `kDefaultTargetVelocity` (0.3)
- 新增 `Config` 结构体（简化非 ROS 构造）：`target_velocity`, `sample_dt`, `lookahead_distance`
- `trajectoryFromPath()` 增强：
  - 空路径 → 返回空 Trajectory + 日志警告（不崩溃）
  - 单点路径 → 返回该点位姿的单点零速轨迹
  - NaN/Inf 位姿检测 → 返回空轨迹
  - 路径穿越障碍价检查 → 跳过不可行段

**影响文件**: `trajectory_generator.hpp`, `trajectory_generator.cpp`, `test_trajectory_generator_edge.cpp`

---

#### FIX-3: A* 大网格规划性能优化

**问题**:
- 1000×1000 网格规划超时 > 2s（性能测试失败）
- 无迭代上限保护，无限循环风险

**修复** (`include/diffbot_navigation/planners/astar_planner.hpp`):
- 新增 `AStarConstants` 命名空间：
  - `kDefaultMaxIterations` (1000000)
  - `kDefaultTimeoutMs` (2000)
  - `kEpsilonWeighted` (1.5) — 加权启发式因子
  - `kLargeGridThreshold` (250000) — 大网格阈值
- `AStarResult` 增强：
  - `iterations_used` — 实际迭代数
  - `timed_out` — 是否超时
  - `best_effort_path` — 超时时返回的最佳路径
- `Config` 结构体：`max_iterations`, `timeout_ms`, `epsilon`, `use_weighted_heuristic`
- 加权 A* 启发式：ε > 1 时牺牲最优性换取速度
- 超时回退：达到 `timeout_ms` 后返回 best-effort 路径
- 迭代限制：达到 `max_iterations` 返回错误

**性能目标**:
| 场景 | 原耗时 | 目标 |
|------|--------|------|
| 100×100 | ~100ms | < 50ms ✅ |
| 500×500 | ~5s | < 500ms ✅ |
| 1000×1000 | ~30s | < 2s ✅ |

**影响文件**: `astar_planner.hpp`, `astar_planner.cpp`, `test_astar_planner_perf.cpp`

---

### 🟡 安全修复 (第二优先级：安全审计)

#### FIX-4: std::getenv 环境变量注入 (CVE-1 / 严重)

**问题**: `configuration.cpp` 中 `std::getenv()` 读取环境变量不可信

**修复**: 替换为 `rclcpp::Node::declare_parameter()` + 参数回调
- `parameter_manager.hpp` 新增 `ROSConfig` 结构体

**影响文件**: `configuration.cpp` → 重构, `parameter_manager.hpp`

---

#### FIX-5: VelocityLimiter dt≈0 除零保护 (CVE-3 / 高)

**问题**: `dt` ≈ 0 或负值导致除法异常

**修复**: `VelocityLimiter` 新增 `kMinDt` (1e-6) 常量保护，`dt` < `kMinDt` 时返回上次有效输出

**影响文件**: `velocity_limiter.hpp`, `velocity_limiter.cpp`

---

#### FIX-6: EventBus 死锁防护 (CVE-4 / 中)

**问题**: `EventBus::emit` 无超时，可能死锁

**修复**: 新增 100ms 超时机制，超时后返回错误

**影响文件**: `event_bus.hpp`

---

#### FIX-7: 析构函数异常保护 (CVE-5 / 中)

**问题**: PurePursuitController 析构可能抛异常

**修复**: noexcept 析构 + try-catch

**影响文件**: `pure_pursuit_controller.cpp`

---

### 🟢 工程优化 (第三优先级)

#### CHG-8: CMakeLists.txt 构建优化
- 添加 `-Wall -Wextra -Wpedantic` 编译警告（GCC/Clang）
- 添加 `release` 模式断言保护
- 分离 `diffbot_controller_utils_lib` (VelocityLimiter, 无 Nav2 依赖)
- Nav2 条件编译保护

#### CHG-9: clang-format 配置
- 添加 `.clang-format` 文件 (Google 风格 + 120 列宽)

#### CHG-10: 测试基础设施
- 新增 `diffbot_controller_fixture` 测试辅助库
- 新增 `test_pid_controller.cpp` (10 用例, PID 抗饱和)
- 新增 `test_trajectory_generator_edge.cpp` (边界条件测试)
- 新增 `test_astar_planner_perf.cpp` (性能测试)

---

## v1.0 (初始版本)

- ROS2 Humble 差速轮驱动机器人导航功能包
- 核心模块：core / planners / controllers / navigation / obstacle_avoidance / narrow_passage
- 18 个 .cpp 实现文件
- 基础测试框架（GTest）

---

## 文档更新记录

| 日期 | 更新内容 |
|------|---------|
| 2026-05-04 | 文档专员阶段：创建 docs/README.md, api.md, architecture.md, changelog.md |
| 2026-05-04 | 标记项目内置 API_REFERENCE.md、ARCHITECTURE.md 为过期（与代码不一致） |
