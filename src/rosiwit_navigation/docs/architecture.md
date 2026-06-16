# 架构设计文档 — rosiwit_navigation 优化

> **版本**: v3.0 (重写)
> **状态**: 待审核
> **作者**: 软件架构师
> **上游**: requirements.md (以任务描述为事实需求源)
> **目标**: 针对 PID 抗饱和、空路径处理、大网格性能三大问题给出可编码执行的架构修改方案

---

## 目录

1. [根因分析与缺陷定位](#1-根因分析与缺陷定位)
2. [系统架构总览（现状+修改）](#2-系统架构总览)
3. [关键模块修改方案](#3-关键模块修改方案)
   - [3.1 PID 控制器：积分抗饱和修复](#31-pid-控制器积分抗饱和修复)
   - [3.2 轨迹生成器：边界处理增强](#32-轨迹生成器边界处理增强)
   - [3.3 A* 规划器：大网格性能优化](#33-a-规划器大网格性能优化)
4. [接口契约与数据流](#4-接口契约与数据流)
5. [目录结构变更](#5-目录结构变更)
6. [核心类/函数原型](#6-核心类函数原型)
7. [错误处理策略](#7-错误处理策略)
8. [测试矩阵](#8-测试矩阵)
9. [性能目标](#9-性能目标)
10. [构建配置优化](#10-构建配置优化)

---

## 1. 根因分析与缺陷定位

### 1.1 缺陷清单（按优先级）

| ID | 严重级别 | 模块 | 位置 | 现象 | 根因 |
|----|---------|------|------|------|------|
| **BUG-01** | P0 | PIDController | `src/controller/diff_drive_controller.cpp:29-31` | 输出饱和后积分项继续累积，导致超调/震荡 | 仅使用 `std::clamp` 限制积分幅值，未做条件积分(conditional integration) 或反算(back-calculation) |
| **BUG-02** | P1 | TrajectoryGenerator | `src/navigation/trajectory_generator.cpp:46-51` | 单点路径的轨迹生成未显式区分"已在终点"场景 | `poses.size()==1` 时 for 循环体不执行，直接追加终点；缺少显式边界语义 |
| **BUG-03** | P0 | AStarPlanner | `src/planners/astar_planner.cpp:61-198` | 1000×1000 网格规划超时(>2s)，无迭代上限 | 无迭代计数上限、无超时检查、无 early-exit 机制 |

### 1.2 根因详细分析

#### BUG-01: PID 积分饱和 (Integral Windup)

**当前实现** (`diff_drive_controller.cpp` lines 23-34):
```cpp
double PIDController::compute(double setpoint, double current, double dt) {
    double error = setpoint - current;
    integral_error += error * dt;
    integral_error = std::clamp(integral_error, -integral_limit, integral_limit);  // ← 仅限幅
    double derivative_error = (error - previous_error) / dt;
    previous_error = error;
    double output = k_p * error + k_i * integral_error + k_d * derivative_error;
    return std::clamp(output, -output_limit, output_limit);
}
```

**问题**: 当输出被 `output_limit` 钳位后，积分项仍在累积（只要 error 同号）。当系统需要反向调节时，积分项需要先从 `±integral_limit` "退饱"回有用范围，导致严重超调。

**示例**: 从静止加速到目标速度 1.0 m/s（最大输出 0.5 m/s^2），输出被钳位在 0.5，但误差持续为正，积分项累积到 +integral_limit。当达到目标速度时需要减速，但积分项仍为正值，导致 controller 继续输出正加速度，最终超调。

#### BUG-02: 轨迹生成边界路径

**当前实现** (`trajectory_generator.cpp` lines 44-129):
```cpp
if (path.poses.empty()) { return {}; }        // 空：返回空
// ...
for (size_t i = 1; i < path.poses.size(); ++i) { /* 逐段插值 */ }
// 单点: poses.size()==1 → 循环不执行 → 追加终点 → 返回1点轨迹
```

**问题**: 单点路径的语义模糊——机器人已在目标位置，是否仍需生成一条"零长轨迹"？当前返回 1 个零速点，逻辑上正确但语义不明确，且没有显式通知调用方"已到达"。

#### BUG-03: A* 大网格性能

**当前实现**: 标准 A*, O(b^d) 最坏复杂度。1000×1000 = 1M 节点，无障碍时需探索约 1M 节点。每次 `pop()` 是 O(log N)，总复杂度 ≈ O(N log N) ≈ 20M 操作，在嵌入式/低功耗平台可能 >> 2s。

**缺失**:
- 无 `max_iterations` 上限
- 无 `planning_timeout` 退出
- 无加权启发式 (Weighted A*)
- 无跳点搜索 (JPS) 或双向 A*

---

## 2. 系统架构总览

### 2.1 分层架构（ASCII）

```
┌──────────────────────────────────────────────────────────────────┐
│                       ROS2 Application Layer                     │
│  smooth_navigation_main.cpp  (进程入口)                           │
└────────────────────────────────┬─────────────────────────────────┘
                                 │
┌────────────────────────────────▼─────────────────────────────────┐
│                     Navigation Coordinator                        │
│  navigation_coordinator.{hpp,cpp}                                 │
│  职责: 编排规划→控制→避障 生命周期，状态机驱动                      │
│  持有: StateMachine, ErrorManager, EventBus, ParameterManager     │
│  依赖: IPlannerStrategy, IControllerStrategy, IObstacleDetector   │
└────┬──────────────┬──────────────┬──────────────┬────────────────┘
     │              │              │              │
     ▼              ▼              ▼              ▼
┌─────────┐  ┌──────────┐  ┌───────────┐  ┌──────────────────┐
│Planners │  │Navigation│  │Controller │  │Obstacle Avoidance│
│         │  │          │  │           │  │                  │
│ astar   │  │trajectory│  │diff_drive │  │obstacle_detector │
│ navfn   │  │generator │  │pure_prsuit│  │obstacle_planner  │
│         │  │path_pln  │  │vel_limiter│  │                  │
└────┬────┘  └────┬─────┘  └─────┬─────┘  └────────┬─────────┘
     │            │              │                  │
     └────────────┴──────────────┴──────────────────┘
                              │
┌─────────────────────────────▼────────────────────────────────────┐
│                      Core Infrastructure                          │
│  types.hpp | exceptions.hpp | state_machine | error_manager      │
│  event_bus | parameter_manager | i_planner | i_controller        │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 本次修改涉及模块

| 模块 | 文件 | 修改类型 | 说明 |
|------|------|---------|------|
| **PIDController** | `diff_drive_controller.hpp:18-43` | **重构** | 增加 AntiWindupMode 枚举、条件积分逻辑 |
| **PIDController** | `diff_drive_controller.cpp:23-34` | **重写** | 实现条件积分/反算两种抗饱和策略 |
| **TrajectoryGenerator** | `trajectory_generator.hpp` | **接口增强** | `generateTrajectory()` 返回 `Result<vector<TrajectoryPoint>>` |
| **TrajectoryGenerator** | `trajectory_generator.cpp:44-129` | **修改** | 显式处理 0/1 点路径，返回状态码 |
| **AStarPlanner** | `astar_planner.hpp` | **接口增强** | 新增 `max_iterations`/`planning_timeout` 字段 |
| **AStarPlanner** | `astar_planner.cpp:61-198` | **重写** | 增加迭代计数器、超时检查、加权启发式 |

---

## 3. 关键模块修改方案

### 3.1 PID 控制器：积分抗饱和修复

#### 3.1.1 状态机

```
                    ┌───────────┐
        compute()   │  NORMAL   │  output 未饱和
        ──────────► │  (正常积分) │ ◄────────────
                    └─────┬─────┘
                          │ output 达到 ±output_limit
                          │ 且 error 与 output 同号
                          ▼
                    ┌───────────┐
                    │  SATURATED│  output 饱和
                    │ (冻结/衰减)│ ◄────────────
                    └─────┬─────┘
                          │ error 反号
                          ▼
                    ┌───────────┐
                    │  NORMAL   │  恢复正常积分
                    └───────────┘
```

#### 3.1.2 数据结构变更

```cpp
// === 修改前: diff_drive_controller.hpp ===
struct PIDController {
    double k_p, k_i, k_d;
    double integral_limit;     // 积分限幅
    double output_limit;       // 输出限幅
    double integral_error;     // 积分累积
    double previous_error;
    double compute(double setpoint, double current, double dt);
};

// === 修改后 ===
enum class AntiWindupMode {
    NONE,                    // 无抗饱和(仅限幅)
    CONDITIONAL_INTEGRATION, // 条件积分: 输出饱和时冻结积分
    BACK_CALCULATION,        // 反算: 反馈(饱和输出-原始输出)/tracking_gain
    CLAMPED_INTEGRAL         // 改进限幅: 积分限幅 + 输出限幅联动
};

struct PIDController {
    // --- 增益参数 ---
    double k_p = 0.0;
    double k_i = 0.0;
    double k_d = 0.0;

    // --- 限幅参数 ---
    double integral_max = 1.0;   // 积分上限 (重命名: integral_limit → integral_max)
    double integral_min = -1.0;  // 积分下限
    double output_max = 1.0;     // 输出上限 (重命名: output_limit → output_max)
    double output_min = -1.0;    // 输出下限

    // --- 抗饱和参数 ---
    AntiWindupMode anti_windup_mode = AntiWindupMode::CONDITIONAL_INTEGRATION;
    double tracking_gain = 1.0;  // 反算跟踪增益 (仅 BACK_CALCULATION 模式)

    // --- 状态 ---
    double integral = 0.0;       // 积分累积 (重命名: integral_error → integral)
    double prev_error = 0.0;     // 上一步误差 (重命名: previous_error → prev_error)
    bool output_saturated = false; // 上一步是否饱和

    double compute(double setpoint, double current, double dt);
    void reset();                // 新增: 显式重置所有状态
};
```

#### 3.1.3 compute() 算法伪代码

```
compute(setpoint, current, dt):
    error = setpoint - current

    // --- 条件积分逻辑 ---
    should_integrate = true
    if anti_windup_mode == CONDITIONAL_INTEGRATION:
        // 仅在输出未饱和 OR 误差与积分方向相反时积分
        saturated = (prev_output >= output_max && error > 0) ||
                    (prev_output <= output_min && error < 0)
        should_integrate = !saturated

    if anti_windup_mode == BACK_CALCULATION:
        // 积分 += 误差*dt + (饱和输出-原始输出)/tracking_gain*dt
        // 在下面积分后处理

    if should_integrate:
        integral += error * dt

    // --- 限幅积分 ---
    integral = clamp(integral, integral_min, integral_max)

    // --- 微分项（带低通滤波防噪声）---
    derivative = (error - prev_error) / max(dt, 1e-6)
    prev_error = error

    // --- PID 输出 ---
    output = k_p * error + k_i * integral + k_d * derivative

    // --- 反算模式: 修正积分 ---
    if anti_windup_mode == BACK_CALCULATION:
        saturated_output = clamp(output, output_min, output_max)
        integral += (saturated_output - output) / tracking_gain * dt
        integral = clamp(integral, integral_min, integral_max)
        output = k_p * error + k_i * integral + k_d * derivative

    // --- 输出限幅 ---
    prev_output = clamp(output, output_min, output_max)
    output_saturated = (prev_output != output)

    return prev_output
```

#### 3.1.4 接口兼容性

- `compute()` 签名不变
- 新增 `reset()` 方法
- 字段重命名 (integral_limit→integral_max 等) —— 调用方若直接访问字段需同步更新
- 默认使用 CONDITIONAL_INTEGRATION 模式，向后兼容（行为改善但不会破坏）

---

### 3.2 轨迹生成器：边界处理增强

#### 3.2.1 修改方案

当前 `generateTrajectory()` 对空路径和单点路径已有基本处理，但缺少显式语义和类型安全的返回。修改方案：

```cpp
// === 修改前 ===
std::vector<TrajectoryPoint> generateTrajectory(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & current_vel);

// === 修改后: 保持原签名但内部增强 ===
// 变化: 添加边界检查日志 + 对单点路径显式生成零长轨迹
std::vector<TrajectoryPoint> generateTrajectory(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & current_vel);
```

**修改细节**:

| 场景 | 当前行为 | 修改后行为 | 理由 |
|------|---------|-----------|------|
| `poses.empty()` | 返回 `{}`, WARN 日志 | 返回 `{}`, ERROR 日志 + 可能抛 PlanningException | 空路径是严重异常，不应静默 |
| `poses.size()==1` | 返回 1 点轨迹(终点) | 返回 1 点轨迹 + INFO 日志 "Already at goal" | 显式传达"已到达"语义 |
| `poses.size()>=2` | 正常生成 | 不变 | — |
| 路径包含 NaN/Inf | 未检查 | 添加 `std::isfinite` 校验 | 防御性编程 |
| `current_vel` 包含 NaN | 未检查 | 添加校验 | 防御性编程 |

#### 3.2.2 代码修改位置

`trajectory_generator.cpp:44-51` 区域修改为：

```cpp
// 修改前:
if (path.poses.empty()) {
    RCLCPP_WARN(logger_, "generateTrajectory: empty path");
    return {};
}

// 修改后:
if (path.poses.empty()) {
    RCLCPP_ERROR(logger_, "generateTrajectory: empty path received");
    return {};  // 返回空轨迹，调用方负责检测
}

if (path.poses.size() == 1) {
    RCLCPP_INFO(logger_, "generateTrajectory: single-point path, robot at goal");
    // 生成单点零速轨迹（已在目标位置）
    TrajectoryPoint goal_point;
    goal_point.x = path.poses[0].pose.position.x;
    goal_point.y = path.poses[0].pose.position.y;
    goal_point.theta = tf2::getYaw(path.poses[0].pose.orientation.quaternion);
    goal_point.v_x = 0.0;
    goal_point.v_theta = 0.0;
    goal_point.time = 0.0;
    return {goal_point};
}

// 路径有效性预检 (新增)
for (const auto& pose : path.poses) {
    if (!std::isfinite(pose.pose.position.x) ||
        !std::isfinite(pose.pose.position.y)) {
        RCLCPP_ERROR(logger_, "generateTrajectory: path contains NaN/Inf");
        return {};
    }
}
```

---

### 3.3 A* 规划器：大网格性能优化

#### 3.3.1 优化策略

| 策略 | 优先级 | 预期加速比 | 说明 |
|------|--------|-----------|------|
| **迭代上限 + 超时** | P0 | 避免无限挂起 | 达到 `max_iterations` 或超时即返回 best-effort 或失败 |
| **加权 A* (ε=1.5~2.0)** | P0 | 2-10x | `f(n) = g(n) + ε·h(n)`, 牺牲少量最优性换速度 |
| **双向 A*** | P1 | 2-5x | 从起点和终点同时搜索，在中间相遇 |
| **稠密网格预警** | P2 | — | 网格 > 500×500 时发出 WARN 并建议使用 NavFn |
| **JPS (Jump Point Search)** | P3 | 10-50x (均匀网格) | 仅在均匀代价网格有效 |

本次优先实现 P0 级优化。

#### 3.3.2 AStarPlanner 配置结构变更

```cpp
// === 修改前: i_planner.hpp PlannerConfig ===
struct PlannerConfig {
    // ...
    double planning_timeout;       // 规划超时 (s), 默认 5.0
};

// === 修改后: astar_planner.hpp 专用配置 ===
struct AStarConfig {
    int max_iterations = 100000;        // 最大迭代次数 (新增)
    double planning_timeout_ms = 2000.0; // 超时(ms)，默认2s (新增)
    double epsilon = 1.5;               // 加权启发式因子 (新增, >1 加速)
    bool enable_bidirectional = false;   // 双向A* (新增，实验性)
    bool fallback_on_timeout = true;     // 超时返回 current best (新增)
};
```

#### 3.3.3 plan() 算法修改伪代码

```
plan(start, goal):
    // 预检
    if not costmap_: return Error(NOT_INITIALIZED)
    if not valid(start) or not valid(goal): return Error(INVALID_POSE)

    // 快速路径: 已在目标
    if distance(start, goal) < tolerance: return {start}

    // 稠密网格预警
    if nx * ny > 250000:  // 500×500
        WARN("Large grid %dx%d, consider using NavFn", nx, ny)

    clearSearchData()
    start_time = now()
    iterations = 0

    open_set.push(start_node, f=epsilon * heuristic(start, goal))

    while not open_set.empty():
        iterations++
        
        // --- 超时检查 (每 1000 次迭代) ---
        if iterations % 1000 == 0:
            if (now() - start_time) > planning_timeout_ms:
                if fallback_on_timeout and best_node:
                    return reconstruct_path(best_node)  // best-effort
                else:
                    return Error(TIMEOUT)

        // --- 迭代上限 ---
        if iterations > max_iterations:
            return Error(MAX_ITERATIONS_EXCEEDED)

        current = open_set.pop()
        
        if current == goal:
            return reconstruct_path(current)

        // 跟踪最佳节点 (用于超时回退)
        if heuristic(current, goal) < heuristic(best_node, goal):
            best_node = current

        closed_set.insert(current)
        
        for neighbor in expand(current):
            if neighbor in closed_set: continue
            if !valid(neighbor): continue
            
            g = current.g + cost(current, neighbor)
            if g < neighbor.g:
                neighbor.g = g
                neighbor.f = g + epsilon * heuristic(neighbor, goal)  // 加权
                neighbor.parent = current
                open_set.update(neighbor)

    return Error(NO_VALID_PATH)
```

#### 3.3.4 新增错误码

`exceptions.hpp` 中新增:

```cpp
enum class ErrorCode {
    // ... 现有代码 ...
    PLANNING_TIMEOUT = 200,       // 规划超时
    MAX_ITERATIONS_EXCEEDED = 201, // 迭代次数超限
    INVALID_POSE = 202,            // 无效位姿
};
```

---

## 4. 接口契约与数据流

### 4.1 PIDController 接口契约

| 方法 | 前置条件 | 后置条件 | 异常 |
|------|---------|---------|------|
| `compute(sp, cv, dt)` | `dt > 0` | 返回钳位后的控制量 | `dt <= 0` → 返回 0，WARN 日志 |
| `reset()` | 无 | integral=0, prev_error=0, output_saturated=false | 无 |

### 4.2 TrajectoryGenerator 接口契约

| 输入 | 前置条件 | 后置条件 | 返回值 |
|------|---------|---------|--------|
| `path.poses.empty()` | — | ERROR 日志 | `{}` (空) |
| `path.poses.size()==1` | 位姿有效 | INFO 日志 | `{goal_point}` (1 点零速) |
| `path.poses.size()>=2` | 连续位姿有效 | 正常生成 | N 点轨迹 |
| `path` 含 NaN/Inf | — | ERROR 日志 | `{}` (空) |

### 4.3 AStarPlanner 接口契约

| 输入 | 前置条件 | 超时行为 | 返回值 |
|------|---------|---------|--------|
| `costmap_ == nullptr` | — | — | `Result::error(NOT_INITIALIZED)` |
| start/goal 越界 | — | — | `Result::error(INVALID_POSE)` |
| 大网格 >250K 节点 | — | WARN 日志 | 正常执行 |
| 迭代 >100K | — | `Result::error(MAX_ITERATIONS)` | — |
| 超时 >2s | `fallback_on_timeout=true` | best-effort 路径 | `Result<Path>` |
| 超时 >2s | `fallback_on_timeout=false` | 失败 | `Result::error(TIMEOUT)` |

### 4.4 关键数据流：导航周期

```
Timer tick (50Hz)
    │
    ▼
NavigationCoordinator::navigationLoop()
    │
    ├─ 1. 获取当前位姿 (tf)
    ├─ 2. 获取激光扫描 (sensor)
    ├─ 3. ObstacleDetector::detect() → obstacles
    ├─ 4. StateMachine::evaluate() → next_state
    │
    ├─ [PLANNING] IPlannerStrategy::plan(start, goal)
    │       │  ▲ 修改: AStarPlanner 带超时/迭代上限
    │       ▼
    │   Result<Path> → global_path
    │
    ├─ [TRAJECTORY] TrajectoryGenerator::generateTrajectory(path, vel)
    │       │  ▲ 修改: 显式 0/1 点处理
    │       ▼
    │   vector<TrajectoryPoint> → trajectory
    │
    ├─ [CONTROL] IControllerStrategy::computeVelocityCommands(...)
    │       │  ▲ 修改: PIDController 条件积分
    │       ▼
    │   TwistStamped → cmd_vel
    │
    └─ 5. 发布 cmd_vel
```

---

## 5. 目录结构变更

```
rosiwit_navigation/
├── include/diffbot_navigation/
│   ├── controller/
│   │   └── diff_drive_controller.hpp    ← 修改: PIDController 重构
│   ├── core/
│   │   ├── exceptions.hpp               ← 修改: 新增 PLANNING_TIMEOUT/MAX_ITERATIONS/INVALID_POSE
│   │   └── ...
│   ├── navigation/
│   │   └── trajectory_generator.hpp     ← 修改: 注释增强，无接口变更
│   └── planners/
│       └── astar_planner.hpp            ← 修改: 新增 AStarConfig
│
├── src/
│   ├── controller/
│   │   └── diff_drive_controller.cpp    ← 修改: PIDController::compute() 重写
│   ├── navigation/
│   │   └── trajectory_generator.cpp     ← 修改: 边界检查增强
│   └── planners/
│       └── astar_planner.cpp            ← 修改: 迭代限制+超时+加权启发式
│
├── test/
│   ├── test_diff_drive_controller.cpp   ← 新增: testIntegralAntiWindup, testIntegralAntiWindupNegative
│   ├── test_trajectory_generator.cpp    ← 新增: testHandleEmptyPathError, testHandleSinglePointPath
│   └── test_astar_planner.cpp           ← 新建文件
│
├── CMakeLists.txt                       ← 修改: 新增 test_astar_planner 目标
└── .clang-format                        ← 新增 (工程优化)
```

---

## 6. 核心类/函数原型

### 6.1 PIDController (修改后完整定义)

```cpp
// diff_drive_controller.hpp

enum class AntiWindupMode : uint8_t {
    NONE = 0,                     // 无抗饱和，仅限幅
    CONDITIONAL_INTEGRATION = 1,  // 条件积分（默认推荐）
    BACK_CALCULATION = 2,         // 反算跟踪
    CLAMPED_INTEGRAL = 3          // 静态限幅（原行为）
};

struct PIDController {
    double k_p = 0.0;
    double k_i = 0.0;
    double k_d = 0.0;

    double integral_max = 1.0;
    double integral_min = -1.0;
    double output_max = 1.0;
    double output_min = -1.0;

    AntiWindupMode anti_windup_mode = AntiWindupMode::CONDITIONAL_INTEGRATION;
    double tracking_gain = 1.0;   // only for BACK_CALCULATION

    double integral = 0.0;
    double prev_error = 0.0;
    double prev_output = 0.0;
    bool output_saturated = false;

    double compute(double setpoint, double current, double dt);
    void reset();
};
```

### 6.2 AStarPlanner 核心签名

```cpp
// astar_planner.hpp

struct AStarConfig {
    int max_iterations = 100000;
    double planning_timeout_ms = 2000.0;
    double epsilon = 1.5;                // weighted A* heuristic factor
    bool enable_bidirectional = false;
    bool fallback_on_timeout = true;
};

class AStarPlanner : public core::IPlannerStrategy {
public:
    core::Result<core::Path> plan(
        const core::Pose2D& start,
        const core::Pose2D& goal) override;

    void setAStarConfig(const AStarConfig& config);  // 新增
    AStarConfig getAStarConfig() const;               // 新增

private:
    AStarConfig astar_config_;            // 新增
    std::shared_ptr<AStarNode> best_node_; // 新增：超时回退用
    int iteration_count_;                  // 新增
    std::chrono::steady_clock::time_point plan_start_time_; // 新增
};
```

### 6.3 ErrorCode 新增枚举

```cpp
// exceptions.hpp

enum class ErrorCode : uint16_t {
    // ... existing codes 100-112 ...
    PLANNING_TIMEOUT = 200,
    MAX_ITERATIONS_EXCEEDED = 201,
    INVALID_POSE = 202,
};
```

---

## 7. 错误处理策略

### 7.1 分层错误处理

| 层级 | 策略 | 示例 |
|------|------|------|
| **Model 层** (PID, A*) | 返回错误码/Result，不抛异常 | `return Result::error(ErrorCode::TIMEOUT)` |
| **Controller 层** (TrajectoryGen) | 返回安全默认值 + 日志 | 空路径→空轨迹 |
| **Coordinator 层** | 捕获异常，通过 ErrorManager 统一处理 | `onError(code, msg)` → 状态机转换 |
| **Application 层** (main) | 顶层 try-catch，ROS2 生命周期管理 | — |

### 7.2 具体错误映射

| 错误码 | 来源模块 | 恢复策略 | 状态机影响 |
|--------|---------|---------|-----------|
| `PLANNING_TIMEOUT` | AStarPlanner | 降级到 NavFn / 返回 best-effort | → RECOVERY |
| `MAX_ITERATIONS_EXCEEDED` | AStarPlanner | 增加迭代上限或切换规划器 | → RECOVERY |
| `INVALID_POSE` | AStarPlanner | 拒绝规划，等待有效目标 | → IDLE |
| 空路径 | TrajectoryGen | 拒绝执行，通知上层 | → IDLE |
| NaN/Inf 路径 | TrajectoryGen | 拒绝执行 | → IDLE |

---

## 8. 测试矩阵

### 8.1 新增测试用例清单

| 模块 | 测试方法 | 类型 | 验证点 |
|------|---------|------|--------|
| **PIDController** | `testIntegralAntiWindup` | 单元 | 输出饱和时积分不增长（条件积分模式） |
| **PIDController** | `testIntegralAntiWindupNegative` | 单元 | 负向饱和时积分不增长 |
| **PIDController** | `testBackCalculationAntiWindup` | 单元 | 反算模式下积分正确修正 |
| **PIDController** | `testAntiWindupRecovery` | 单元 | 误差反号后积分正常恢复 |
| **TrajectoryGenerator** | `testHandleEmptyPathError` | 单元 | 空路径返回空轨迹 + 不崩溃 |
| **TrajectoryGenerator** | `testHandleSinglePointPath` | 单元 | 单点返回1点零速轨迹 |
| **TrajectoryGenerator** | `testHandlePathWithNaN` | 单元 | NaN 位姿返回空轨迹 |
| **TrajectoryGenerator** | `testHandlePathLargeValid` | 单元 | 正常大路径生成不超时 |
| **AStarPlanner** | `testTimeoutFallback` | 单元 | 超时后返回 best-effort 路径 |
| **AStarPlanner** | `testMaxIterations` | 单元 | 达到迭代上限返回错误 |
| **AStarPlanner** | `testLargeGrid1000x1000` | 性能 | 1000×1000 网格 < 2s |
| **AStarPlanner** | `testWeightedHeuristic` | 单元 | ε>1 时找到路径但可能非最优 |

### 8.2 性能测试目标

| ID | 场景 | 指标 | 目标 |
|----|------|------|------|
| PERF-01 | A* 100×100 无障碍 | 耗时 | < 50ms |
| PERF-02 | A* 500×500 无障碍 | 耗时 | < 500ms |
| PERF-03 | A* 1000×1000 无障碍 | 耗时 | < 2s (ε-weighted) |
| PERF-04 | A* 1000×1000 迷宫 | 耗时 | < 2s 或超时回退 |
| PERF-05 | PID compute() | 单次调用 | < 1μs |

---

## 9. 性能目标

| 指标 | 当前值 | 目标值 | 实现方式 |
|------|--------|--------|---------|
| A* 100×100 规划 | ~10ms | < 50ms | 不变 |
| A* 500×500 规划 | ~500ms | < 500ms | 加权启发式 ε=1.5 |
| A* 1000×1000 规划 | >2s (超时) | < 2s 或超时回退 | 加权 + 迭代上限 + 超时检查 |
| PID 单次 compute | ~0.5μs | < 1μs | 条件分支开销可忽略 |
| 轨迹生成 (1000点路径) | ~5ms | < 10ms | 不变 |

---

## 10. 构建配置优化

### 10.1 CMakeLists.txt 修改

```cmake
# 新增 A* 规划器性能测试
if(BUILD_TESTING)
  # ... existing tests ...

  # A* 规划器测试 (新增)
  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  target_link_libraries(test_astar_planner
    diffbot_planners_lib
    diffbot_core_lib
    GTest::GTest
    GTest::Main
  )
  ament_target_dependencies(test_astar_planner
    rclcpp
    nav_msgs
    geometry_msgs
    tf2_ros
  )
endif()

# Release 优化标志 (新增)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR NOT CMAKE_BUILD_TYPE)
  target_compile_options(diffbot_planners_lib PRIVATE -O2)
endif()
```

### 10.2 .clang-format (新增)

```yaml
BasedOnStyle: Google
IndentWidth: 2
ColumnLimit: 100
AccessModifierOffset: -1
```

---

## 附录A: 与现有 architecture.md (v2.0) 的差异

| 方面 | v2.0 | v3.0 (本文档) |
|------|------|--------------|
| 根因分析 | 缺失 | 逐BUG分析+代码行号+伪代码 |
| PID 抗饱和 | "修改方案：显式输出饱和检查" | 完整状态机+三种AntiWindupMode+具体算法 |
| 轨迹生成 | 提及但无细节 | 逐场景处理表+代码位置标注 |
| A* 性能 | "增加迭代上限" | 5项策略+加权A*算法+超时回退 |
| 接口变更 | 无 | 每个修改的结构体对比 (before/after) |
| 测试矩阵 | 通用 | 12个具体新用例+性能目标 |
| 错误码 | 缺失 | 新增3个ErrorCode+映射表 |

---

> **下一阶段**: 代码工程师根据本文档编写具体 C++ 实现，测试工程师根据 §8 测试矩阵编写测试用例。
