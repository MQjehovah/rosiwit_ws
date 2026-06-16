# API 参考 — rosiwit_navigation v1.1

> **从代码自动提取** | 命名空间: `diffbot_navigation`  
> **代码路径**: `include/diffbot_navigation/`

---

## 目录

1. [Core 层](#1-core-层) — 事件总线、状态机、异常、类型
2. [Controller 层](#2-controller-层) — PID、Pure Pursuit、速度限制器、差速驱动
3. [Planners 层](#3-planners-层) — A\* 规划器、RRT 规划器
4. [Navigation 层](#4-navigation-层) — 轨迹生成器、路径规划接口、平滑导航
5. [Obstacle Avoidance 层](#5-obstacle-avoidance-层) — 障碍物检测与避障
6. [Narrow Passage 层](#6-narrow-passage-层) — 窄道检测与通行

---

## 1. Core 层

### 1.1 事件总线 (`event_bus.hpp`)

```cpp
namespace diffbot_navigation::core {

struct EventBase {
    std::string type;
    double timestamp;
    virtual ~EventBase() = default;
};

using EventHandler = std::function<void(const EventBase&)>;

class EventBus {
public:
    void subscribe(const std::string& event_type, EventHandler handler);
    void unsubscribe(const std::string& event_type, EventHandler handler);
    bool emit(const EventBase& event);  // 异步，超时 100ms → 返回 false
    void clear();
};
}
```

> **安全变更 (v1.1)**: `emit()` 新增 100ms 超时，防止死锁。

### 1.2 状态机 (`state_machine.hpp`)

```cpp
namespace diffbot_navigation::core {

enum class StateEvent {
    START, GOAL_RECEIVED, PATH_COMPUTED, PATH_FAILED,
    OBSTACLE_DETECTED, OBSTACLE_CLEARED,
    NARROW_PASSAGE_ENTER, NARROW_PASSAGE_EXIT,
    GOAL_REACHED, GOAL_FAILED,
    CANCEL, ERROR, RESET
};

std::string eventToString(StateEvent event);

struct TransitionResult {
    bool success;
    StateEvent from;
    StateEvent to;
    std::string error;
};

class NavigationStateMachine {
public:
    TransitionResult transition(StateEvent event);
    StateEvent currentState();
    bool canTransitionTo(StateEvent target);
    void reset();
};
}
```

### 1.3 异常与错误码 (`exceptions.hpp`)

```cpp
namespace diffbot_navigation::core {

enum class ErrorCode : uint16_t {
    // 通用 (100-112)
    SUCCESS = 0,
    UNKNOWN_ERROR = 100,
    INVALID_PARAMETER = 101,
    TIMEOUT = 102,
    // 新增 v1.1
    PLANNING_TIMEOUT = 200,
    MAX_ITERATIONS_EXCEEDED = 201,
    INVALID_POSE = 202,
};

struct ExceptionContext {
    ErrorCode code;
    std::string message;
    std::string file;
    int line;
    double timestamp;
};

class NavigationException : public std::runtime_error {
public:
    explicit NavigationException(const ExceptionContext& ctx);
    const ExceptionContext& context() const;
    ErrorCode code() const;
};
}
```

### 1.4 核心类型 (`types.hpp`)

```cpp
namespace diffbot_navigation::core {

struct Pose2D { double x, y, theta; };
struct Velocity2D { double linear, angular; };
struct Point2D { double x, y; };

using Path = std::vector<Pose2D>;
using Trajectory = std::vector<Pose2D>;  // 带时间戳的位姿序列

struct Costmap {
    int width, height;
    double resolution;
    std::vector<uint8_t> data;
    Pose2D origin;
};

struct Obstacle {
    Point2D center;
    double radius;
    Velocity2D velocity;  // 动态障碍物
};

struct GridCell {
    int x, y;
    double cost;
};
}
```

### 1.5 参数管理 (`parameter_manager.hpp`)

```cpp
namespace diffbot_navigation::core {

struct ROSConfig {
    // 通过 rclcpp::Node::declare_parameter 声明，替代 std::getenv
    double controller_frequency{20.0};
    double planner_frequency{1.0};
    std::string base_frame{"base_link"};
    std::string global_frame{"map"};
    // ... 其余参数
};

class ParameterManager {
public:
    explicit ParameterManager(rclcpp::Node* node);
    void declareAll();
    void setCallback(std::function<void(const ROSConfig&)> cb);
};
}
```

> **安全变更 (v1.1)**: `std::getenv` 已替换为 ROS2 参数系统。

---

## 2. Controller 层

### 2.1 PID 控制器 (`pid_controller.hpp`) ⭐ 重大变更

```cpp
namespace diffbot_navigation::controller {

// === 抗饱和模式 ===
enum class AntiWindupMode {
    None,                  // 无抗饱和保护
    ConditionalIntegration,// 输出饱和时冻结积分
    Clamping,              // 限制积分上限
    BackCalculation        // 反算修正积分
};

// === 常量 ===
namespace PIDConstants {
    constexpr double kMinDt = 1e-6;           // 最小时间步长
    constexpr double kDefaultIntegralLimit = 10.0;
    constexpr double kDefaultOutputLimit = 1.0;
    constexpr double kBackCalcGain = 0.1;
}

// === 无 ROS 依赖的 PID 配置 ===
struct PIDConfig {
    double kp{1.0}, ki{0.0}, kd{0.0};
    double integral_limit{PIDConstants::kDefaultIntegralLimit};
    double output_limit{PIDConstants::kDefaultOutputLimit};
    AntiWindupMode anti_windup_mode{AntiWindupMode::ConditionalIntegration};
};

// === PID 计算核心（header-only） ===
struct PIDController {
    double kp, ki, kd;
    double integral_limit;
    double output_limit;
    AntiWindupMode mode;

    double integral{0.0};
    double prev_error{0.0};
    bool output_saturated{[] → bool};

    PIDController(double p, double i, double d);
    explicit PIDController(const PIDConfig& cfg);

    double compute(double setpoint, double measurement, double dt);
    void reset();                  // 显式重置积分和状态
    void setMode(AntiWindupMode m); // 运行时切换抗饱和模式
};
}
```

**原 API (v1.0) — 已废弃**:
```cpp
// 旧版自由函数，无抗饱和保护
double computePID(double error, double dt, double& integral, double& prev, 
                  double kp, double ki, double kd);
void resetPID(double& integral, double& prev_error);
```

### 2.2 Pure Pursuit 控制器 (`pure_pursuit_controller.hpp`)

```cpp
namespace diffbot_navigation::controller {

struct PurePursuitConfig {
    double lookahead_distance{0.5};
    double min_lookahead{0.1};
    double max_lookahead{2.0};
    double wheel_base{0.3};
};

struct PurePursuitResult {
    double linear_velocity;
    double angular_velocity;
    Pose2D lookahead_point;
    bool path_end_reached;
};

class PurePursuitController {
public:
    explicit PurePursuitController(const PurePursuitConfig& cfg);
    PurePursuitResult compute(const Pose2D& current, const Path& path);
    void setLookaheadDistance(double d);
    double currentLookahead() const;
};
}
```

### 2.3 速度限制器 (`velocity_limiter.hpp`)

```cpp
namespace diffbot_navigation::controller {

struct VelocityLimits {
    double max_velocity_x{0.5}, max_velocity_theta{1.0};
    double min_velocity_x{-0.5}, min_velocity_theta{-1.0};
    double max_accel_x{0.5}, max_accel_theta{1.0};
    double min_accel_x{-0.5}, min_accel_theta{-1.0};
    double max_decel_x{0.5}, max_decel_theta{1.0};
};

class VelocityLimiter {
public:
    VelocityLimiter();
    explicit VelocityLimiter(const VelocityLimits& limits);
    Velocity2D limit(const Velocity2D& cmd, double dt);
    void setLimits(const VelocityLimits& limits);
    void reset();
};
}
```

> **安全变更 (v1.1)**: `dt` < `kMinDt` (1e-6) 时返回上次有效输出，防止除零。

### 2.4 差速驱动控制器 (`diff_drive_controller.hpp`)

```cpp
namespace diffbot_navigation::controller {

struct DiffDriveConfig {
    double wheel_radius{0.05};
    double wheel_separation{0.3};
    double max_velocity{0.5};
    PIDConfig linear_pid;
    PIDConfig angular_pid;
};

// 简化版 (无 ROS/Nav2 依赖)
class DiffDriveController {
public:
    explicit DiffDriveController(const DiffDriveConfig& cfg);
    Velocity2D compute(const Pose2D& current, const Pose2D& target, double dt);
    void reset();
};

// ROS2 LifecycleNode 版 (需要 Nav2)
class DiffDriveControllerNode : public nav2_core::Controller {
    // ... Nav2 接口
};
}
```

---

## 3. Planners 层

### 3.1 A\* 规划器 (`astar_planner.hpp`) ⭐ v1.1 新增/增强

```cpp
namespace diffbot_navigation::planners {

// === 常量 ===
namespace AStarConstants {
    constexpr int kDefaultMaxIterations = 1'000'000;
    constexpr double kDefaultTimeoutMs = 2000.0;
    constexpr double kEpsilonWeighted = 1.5;     // 加权启发式因子
    constexpr int kLargeGridThreshold = 250'000;  // 大网格阈值 (500×500)
    constexpr double kCostFactorScale = 0.5;
}

// === 规划结果 ===
struct AStarResult {
    Path path;
    bool success;
    int iterations_used;
    bool timed_out;
    Path best_effort_path;       // 超时时返回的最佳路径
    double elapsed_ms;

    static AStarResult error(ErrorCode code);
    static AStarResult timeout(const Path& best_effort);
};

// === 简化配置 ===
struct AStarConfig {
    int max_iterations{AStarConstants::kDefaultMaxIterations};
    double timeout_ms{AStarConstants::kDefaultTimeoutMs};
    double epsilon{AStarConstants::kEpsilonWeighted};
    bool use_weighted_heuristic{true};
};

// === 规划器 ===
class AStarPlanner {
public:
    AStarPlanner();
    explicit AStarPlanner(const AStarConfig& cfg);

    AStarResult plan(const Point2D& start, const Point2D& goal,
                     const Costmap& costmap);

    void setConfig(const AStarConfig& cfg);
    const AStarConfig& config() const;
};
}
```

**关键改进 (v1.1)**:
- 加权启发式 (ε = 1.5)：大网格 (≥250k 单元) 自动启用，牺牲最优性换取速度
- 超时检测：`std::chrono::steady_clock` 每隔 N 次迭代检查
- 迭代上限：超限返回 `MAX_ITERATIONS_EXCEEDED` 错误码
- 文件内联优化：`GridCell::operator<` 等热路径标记为 `inline`

### 3.2 RRT 规划器 (`rrt_planner.hpp`)

```cpp
namespace diffbot_navigation::planners {

class RRTPlanner {
public:
    Path plan(const Point2D& start, const Point2D& goal, const Costmap& costmap,
              int max_iterations = 10000, double step_size = 0.5);
};
}
```

---

## 4. Navigation 层

### 4.1 轨迹生成器 (`trajectory_generator.hpp`) ⭐ v1.1 边界增强

```cpp
namespace diffbot_navigation::navigation {

namespace TrajectoryConstants {
    constexpr int kMinPathSize = 2;
    constexpr double kDefaultTargetVelocity = 0.3;
}

struct TrajectoryConfig {
    double target_velocity{TrajectoryConstants::kDefaultTargetVelocity};
    double sample_dt{0.05};
    double lookahead_distance{0.5};
};

class TrajectoryGenerator {
public:
    explicit TrajectoryGenerator(const TrajectoryConfig& cfg);

    Trajectory trajectoryFromPath(const Path& path);
    // 空路径 → 空轨迹 + 警告 (不崩溃)
    // 单点路径 → 1点零速轨迹
    // NaN/Inf 检测 → 空轨迹

    void setConfig(const TrajectoryConfig& cfg);
};
}
```

**边界处理改进 (v1.1)**:
| 输入 | v1.0 行为 | v1.1 行为 |
|------|----------|----------|
| 空 `Path` (0 点) | 段错误/崩溃 | 返回空 `Trajectory` + RCLCPP_WARN |
| 单点 `Path` (1 点) | 除零/未定义 | 返回该点的零速轨迹 |
| 含 NaN 位姿 | 传播 NaN | 返回空轨迹 + 日志 |
| 合法大路径 | 正常 | 正常（无回归） |

### 4.2 路径规划接口 (`path_planner.hpp`)

```cpp
namespace diffbot_navigation::navigation {

class PathPlanner {
public:
    Path plan(const Pose2D& start, const Pose2D& goal, const Costmap& map);
    bool isPlanning() const;
    void cancelPlanning();
};
}
```

### 4.3 平滑导航 (`smooth_navigation.hpp`)

```cpp
namespace diffbot_navigation::navigation {

enum class NavigationState {
    IDLE, PLANNING, CONTROLLING,
    OBSTACLE_AVOIDANCE, NARROW_PASSAGE,
    GOAL_REACHED, FAILED
};

struct NavigationParams {
    double controller_frequency, planner_frequency;
    double max_velocity_x, max_velocity_theta, min_velocity_x;
    double max_accel_x, max_accel_theta;
    double goal_tolerance_xy, goal_tolerance_yaw;
    double lookahead_distance, min_lookahead_distance, max_lookahead_distance;
    double replanning_distance, replanning_time;
};

class SmoothNavigation : public rclcpp_lifecycle::LifecycleNode {
public:
    SmoothNavigation();
    bool navigateToPose(const Pose2D& goal);
    void cancelNavigation();
    NavigationState state();
};
}
```

---

## 5. Obstacle Avoidance 层

### 5.1 障碍物检测器 (`obstacle_detector.hpp`)

```cpp
namespace diffbot_navigation::obstacle_avoidance {

class ObstacleDetector {
public:
    std::vector<Obstacle> detect(const sensor_msgs::msg::LaserScan& scan,
                                  double robot_radius = 0.3);
    bool isPathBlocked(const Path& path, const std::vector<Obstacle>& obstacles);
};
}
```

### 5.2 避障规划器 (`obstacle_avoidance_planner.hpp`)

```cpp
namespace diffbot_navigation::obstacle_avoidance {

class ObstacleAvoidancePlanner {
public:
    Path replanAroundObstacle(const Path& original,
                               const std::vector<Obstacle>& obstacles,
                               const Costmap& map);
    Path emergencyStopPath(const Pose2D& current);
};
}
```

---

## 6. Narrow Passage 层

### 6.1 窄道检测器 (`narrow_passage_detector.hpp`)

```cpp
namespace diffbot_navigation::narrow_passage {

class NarrowPassageDetector {
public:
    bool isNarrowPassage(const std::vector<Obstacle>& obstacles,
                          double robot_width);
    double narrowestWidth(const std::vector<Obstacle>& obstacles);
};
}
```

### 6.2 窄道规划器 (`narrow_passage_planner.hpp`)

```cpp
namespace diffbot_navigation::narrow_passage {

class NarrowPassagePlanner {
public:
    Path planThroughPassage(const Pose2D& entry, const Pose2D& exit,
                             const std::vector<Obstacle>& walls);
};
}
```

---

## 版本兼容性

| API | v1.0 | v1.1 | 说明 |
|-----|------|------|------|
| `computePID/3` 自由函数 | ✅ | ⚠️ 保留但标记废弃 | 推导到 `PIDController::compute` |
| `PIDController` 结构体 | ❌ | ✅ | 新 API，替代自由函数 |
| `AntiWindupMode` 枚举 | ❌ | ✅ | 新增 |
| `AStarConfig` / `AStarConstants` | ❌ | ✅ | 新增 |
| `TrajectoryConfig` / `TrajectoryConstants` | ❌ | ✅ | 新增 |
| `std::getenv` 读参 | ✅ | ❌ 已移除 | `ROSConfig` + `ParameterManager` |

---

## 废弃 API 迁移指南

```cpp
// v1.0 (旧)
double integral = 0, prev_error = 0;
double output = computePID(error, dt, integral, prev_error, 1.0, 0.1, 0.01);

// v1.1 (新)
PIDController pid(1.0, 0.1, 0.01);
pid.setMode(AntiWindupMode::ConditionalIntegration);
double output = pid.compute(setpoint, measurement, dt);
```
