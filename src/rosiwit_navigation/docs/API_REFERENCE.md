# API 参考文档

> ⚠️ **文档状态**: 部分章节已过期（2026-05-04 审计标记）
> 以下内容需对照最新代码 `include/diffbot_navigation/` 进行更新：
> - **PIDController** (§4.2): 描述的是旧版 `computePID` 自由函数，实际代码已重构为 `PIDController` 结构体 + `AntiWindupMode` 枚举。见团队文档 `docs/api.md`。
> - **AStarPlanner**: 完全缺失。实际代码有 `AStarPlanner` 类 + `AStarConstants` + `AStarConfig`。
> - **TrajectoryGenerator**: 未描述空路径/单点路径/NaN 处理逻辑。已增强。
>
> 最新完整 API 参考见: `E:\xzkj\agent\workspace\agents\AI开发团队\docs\api.md`

本文档详细描述了 Diffbot Navigation 功能包的所有核心 API 接口。

---

## 目录

- [导航模块 (Navigation)](#导航模块-navigation)
  - [SmoothNavigation](#smoothnavigation)
  - [PathPlanner](#pathplanner)
  - [TrajectoryGenerator](#trajectorygenerator)
- [运动控制模块 (Controller)](#运动控制模块-controller)
  - [DiffDriveController](#diffdrivecontroller)
  - [VelocityLimiter](#velocitylimiter)
- [避障模块 (Obstacle Avoidance)](#避障模块-obstacle-avoidance)
  - [ObstacleDetector](#obstacledetector)
  - [ObstacleAvoidancePlanner](#obstacleavoidanceplanner)
- [窄道通行模块 (Narrow Passage)](#窄道通行模块-narrow-passage)
  - [NarrowPassageDetector](#narrowpassagedetector)
  - [NarrowPassagePlanner](#narrowpassageplanner)
- [数据结构](#数据结构)
- [枚举类型](#枚举类型)

---

## 导航模块 (Navigation)

### SmoothNavigation

**头文件**: `diffbot_navigation/navigation/smooth_navigation.hpp`

平滑导航主控制器，协调路径规划和轨迹跟踪，实现丝滑的单点导航功能。

#### 类定义

```cpp
class SmoothNavigation : public rclcpp_lifecycle::LifecycleNode
```

#### 构造函数

```cpp
explicit SmoothNavigation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
```

#### 主要方法

| 方法 | 返回类型 | 描述 |
|------|----------|------|
| `configure()` | `nav2_util::CallbackReturn` | 配置节点，初始化参数和订阅 |
| `activate()` | `nav2_util::CallbackReturn` | 激活节点，启动导航服务 |
| `deactivate()` | `nav2_util::CallbackReturn` | 停用节点，暂停导航 |
| `cleanup()` | `nav2_util::CallbackReturn` | 清理资源 |
| `shutdown()` | `nav2_util::CallbackReturn` | 关闭节点 |

#### 核心功能

```cpp
// 导航到目标点
void navigateToGoal(const geometry_msgs::msg::PoseStamped & goal);

// 停止导航
void stopNavigation();

// 暂停/恢复导航
void pauseNavigation();
void resumeNavigation();

// 获取导航状态
NavigationState getState() const;

// 获取当前路径
nav_msgs::msg::Path getCurrentPath() const;
```

#### 话题订阅

| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `~/goal_pose` | `geometry_msgs/msg/PoseStamped` | 目标位姿 |
| `~/odom` | `nav_msgs/msg/Odometry` | 里程计 |
| `~/scan` | `sensor_msgs/msg/LaserScan` | 激光雷达数据 |

#### 话题发布

| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `~/cmd_vel` | `geometry_msgs/msg/Twist` | 速度命令 |
| `~/path` | `nav_msgs/msg/Path` | 规划路径 |
| `~/trajectory` | `nav_msgs/msg/Path` | 当前轨迹 |

#### 服务

| 服务 | 类型 | 描述 |
|------|------|------|
| `~/navigate` | `nav2_msgs/action/NavigateToPose` | 导航 Action |

---

### PathPlanner

**头文件**: `diffbot_navigation/navigation/path_planner.hpp`

全局路径规划器，基于 Smac Hybrid A* 算法，支持差速轮运动学约束。

#### 类定义

```cpp
class PathPlanner : public nav2_core::GlobalPlanner
```

#### 构造函数

```cpp
PathPlanner();
~PathPlanner() override = default;
```

#### 主要方法

```cpp
// 配置规划器
void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

// 清理规划器
void cleanup() override;

// 激活规划器
void activate() override;

// 停用规划器
void deactivate() override;

// 计算路径
nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
```

#### 配置参数

```cpp
struct PlannerConfig {
    double wheel_separation;    // 轮间距 (m)
    double wheel_radius;        // 轮半径 (m)
    double tolerance;           // 目标容差 (m)
    bool use_astar;            // 是否使用 A*
    bool allow_unknown;        // 是否允许未知区域
    double path_resolution;    // 路径分辨率 (m)
    double smooth_cost;        // 平滑代价权重
    double obstacle_cost;      // 障碍物代价权重
};
```

#### 路径点结构

```cpp
struct PathPoint {
    double x;          // x 坐标 (m)
    double y;          // y 坐标 (m)
    double theta;      // 朝向角 (rad)
    double velocity;   // 建议速度 (m/s)
    double curvature;  // 曲率 (rad/m)
};
```

---

### TrajectoryGenerator

**头文件**: `diffbot_navigation/navigation/trajectory_generator.hpp`

轨迹生成器，生成平滑的、符合运动学约束的轨迹，采用五次多项式插值。

#### 类定义

```cpp
class TrajectoryGenerator
```

#### 构造函数

```cpp
TrajectoryGenerator();
explicit TrajectoryGenerator(const TrajectoryConfig & config);
```

#### 主要方法

```cpp
// 设置配置参数
void setConfig(const TrajectoryConfig & config);

// 从路径生成轨迹
std::vector<TrajectoryPoint> generateTrajectory(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & current_velocity);

// 生成单段轨迹
std::vector<TrajectoryPoint> generateSegment(
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & end,
    double start_velocity,
    double end_velocity);

// 优化轨迹
void optimizeTrajectory(std::vector<TrajectoryPoint> & trajectory);

// 验证轨迹
bool validateTrajectory(const std::vector<TrajectoryPoint> & trajectory);
```

#### 轨迹点结构

```cpp
struct TrajectoryPoint {
    double x;          // x 坐标 (m)
    double y;          // y 坐标 (m)
    double theta;      // 朝向角 (rad)
    double v_x;        // x 方向速度 (m/s)
    double v_theta;    // 角速度 (rad/s)
    double time;       // 时间戳 (s)
};
```

#### 配置参数

```cpp
struct TrajectoryConfig {
    double max_velocity_x;      // 最大线速度 (m/s)
    double max_velocity_theta;  // 最大角速度 (rad/s)
    double min_velocity_x;      // 最小线速度 (m/s)
    double max_accel_x;         // 最大线加速度 (m/s²)
    double max_accel_theta;     // 最大角加速度 (rad/s²)
    double sim_time;           // 模拟时间 (s)
    double sim_granularity;    // 模拟粒度 (s)
    double path_resolution;    // 路径分辨率 (m)
    double xy_goal_tolerance;   // 位置目标容差 (m)
    double yaw_goal_tolerance;  // 角度目标容差 (rad)
};
```

---

## 运动控制模块 (Controller)

### DiffDriveController

**头文件**: `diffbot_navigation/controller/diff_drive_controller.hpp`

差速轮运动控制器，基于 Pure Pursuit 算法和 PID 控制。

#### 类定义

```cpp
class DiffDriveController : public nav2_core::Controller
```

#### 构造函数

```cpp
DiffDriveController();
~DiffDriveController() override = default;
```

#### 主要方法

```cpp
// 配置控制器
void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

// 清理控制器
void cleanup() override;

// 激活控制器
void activate() override;

// 停用控制器
void deactivate() override;

// 计算速度命令
geometry_msgs::msg::Twist computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

// 设置规划路径
void setPlan(const nav_msgs::msg::Path & path) override;
```

#### Pure Pursuit 控制

```cpp
// 计算前视点
geometry_msgs::msg::PoseStamped computeLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & path);

// 计算转向角
double computeSteeringAngle(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

// 计算曲率
double computeCurvature(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);
```

#### PID 控制

```cpp
struct PIDController {
    double k_p;              // 比例增益
    double k_i;              // 积分增益
    double k_d;              // 微分增益
    double integral_error;   // 积分误差
    double previous_error;    // 前一时刻误差
    double integral_limit;   // 积分限幅
    double output_limit;     // 输出限幅
};

// 计算 PID 输出
double computePID(PIDController & pid, double error, double dt);

// 重置 PID 控制器
void resetPID(PIDController & pid);
```

#### 配置参数

```cpp
struct ControllerConfig {
    // 运动学参数
    double wheel_separation;         // 轮间距 (m)
    double wheel_radius;             // 轮半径 (m)

    // 速度限制
    double max_velocity_x;           // 最大前进速度 (m/s)
    double max_velocity_theta;       // 最大旋转速度 (rad/s)
    double min_velocity_x;           // 最小前进速度 (m/s)

    // 加速度限制
    double max_accel_x;              // 最大线加速度 (m/s²)
    double max_accel_theta;          // 最大角加速度 (rad/s²)

    // Pure Pursuit 参数
    double lookahead_distance;       // 前视距离 (m)
    double min_lookahead_distance;   // 最小前视距离 (m)
    double max_lookahead_distance;   // 最大前视距离 (m)
    double lookahead_gain;           // 前视距离增益

    // PID 参数
    double k_p_linear;              // 线速度比例增益
    double k_i_linear;              // 线速度积分增益
    double k_d_linear;              // 线速度微分增益
    double k_p_angular;             // 角速度比例增益
    double k_i_angular;             // 角速度积分增益
    double k_d_angular;             // 角速度微分增益

    // 目标容差
    double xy_goal_tolerance;        // 位置容差 (m)
    double yaw_goal_tolerance;       // 角度容差 (rad)
};
```

---

### VelocityLimiter

**头文件**: `diffbot_navigation/controller/velocity_limiter.hpp`

速度限制器，确保速度命令符合物理约束和安全限制。

#### 类定义

```cpp
class VelocityLimiter
```

#### 构造函数

```cpp
VelocityLimiter();
explicit VelocityLimiter(const VelocityLimits & limits);
```

#### 主要方法

```cpp
// 设置速度限制
void setLimits(const VelocityLimits & limits);

// 获取当前限制
const VelocityLimits & getLimits() const;

// 应用速度限制
geometry_msgs::msg::Twist limitVelocity(
    const geometry_msgs::msg::Twist & velocity);

// 应用加速度限制
geometry_msgs::msg::Twist limitAcceleration(
    const geometry_msgs::msg::Twist & current_velocity,
    const geometry_msgs::msg::Twist & target_velocity,
    double dt);

// 应用急动限制
geometry_msgs::msg::Twist limitJerk(
    const geometry_msgs::msg::Twist & current_velocity,
    const geometry_msgs::msg::Twist & target_velocity,
    double dt);

// 计算平滑速度
geometry_msgs::msg::Twist computeSmoothVelocity(
    const geometry_msgs::msg::Twist & current_velocity,
    const geometry_msgs::msg::Twist & target_velocity,
    double dt);
```

#### 速度限制参数

```cpp
struct VelocityLimits {
    double max_velocity_x;      // 最大前进速度 (m/s)
    double max_velocity_theta;  // 最大旋转速度 (rad/s)
    double min_velocity_x;      // 最小前进速度 (m/s)
    double min_velocity_theta;  // 最小旋转速度 (rad/s)

    double max_accel_x;         // 最大线加速度 (m/s²)
    double max_accel_theta;     // 最大角加速度 (rad/s²)
    double min_accel_x;         // 最小线加速度 (m/s²)
    double min_accel_theta;     // 最小角加速度 (rad/s²)

    double max_decel_x;         // 最大线减速度 (m/s²)
    double max_decel_theta;     // 最大角减速度 (rad/s²)
};
```

---

## 避障模块 (Obstacle Avoidance)

### ObstacleDetector

**头文件**: `diffbot_navigation/obstacle_avoidance/obstacle_detector.hpp`

障碍物检测器，基于激光雷达和点云数据检测障碍物。

#### 类定义

```cpp
class ObstacleDetector
```

#### 构造函数

```cpp
ObstacleDetector();
explicit ObstacleDetector(const DetectorConfig & config);
```

#### 主要方法

```cpp
// 配置检测器
void configure(const DetectorConfig & config);

// 处理激光扫描数据
std::vector<Obstacle> processLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);

// 处理点云数据
std::vector<Obstacle> processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

// 获取障碍物列表
std::vector<Obstacle> getObstacles() const;

// 获取最近障碍物
Obstacle getNearestObstacle() const;

// 检测前方障碍物
bool hasObstacleInFront(double distance, double angle_range);

// 设置代价地图
void setCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);
```

#### 障碍物信息

```cpp
struct Obstacle {
    double x;              // x 坐标 (m)
    double y;              // y 坐标 (m)
    double z;              // z 坐标 (m)
    double distance;       // 到机器人的距离 (m)
    double angle;          // 相对机器人的角度 (rad)
    double radius;         // 障碍物半径估计 (m)
    bool is_dynamic;       // 是否为动态障碍物
    double velocity_x;     // 障碍物速度 x 分量 (m/s)
    double velocity_y;     // 障碍物速度 y 分量 (m/s)
    double confidence;     // 检测置信度 [0, 1]
};
```

#### 配置参数

```cpp
struct DetectorConfig {
    // 检测范围
    double detection_range;            // 检测范围 (m)
    double min_obstacle_distance;      // 最小障碍物距离 (m)
    double safe_distance;              // 安全距离 (m)

    // 激光雷达参数
    double laser_min_range;
    double laser_max_range;
    double laser_min_angle;
    double laser_max_angle;

    // 点云参数
    double point_cloud_min_height;
    double point_cloud_max_height;

    // 聚类参数
    double cluster_tolerance;          // 聚类容差 (m)
    int min_cluster_size;              // 最小聚类点数
    int max_cluster_size;              // 最大聚类点数

    // 动态障碍物检测
    bool enable_dynamic_detection;
    double dynamic_velocity_threshold;  // 动态障碍物速度阈值 (m/s)
};
```

---

### ObstacleAvoidancePlanner

**头文件**: `diffbot_navigation/obstacle_avoidance/obstacle_avoidance_planner.hpp`

避障规划器，采用动态窗口法 (DWA) 和人工势场法混合避障。

#### 类定义

```cpp
class ObstacleAvoidancePlanner : public nav2_core::Controller
```

#### 构造函数

```cpp
ObstacleAvoidancePlanner();
~ObstacleAvoidancePlanner() override = default;
```

#### 主要方法

```cpp
// 配置规划器
void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

// 计算避障速度
geometry_msgs::msg::Twist computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

// 设置障碍物检测器
void setObstacleDetector(std::shared_ptr<ObstacleDetector> detector);

// 设置避障模式
void setAvoidanceMode(AvoidanceMode mode);

// 检查避障状态
bool isAvoidanceActive() const;
```

#### 动态窗口法 (DWA)

```cpp
// 速度采样
struct VelocitySample {
    double v_x;        // 线速度 (m/s)
    double v_theta;    // 角速度 (rad/s)
    double score;      // 评分
};

// 生成速度采样
std::vector<VelocitySample> generateVelocitySamples(
    const geometry_msgs::msg::Twist & current_velocity);

// 评估速度样本
double evaluateVelocitySample(
    const VelocitySample & sample,
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & path,
    const std::vector<Obstacle> & obstacles);

// 选择最优速度
VelocitySample selectBestVelocity(
    const std::vector<VelocitySample> & samples);
```

#### 人工势场法

```cpp
// 计算斥力
geometry_msgs::msg::Twist computeRepulsiveForce(
    const std::vector<Obstacle> & obstacles);

// 计算引力
geometry_msgs::msg::Twist computeAttractiveForce(
    const geometry_msgs::msg::PoseStamped & goal);

// 合成势场力
geometry_msgs::msg::Twist computePotentialField(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<Obstacle> & obstacles);
```

#### 配置参数

```cpp
struct AvoidanceConfig {
    AvoidanceMode mode;                   // 避障模式
    double avoidance_radius;              // 避障半径 (m)
    double avoidance_velocity;           // 避障速度 (m/s)
    double min_avoidance_distance;       // 最小避障距离 (m)
    bool allow_reverse;                  // 是否允许倒退
    double max_avoidance_time;           // 最大避障时间 (s)

    // 动态障碍物预测
    bool enable_prediction;
    double prediction_time;              // 预测时间 (s)
    double prediction_step;              // 预测步长 (s)

    // 代价函数权重
    double weight_obstacle_distance;
    double weight_path_alignment;
    double weight_goal_distance;
    double weight_velocity_alignment;
    double weight_acceleration;
};
```

---

## 窄道通行模块 (Narrow Passage)

### NarrowPassageDetector

**头文件**: `diffbot_navigation/narrow_passage/narrow_passage_detector.hpp`

窄道检测器，识别和评估窄道区域。

#### 类定义

```cpp
class NarrowPassageDetector
```

#### 构造函数

```cpp
NarrowPassageDetector();
explicit NarrowPassageDetector(const DetectorConfig & config);
```

#### 主要方法

```cpp
// 配置检测器
void configure(const DetectorConfig & config);

// 检测窄道
std::vector<NarrowPassage> detectPassages(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);

// 检测前方窄道
std::vector<NarrowPassage> detectPassagesInFront(
    const geometry_msgs::msg::PoseStamped & pose,
    double look_ahead_distance);

// 评估窄道可通过性
bool evaluatePassageTraversability(const NarrowPassage & passage);

// 计算安全余量
double calculateSafetyMargin(const NarrowPassage & passage);

// 检查是否在窄道中
bool isInNarrowPassage() const;

// 获取当前窄道
NarrowPassage getCurrentPassage() const;
```

#### 窄道信息

```cpp
struct NarrowPassage {
    double start_x;              // 起点 x (m)
    double start_y;              // 起点 y (m)
    double end_x;                // 终点 x (m)
    double end_y;                // 终点 y (m)
    double width;                // 通道宽度 (m)
    double length;               // 通道长度 (m)
    double orientation;          // 通道方向 (rad)
    bool is_traversable;         // 是否可通过
    double safety_margin;        // 安全余量 (m)
    double recommended_velocity; // 建议速度 (m/s)
    double centerline_x;         // 中心线 x
    double centerline_y;         // 中心线 y
};
```

#### 配置参数

```cpp
struct DetectorConfig {
    // 机器人参数
    double robot_width;              // 机器人宽度 (m)
    double robot_length;             // 机器人长度 (m)

    // 窄道检测参数
    double min_passage_width;        // 最小通道宽度 (m)
    double detection_range;          // 检测范围 (m)
    double safety_margin;            // 安全余量 (m)

    // 通道识别参数
    double width_threshold;          // 宽度阈值
    double length_threshold;         // 长度阈值

    // 激光雷达参数
    double laser_min_range;
    double laser_max_range;
    double laser_angular_resolution;

    // 精确模式参数
    bool precision_mode;
    double precision_safety_distance;
};
```

---

### NarrowPassagePlanner

**头文件**: `diffbot_navigation/narrow_passage/narrow_passage_planner.hpp`

窄道通行规划器，生成安全通过窄道的路径。

#### 类定义

```cpp
class NarrowPassagePlanner : public nav2_core::Controller
```

#### 构造函数

```cpp
NarrowPassagePlanner();
~NarrowPassagePlanner() override = default;
```

#### 主要方法

```cpp
// 配置规划器
void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

// 计算通行速度
geometry_msgs::msg::Twist computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

// 设置窄道检测器
void setPassageDetector(std::shared_ptr<NarrowPassageDetector> detector);

// 开始通行
bool startPassage(const NarrowPassage & passage);

// 停止通行
void stopPassage();

// 获取通行状态
PassageState getPassageState() const;

// 检查是否需要通行
bool needsPassage(const geometry_msgs::msg::PoseStamped & pose);
```

#### 通行控制

```cpp
// 计算通行路径
nav_msgs::msg::Path computePassagePath(const NarrowPassage & passage);

// 计算中心线跟踪
geometry_msgs::msg::Twist computeCenterlineTracking(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

// 计算角度修正
double computeAngleCorrection(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

// 计算横向修正
double computeLateralCorrection(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

// 计算安全速度
double computeSafeVelocity(
    const NarrowPassage & passage,
    const geometry_msgs::msg::Twist & current_velocity);
```

#### 配置参数

```cpp
struct PassageConfig {
    // 通行策略参数
    double passage_velocity;          // 通行速度 (m/s)
    double max_passage_velocity;      // 最大通行速度 (m/s)
    double angle_correction_gain;     // 角度修正增益
    double lateral_correction_gain;   // 横向修正增益

    // 精确模式参数
    bool precision_mode;
    double precision_safety_distance;

    // 通行检查参数
    double min_passage_time;          // 最小通行时间 (s)
    bool check_exit_space;           // 是否检查出口空间

    // 机器人参数
    double robot_width;
    double robot_length;
    double wheel_separation;

    // 安全参数
    double safety_margin;
    double emergency_stop_distance;
};
```

---

## 数据结构

### 导航参数

```cpp
struct NavigationParams {
    // 控制频率
    double controller_frequency;      // 控制频率 (Hz)
    double planner_frequency;         // 规划频率 (Hz)

    // 速度限制
    double max_velocity_x;            // 最大前进速度 (m/s)
    double max_velocity_theta;         // 最大旋转速度 (rad/s)
    double min_velocity_x;            // 最小前进速度 (m/s)

    // 加速度限制
    double max_accel_x;               // 最大线加速度 (m/s²)
    double max_accel_theta;           // 最大角加速度 (rad/s²)

    // 目标容差
    double goal_tolerance_xy;         // 位置容差 (m)
    double goal_tolerance_yaw;        // 角度容差 (rad)

    // 轨迹跟踪参数
    double lookahead_distance;        // 前视距离 (m)
    double min_lookahead_distance;    // 最小前视距离 (m)
    double max_lookahead_distance;    // 最大前视距离 (m)

    // 重规划参数
    double replanning_distance;       // 重规划距离 (m)
    double replanning_time;           // 重规划时间 (s)
};
```

---

## 枚举类型

### 导航状态

```cpp
enum class NavigationState {
    IDLE,                // 空闲
    PLANNING,            // 规划中
    CONTROLLING,         // 控制中
    OBSTACLE_AVOIDANCE,  // 避障中
    NARROW_PASSAGE,      // 窄道通行中
    GOAL_REACHED,        // 到达目标
    FAILED               // 失败
};
```

### 避障模式

```cpp
enum class AvoidanceMode {
    DYNAMIC_WINDOW,      // 动态窗口法
    POTENTIAL_FIELD,     // 人工势场法
    HYBRID               // 混合模式
};
```

### 通行状态

```cpp
enum class PassageState {
    IDLE,         // 空闲
    APPROACHING,  // 接近中
    ENTERING,     // 进入中
    PASSING,      // 通行中
    EXITING,      // 退出中
    COMPLETED,    // 完成
    FAILED        // 失败
};
```

---

## ROS 接口

### 话题

| 话题名称 | 方向 | 消息类型 | 描述 |
|----------|------|----------|------|
| `~/cmd_vel` | 发布 | `geometry_msgs/msg/Twist` | 速度命令 |
| `~/odom` | 订阅 | `nav_msgs/msg/Odometry` | 里程计 |
| `~/scan` | 订阅 | `sensor_msgs/msg/LaserScan` | 激光雷达数据 |
| `~/map` | 订阅 | `nav_msgs/msg/OccupancyGrid` | 地图 |
| `~/goal_pose` | 订阅 | `geometry_msgs/msg/PoseStamped` | 目标位姿 |
| `~/path` | 发布 | `nav_msgs/msg/Path` | 规划路径 |
| `~/trajectory` | 发布 | `nav_msgs/msg/Path` | 当前轨迹 |

### 服务

| 服务名称 | 类型 | 描述 |
|----------|------|------|
| `~/navigate` | `nav2_msgs/action/NavigateToPose` | 导航 Action |
| `~/stop` | `std_srvs/srv/Empty` | 停止导航 |
| `~/pause` | `std_srvs/srv/Empty` | 暂停导航 |
| `~/resume` | `std_srvs/srv/Empty` | 恢复导航 |

### 参数

| 参数名称 | 类型 | 默认值 | 描述 |
|----------|------|--------|------|
| `controller_frequency` | double | 20.0 | 控制频率 (Hz) |
| `planner_frequency` | double | 2.0 | 规划频率 (Hz) |
| `max_velocity_x` | double | 0.5 | 最大前进速度 (m/s) |
| `max_velocity_theta` | double | 1.0 | 最大旋转速度 (rad/s) |
| `max_accel_x` | double | 0.5 | 最大线加速度 (m/s²) |
| `max_accel_theta` | double | 1.0 | 最大角加速度 (rad/s²) |
| `goal_tolerance_xy` | double | 0.1 | 位置容差 (m) |
| `goal_tolerance_yaw` | double | 0.1 | 角度容差 (rad) |

---

## 使用示例

### 基本导航

```cpp
#include "diffbot_navigation/navigation/smooth_navigation.hpp"

// 创建导航节点
auto nav_node = std::make_shared<diffbot_navigation::navigation::SmoothNavigation>();

// 设置目标位姿
geometry_msgs::msg::PoseStamped goal;
goal.header.frame_id = "map";
goal.pose.position.x = 1.0;
goal.pose.position.y = 2.0;
goal.pose.orientation.w = 1.0;

// 开始导航
nav_node->navigateToGoal(goal);
```

### 使用路径规划器

```cpp
#include "diffbot_navigation/navigation/path_planner.hpp"

// 创建规划器
diffbot_navigation::navigation::PathPlanner planner;

// 设置起点和终点
geometry_msgs::msg::PoseStamped start, goal;
// ... 设置位姿

// 计算路径
nav_msgs::msg::Path path = planner.createPlan(start, goal);
```

### 使用轨迹生成器

```cpp
#include "diffbot_navigation/navigation/trajectory_generator.hpp"

// 配置参数
diffbot_navigation::navigation::TrajectoryConfig config;
config.max_velocity_x = 0.5;
config.max_velocity_theta = 1.0;

// 创建生成器
diffbot_navigation::navigation::TrajectoryGenerator generator(config);

// 生成轨迹
geometry_msgs::msg::Twist current_velocity;
nav_msgs::msg::Path path;

auto trajectory = generator.generateTrajectory(path, current_velocity);
```

---

**文档版本**: 0.1.0
**最后更新**: 2026-04-25
**维护者**: AI Development Team