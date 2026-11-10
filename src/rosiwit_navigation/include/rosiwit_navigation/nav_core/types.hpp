#ifndef ROSIWIT_NAVIGATION__CORE__TYPES_HPP_
#define ROSIWIT_NAVIGATION__CORE__TYPES_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <cmath>
#include <utility>

namespace rosiwit_navigation
{
namespace core
{

struct Pose2D
{
  double x;
  double y;
  double theta;

  Pose2D() : x(0.0), y(0.0), theta(0.0) {}
  Pose2D(double x_in, double y_in, double theta_in) : x(x_in), y(y_in), theta(theta_in) {}

  double distanceTo(const Pose2D & other) const
  {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

struct PathPoint
{
  Pose2D pose;
  double velocity_x;
  double velocity_theta;

  PathPoint() : velocity_x(0.0), velocity_theta(0.0) {}
  PathPoint(const Pose2D & p) : pose(p), velocity_x(0.0), velocity_theta(0.0) {}
};

struct Path
{
  std::vector<PathPoint> points;
  bool empty() const { return points.empty(); }
  size_t size() const { return points.size(); }
};

struct VelocityCommand
{
  double linear_x;
  double lateral_y;
  double angular_z;

  VelocityCommand() : linear_x(0.0), lateral_y(0.0), angular_z(0.0) {}
  VelocityCommand(double vx, double vy, double wz)
    : linear_x(vx), lateral_y(vy), angular_z(wz) {}
};

struct PlannerConfig
{
  std::string name;
  double resolution;
  int max_iterations;
  double timeout;
  /// Hybrid A*: allow reverse motion primitives (D* forward-only if false).
  bool allow_reverse = true;
  /// Optional cap for a single reverse maneuver (0 = no explicit cap).
  double max_reverse_distance = 1.0;
  /// Recovery-only: only expand reverse primitives.
  bool reverse_only = false;
  /// Recovery-only start clearance (m); normal planning should leave 0.
  double start_relax_radius = 0.0;
  /// 加权 A* 的启发式倍率. 1.0 = 最优; >1 更贪心(次优但快很多).
  /// 膨胀代价会把 g 抬高 3~4 倍, 而欧氏启发式不变 -> h/g 失衡, A* 退化成
  /// Dijkstra: 实测 0.8m 门洞 8m 行程从 0.4ms 涨到 12.8s (节点超时 -> "规划失败").
  double heuristic_weight = 1.0;
  /// Hybrid A*: 原地转向基元的步长 (deg). 30° 时窄处"转不动"会直接搜不到解,
  /// 15° 给搜索更多可行中间姿态; 代价是分支数翻倍 (配合 timeout 调大)。
  double turn_step_deg = 30.0;
  /// 目标点容错半径 (m): 目标落在障碍/膨胀区时, 在半径内搜索最近的
  /// footprint-free 格作为实际规划目标 (对齐 nav2_navfn_planner 的 tolerance 语义)。
  /// 0 = 关闭 (保持旧的"目标在障碍里直接失败")。
  double goal_tolerance = 0.5;
};

struct KinematicsLimits
{
  double max_velocity_x;
  double max_velocity_theta;
};

// 机器人坐标系下的多边形 footprint: (x, y) 米. 空 = 点模型.
using Footprint = std::vector<std::pair<double, double>>;

struct ControllerConfig
{
  double lookahead_distance;
  double xy_goal_tolerance;
  double slow_down_distance;
  double max_lookahead_distance;
  double min_lookahead_distance;
  double lookahead_gain;
  KinematicsLimits kinematics;
  double max_linear_velocity;
  double max_angular_velocity;
  double max_accel_x = 0.0;
  double max_accel_theta = 0.0;
  // 长方形车体: 控制器在候选轨迹上按这个多边形做碰撞/代价检查.
  // 为空时退化为原来的"只看中心点"行为.
  Footprint footprint;
  double footprint_sample_spacing = 0.10;  // 多边形边采样间距 (m)
  /// 外部路径跟踪时把稀疏 waypoint 插值到最大间距(m); 0=关闭.
  double path_resample_step = 0.08;
  // ---- 以下对齐 nav2_regulated_pure_pursuit_controller 的参数语义 ----
  bool use_velocity_scaled_lookahead_dist = true;
  double lookahead_time = 1.5;
  /// 航向差超过它且车没在走 -> 原地转到路径朝向。
  /// 取 nav2 rotation_shim 的 angular_disengage_threshold = 0.3925 (22.5°):
  /// 只转到这个窗口就交给跟踪, 但残余的 20°+ 航向差在窄门里会把车体斜置
  /// 带进扫掠区, 所以门洞场景建议再小一些 (见 cfg_door_test.yaml)。
  bool use_rotate_to_heading = true;
  double rotate_to_heading_min_angle = 0.3925;
  double rotate_to_heading_angular_vel = 1.8;
  /// 曲率调速: 转弯半径小于它时降速
  bool use_regulated_linear_velocity_scaling = true;
  double regulated_linear_scaling_min_radius = 0.9;
  /// 任何调速的下限 (nav2 要求 > 0.1, 防止把车压死)
  double regulated_linear_scaling_min_speed = 0.2;
  /// 碰撞检测调速: 沿指令弧 max_allowed_time_to_collision_up_to_carrot 内
  /// 会撞到真实障碍就降速 (v,ω 同比例缩放, 曲率不变)
  bool use_collision_detection = true;
  double max_allowed_time_to_collision_up_to_carrot = 1.0;
  /// 末段调速 (nav2 approach_velocity_scaling_dist / min_approach_linear_velocity)
  double approach_velocity_scaling_dist = 0.6;
  double min_approach_linear_velocity = 0.05;
  // ---- 门区直穿 (窄口 lane): 行业里的 "door mode" ----
  // 窄口内可用横向余量只有几厘米, 而"边转边走"会把车体斜置撑宽扫掠区;
  // 同时高角速度会让雷达点云与位姿的时间偏斜放大 (实测 ω=0.8rad/s + 120ms
  // 时延 ≈ 5.5°, 在 0.6m 处把门框点"转"进门洞 7cm -> 幻影障碍 -> 停车)。
  // 所以: 窄口前 lane_engage_dist 内先限速把偏差收敛掉, 窄口内冻结 ω 直穿。
  bool door_lane_enabled = true;
  /// 净宽低于它就算窄口 (m)
  double lane_width = 0.95;
  /// 窄口前方的收敛段长度 (m): 只限速, 允许转向
  double lane_engage_dist = 1.0;
  /// 窄口后继续直穿的长度 (m)
  double lane_exit_dist = 0.4;
  double lane_max_speed = 0.20;
  /// 窄口内允许的角速度上限 (rad/s); 0 = 不冻结
  double lane_max_omega = 0.03;
  /// 航向差超过它就不冻结 (车本来就歪, 必须先转正)
  double lane_freeze_max_heading = 0.25;
  bool allow_reverse = true;
  // ---- LQR (ignored by other controllers) ----
  double lqr_q_x = 5.0;      // lateral error weight (x)
  double lqr_q_y = 5.0;      // lateral error weight (y)
  double lqr_q_theta = 1.0;  // heading error weight
  double lqr_r_v = 1.0;      // linear velocity control weight
  double lqr_r_w = 1.0;      // angular velocity control weight
  // MPPI tuning (ignored by other controllers).
  int mppi_horizon = 20;
  int mppi_samples = 160;
  double mppi_dt = 0.1;
  double mppi_lambda = 0.8;
  double mppi_sigma_v = 0.18;
  double mppi_sigma_w = 0.35;
  double mppi_path_weight = 2.0;
  double mppi_heading_weight = 0.4;
  double mppi_obstacle_weight = 0.03;
  double mppi_control_weight = 0.03;
  double mppi_speed_weight = 2.0;
  double mppi_goal_weight = 1.5;
  // ---- TEB (定时弹性带) ----
  double teb_weight_path = 20.0;
  double teb_weight_obstacle = 30.0;
  double teb_weight_kinematics = 6.0;
  double teb_weight_velocity = 4.0;
  double teb_weight_acceleration = 3.0;
  double teb_weight_time = 1.0;
  double teb_weight_shortest = 1.0;
  double teb_min_obstacle_dist = 0.18;
  double teb_inflation_dist = 0.30;
  double teb_max_deviation = 0.60;
  int teb_homotopy_classes = 3;
  double teb_homotopy_offset = 0.35;
  int teb_max_iterations = 3;
  int teb_max_samples = 80;
  double teb_dt_ref = 0.30;
  /// 控制器内部周期性调试输出 ([RPPProbe]/[RPPClr] 等). 默认关:
  /// 以前这两处 std::cout 常驻, 每 ~5s 往控制线程的 stdout 写一次, 仿真日志上百 MB.
  bool debug_probe = false;
};

struct CostmapGridInfo
{
  unsigned int width;
  unsigned int height;
  double resolution;
  struct
  {
    struct
    {
      double x;
      double y;
    } position;
  } origin;
};

struct CostmapGrid
{
  CostmapGridInfo info;
  std::vector<int8_t> data;
};

struct Costmap
{
  std::shared_ptr<CostmapGrid> grid;
  unsigned int width;
  unsigned int height;
  double resolution;
  double origin_x;
  double origin_y;
  // 与 nav_msgs/OccupancyGrid 同一套语义: -1 unknown, 0..100.
  std::vector<int8_t> data;
};

struct Obstacle
{
  double x;
  double y;
  double radius;
};

using ObstacleArray = std::vector<Obstacle>;

/**
 * @brief 激光扫描数据（纯C++，替代 sensor_msgs::LaserScan）
 */
struct RangeScan
{
  std::vector<float> ranges;
  std::vector<float> intensities;
  float angle_min = -3.14159f;
  float angle_max = 3.14159f;
  float angle_increment = 0.0174533f;
  float range_min = 0.1f;
  float range_max = 10.0f;
  size_t size() const { return ranges.size(); }
};

/**
 * @brief 点云数据（纯C++，替代 sensor_msgs::PointCloud2）
 */
struct PointCloud
{
  struct Point { float x, y, z; };
  std::vector<Point> points;
  bool empty() const { return points.empty(); }
  size_t size() const { return points.size(); }
};

struct TrajectoryPoint
{
  double x;
  double y;
  double theta;
  double vx;
  double vtheta;
  double time;
  double acceleration;
};

enum class ErrorCode
{
  SUCCESS = 0,
  NOT_INITIALIZED,
  PLANNING_FAILED,
  START_IN_OBSTACLE,
  GOAL_IN_OBSTACLE,
  TIMEOUT,
  NO_VALID_PATH,
  UNKNOWN
};

template<typename T>
class Result
{
public:
  static Result ok(const T & value) { return Result(value); }
  static Result error(ErrorCode code, const std::string & msg) { return Result(code, msg); }

  bool is_ok() const { return success_; }
  bool is_error() const { return !success_; }
  const T & value() const { return value_; }
  ErrorCode error_code() const { return error_code_; }
  std::string error_message() const { return error_message_; }

private:
  Result(const T & value) : success_(true), value_(value), error_code_(ErrorCode::SUCCESS) {}
  Result(ErrorCode code, const std::string & msg)
    : success_(false), error_code_(code), error_message_(msg) {}

  bool success_;
  T value_;
  ErrorCode error_code_;
  std::string error_message_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CORE__TYPES_HPP_
