// ============================================================
// Diffbot Navigation - 轨迹生成器
// 生成平滑的、符合运动学约束的轨迹
// ============================================================

#ifndef DIFFBOT_NAVIGATION__NAVIGATION__TRAJECTORY_GENERATOR_HPP_
#define DIFFBOT_NAVIGATION__NAVIGATION__TRAJECTORY_GENERATOR_HPP_

#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/path.hpp"

#include "diffbot_navigation/core/types.hpp"

namespace diffbot_navigation
{
namespace navigation
{

// 向后兼容：navigation::TrajectoryPoint 别名指向 core::TrajectoryPoint
using TrajectoryPoint = core::TrajectoryPoint;

/**
 * @brief 轨迹生成器常量
 */
namespace TrajectoryConstants
{
  constexpr double kMinDt = 0.01;          // 最小时间步长
  constexpr double kDefaultDt = 0.05;      // 默认时间步长
  constexpr double kEpsilon = 1e-9;        // 浮点比较容差
}  // namespace TrajectoryConstants

/**
 * @brief 轨迹生成参数
 */
struct TrajectoryConfig
{
  // 速度限制
  double max_velocity_x;
  double max_velocity_theta;
  double min_velocity_x;

  // 加速度限制
  double max_accel_x;
  double max_accel_theta;

  // 轨迹参数
  double sim_time;              // 模拟时间 (s)
  double sim_granularity;       // 模拟粒度 (s)
  double path_resolution;       // 路径分辨率 (m)

  // 目标容差
  double xy_goal_tolerance;
  double yaw_goal_tolerance;
};

/**
 * @class TrajectoryGenerator
 * @brief 轨迹生成器，支持差速轮模型
 */
class TrajectoryGenerator
{
public:
  /// DEF-003: 极短路径的最小轨迹点数，确保控制器有足够采样点
  static constexpr size_t kMinTrajectoryPoints = 3;

  /**
   * @brief 简化的轨迹生成配置（用于非ROS直接调用场景）
   */
  struct Config
  {
    double max_linear_velocity = 1.0;
    double max_angular_velocity = 1.5;
    double max_linear_accel = 3.0;
    double max_angular_accel = 4.5;
    double dt = 0.05;
    double xy_goal_tolerance = 0.1;
    double yaw_goal_tolerance = 0.1;
    double min_lookahead_distance = 0.3;
    double max_lookahead_distance = 0.9;
  };

  /**
   * @brief 构造函数
   */
  TrajectoryGenerator();

  /**
   * @brief 构造函数，带参数配置
   */
  explicit TrajectoryGenerator(const TrajectoryConfig & config);

  /**
   * @brief 析构函数
   */
  ~TrajectoryGenerator() = default;

  /**
   * @brief 设置配置参数
   */
  void setConfig(const TrajectoryConfig & config);

  /**
   * @brief 获取当前配置参数
   */
  const TrajectoryConfig & getConfig() const { return config_; }

  /**
   * @brief 配置简化参数
   */
  void configure(const Config& config);

  /**
   * @brief 从路径生成轨迹
   * @param path 输入路径
   * @param current_velocity 当前速度
   * @return 生成的轨迹点序列
   *
   * 边界处理：
   * - 空路径 → 返回空向量
   * - 单点路径 → 返回只含起点的轨迹，速度为0
   * - NaN/Inf 坐标 → 跳过异常点，记录日志
   * - 初始速度超过最大值 → 裁剪到允许范围
   */
  std::vector<TrajectoryPoint> generateTrajectory(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & current_velocity);

  /**
   * @brief 生成从起点到终点的轨迹
   * @param start 起点
   * @param goal 终点
   * @param start_vel 起始速度
   * @param end_vel 终点速度
   * @return 生成的轨迹点序列
   */
  std::vector<TrajectoryPoint> generateTrajectory(
    const geometry_msgs::msg::Pose2D & start,
    const geometry_msgs::msg::Pose2D & goal,
    const geometry_msgs::msg::Twist & start_vel,
    const geometry_msgs::msg::Twist & end_vel);

  /**
   * @brief 平滑速度曲线
   * @param trajectory 待平滑轨迹
   * @return 平滑后的轨迹
   */
  std::vector<TrajectoryPoint> smoothVelocity(
    const std::vector<TrajectoryPoint> & trajectory);

  /**
   * @brief 应用加速度约束
   * @param trajectory 待约束轨迹
   * @return 约束后的轨迹
   */
  std::vector<TrajectoryPoint> applyAccelerationConstraints(
    const std::vector<TrajectoryPoint> & trajectory);

  /**
   * @brief 检查轨迹是否有效
   * @param trajectory 待检查轨迹
   * @return 是否有效
   */
  bool validateTrajectory(const std::vector<TrajectoryPoint> & trajectory);

  /**
   * @brief 将轨迹转换为ROS Path消息
   * @param trajectory 轨迹点序列
   * @return ROS Path消息
   */
  nav_msgs::msg::Path toPathMsg(const std::vector<TrajectoryPoint> & trajectory);

  /**
   * @brief 设置轨迹 frame_id
   */
  void setFrameId(const std::string & frame_id) { frame_id_ = frame_id; }

private:
  /**
   * @brief 计算两点之间的距离
   */
  double distance(
    const geometry_msgs::msg::Pose2D & p1,
    const geometry_msgs::msg::Pose2D & p2);

  /**
   * @brief 规范化角度到 [-pi, pi]
   */
  double normalizeAngle(double angle);

  /**
   * @brief 计算转向角
   */
  double computeSteeringAngle(
    const geometry_msgs::msg::Pose2D & current,
    const geometry_msgs::msg::Pose2D & target);

  /**
   * @brief 应用速度限制
   */
  double applyVelocityLimit(double velocity, double max, double min);

  /**
   * @brief 计算停止距离
   */
  double computeStoppingDistance(double velocity);

  // 成员变量
  TrajectoryConfig config_;
  Config simple_config_;
  std::string frame_id_ = "odom";
  rclcpp::Logger logger_{rclcpp::get_logger("trajectory_generator")};
};

} // namespace navigation
} // namespace diffbot_navigation

#endif // DIFFBOT_NAVIGATION__NAVIGATION__TRAJECTORY_GENERATOR_HPP_