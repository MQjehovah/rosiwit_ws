// ============================================================
// Diffbot Navigation - 差速轮控制器
// 基于Pure Pursuit算法的差速轮运动控制
// ============================================================

#ifndef DIFFBOT_NAVIGATION__CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define DIFFBOT_NAVIGATION__CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>
#include <vector>
#include <string>

#include "diffbot_navigation/controller/pid_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace diffbot_navigation
{
namespace controller
{

/**
 * @brief 控制器配置参数
 */
struct ControllerConfig
{
  // 运动学参数
  double wheel_separation;        // 轮间距 (m)
  double wheel_radius;           // 轮半径 (m)

  // 速度限制
  double max_velocity_x;
  double max_velocity_theta;
  double min_velocity_x;
  double min_velocity_theta;

  // 加速度限制
  double max_accel_x;
  double max_accel_theta;
  double min_accel_x;
  double min_accel_theta;

  // Pure Pursuit参数
  double lookahead_distance;
  double min_lookahead_distance;
  double max_lookahead_distance;
  double lookahead_gain;

  // PID参数 (用于速度控制)
  double k_p_linear;
  double k_i_linear;
  double k_d_linear;
  double k_p_angular;
  double k_i_angular;
  double k_d_angular;

  // 目标容差
  double xy_goal_tolerance;
  double yaw_goal_tolerance;

  // 控制频率
  double controller_frequency;
};

// PIDController 和 PIDConstants 已提取到 pid_controller.hpp
// （通过上方 #include 引入，与 Nav2 解耦，支持独立测试）

/**
 * @class DiffDriveController
 * @brief 差速轮控制器，实现Pure Pursuit + PID控制
 */
class DiffDriveController : public nav2_core::Controller
{
public:
  /**
   * @brief 简化的控制配置（用于非ROS直接调用场景，含PID参数和限幅）
   */
  struct Config
  {
    double kp_linear = 1.0;
    double ki_linear = 0.0;
    double kd_linear = 0.0;
    double kp_angular = 1.0;
    double ki_angular = 0.0;
    double kd_angular = 0.0;
    double linear_integral_limit = 1.0;
    double angular_integral_limit = 1.0;
    double max_linear_velocity = 1.0;
    double max_angular_velocity = 1.5;
    double max_linear_accel = 3.0;
    double max_angular_accel = 4.5;
  };

  /**
   * @brief 构造函数
   */
  DiffDriveController();

  /**
   * @brief 析构函数
   */
  ~DiffDriveController() override = default;

  /**
   * @brief 配置控制器（简化接口，用于直接调用场景）
   * @param config 简化的控制配置参数
   */
  void configure(const Config& config);

  /**
   * @brief 单误差PID计算（简化接口，带抗饱和）
   * @param error 当前误差（setpoint - measured）
   * @param dt 时间步长（秒），dt <= 0 时返回零输出且不更新内部状态
   * @return 控制输出速度命令（线性用 linear.x，角速度用 angular.z）
   *
   * 内置抗饱和逻辑：
   * - 积分项独立限幅（clamping anti-windup）
   * - 条件积分：误差和积分符号相反时允许积分修正
   * - 输出限幅到 max_linear_velocity / max_angular_velocity
   * - 加速度限制（通过增量裁剪）
   */
  geometry_msgs::msg::Twist compute(double error, double dt);

  /**
   * @brief 简化模式计算控制输出（独立线速度/角速度误差）
   * @param linear_error 线性误差（目标 - 当前）
   * @param dt 时间步长
   * @param angular_error 角速度误差（目标 - 当前）
   * @return 控制输出速度命令
   */
  geometry_msgs::msg::Twist compute(double linear_error, double dt, double angular_error);

  /**
   * @brief 重置简化模式PID内部状态（积分项、上一次误差、饱和标志）
   */
  void reset();

  /**
   * @brief 配置控制器（ROS2生命周期接口）
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理资源
   */
  void cleanup() override;

  /**
   * @brief 激活控制器
   */
  void activate() override;

  /**
   * @brief 停用控制器
   */
  void deactivate() override;

  /**
   * @brief 设置速度限制
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief 计算速度命令
   * @param pose 当前位姿
   * @param velocity 当前速度
   * @param goal_waypoints 目标路径点
   * @return 速度命令
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置规划路径
   * @param path 全局路径
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 获取当前路径
   */
  const nav_msgs::msg::Path & getPath() const { return current_path_; }

  /**
   * @brief 重置控制器状态
   */
  void resetController();

  /**
   * @brief 是否到达目标
   */
  bool isGoalReached(
    const geometry_msgs::msg::PoseStamped & pose,
    nav2_core::GoalChecker * goal_checker);

private:
  /**
   * @brief 计算前视点
   */
  geometry_msgs::msg::PoseStamped computeLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & current_pose);

  /**
   * @brief 计算到目标的距离和角度
   */
  void computeDistanceAndAngle(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to,
    double & distance,
    double & angle);

  /**
   * @brief 计算Pure Pursuit曲率
   */
  double computePurePursuitCurvature(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & lookahead_point);

  /**
   * @brief 从曲率计算速度命令
   */
  geometry_msgs::msg::Twist computeVelocityFromCurvature(double curvature);

  /**
   * @brief 应用加速度约束
   */
  geometry_msgs::msg::Twist applyAccelerationConstraints(
    const geometry_msgs::msg::Twist & cmd_vel,
    const geometry_msgs::msg::Twist & current_vel,
    double dt);

  /**
   * @brief 规范化角度到 [-pi, pi]
   */
  double normalizeAngle(double angle);

  /**
   * @brief 找到路径上最近的点
   */
  int findClosestPointOnPath(
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 插值计算前视点
   */
  geometry_msgs::msg::PoseStamped interpolateLookaheadPoint(
    const geometry_msgs::msg::PoseStamped & pose,
    int closest_idx);

  /**
   * @brief 应用速度限制
   */
  void applySpeedLimit(geometry_msgs::msg::Twist & cmd_vel);

  // 成员变量
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string controller_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  ControllerConfig config_;
  bool configured_;
  bool active_;
  double speed_limit_;
  bool speed_limit_percentage_;

  // 当前路径
  nav_msgs::msg::Path current_path_;
  int closest_point_idx_;
  bool goal_reached_;

  // PID控制器（Pure Pursuit模式）
  PIDController linear_pid_;
  PIDController angular_pid_;

  // 简化模式 PID 状态（用于 compute(error, dt) 接口）
  Config simple_config_;
  double prev_linear_output_;
  double prev_angular_output_;

  // 时间记录
  rclcpp::Time last_time_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("diff_drive_controller")};
};

} // namespace controller
} // namespace diffbot_navigation

#endif // DIFFBOT_NAVIGATION__CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_