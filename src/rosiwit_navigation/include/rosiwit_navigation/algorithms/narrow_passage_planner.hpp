// ============================================================
// Diffbot Navigation - 窄道通行规划器
// 安全通过窄道的路径规划
// ============================================================

#ifndef ROSIWIT_NAVIGATION__NARROW_PASSAGE__NARROW_PASSAGE_PLANNER_HPP_
#define ROSIWIT_NAVIGATION__NARROW_PASSAGE__NARROW_PASSAGE_PLANNER_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

#include "rosiwit_navigation/algorithms/narrow_passage_detector.hpp"

namespace rosiwit_navigation
{
namespace narrow_passage
{

/**
 * @brief 窄道通行参数
 */
struct PassageConfig
{
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

/**
 * @brief 通行状态
 */
enum class PassageState
{
  IDLE,
  APPROACHING,
  ENTERING,
  PASSING,
  EXITING,
  COMPLETED,
  FAILED
};

/**
 * @class NarrowPassagePlanner
 * @brief 窄道通行规划器
 */
class NarrowPassagePlanner : public nav2_core::Controller
{
public:
  /**
   * @brief 构造函数
   */
  NarrowPassagePlanner();

  /**
   * @brief 析构函数
   */
  ~NarrowPassagePlanner() override = default;

  /**
   * @brief 配置规划器
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
   * @brief 激活规划器
   */
  void activate() override;

  /**
   * @brief 停用规划器
   */
  void deactivate() override;

  /**
   * @brief 设置速度限制
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief 计算速度命令
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置规划路径
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 设置窄道检测器
   */
  void setPassageDetector(std::shared_ptr<NarrowPassageDetector> detector);

  /**
   * @brief 检查是否需要窄道通行模式
   */
  bool needsPassageMode(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief 获取当前通行状态
   */
  PassageState getState() const { return current_state_; }

  /**
   * @brief 重置通行状态
   */
  void resetState() { current_state_ = PassageState::IDLE; }

private:
  /**
   * @brief 计算接近阶段的控制
   */
  geometry_msgs::msg::Twist computeApproachingControl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const NarrowPassage & passage);

  /**
   * @brief 计算进入阶段的控制
   */
  geometry_msgs::msg::Twist computeEnteringControl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const NarrowPassage & passage);

  /**
   * @brief 计算通过阶段的控制
   */
  geometry_msgs::msg::Twist computePassingControl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const NarrowPassage & passage);

  /**
   * @brief 计算离开阶段的控制
   */
  geometry_msgs::msg::Twist computeExitingControl(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const NarrowPassage & passage);

  /**
   * @brief 计算中心线跟踪控制
   */
  geometry_msgs::msg::Twist computeCenterlineTracking(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 计算姿态修正
   */
  geometry_msgs::msg::Twist computePoseCorrection(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 计算到中心线的横向误差
   */
  double computeLateralError(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 计算航向误差
   */
  double computeHeadingError(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 更新通行状态
   */
  void updatePassageState(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 检查是否完成通行
   */
  bool isPassageCompleted(
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 计算建议速度
   */
  double computeRecommendedVelocity(
    const NarrowPassage & passage,
    double lateral_error);

  /**
   * @brief 应用安全约束
   */
  geometry_msgs::msg::Twist applySafetyConstraints(
    const geometry_msgs::msg::Twist & cmd_vel,
    const geometry_msgs::msg::PoseStamped & pose,
    const NarrowPassage & passage);

  /**
   * @brief 规范化角度到 [-pi, pi]
   */
  double normalizeAngle(double angle);

  /**
   * @brief 检查是否紧急停止
   */
  bool checkEmergencyStop(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const NarrowPassage & passage);

  // 成员变量
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string planner_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<NarrowPassageDetector> passage_detector_;

  PassageConfig config_;
  PassageState current_state_;
  bool configured_;
  bool active_;
  double speed_limit_;
  bool speed_limit_percentage_;

  // 当前路径
  nav_msgs::msg::Path current_path_;

  // 当前通道
  NarrowPassage current_passage_;

  // 时间记录
  rclcpp::Time passage_start_time_;
  rclcpp::Time last_time_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("narrow_passage_planner")};
};

} // namespace narrow_passage
} // namespace rosiwit_navigation

#endif // ROSIWIT_NAVIGATION__NARROW_PASSAGE__NARROW_PASSAGE_PLANNER_HPP_