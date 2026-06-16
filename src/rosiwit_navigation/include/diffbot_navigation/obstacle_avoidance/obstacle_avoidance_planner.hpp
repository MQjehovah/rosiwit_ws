// ============================================================
// Diffbot Navigation - 避障规划器
// 动态窗口法和人工势场法混合避障
// ============================================================

#ifndef DIFFBOT_NAVIGATION__OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_PLANNER_HPP_
#define DIFFBOT_NAVIGATION__OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_PLANNER_HPP_

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

#include "diffbot_navigation/obstacle_avoidance/obstacle_detector.hpp"

namespace diffbot_navigation
{
namespace obstacle_avoidance
{

/**
 * @brief 避障模式
 */
enum class AvoidanceMode
{
  DYNAMIC_WINDOW,       // 动态窗口法
  POTENTIAL_FIELD,      // 人工势场法
  HYBRID               // 混合模式
};

/**
 * @brief 避障参数
 */
struct AvoidanceConfig
{
  // 避障模式
  AvoidanceMode mode;

  // 避障半径
  double avoidance_radius;

  // 绕障速度
  double avoidance_velocity;

  // 最小绕障距离
  double min_avoidance_distance;

  // 是否允许反向运动
  bool allow_reverse;

  // 最大绕障时间
  double max_avoidance_time;

  // 动态障碍物预测
  bool enable_prediction;
  double prediction_time;
  double prediction_step;

  // 代价函数权重
  double weight_obstacle_distance;
  double weight_path_alignment;
  double weight_goal_distance;
  double weight_velocity_alignment;
  double weight_acceleration;

  // 速度限制
  double max_velocity_x;
  double max_velocity_theta;

  // 安全参数
  double safe_distance;
  double emergency_stop_distance;
};

/**
 * @brief 速度采样
 */
struct VelocitySample
{
  double v_x;
  double v_theta;
  double score;

  VelocitySample() : v_x(0.0), v_theta(0.0), score(0.0) {}
  VelocitySample(double vx, double vth) : v_x(vx), v_theta(vth), score(0.0) {}
};

/**
 * @class ObstacleAvoidancePlanner
 * @brief 避障规划器
 */
class ObstacleAvoidancePlanner : public nav2_core::Controller
{
public:
  /**
   * @brief 构造函数
   */
  ObstacleAvoidancePlanner();

  /**
   * @brief 析构函数
   */
  ~ObstacleAvoidancePlanner() override = default;

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
   * @brief 计算避障速度命令
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
   * @brief 设置障碍物检测器
   */
  void setObstacleDetector(std::shared_ptr<ObstacleDetector> detector);

  /**
   * @brief 检查是否需要避障
   */
  bool needsAvoidance(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief 获取避障模式
   */
  AvoidanceMode getMode() const { return config_.mode; }

  /**
   * @brief 设置避障模式
   */
  void setMode(AvoidanceMode mode) { config_.mode = mode; }

private:
  /**
   * @brief 动态窗口法避障
   */
  geometry_msgs::msg::Twist computeDynamicWindowAvoidance(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief 人工势场法避障
   */
  geometry_msgs::msg::Twist computePotentialFieldAvoidance(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief 混合模式避障
   */
  geometry_msgs::msg::Twist computeHybridAvoidance(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  /**
   * @brief 生成速度样本
   */
  std::vector<VelocitySample> generateVelocitySamples(
    const geometry_msgs::msg::Twist & current_velocity);

  /**
   * @brief 评估速度样本
   */
  double evaluateVelocitySample(
    const VelocitySample & sample,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::vector<Obstacle> & obstacles);

  /**
   * @brief 计算障碍物代价
   */
  double computeObstacleCost(
    const VelocitySample & sample,
    const geometry_msgs::msg::PoseStamped & pose,
    const std::vector<Obstacle> & obstacles);

  /**
   * @brief 计算路径对齐代价
   */
  double computePathAlignmentCost(
    const VelocitySample & sample,
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 计算目标距离代价
   */
  double computeGoalDistanceCost(
    const VelocitySample & sample,
    const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief 预测障碍物位置
   */
  std::vector<Obstacle> predictObstacles(
    const std::vector<Obstacle> & obstacles,
    double time);

  /**
   * @brief 计算斥力场（人工势场法）
   */
  geometry_msgs::msg::Point computeRepulsiveForce(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::vector<Obstacle> & obstacles);

  /**
   * @brief 计算引力场（人工势场法）
   */
  geometry_msgs::msg::Point computeAttractiveForce(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief 选择最优速度样本
   */
  VelocitySample selectBestVelocitySample(
    const std::vector<VelocitySample> & samples);

  /**
   * @brief 检查是否紧急停止
   */
  bool checkEmergencyStop(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity);

  // 成员变量
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string planner_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<ObstacleDetector> obstacle_detector_;

  AvoidanceConfig config_;
  bool configured_;
  bool active_;
  double speed_limit_;
  bool speed_limit_percentage_;

  // 当前路径
  nav_msgs::msg::Path current_path_;

  // 时间记录
  rclcpp::Time last_time_;
  double avoidance_start_time_;
  bool in_avoidance_mode_;

  // 日志
  rclcpp::Logger logger_{rclcpp::get_logger("obstacle_avoidance_planner")};
};

} // namespace obstacle_avoidance
} // namespace diffbot_navigation

#endif // DIFFBOT_NAVIGATION__OBSTACLE_AVOIDANCE__OBSTACLE_AVOIDANCE_PLANNER_HPP_