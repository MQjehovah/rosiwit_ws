#ifndef ROSIWIT_NAVIGATION__CONTROLLERS__LQR_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLERS__LQR_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/i_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"

namespace rosiwit_navigation
{
namespace controllers
{

/**
 * @class LqrController
 * @brief 路径跟踪用离散 LQR 控制器 (参考 ai-winter/ros_motion_planning 的
 *        lqr_controller, 去掉 Eigen 依赖, 3x3 Riccati 手写)
 *
 * 与 MPPI/DWA 的区别: 不是"采样+软代价", 而是对路径误差状态做最优反馈,
 * 横向误差理论上收敛到 0, 并用参考曲率前馈, 弯道不落后。
 */
class LqrController : public core::IControllerStrategy
{
public:
  LqrController();
  ~LqrController() override;

  bool initialize(const core::ControllerConfig& config) override;
  void setPath(const core::Path& path) override;
  core::VelocityCommand computeVelocityCommand(
      const core::Pose2D& current_pose,
      const core::VelocityCommand& current_velocity) override;
  bool isGoalReached(const core::Pose2D& current_pose) const override;
  size_t getCurrentWaypointIndex() const override;
  double getProgress() const override;
  void reset() override;
  void setVelocityLimit(double linear_x, double angular_z) override;
  std::string getName() const override;
  std::string getVersion() const override;
  std::string getDiagnostics() const override;
  void setObstacles(const core::ObstacleArray& obstacles) override { (void)obstacles; }

private:
  size_t findNearest(const core::Pose2D& pose) const;
  size_t lookaheadIndex(size_t from, double dist) const;
  double pathCurvature(size_t i) const;
  double pathTangent(size_t i) const;
  double normAngle(double a) const;

  std::string name_;
  core::Logger logger_;
  core::ControllerConfig cfg_;
  core::Path path_;
  size_t idx_ = 0;
  double max_vx_ = 0.30;
  double max_vw_ = 0.50;
  double max_ax_ = 0.30;
  double max_aw_ = 0.40;
  double goal_tol_ = 0.10;
  double lookahead_time_ = 1.2;
  double min_lookahead_ = 0.30;
  double max_lookahead_ = 0.90;
  double control_dt_ = 0.10;
  double q_x_ = 5.0, q_y_ = 5.0, q_theta_ = 1.0;
  double r_v_ = 1.0, r_w_ = 1.0;
  double last_v_ = 0.0, last_w_ = 0.0;
  bool initialized_ = false;
  bool goal_reached_ = false;
  // diagnostics
  double last_lateral_ = 0.0, last_heading_ = 0.0, last_kappa_ = 0.0;
};

}  // namespace controllers
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CONTROLLERS__LQR_CONTROLLER_HPP_
