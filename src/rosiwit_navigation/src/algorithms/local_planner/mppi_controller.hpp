#ifndef ROSIWIT_NAVIGATION__CONTROLLERS__MPPI_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLERS__MPPI_CONTROLLER_HPP_

#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/i_controller.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"
#include "rosiwit_navigation/nav_core/footprint.hpp"

namespace rosiwit_navigation {
namespace controllers {

/// 轻量 MPPI 采样控制器 (核心库版本, 不依赖 Nav2).
/// 用局部 costmap + 路径跟踪代价采样控制序列, 输出加权平均的第一个控制量.
class MppiController : public core::IControllerStrategy {
public:
  MppiController();
  ~MppiController() override = default;

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
  void setObstacles(const core::ObstacleArray& obstacles) override;
  void setLocalCostmap(const core::CostmapGrid& costmap) override;
  bool supportsFootprint() const override { return true; }
  std::string getDiagnostics() const override;

private:
  struct Control { double v{0.0}; double w{0.0}; };

  size_t nearestPathIndex(const core::Pose2D& pose) const;
  size_t nearestPathIndex(const core::Pose2D& pose, size_t begin, size_t end) const;
  static double normAngle(double a);
  static core::Pose2D integrate(
    const core::Pose2D& pose, double v, double w, double dt);
  void resetNominal();

  std::string name_;
  core::Logger logger_;
  core::ControllerConfig cfg_;
  core::Path path_;
  core::CostmapGrid local_costmap_;
  core::ObstacleArray obstacles_;
  core::Footprint footprint_;
  std::vector<std::pair<double, double>> footprint_samples_;
  bool has_local_costmap_{false};
  bool initialized_{false};
  bool reverse_mode_{false};
  bool allow_reverse_{true};
  size_t idx_{0};

  double max_vx_{0.5};
  double max_vw_{1.0};
  double max_ax_{0.4};
  double max_aw_{0.8};
  double goal_tol_{0.2};

  int horizon_{20};
  int samples_{160};
  double dt_{0.1};
  double lambda_{0.8};
  double sigma_v_{0.18};
  double sigma_w_{0.35};
  double path_weight_{2.0};
  double heading_weight_{0.4};
  double obstacle_weight_{0.03};
  double control_weight_{0.03};
  double speed_weight_{2.0};
  double goal_weight_{1.5};

  std::vector<Control> nominal_;
  std::mt19937 rng_{5489u};
  double last_min_cost_{0.0};
  double last_mean_cost_{0.0};
  double last_collision_ratio_{0.0};
  double last_best_v_{0.0};
  double last_best_w_{0.0};
  double last_output_v_{0.0};
  double last_output_w_{0.0};
  double last_goal_error_{0.0};
  size_t last_nearest_idx_{0};
};

}}  // namespace rosiwit_navigation::controllers

#endif  // ROSIWIT_NAVIGATION__CONTROLLERS__MPPI_CONTROLLER_HPP_
