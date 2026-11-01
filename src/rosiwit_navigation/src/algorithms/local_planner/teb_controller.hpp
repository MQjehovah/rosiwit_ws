#ifndef ROSIWIT_NAVIGATION__CONTROLLERS__TEB_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CONTROLLERS__TEB_CONTROLLER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/i_controller.hpp"
#include "rosiwit_navigation/nav_core/logger.hpp"
#include "rosiwit_navigation/nav_core/obstacle_distance.hpp"
#include "rosiwit_navigation/nav_core/teb_types.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace controllers
{

/// 定时弹性带 (Timed Elastic Band)。
///
/// 与 teb_local_planner 的对应关系:
///   * 顶点 = (x, y, theta, dt); 边见 core/teb_types.hpp
///   * 稀疏图用块三对角 Gauss-Newton + LM 求解 (core/banded_solver.hpp)
///   * 同伦多候选用"全局路径左右偏移的初值"近似 (不是真正的 homotopy 判定)
///   * dt 不参与 GN 求解, 按"该段长度 / 允许速度"闭式更新 (含加速度限幅),
///     等价于 TEB 的 EdgeTimeOptimal 在固定拓扑下的解
///
/// 与现有 node 的契约:
///   * setPath() 只存路径并预热 (不跑优化) —— 规划/控制同拍
///   * computeVelocityCommand() 每拍做 max_iterations 次 GN, warm start
///   * getCurrentWaypointIndex() 返回**路径**索引 (node 用它索引 current_path_.poses)
class TebController : public core::IControllerStrategy
{
public:
  TebController();
  ~TebController() override;

  bool initialize(const core::ControllerConfig & config) override;
  void setPath(const core::Path & path) override;
  core::VelocityCommand computeVelocityCommand(
      const core::Pose2D & current_pose,
      const core::VelocityCommand & current_velocity) override;
  bool isGoalReached(const core::Pose2D & current_pose) const override;
  size_t getCurrentWaypointIndex() const override;
  double getProgress() const override;
  void reset() override;
  void setVelocityLimit(double linear_x, double angular_z) override;
  std::string getName() const override;
  std::string getVersion() const override;
  void setObstacles(const core::ObstacleArray & obstacles) override;
  void setLocalCostmap(const core::CostmapGrid & costmap) override;
  bool supportsFootprint() const override { return true; }
  bool supportsMultiDirection() const override { return true; }
  std::string getDiagnostics() const override;

private:
  using Band = std::vector<core::TebVertex>;

  void rebuildCandidates(const core::Pose2D & pose);
  Band makeBand(const core::Pose2D & pose, double lateral_offset) const;
  void optimizeBand(Band & band, int iterations) const;
  double bandCost(const Band & band) const;
  bool bandFeasible(const Band & band) const;
  double obstacleDistanceAt(const core::TebVertex & v, double * dir_x, double * dir_y) const;
  size_t updatePathIndex(const core::Pose2D & pose);
  double segmentSpeed(const Band & band, size_t i) const;

  std::string name_;
  core::Logger logger_;
  core::ControllerConfig cfg_;
  core::TebWeights weights_;
  core::Path path_;
  core::ObstacleArray obstacles_;
  core::ObstacleDistance obstacle_distance_;
  std::vector<std::pair<double, double>> footprint_samples_;

  std::vector<Band> candidates_;
  core::Pose2D last_pose_;
  size_t path_idx_ = 0;
  int best_candidate_ = 0;

  double max_vx_ = 0.5;
  double max_vw_ = 1.0;
  double max_ax_ = 0.8;
  double goal_tol_ = 0.1;
  int homotopy_classes_ = 3;
  double homotopy_offset_ = 0.35;
  int max_iterations_ = 3;
  size_t max_samples_ = 80;
  double dt_ref_ = 0.3;
  double prune_distance_ = 0.3;
  double last_v_ = 0.0;   // 单拍加速度限幅用
  double last_w_ = 0.0;

  bool initialized_ = false;
  bool goal_reached_ = false;
  mutable std::string diagnostics_;
};

}  // namespace controllers
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CONTROLLERS__TEB_CONTROLLER_HPP_
