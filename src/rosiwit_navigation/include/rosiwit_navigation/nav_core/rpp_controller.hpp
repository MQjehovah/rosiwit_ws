// Regulated Pure Pursuit (RPP) —— 差速轮路径跟踪器 (弧长参数化 + 段方向).
// 与 MPPI 的区别: 解析反馈律, 方向来自路径 (锁存), cusp 是显式状态,
// 天然支持前进/倒车/原地旋转, 不存在采样器在 cusp 处两族样本抵消的问题.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/i_controller.hpp"

namespace rosiwit_navigation
{
namespace controllers
{

class RppController : public core::IControllerStrategy
{
public:
  bool initialize(const core::ControllerConfig & config) override;
  void setPath(const core::Path & path) override;
  core::VelocityCommand computeVelocityCommand(
      const core::Pose2D & current_pose,
      const core::VelocityCommand & current_velocity) override;
  bool isGoalReached(const core::Pose2D & current_pose) const override;
  size_t getCurrentWaypointIndex() const override { return idx_; }
  double getProgress() const override;
  void reset() override;
  void setVelocityLimit(double linear_x, double angular_z) override;
  std::string getName() const override { return "rpp"; }
  std::string getVersion() const override { return "1.0"; }
  void setObstacles(const core::ObstacleArray & /*obstacles*/) override {}
  void setLocalCostmap(const core::CostmapGrid & grid) override { costmap_ = grid; }
  bool supportsMultiDirection() const override { return true; }
  bool supportsInPlaceRotation() const override { return true; }

private:
  void rebuild();
  void updateProgress(const core::Pose2D & pose);
  size_t lookaheadIndex(double s) const;
  int dirAt(size_t i) const;

  core::ControllerConfig cfg_;
  core::Path path_;
  std::vector<double> s_;      // 累计弧长
  core::CostmapGrid costmap_;
  size_t idx_ = 0;
  double s_cur_ = 0.0;
  double max_vx_ = 0.4;
  double max_vw_ = 0.5;
  double max_ax_ = 0.5;
  double max_aw_ = 0.2;
  double goal_tol_ = 0.2;
  double last_v_ = 0.0;
  double last_w_ = 0.0;
  int align_dir_ = 0;   // 旋转方向锁存 (±π 附近防左右摆)
};

}  // namespace controllers
}  // namespace rosiwit_navigation
