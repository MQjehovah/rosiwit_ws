// Regulated Pure Pursuit (RPP) —— 差速轮路径跟踪器 (弧长参数化 + 段方向).
// 与 MPPI 的区别: 解析反馈律, 方向来自路径 (锁存), cusp 是显式状态,
// 天然支持前进/倒车/原地旋转, 不存在采样器在 cusp 处两族样本抵消的问题.
#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
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
  // footprint-aware: 声明支持后节点会注入**未膨胀**的障碍层 (raw),
  // 由这里的扫掠检查负责车体形状 (以前 rpp 完全不用 costmap, 拐弯会切内角)。
  void setLocalCostmap(const core::CostmapGrid & grid) override;
  bool supportsFootprint() const override { return true; }
  void setTrajectory(const core::TimedTrajectory & traj) override { traj_ = traj; }
  bool supportsMultiDirection() const override { return true; }
  bool supportsInPlaceRotation() const override { return true; }

private:
  void rebuild();
  void updateProgress(const core::Pose2D & pose);
  size_t lookaheadIndex(double s) const;
  int dirAt(size_t i) const;

  /// 沿指令弧的 footprint 扫掠检查。返回 [0,1]: 弧上能安全走完的比例
  /// (1 = 整段无碰撞, 0 = 起步就撞)。只用未膨胀障碍层判定。
  double arcClearanceScale(const core::Pose2D & pose, double v, double w) const;

  core::ControllerConfig cfg_;
  core::Path path_;
  std::vector<double> s_;      // 累计弧长
  // 局部代价地图跨线程更新 (publishLocalCostmap 定时器写, 控制定时器读):
  // 用 shared_ptr + 原子换指针, 读侧只拷指针, 不拷贝 100KB 栅格。
  mutable std::mutex map_mu_;
  std::shared_ptr<const core::CostmapGrid> costmap_ptr_;
  std::vector<std::pair<double, double>> fp_samples_;   // footprint 采样点 (车体系)
  size_t idx_ = 0;
  double s_cur_ = 0.0;
  double max_vx_ = 0.4;
  double max_vw_ = 0.5;
  double max_ax_ = 0.5;
  double max_aw_ = 0.2;
  double goal_tol_ = 0.2;
  double last_v_ = 0.0;
  double last_w_ = 0.0;
  mutable int clr_probe_ = 0;   // 诊断日志节流
  bool map_logged_ = false;     // 首次收到 raw 障碍层时打一条 (确认链路接上)
  int align_dir_ = 0;   // 旋转方向锁存 (±π 附近防左右摆)
  core::TimedTrajectory traj_;   // 可选: 时间参数化轨迹 (v 前馈/末段减速)
};

}  // namespace controllers
}  // namespace rosiwit_navigation
