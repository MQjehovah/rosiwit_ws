#ifndef ROSIWIT_NAVIGATION__CORE__I_CONTROLLER_HPP_
#define ROSIWIT_NAVIGATION__CORE__I_CONTROLLER_HPP_

#include <string>
#include <memory>
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

class IControllerStrategy
{
public:
  virtual ~IControllerStrategy() = default;

  virtual bool initialize(const ControllerConfig & config) = 0;
  virtual void setPath(const Path & path) = 0;
  virtual VelocityCommand computeVelocityCommand(
      const Pose2D & current_pose,
      const VelocityCommand & current_velocity) = 0;
  virtual bool isGoalReached(const Pose2D & current_pose) const = 0;
  virtual size_t getCurrentWaypointIndex() const = 0;
  virtual double getProgress() const = 0;
  virtual void reset() = 0;
  virtual void setVelocityLimit(double linear_x, double angular_z) = 0;
  virtual std::string getName() const = 0;
  virtual std::string getVersion() const = 0;
  virtual void setObstacles(const ObstacleArray & obstacles) = 0;
  virtual void setLocalCostmap(const CostmapGrid & /*costmap*/) {}
  // 控制器是否真的实现了 footprint 代价检查. 只有支持的控制器才能接
  // 未膨胀的局部 costmap, 否则会丢掉圆形膨胀的安全余量.
  virtual bool supportsFootprint() const { return false; }
  // ---- 能力声明: 规划器据此决定生成什么样的路径 ----
  // 是否支持含"前进/倒车切换(cusp)"的多方向路径. false(如 MPPI) 时规划器
  // 必须只生成单向路径 + 原地旋转, 否则采样控制器会在 cusp 处两族样本互相
  // 抵消 (v≈0) 卡死.
  virtual bool supportsMultiDirection() const { return false; }
  // 是否能自己执行原地旋转段 (velocity_x=0). false 时由上层执行器代做.
  virtual bool supportsInPlaceRotation() const { return false; }
  virtual std::string getDiagnostics() const { return {}; }
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CORE__I_CONTROLLER_HPP_
