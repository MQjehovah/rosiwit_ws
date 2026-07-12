#ifndef ROSIWIT_NAVIGATION__CORE__I_PLANNER_HPP_
#define ROSIWIT_NAVIGATION__CORE__I_PLANNER_HPP_

#include <string>
#include <memory>
#include <functional>
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

class IPlannerStrategy
{
public:
  virtual ~IPlannerStrategy() = default;

  virtual bool initialize(const PlannerConfig & config) = 0;
  virtual void setCostmap(const Costmap & costmap) = 0;
  virtual void setInflationRadius(double /*radius_meters*/) {}  // optional
  virtual void setRobotRadius(double /*radius*/) {}
  virtual Result<Path> plan(const Pose2D & start, const Pose2D & goal) = 0;
  virtual void planAsync(const Pose2D & start, const Pose2D & goal,
                          std::function<void(const Result<Path> &)> callback) = 0;
  virtual void cancel() = 0;
  virtual bool isPlanning() const = 0;
  virtual std::string getName() const = 0;
  virtual std::string getVersion() const = 0;
  virtual void reset() = 0;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CORE__I_PLANNER_HPP_
