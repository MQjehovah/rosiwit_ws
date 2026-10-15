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
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CORE__I_CONTROLLER_HPP_
