#include "rosiwit_navigation/nav_core/navigation_factory.hpp"

#include "astar_planner.hpp"
#include "navfn_planner.hpp"
#include "pure_pursuit_controller.hpp"
#include "diff_drive_controller.hpp"
#include "dwa_controller.hpp"
#include "teb_controller.hpp"

namespace rosiwit_navigation {
namespace nav_core {

std::unique_ptr<core::IPlannerStrategy>
NavigationFactory::createPlanner(const std::string & name)
{
  if (name == "astar" || name == "a_star") {
    return std::make_unique<planners::AStarPlanner>();
  }
  if (name == "navfn") {
    return std::make_unique<planners::NavFnPlanner>();
  }
  return nullptr;
}

std::unique_ptr<core::IControllerStrategy>
NavigationFactory::createController(const std::string & name)
{
  if (name == "pure_pursuit") {
    return std::make_unique<controllers::PurePursuitController>();
  }
  if (name == "diff_drive") {
    return std::make_unique<controllers::DiffDriveController>();
  }
  if (name == "dwa") {
    return std::make_unique<controllers::DwaController>();
  }
  if (name == "teb") {
    return std::make_unique<controllers::TebController>();
  }
  return nullptr;
}

std::vector<std::string> NavigationFactory::listPlanners()
{
  return {"astar", "navfn"};
}

std::vector<std::string> NavigationFactory::listControllers()
{
  return {"pure_pursuit", "diff_drive", "dwa", "teb"};
}

} // namespace nav_core
} // namespace rosiwit_navigation
