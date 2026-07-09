#include "rosiwit_navigation/nav_core/navigation_factory.hpp"

#include "rosiwit_navigation/algorithms/astar_planner.hpp"
#include "rosiwit_navigation/algorithms/navfn_planner.hpp"
#include "rosiwit_navigation/algorithms/pure_pursuit_controller.hpp"

namespace rosiwit_navigation {
namespace nav_core {

std::unique_ptr<rosiwit_navigation::core::IPlannerStrategy>
NavigationFactory::createPlanner(const std::string & name)
{
  if (name == "astar" || name == "a_star") {
    return std::make_unique<rosiwit_navigation::planners::AStarPlanner>();
  }
  if (name == "navfn") {
    return std::make_unique<rosiwit_navigation::planners::NavFnPlanner>();
  }
  return nullptr;
}

std::unique_ptr<rosiwit_navigation::core::IControllerStrategy>
NavigationFactory::createController(const std::string & name)
{
  if (name == "pure_pursuit") {
    return std::make_unique<rosiwit_navigation::controllers::PurePursuitController>();
  }
  return nullptr;
}

std::vector<std::string> NavigationFactory::listPlanners()
{
  return {"astar", "navfn"};
}

std::vector<std::string> NavigationFactory::listControllers()
{
  return {"pure_pursuit"};
}

} // namespace nav_core
} // namespace rosiwit_navigation
