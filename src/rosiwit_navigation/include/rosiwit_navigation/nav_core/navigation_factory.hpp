#ifndef ROSIWIT_NAVIGATION__NAV_CORE__NAVIGATION_FACTORY_HPP_
#define ROSIWIT_NAVIGATION__NAV_CORE__NAVIGATION_FACTORY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/i_planner.hpp"
#include "rosiwit_navigation/nav_core/i_controller.hpp"

namespace rosiwit_navigation {
namespace nav_core {

class NavigationFactory
{
public:
  static std::unique_ptr<rosiwit_navigation::core::IPlannerStrategy> createPlanner(
      const std::string & name);
  static std::unique_ptr<rosiwit_navigation::core::IControllerStrategy> createController(
      const std::string & name);
  static std::vector<std::string> listPlanners();
  static std::vector<std::string> listControllers();
};

} // namespace nav_core
} // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__NAV_CORE__NAVIGATION_FACTORY_HPP_
