#ifndef ROSIWIT_NAVIGATION__CORE__EXCEPTIONS_HPP_
#define ROSIWIT_NAVIGATION__CORE__EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include "rosiwit_navigation/nav_core/types.hpp"

namespace rosiwit_navigation
{
namespace core
{

class NavigationException : public std::runtime_error
{
public:
  explicit NavigationException(const std::string & message)
    : std::runtime_error(message), error_code_(ErrorCode::UNKNOWN) {}

  NavigationException(ErrorCode code, const std::string & message)
    : std::runtime_error(message), error_code_(code) {}

  ErrorCode code() const { return error_code_; }

private:
  ErrorCode error_code_;
};

}  // namespace core
}  // namespace rosiwit_navigation

#endif  // ROSIWIT_NAVIGATION__CORE__EXCEPTIONS_HPP_
