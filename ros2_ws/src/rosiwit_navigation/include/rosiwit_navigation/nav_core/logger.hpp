#ifndef ROSIWIT_NAVIGATION__CORE__LOGGER_HPP_
#define ROSIWIT_NAVIGATION__CORE__LOGGER_HPP_

#include <iostream>
#include <string>
#include <sstream>

namespace rosiwit_navigation
{
namespace core
{

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger
{
public:
  explicit Logger(const std::string & name) : name_(name) {}

  class LogStream
  {
  public:
    LogStream(LogLevel level, const std::string & name) : level_(level), name_(name) {}
    ~LogStream() {
      static const char * labels[] = {"DEBUG", "INFO", "WARN", "ERROR"};
      std::cerr << "[" << labels[static_cast<int>(level_)] << "] [" << name_ << "] "
                << ss_.str() << std::endl;
    }
    template<typename T>
    LogStream & operator<<(const T & val) { ss_ << val; return *this; }
    LogStream & operator<<(std::ostream & (*f)(std::ostream &)) { ss_ << f; return *this; }
  private:
    LogLevel level_;
    std::string name_;
    std::ostringstream ss_;
  };

  LogStream info() const { return LogStream(LogLevel::INFO, name_); }
  LogStream warn() const { return LogStream(LogLevel::WARN, name_); }
  LogStream error() const { return LogStream(LogLevel::ERROR, name_); }

private:
  std::string name_;
};

}  // namespace core
}  // namespace rosiwit_navigation


// Log macros (printf-style convenience)
#define LOG_INFO(logger, fmt, ...) \
    do { char buf[2048]; snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
         (logger).info() << buf; } while(0)
#define LOG_WARN(logger, fmt, ...) \
    do { char buf[2048]; snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
         (logger).warn() << buf; } while(0)
#define LOG_ERROR(logger, fmt, ...) \
    do { char buf[2048]; snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
         (logger).error() << buf; } while(0)

#endif
#ifndef ROSIWIT_NAVIGATION__CORE__LOG_MACROS_HPP_
#define ROSIWIT_NAVIGATION__CORE__LOG_MACROS_HPP_

#include "rosiwit_navigation/nav_core/logger.hpp"
#include "rosiwit_navigation/nav_core/types.hpp"
#include <cstdio>

#define LOG_INFO(logger, fmt, ...) \
    do { char buf[2048]; snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
         (logger).info() << buf; } while(0)

#define LOG_WARN(logger, fmt, ...) \
    do { char buf[2048]; snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
         (logger).warn() << buf; } while(0)

#define LOG_ERROR(logger, fmt, ...) \
    do { char buf[2048]; snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
         (logger).error() << buf; } while(0)

#endif
