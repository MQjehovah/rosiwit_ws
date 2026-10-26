// ============================================================================
// periodic.hpp — 固定周期任务线程 (基础设施层)
//
// 与 "while(true){ task(); sleep(period); }" 的区别: 用 sleep_until(next += period)
// 做**无漂移**调度, 单次超时不会累积。适合替代内部用的 ros::Timer
// (让业务线程不依赖 ROS)。
// ============================================================================
#ifndef ROSIWIT_COMMON__PERIODIC_HPP_
#define ROSIWIT_COMMON__PERIODIC_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rosiwit_common/thread.hpp"

namespace rosiwit_common
{

class PeriodicThread
{
public:
  using Tick = std::function<void()>;

  PeriodicThread(std::string name, std::chrono::microseconds period, Tick tick);
  ~PeriodicThread();

  PeriodicThread(const PeriodicThread &) = delete;
  PeriodicThread & operator=(const PeriodicThread &) = delete;

  void requestStop();
  void join();

  uint64_t ticks() const { return ticks_.load(std::memory_order_relaxed); }
  uint64_t overruns() const { return overruns_.load(std::memory_order_relaxed); }

private:
  std::unique_ptr<Thread> thread_;
  std::chrono::microseconds period_;
  std::atomic<uint64_t> ticks_{0};
  std::atomic<uint64_t> overruns_{0};
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__PERIODIC_HPP_
