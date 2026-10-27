#include "rosiwit_common/periodic.hpp"

#include <algorithm>
#include <cstdio>
#include <exception>
#include <utility>

namespace rosiwit_common
{

PeriodicThread::PeriodicThread(std::string name, std::chrono::microseconds period, Tick tick)
: period_(std::max<std::chrono::microseconds>(period, std::chrono::microseconds(1)))
{
  auto * ticks = &ticks_;
  auto * overruns = &overruns_;
  const auto period_copy = period_;
  const std::string log_name = name;   // name 会被 move 进 Thread, 日志要用副本
  thread_.reset(new Thread(std::move(name),
    [period_copy, tick = std::move(tick), ticks, overruns, log_name](CancelToken token) {
      auto next = std::chrono::steady_clock::now() + period_copy;
      while (!token.cancelled()) {
        // tick 抛异常只记一条: 否则 50Hz 发布线程会静默永久停摆 (只剩一行 stderr)
        if (tick) {
          try { tick(); }
          catch (const std::exception & e) {
            std::fprintf(stderr, "[periodic:%s] tick threw: %s\n",
                         log_name.c_str(), e.what());
          } catch (...) {
            std::fprintf(stderr, "[periodic:%s] tick threw (unknown)\n",
                         log_name.c_str());
          }
        }
        ticks->fetch_add(1, std::memory_order_relaxed);
        const auto now = std::chrono::steady_clock::now();
        if (now > next + period_copy) {
          // 落后超过一个周期: 重新对齐, 避免追帧雪崩
          overruns->fetch_add(1, std::memory_order_relaxed);
          next = now + period_copy;
        } else {
          next += period_copy;
        }
        // 分片睡眠, 保证 requestStop 及时生效
        while (!token.cancelled() && std::chrono::steady_clock::now() < next) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }
    }));
}

PeriodicThread::~PeriodicThread()
{
  requestStop();
  join();
}

void PeriodicThread::requestStop()
{
  if (thread_) { thread_->requestStop(); }
}

void PeriodicThread::join()
{
  if (thread_) { thread_->join(); }
}

}  // namespace rosiwit_common
