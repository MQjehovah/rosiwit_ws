// ============================================================================
// control_loop.h — 管理命令线程
//
// 载图 / 切模式 / 存图 / 扩图 这些是"同一时刻只能有一个"的管理操作:
//   * 它们串行是**业务属性**, 不是执行资源属性 -> 用一条线程 + 一个队列表达
//   * 服务线程只负责"投命令 + 有界等待", 不会因为队列忙而卡死
//   * 长耗时的部分仍在这条线程上同步做完 (它不承担任何延迟敏感的工作)
//
// 与线程池的分工:
//   控制线程: 决定 + 应用 (ms~s)
//   线程池  : 业务相关性小/可丢帧的活 (点云转换、栅格渲染、2D 匹配、标定)
// ============================================================================
#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "rosiwit_common/channel.hpp"
#include "rosiwit_common/thread.hpp"

namespace rosiwit_slam_ros1
{

class ControlLoop
{
public:
  void start()
  {
    if (thread_) return;
    thread_.reset(new rosiwit_common::Thread(
      "slam_control", [this](rosiwit_common::CancelToken) { run(); }));
  }

  void stop()
  {
    if (!thread_) return;
    queue_.close();
    thread_->requestStop();
    thread_->join();
    thread_.reset();
  }

  /// 投递命令 (不等待)。队列满时阻塞投递方 —— 管理命令不能丢。
  bool post(std::function<void()> fn)
  {
    return queue_.push(std::move(fn));
  }

  /// 在控制线程执行并等它结束; 返回 false = 超时 (任务可能已排队/仍在跑)。
  bool postAndWait(std::function<void()> fn, int timeout_ms)
  {
    auto p = std::make_shared<std::promise<void>>();
    auto fut = p->get_future();
    if (!post([fn, p] {
          try { fn(); } catch (...) {}
          try { p->set_value(); } catch (...) {}
        })) {
      return false;
    }
    return fut.wait_for(std::chrono::milliseconds(timeout_ms)) ==
           std::future_status::ready;
  }

private:
  void run()
  {
    while (auto fn = queue_.pop()) {
      if (*fn) (*fn)();
    }
  }

  rosiwit_common::Channel<std::function<void()>> queue_{
    32, rosiwit_common::OverflowPolicy::kBlock};
  std::unique_ptr<rosiwit_common::Thread> thread_;
};

}  // namespace rosiwit_slam_ros1
