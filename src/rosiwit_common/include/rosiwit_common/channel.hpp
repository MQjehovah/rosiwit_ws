// ============================================================================
// channel.hpp — 有界多生产者/多消费者队列 (基础设施层, ROS 无关)
//
// 为什么要它: 现在各模块都是 std::thread + condition_variable 手搓队列,
// 没有上限、没有丢弃策略、没有关停语义 (slam_pipeline 还 detach 了线程)。
// 这里只做一件事: 把"有界队列 + 明确的满时策略 + close 唤醒 + 计数"封装好,
// 让上层(前端/后端/任务池)**不用再写 mutex/cv**。
//
// 三种满时策略:
//   kBlock       阻塞 push (调用方自己保证不在热路径阻塞: 例如后台线程)
//   kDropOldest  丢最旧、保新鲜  <- 传感器帧默认 (过期帧对定位/建图没用)
//   kDropNewest  丢新到、保连续  <- 需要顺序完整性时用
//
// close() 之后: push 一律失败并计数, pop 立即返回 nullopt (唤醒所有等待者)。
// ============================================================================
#ifndef ROSIWIT_COMMON__CHANNEL_HPP_
#define ROSIWIT_COMMON__CHANNEL_HPP_

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace rosiwit_common
{

enum class OverflowPolicy
{
  kBlock,        // 满时阻塞 (只有后台生产者可以用)
  kDropOldest,   // 丢最旧 (保新)
  kDropNewest,   // 丢新到 (保序)
};

struct ChannelStats
{
  uint64_t pushed = 0;
  uint64_t popped = 0;
  uint64_t dropped_oldest = 0;
  uint64_t dropped_newest = 0;
  uint64_t cleared = 0;          // 由 clear() 显式丢弃的数量 (与"满时丢弃"分开计数)
  uint64_t rejected_closed = 0;
  size_t peak = 0;
};

template<typename T>
class Channel
{
public:
  explicit Channel(size_t capacity, OverflowPolicy policy = OverflowPolicy::kDropOldest)
  : capacity_(std::max<size_t>(1, capacity)), policy_(policy) {}

  Channel(const Channel &) = delete;
  Channel & operator=(const Channel &) = delete;

  /// 返回 false = 这条没进队列 (kDropNewest 或已 close)。其余情况按策略处理。
  bool push(T value)
  {
    std::unique_lock<std::mutex> lk(mu_);
    if (closed_) {
      ++stats_.rejected_closed;
      return false;
    }
    if (q_.size() >= capacity_) {
      switch (policy_) {
        case OverflowPolicy::kBlock:
          not_full_.wait(lk, [&] { return closed_ || q_.size() < capacity_; });
          if (closed_) {
            ++stats_.rejected_closed;
            return false;
          }
          break;
        case OverflowPolicy::kDropOldest:
          q_.pop_front();
          ++stats_.dropped_oldest;
          break;
        case OverflowPolicy::kDropNewest:
          ++stats_.dropped_newest;
          return false;
      }
    }
    q_.push_back(std::move(value));
    ++stats_.pushed;
    stats_.peak = std::max(stats_.peak, q_.size());
    not_empty_.notify_one();
    return true;
  }

  /// 阻塞直到有数据或 close。close 且已排空时返回 nullopt。
  std::optional<T> pop()
  {
    std::unique_lock<std::mutex> lk(mu_);
    not_empty_.wait(lk, [&] { return closed_ || !q_.empty(); });
    return takeLocked();
  }

  /// 带超时。超时/已关闭返回 nullopt。
  template<typename Rep, typename Period>
  std::optional<T> pop_for(const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lk(mu_);
    if (!not_empty_.wait_for(lk, timeout, [&] { return closed_ || !q_.empty(); })) {
      return std::nullopt;
    }
    return takeLocked();
  }

  std::optional<T> try_pop()
  {
    std::lock_guard<std::mutex> lk(mu_);
    return takeLocked();
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mu_);
    closed_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  bool closed() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return closed_;
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return q_.size();
  }

  size_t capacity() const { return capacity_; }

  /// 丢弃队列里所有元素 (返回丢弃数量)。不改变 closed 状态。
  /// 用途: 模式切换/重建时把"上一轮残留的帧"清掉, 免得新实例处理旧数据。
  size_t clear()
  {
    std::lock_guard<std::mutex> lk(mu_);
    const size_t n = q_.size();
    q_.clear();
    stats_.cleared += n;
    not_full_.notify_all();
    return n;
  }

  ChannelStats stats() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
  }

private:
  std::optional<T> takeLocked()
  {
    if (q_.empty()) { return std::nullopt; }   // 只可能是 closed 且空
    T value = std::move(q_.front());
    q_.pop_front();
    ++stats_.popped;
    not_full_.notify_one();
    return value;
  }

  mutable std::mutex mu_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<T> q_;
  const size_t capacity_;
  const OverflowPolicy policy_;
  bool closed_ = false;
  ChannelStats stats_;
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__CHANNEL_HPP_
