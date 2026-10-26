// ============================================================================
// snapshot.hpp — 不可变快照槽 (基础设施层)
//
// 用途: 生产者 (前端/后端) 产出不可变对象, 消费者 (ROS 发布线程) 无锁读取。
// 语义: publish 换指针, load 拿到某一份完整快照; version 单调递增, 消费端可
// 据此判断"这帧是不是新的", 不用再靠时间戳去重。
// 注意: 只能发布 shared_ptr<const T>, 发布后不得修改 (否则就是数据竞争)。
// ============================================================================
#ifndef ROSIWIT_COMMON__SNAPSHOT_HPP_
#define ROSIWIT_COMMON__SNAPSHOT_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>

namespace rosiwit_common
{

template<typename T>
class SnapshotSlot
{
public:
  void publish(std::shared_ptr<const T> value)
  {
    std::atomic_store_explicit(&slot_, std::move(value), std::memory_order_release);
    version_.fetch_add(1, std::memory_order_acq_rel);
  }

  std::shared_ptr<const T> load() const
  {
    return std::atomic_load_explicit(&slot_, std::memory_order_acquire);
  }

  bool has_value() const { return load() != nullptr; }

  uint64_t version() const { return version_.load(std::memory_order_acquire); }

private:
  std::shared_ptr<const T> slot_;
  std::atomic<uint64_t> version_{0};
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__SNAPSHOT_HPP_
