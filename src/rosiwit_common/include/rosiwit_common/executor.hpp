// ============================================================================
// executor.hpp — 通用执行器接口 (基础设施层)
//
// 设计原则:
//   1. 业务层只声明"这件事要异步、可取消、属于哪条 lane", 由执行器决定
//      线程数/排队/限流策略 —— 业务层不创建线程, 不锁池。
//   2. lane 是**通用概念**, 不含任何业务语义: 用 LaneSpec 描述
//      "这条 lane 几条线程 / 队列多长 / 满了怎么办"。
//      例如"写盘只能串行"= lanes{{name:"disk_write", threads:1}}。
//   3. JobClass 之类的业务分类放在使用方, 不在本层。
// ============================================================================
#ifndef ROSIWIT_COMMON__EXECUTOR_HPP_
#define ROSIWIT_COMMON__EXECUTOR_HPP_

#include <cstdint>
#include <functional>
#include <string>

#include "rosiwit_common/cancel.hpp"

namespace rosiwit_common
{

using JobId = uint64_t;
static constexpr JobId kInvalidJobId = 0;

/// 任务体: 必须周期性检查 token.cancelled()。
using JobFn = std::function<void(CancelToken)>;
/// 完成回调 (可能在线程池线程上执行): ok=false 表示被取消或抛异常。
using DoneFn = std::function<void(JobId, bool ok)>;

enum class LaneOverflow
{
  kBlock,        // 满时 submit 阻塞 (慎用: 不要在 ROS 回调里用)
  kDropOldest,   // 丢最旧排队任务 (保证新任务能进)
  kReject,       // 直接拒绝, submit 返回 kInvalidJobId
};

struct LaneSpec
{
  std::string name;
  int threads = 1;                 // 1 = 串行 lane (顺序执行所有任务)
  size_t capacity = 16;
  LaneOverflow overflow = LaneOverflow::kReject;
};

struct JobOptions
{
  std::string name;
  int priority = 0;                // 大者先执行; 同优先级 FIFO
};

struct ExecutorStats
{
  uint64_t submitted = 0;
  uint64_t started = 0;
  uint64_t completed = 0;
  uint64_t failed = 0;
  uint64_t cancelled = 0;
  uint64_t rejected = 0;
  size_t queued = 0;
};

class IExecutor
{
public:
  virtual ~IExecutor() = default;

  /// 返回 kInvalidJobId = 被拒绝 (队列满 / 已 shutdown / lane 不存在)。
  virtual JobId submit(
    const std::string & lane, JobOptions options,
    JobFn job, DoneFn done = DoneFn()) = 0;

  /// 未开始的任务: 移出队列并回调 done(false);
  /// 已开始的任务: 只置取消位 (任务体自己检查)。
  virtual bool cancel(JobId id) = 0;

  virtual ExecutorStats stats() const = 0;

  /// drain=true: 先跑完已排队的任务; false: 取消排队的任务后立刻退出。
  virtual void shutdown(bool drain = true) = 0;
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__EXECUTOR_HPP_
