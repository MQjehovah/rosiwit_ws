// ============================================================================
// cancel.hpp — 协作式取消标记 (基础设施层)
//
// 约定: 所有长任务/线程函数签名都带 CancelToken, 在循环里检查 (或调用
// throw_if_cancelled())。cancel() 只置位, 不会中断线程——C++ 里没有安全的
// 强制取消, 强杀线程会留下未释放的锁。
// ============================================================================
#ifndef ROSIWIT_COMMON__CANCEL_HPP_
#define ROSIWIT_COMMON__CANCEL_HPP_

#include <atomic>
#include <exception>
#include <memory>
#include <utility>

namespace rosiwit_common
{

class CancelledException : public std::exception
{
public:
  const char * what() const noexcept override { return "cancelled"; }
};

class CancelToken
{
public:
  CancelToken() = default;

  bool cancelled() const
  {
    return state_ && state_->load(std::memory_order_acquire);
  }

  void throw_if_cancelled() const
  {
    if (cancelled()) { throw CancelledException(); }
  }

private:
  friend class CancellationSource;
  explicit CancelToken(std::shared_ptr<std::atomic<bool>> state)
  : state_(std::move(state)) {}

  std::shared_ptr<std::atomic<bool>> state_;
};

class CancellationSource
{
public:
  CancellationSource()
  : state_(std::make_shared<std::atomic<bool>>(false)) {}

  CancelToken token() const { return CancelToken(state_); }

  void cancel() { state_->store(true, std::memory_order_release); }

  bool cancelled() const { return state_->load(std::memory_order_acquire); }

private:
  std::shared_ptr<std::atomic<bool>> state_;
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__CANCEL_HPP_
