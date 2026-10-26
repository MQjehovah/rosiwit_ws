// ============================================================================
// thread.hpp — 带生命周期的线程 (基础设施层)
//
// 只做三件事:
//   1. RAII: 析构 = requestStop + join (**永不 detach**)
//   2. 取消: 线程体拿到 CancelToken, 循环里检查
//   3. 命名: 通过 pthread_setname_np 写进 OS, htop/top 里能看到 (上限 15 字符)
//
// 为什么不用裸 std::thread: 现有代码里出现过 std::thread(...).detach() 捕获 this,
// 退出时对象已析构线程还在跑 -> 必崩。这里从类型上就不允许。
// ============================================================================
#ifndef ROSIWIT_COMMON__THREAD_HPP_
#define ROSIWIT_COMMON__THREAD_HPP_

#include <functional>
#include <string>
#include <thread>

#include "rosiwit_common/cancel.hpp"

namespace rosiwit_common
{

class Thread
{
public:
  using Body = std::function<void(CancelToken)>;

  /// body 在独立线程里执行; 返回后线程结束。body 抛异常会被捕获并打到 stderr。
  Thread(std::string name, Body body);
  ~Thread();

  Thread(const Thread &) = delete;
  Thread & operator=(const Thread &) = delete;

  /// 置取消位并唤醒 (线程体自己退出循环)。不阻塞。
  void requestStop() { source_.cancel(); }

  /// 等待线程结束 (幂等)。requestStop 之后调用; 析构里会自动调。
  void join();

  /// 放弃等待: 置取消位后 detach, 析构/join 不再阻塞。
  /// **使用条件(缺一不可)**:
  ///   1. 线程体只访问由 shared_ptr 保活的独立状态 —— 里面**不能**有
  ///      "可能先于本线程析构的 owner 的 this";
  ///   2. 调用方是在不能阻塞的线程上 (ROS 服务回调 / 关停路径)。
  /// 典型场景: 后端 worker 正卡在一次 optimize 里, 关停时不能把服务线程挂死。
  /// 默认析构仍然 join; 这个方法只是显式放弃, 不会静默发生。
  void abandon();

  bool joinable() const { return thread_.joinable(); }
  bool stopRequested() const { return source_.cancelled(); }
  const std::string & name() const { return name_; }

private:
  std::string name_;
  Body body_;
  CancellationSource source_;
  std::thread thread_;
};

}  // namespace rosiwit_common

#endif  // ROSIWIT_COMMON__THREAD_HPP_
