#include "rosiwit_common/thread.hpp"

#include <cstdio>
#include <cstring>
#include <exception>
#include <utility>

#if defined(__linux__)
#include <pthread.h>
#endif

namespace rosiwit_common
{

namespace
{
void setCurrentThreadName(const std::string & name)
{
#if defined(__linux__)
  // pthread 限制 16 字节 (含 '\0')
  char buf[16] = {0};
  std::strncpy(buf, name.c_str(), sizeof(buf) - 1);
  pthread_setname_np(pthread_self(), buf);
#else
  (void)name;
#endif
}
}  // namespace

Thread::Thread(std::string name, Body body)
: name_(std::move(name)), body_(std::move(body))
{
  const std::string thread_name = name_;
  const Body body_copy = body_;
  CancellationSource source = source_;
  thread_ = std::thread([thread_name, body_copy, source]() {
    setCurrentThreadName(thread_name);
    try {
      if (body_copy) { body_copy(source.token()); }
    } catch (const CancelledException &) {
      // 正常退出路径
    } catch (const std::exception & e) {
      std::fprintf(stderr, "[thread:%s] uncaught exception: %s\n",
                   thread_name.c_str(), e.what());
    } catch (...) {
      std::fprintf(stderr, "[thread:%s] uncaught unknown exception\n", thread_name.c_str());
    }
  });
}

Thread::~Thread()
{
  requestStop();
  join();
}

void Thread::join()
{
  if (thread_.joinable()) { thread_.join(); }
}

}  // namespace rosiwit_common
