// ============================================================================
// crash_handler.cpp — 见 crash_handler.h
// ============================================================================
#include "crash_handler.h"

#include <csignal>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>

namespace rosiwit_navigation_ros1 {
namespace {

void writeStderr(const char * s)
{
  if (!s) return;
  const size_t len = std::strlen(s);
  if (len == 0) return;
  const ssize_t ignored = ::write(STDERR_FILENO, s, len);
  (void)ignored;
}

void crashHandler(int sig)
{
  // 只做异步信号安全的事: write + backtrace_symbols_fd (内部不 malloc)
  char head[160];
  std::snprintf(head, sizeof(head),
                "\n[nav] FATAL signal %d (%s), backtrace follows:\n",
                sig, ::strsignal(sig));
  writeStderr(head);

  void * frames[64];
  const int n = ::backtrace(frames, 64);
  if (n > 0) { ::backtrace_symbols_fd(frames, n, STDERR_FILENO); }
  writeStderr("[nav] end of backtrace\n");

  // 再落一份固定文件: roslaunch 下节点 stdout/stderr 的去向不可靠
  // (output=log/screen 都可能把 stderr 吞掉), 现场也只能靠这个文件回溯。
  const char * path = "/tmp/rosiwit_nav_crash.txt";
  const int fd = ::open(path, O_CREAT | O_WRONLY | O_APPEND, 0644);
  if (fd >= 0) {
    const ssize_t w1 = ::write(fd, head, std::strlen(head));
    (void)w1;
    if (n > 0) { ::backtrace_symbols_fd(frames, n, fd); }
    const ssize_t w2 = ::write(fd, "[nav] end of backtrace\n", 23);
    (void)w2;
    ::close(fd);
  }

  // 恢复默认行为并重新触发一次, 让 core dump / 退出码保持原样
  ::signal(sig, SIG_DFL);
  ::raise(sig);
}

}  // namespace

void installCrashHandler()
{
  for (int sig : {SIGSEGV, SIGABRT, SIGBUS, SIGFPE}) {
    ::signal(sig, crashHandler);
  }
}

}  // namespace rosiwit_navigation_ros1
