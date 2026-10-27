// ============================================================================
// crash_handler.h — 段错误/abort 时打印带符号的调用栈
//
// 为什么需要: nav 节点在机器人上是"出问题就没日志"的典型 —— 进程被信号杀掉,
// roslaunch 里只剩一行 "process has died, exit code -11", 没有栈就无法定位。
// 这里注册 SIGSEGV/SIGABRT/SIGBUS/SIGFPE 的处理函数, 直接向 stderr 写 backtrace,
// 然后恢复默认处理并重新抛给自己 (保留 core dump 行为)。
//
// 前提: 可执行文件带 -rdynamic (CMakeLists 里已加), 否则只能打出地址。
// ============================================================================
#pragma once

namespace rosiwit_navigation_ros1 {

/// 安装崩溃处理器 (进程级, 只需在 main 里调一次)
void installCrashHandler();

}  // namespace rosiwit_navigation_ros1
