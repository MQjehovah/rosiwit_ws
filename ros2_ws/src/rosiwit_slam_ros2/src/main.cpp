// src/main.cpp
#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "ros_interface/slam_node.h"

namespace {

// 崩溃时打印 C 栈回溯: 现场没有 gdb 时, 这是唯一能定位 SIGSEGV/SIGFPE 的手段.
void installCrashHandler()
{
    static auto handler = [](int sig) {
        void * frames[40];
        const int n = ::backtrace(frames, 40);
        char buf[128];
        const int len = std::snprintf(buf, sizeof(buf), "\n[rosiwit_slam] signal %d, backtrace:\n", sig);
        if (len > 0) {::write(STDERR_FILENO, buf, static_cast<size_t>(len));}
        ::backtrace_symbols_fd(frames, n, STDERR_FILENO);
        ::signal(sig, SIG_DFL);
        ::raise(sig);
    };
    ::signal(SIGSEGV, handler);
    ::signal(SIGFPE, handler);
    ::signal(SIGABRT, handler);
    ::signal(SIGBUS, handler);
    ::signal(SIGILL, handler);
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 必须在 rclcpp::init 之后: init 会自己装 SIGSEGV/SIGFPE 等信号处理, 先装的会被覆盖.
    installCrashHandler();
    auto node = std::make_shared<rosiwit_slam::SlamNode>();
    bool use_mt = true;
    node->declare_parameter("use_multi_threaded", true);
    node->get_parameter("use_multi_threaded", use_mt);
    if (use_mt) {
        rclcpp::executors::MultiThreadedExecutor exec;
        exec.add_node(node);
        exec.spin();
    } else {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
