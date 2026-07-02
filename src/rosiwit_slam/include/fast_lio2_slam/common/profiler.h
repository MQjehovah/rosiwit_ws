/**
 * @file profiler.h
 * @brief FAST-LIO2 SLAM - 性能监控工具
 * @author AI Development Team
 * @date 2026-04-27
 *
 * 提供精确的性能监控和分析功能:
 * - 函数执行时间统计
 * - 周期性性能报告输出
 * - 内存使用监控
 * - 自定义性能指标
 */

#pragma once

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <fstream>
#include <memory>
#include <atomic>
#include <algorithm>

#ifdef __linux__
#include <sys/resource.h>
#else
// Windows内存获取需要其他方法
#include <windows.h>
#include <psapi.h>
#endif

namespace fast_lio2_slam {

/**
 * @brief 性能统计数据结构
 */
struct ProfileStats {
    std::string name;               // 统计项名称
    int64_t total_time_us = 0;      // 总耗时 (微秒)
    int64_t max_time_us = 0;        // 最大耗时
    int64_t min_time_us = INT64_MAX; // 最小耗时
    int64_t avg_time_us = 0;        // 平均耗时
    int call_count = 0;             // 调用次数

    // 百分位数统计
    int64_t p50_time_us = 0;        // 50分位
    int64_t p95_time_us = 0;        // 95分位
    int64_t p99_time_us = 0;        // 99分位

    void reset();

    void update(int64_t elapsed_us);
};

/**
 * @brief 性能监控配置
 */
struct ProfilerConfig {
    bool enable = true;             // 是否启用监控
    bool print_on_exit = true;      // 程序退出时打印报告
    int report_interval_ms = 5000;  // 周期性报告间隔 (毫秒)
    int max_history_size = 1000;    // 单项最大历史记录数
    std::string output_file = "";   // 输出文件路径 (可选)
};

/**
 * @brief 线程安全的性能监控类
 *
 * 使用方法:
 * 1. 全局启用: Profiler::instance().enable()
 * 2. 函数埋点: PROFILE_FUNCTION("function_name")
 * 3. 代码段埋点: PROFILE_SCOPE("code_block")
 * 4. 获取报告: Profiler::instance().getReport()
 */
class Profiler {
public:
    /**
     * @brief 获取单例实例
     */
    static Profiler& instance();

    /**
     * @brief 配置性能监控器
     */
    void configure(const ProfilerConfig& config);

    /**
     * @brief 启用性能监控
     */
    void enable() { enabled_ = true; }

    /**
     * @brief 禁用性能监控
     */
    void disable() { enabled_ = false; }

    /**
     * @brief 检查是否启用
     */
    bool isEnabled() const { return enabled_; }

    /**
     * @brief 开始计时
     * @param name 统计项名称
     * @return 计时ID
     */
    int64_t startTiming(const std::string& name);

    /**
     * @brief 结束计时并记录
     * @param name 统计项名称
     * @param start_time 开始时间点
     */
    void endTiming(const std::string& name, int64_t start_time);

    /**
     * @brief 直接记录耗时
     * @param name 统计项名称
     * @param elapsed_us 耗时 (微秒)
     */
    void recordTime(const std::string& name, int64_t elapsed_us);

    /**
     * @brief 获取统计数据
     * @param name 统计项名称
     * @return 统计数据
     */
    ProfileStats getStats(const std::string& name);

    /**
     * @brief 获取所有统计数据
     */
    std::unordered_map<std::string, ProfileStats> getAllStats();

    /**
     * @brief 重置统计数据
     */
    void reset();

    /**
     * @brief 生成性能报告字符串
     */
    std::string getReport();

    /**
     * @brief 打印性能报告到日志
     */
    void printReport();

    /**
     * @brief 保存报告到文件
     */
    void saveReport(const std::string& filename);

    /**
     * @brief 获取当前内存使用量 (MB)
     */
    double getMemoryUsageMB();

    /**
     * @brief 获取CPU使用率 (%)
     */
    double getCpuUsagePercent();

private:
    Profiler() = default;
    ~Profiler();

    // 更新百分位数统计
    void updatePercentiles(const std::string& name, int64_t elapsed_us);

    // 自动保存报告
    void autoSaveReport();

private:
    ProfilerConfig config_;
    std::atomic<bool> enabled_;

    std::unordered_map<std::string, ProfileStats> stats_;
    std::unordered_map<std::string, std::vector<int64_t>> history_;  // 用于百分位数计算

    std::mutex mutex_;
    std::mutex history_mutex_;

    std::ofstream output_file_;

    // CPU使用率计算辅助
    std::chrono::time_point<std::chrono::high_resolution_clock> last_cpu_check_;
    long last_cpu_time_;
};

/**
 * @brief 计时辅助类 (RAII风格)
 *
 * 构造时开始计时，析构时自动结束并记录
 */
class ScopedTimer {
public:
    explicit ScopedTimer(const std::string& name);
    ~ScopedTimer();

    /**
     * @brief 手动停止计时
     */
    void stop();

    /**
     * @brief 获取当前耗时 (不停止计时)
     */
    int64_t elapsed() const;

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
    bool stopped_;
};

/**
 * @brief 函数级别计时宏
 */
#define PROFILE_FUNCTION(name) \
    ScopedTimer _timer_##name(#name)

/**
 * @brief 代码块计时宏
 */
#define PROFILE_SCOPE(name) \
    ScopedTimer _timer_scope_##name(#name)

/**
 * @brief 手动计时开始宏
 */
#define PROFILE_START(name) \
    int64_t _time_##name = Profiler::instance().startTiming(#name)

/**
 * @brief 手动计时结束宏
 */
#define PROFILE_END(name) \
    Profiler::instance().endTiming(#name, _time_##name)

/**
 * @brief 性能监控初始化辅助函数
 */
inline void initProfiler(const ProfilerConfig& config = ProfilerConfig()) {
    Profiler::instance().configure(config);
    Profiler::instance().enable();
}

}  // namespace fast_lio2_slam
