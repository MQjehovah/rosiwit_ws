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

    void reset() {
        total_time_us = 0;
        max_time_us = 0;
        min_time_us = INT64_MAX;
        avg_time_us = 0;
        call_count = 0;
        p50_time_us = 0;
        p95_time_us = 0;
        p99_time_us = 0;
    }

    void update(int64_t elapsed_us) {
        total_time_us += elapsed_us;
        max_time_us = std::max(max_time_us, elapsed_us);
        min_time_us = std::min(min_time_us, elapsed_us);
        call_count++;

        if (call_count > 0) {
            avg_time_us = total_time_us / call_count;
        }
    }
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

// ==================== 实现部分 ====================

inline Profiler& Profiler::instance() {
    static Profiler profiler;
    return profiler;
}

inline Profiler::~Profiler() {
    if (config_.print_on_exit && enabled_) {
        printReport();
    }

    if (output_file_.is_open()) {
        output_file_.close();
    }
}

inline void Profiler::configure(const ProfilerConfig& config) {
    config_ = config;

    if (!config_.output_file.empty()) {
        output_file_.open(config_.output_file);
    }
}

inline int64_t Profiler::startTiming(const std::string& name) {
    if (!enabled_) return 0;

    auto start = std::chrono::high_resolution_clock::now();
    return start.time_since_epoch().count();
}

inline void Profiler::endTiming(const std::string& name, int64_t start_time) {
    if (!enabled_) return;

    auto end = std::chrono::high_resolution_clock::now();
    int64_t elapsed_us = (end.time_since_epoch().count() - start_time) / 1000;

    recordTime(name, elapsed_us);
}

inline void Profiler::recordTime(const std::string& name, int64_t elapsed_us) {
    if (!enabled_) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_[name].name = name;
        stats_[name].update(elapsed_us);
    }

    // 更百分位数历史
    updatePercentiles(name, elapsed_us);
}

inline void Profiler::updatePercentiles(const std::string& name, int64_t elapsed_us) {
    std::lock_guard<std::mutex> lock(history_mutex_);

    history_[name].push_back(elapsed_us);

    // 保持历史记录大小限制
    if (history_[name].size() > static_cast<size_t>(config_.max_history_size)) {
        history_[name].erase(history_[name].begin());
    }
}

inline ProfileStats Profiler::getStats(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stats_.find(name) != stats_.end()) {
        ProfileStats result = stats_[name];

        // 计算百分位数
        std::lock_guard<std::mutex> hist_lock(history_mutex_);
        if (history_.find(name) != history_.end() && !history_[name].empty()) {
            auto& hist = history_[name];
            std::vector<int64_t> sorted = hist;
            std::sort(sorted.begin(), sorted.end());

            size_t n = sorted.size();
            result.p50_time_us = sorted[n * 50 / 100];
            result.p95_time_us = sorted[n * 95 / 100];
            result.p99_time_us = sorted[n * 99 / 100];
        }

        return result;
    }
    return ProfileStats();
}

inline std::unordered_map<std::string, ProfileStats> Profiler::getAllStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

inline void Profiler::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::lock_guard<std::mutex> hist_lock(history_mutex_);

    for (auto& stat : stats_) {
        stat.second.reset();
    }
    history_.clear();
}

inline std::string Profiler::getReport() {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string report = "\n==================== Performance Report ====================\n";

    // 按总耗时排序
    std::vector<std::pair<std::string, ProfileStats>> sorted_stats;
    for (const auto& stat : stats_) {
        sorted_stats.push_back(stat);
    }
    std::sort(sorted_stats.begin(), sorted_stats.end(),
              [](const auto& a, const auto& b) {
                  return a.second.total_time_us > b.second.total_time_us;
              });

    // 表头
    report += "Name                        | Calls | Total(ms) | Avg(ms) | Max(ms) | Min(ms) | P50(ms) | P95(ms)\n";
    report += "----------------------------|-------|-----------|---------|---------|---------|---------|---------\n";

    // 数据行
    for (const auto& stat : sorted_stats) {
        const ProfileStats& s = stat.second;
        char line[256];
        snprintf(line, sizeof(line),
                 "%-27s | %5d | %9.2f | %7.3f | %7.3f | %7.3f | %7.3f | %7.3f\n",
                 s.name.c_str(),
                 s.call_count,
                 s.total_time_us / 1000.0,
                 s.avg_time_us / 1000.0,
                 s.max_time_us / 1000.0,
                 s.min_time_us / 1000.0,
                 s.p50_time_us / 1000.0,
                 s.p95_time_us / 1000.0);
        report += line;
    }

    // 内存和CPU信息
    report += "\n--- System Resources ---\n";
    report += "Memory Usage: " + std::to_string(getMemoryUsageMB()) + " MB\n";

    report += "============================================================\n";

    return report;
}

inline void Profiler::printReport() {
    // 直接输出报告 (ROS2环境应该使用RCLCPP_INFO)
    std::string report = getReport();
    printf("%s\n", report.c_str());
}

inline void Profiler::saveReport(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << getReport();
        file.close();
    }
}

inline double Profiler::getMemoryUsageMB() {
#ifdef __linux__
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss / 1024.0;  // KB转MB
#else
    // Windows实现
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        return pmc.WorkingSetSize / (1024.0 * 1024.0);  // Bytes转MB
    }
    return 0.0;
#endif
}

inline double Profiler::getCpuUsagePercent() {
    // 简化实现，返回0 (完整实现需要跨平台CPU时间计算)
    return 0.0;
}

inline ScopedTimer::ScopedTimer(const std::string& name)
    : name_(name), stopped_(false) {
    start_ = std::chrono::high_resolution_clock::now();
}

inline ScopedTimer::~ScopedTimer() {
    if (!stopped_) {
        stop();
    }
}

inline void ScopedTimer::stop() {
    auto end = std::chrono::high_resolution_clock::now();
    int64_t elapsed_us =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();

    Profiler::instance().recordTime(name_, elapsed_us);
    stopped_ = true;
}

inline int64_t ScopedTimer::elapsed() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start_).count();
}

/**
 * @brief 性能监控初始化辅助函数
 */
inline void initProfiler(const ProfilerConfig& config = ProfilerConfig()) {
    Profiler::instance().configure(config);
    Profiler::instance().enable();
}

}  // namespace fast_lio2_slam