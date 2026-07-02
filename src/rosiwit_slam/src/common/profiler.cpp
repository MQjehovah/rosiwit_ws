/**
 * @file profiler.cpp
 * @brief FAST-LIO2 SLAM - 性能监控工具实现
 */

#include "fast_lio2_slam/common/profiler.h"

#include <cstdio>

namespace fast_lio2_slam {

void ProfileStats::reset() {
    total_time_us = 0;
    max_time_us = 0;
    min_time_us = INT64_MAX;
    avg_time_us = 0;
    call_count = 0;
    p50_time_us = 0;
    p95_time_us = 0;
    p99_time_us = 0;
}

void ProfileStats::update(int64_t elapsed_us) {
    total_time_us += elapsed_us;
    max_time_us = std::max(max_time_us, elapsed_us);
    min_time_us = std::min(min_time_us, elapsed_us);
    call_count++;

    if (call_count > 0) {
        avg_time_us = total_time_us / call_count;
    }
}

Profiler& Profiler::instance() {
    static Profiler profiler;
    return profiler;
}

Profiler::~Profiler() {
    if (config_.print_on_exit && enabled_) {
        printReport();
    }

    if (output_file_.is_open()) {
        output_file_.close();
    }
}

void Profiler::configure(const ProfilerConfig& config) {
    config_ = config;

    if (!config_.output_file.empty()) {
        output_file_.open(config_.output_file);
    }
}

int64_t Profiler::startTiming(const std::string& name) {
    if (!enabled_) return 0;

    auto start = std::chrono::high_resolution_clock::now();
    return start.time_since_epoch().count();
}

void Profiler::endTiming(const std::string& name, int64_t start_time) {
    if (!enabled_) return;

    auto end = std::chrono::high_resolution_clock::now();
    int64_t elapsed_us = (end.time_since_epoch().count() - start_time) / 1000;

    recordTime(name, elapsed_us);
}

void Profiler::recordTime(const std::string& name, int64_t elapsed_us) {
    if (!enabled_) return;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_[name].name = name;
        stats_[name].update(elapsed_us);
    }

    // 更百分位数历史
    updatePercentiles(name, elapsed_us);
}

void Profiler::updatePercentiles(const std::string& name, int64_t elapsed_us) {
    std::lock_guard<std::mutex> lock(history_mutex_);

    history_[name].push_back(elapsed_us);

    // 保持历史记录大小限制
    if (history_[name].size() > static_cast<size_t>(config_.max_history_size)) {
        history_[name].erase(history_[name].begin());
    }
}

ProfileStats Profiler::getStats(const std::string& name) {
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

std::unordered_map<std::string, ProfileStats> Profiler::getAllStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void Profiler::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::lock_guard<std::mutex> hist_lock(history_mutex_);

    for (auto& stat : stats_) {
        stat.second.reset();
    }
    history_.clear();
}

std::string Profiler::getReport() {
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

void Profiler::printReport() {
    // 直接输出报告 (ROS2环境应该使用RCLCPP_INFO)
    std::string report = getReport();
    printf("%s\n", report.c_str());
}

void Profiler::saveReport(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << getReport();
        file.close();
    }
}

double Profiler::getMemoryUsageMB() {
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

double Profiler::getCpuUsagePercent() {
    // 简化实现，返回0 (完整实现需要跨平台CPU时间计算)
    return 0.0;
}

ScopedTimer::ScopedTimer(const std::string& name)
    : name_(name), stopped_(false) {
    start_ = std::chrono::high_resolution_clock::now();
}

ScopedTimer::~ScopedTimer() {
    if (!stopped_) {
        stop();
    }
}

void ScopedTimer::stop() {
    auto end = std::chrono::high_resolution_clock::now();
    int64_t elapsed_us =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();

    Profiler::instance().recordTime(name_, elapsed_us);
    stopped_ = true;
}

int64_t ScopedTimer::elapsed() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start_).count();
}

}  // namespace fast_lio2_slam
