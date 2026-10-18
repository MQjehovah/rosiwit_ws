// src/slam_core/diagnostics.cpp
#include "slam_core/diagnostics.h"
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

namespace rosiwit_slam {
namespace diag {

namespace {
std::string wallClockMs() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t sec = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count() % 1000;
    std::tm tm_buf;
    localtime_r(&sec, &tm_buf);
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
                  tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, static_cast<int>(ms));
    return std::string(buf);
}
}   // namespace

void log(char level, const char* tag, const std::string& msg) {
    // 单条 fprintf: 保证与行内其他输出不交错 (std::cout 多段 << 会撕裂)
    std::fprintf(stdout, "[%s][%c][%s] %s\n", wallClockMs().c_str(), level, tag, msg.c_str());
    std::fflush(stdout);
}

FlightRecorder& FlightRecorder::instance() {
    static FlightRecorder inst;
    return inst;
}

void FlightRecorder::configure(size_t max_frames, const std::string& auto_dump_dir) {
    std::lock_guard<std::mutex> lk(m_mu);
    if (max_frames > 0) m_max_frames = max_frames;
    m_auto_dump_dir = auto_dump_dir;
}

void FlightRecorder::record(const FrameRecord& r) {
    std::lock_guard<std::mutex> lk(m_mu);
    m_frames.push_back(r);
    while (m_frames.size() > m_max_frames) m_frames.pop_front();
}

void FlightRecorder::event(const std::string& tag, std::string msg) {
    EventRecord e;
    e.t = std::chrono::duration<double>(
              std::chrono::system_clock::now().time_since_epoch()).count();
    e.tag = tag;
    e.msg = std::move(msg);
    std::lock_guard<std::mutex> lk(m_mu);
    m_events.push_back(std::move(e));
    while (m_events.size() > 512) m_events.pop_front();
}

void FlightRecorder::onStatusChange(bool entered_lost) {
    std::string dir;
    {
        std::lock_guard<std::mutex> lk(m_mu);
        if (!entered_lost || m_auto_dump_dir.empty()) return;
        dir = m_auto_dump_dir;
    }
    dump("LOST");
}

std::string FlightRecorder::dump(const std::string& reason) {
    std::lock_guard<std::mutex> lk(m_mu);
    if (m_frames.empty()) return std::string();

    ::mkdir(m_auto_dump_dir.c_str(), 0755);   // 已存在则忽略错误
    char name[256];
    std::snprintf(name, sizeof(name), "%s/flight_%llu_%s.csv",
                  m_auto_dump_dir.c_str(),
                  static_cast<unsigned long long>(++m_dump_seq), reason.c_str());
    std::FILE* f = std::fopen(name, "w");
    if (!f) return std::string();

    // 事件表 (时间用系统钟, 与 diag::log 的墙上时钟一致)
    std::fprintf(f, "# events\n");
    std::fprintf(f, "# wall_time,tag,message\n");
    for (const auto& e : m_events)
        std::fprintf(f, "# %.3f,%s,%s\n", e.t, e.tag.c_str(), e.msg.c_str());

    // 帧表 (时间用雷达时基)
    std::fprintf(f, "# frames\n");
    std::fprintf(f, "frame_time,status,fitness,lag,corr,tgt,accepted,coast,hyp\n");
    for (const auto& r : m_frames)
        std::fprintf(f, "%.3f,%d,%.4f,%.4f,%d,%d,%d,%d,%d\n",
                     r.t, r.status, r.fitness, r.lag, r.corr, r.tgt,
                     r.accepted, r.coast, r.hyp);
    std::fclose(f);

    log('I', "FlightRecorder", std::string("dumped ") + std::to_string(m_frames.size()) +
                                 " frames / " + std::to_string(m_events.size()) +
                                 " events -> " + name);
    return std::string(name);
}

size_t FlightRecorder::frameCount() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_frames.size();
}
size_t FlightRecorder::eventCount() const {
    std::lock_guard<std::mutex> lk(m_mu);
    return m_events.size();
}

}   // namespace diag
}   // namespace rosiwit_slam
