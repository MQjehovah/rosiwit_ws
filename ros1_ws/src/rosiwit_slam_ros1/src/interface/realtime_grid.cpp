// ============================================================================
// 建图模式实时栅格的实现 (设计文档 §5.11 A)
// 逐行复刻核心 PcdMapManager::generateGridMap() (pcd_map_manager.cpp:264-293)
// ============================================================================
#include "realtime_grid.h"

#include <ros/ros.h>


#include <algorithm>
#include <fstream>
#include <memory>
#include <vector>


namespace rosiwit_slam_ros1 {


RealtimeGrid::RealtimeGrid() = default;

RealtimeGrid::~RealtimeGrid() { stop(); }

void RealtimeGrid::setGridSource(GridSource s) {
    std::lock_guard<std::mutex> lk(m_mtx_);
    m_grid_source_ = std::move(s);
}

void RealtimeGrid::start(double hz) {
    if (m_thread_) return;   // 已经在跑
    const double rate = (hz > 0.0) ? hz : 1.0;
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / rate));
    // PeriodicThread: sleep_until(next += period), 单次重算超时不会累积漂移
    m_thread_.reset(new rosiwit_common::PeriodicThread(
        "rt_grid", period, [this] { runOnce(); }));
}

void RealtimeGrid::stop() {
    if (!m_thread_) return;
    m_thread_->requestStop();
    m_thread_->join();
    m_thread_.reset();
}

bool RealtimeGrid::latest(rosiwit_slam::GridInfo& out) const {
    const auto snap = m_slot_.load();
    if (!snap) return false;
    out = *snap;
    return true;
}

void RealtimeGrid::clear() {
    m_slot_.publish(nullptr);   // 等价于"从没算出过"
}

void RealtimeGrid::runOnce() {
    // 只取数据源; 栅格参数由核心自己持有 (S3: 这层不再镜像一份配置)
    GridSource gsrc;
    {
        std::lock_guard<std::mutex> lk(m_mtx_);
        gsrc = m_grid_source_;
    }
    if (!gsrc) return;
    try {
        rosiwit_slam::GridInfo info;
        // 直接取核心生成好的栅格 (与存 pgm 同一条路径, 有逐帧地面处理)
        if (gsrc(info)) {
            m_slot_.publish(std::make_shared<const rosiwit_slam::GridInfo>(std::move(info)));
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "[realtime_grid] build failed: " << e.what());
    }
}

} // namespace rosiwit_slam_ros1
