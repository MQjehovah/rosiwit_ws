// ============================================================================
// 定位丢失状态机的实现 (设计文档 §5.11 B)
// 规则表 (状态 -> location_status):
//   !map_loaded                    -> RECOVERING(1)   定位链路还没就绪
//   !ever_localized_               -> RECOVERING(1)   等价于旧 relocating_ == true
//   ever_localized_ && !pose_cov_ok-> FAILED(2)       并且置一次 lost_edge
//   ever_localized_ &&  pose_cov_ok-> SUCCESS(0)
// ============================================================================
#include "lost_monitor.h"

namespace rosiwit_slam_ros1 {

void LostMonitor::reset() {
    ever_localized_ = false;
    last_failed_ = false;
    lost_edge_ = false;
}

void LostMonitor::onLocalizedPose() {
    ever_localized_ = true;
    // 恢复不算"新丢失": 上一次丢失还没被消费掉就恢复了, 那个边沿作废 ——
    // 否则"丢失->立刻恢复->发布方稍后才发现边沿"会报出一个已经结束的丢失。
    // (T11 里消费方和这里同线程且每 tick 都看, 所以真实的丢失不会被漏报。)
    if (last_failed_) lost_edge_ = false;
}

void LostMonitor::setMapLoaded(bool loaded) { map_loaded_ = loaded; }

LocStatus LostMonitor::update(bool pose_cov_ok) {
    if (!map_loaded_ || !ever_localized_) {
        // 还没到"能谈丢失"的阶段: 一律 1, 并清掉可能挂着的边沿
        last_failed_ = false;
        lost_edge_ = false;
        return LocStatus::RECOVERING;
    }

    if (!pose_cov_ok) {
        if (!last_failed_) lost_edge_ = true;   // 只在 非FAILED -> FAILED 的边沿置位
        last_failed_ = true;
        return LocStatus::FAILED;
    }

    // 已恢复, 上一次的丢失不再上报
    last_failed_ = false;
    lost_edge_ = false;
    return LocStatus::SUCCESS;
}

bool LostMonitor::consumeLostEdge() {
    if (!lost_edge_) return false;
    lost_edge_ = false;
    return true;
}

} // namespace rosiwit_slam_ros1
