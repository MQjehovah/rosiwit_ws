// src/algorithms/localization/monitor/threshold_monitor.cpp
#include "algorithms/localization/monitor/threshold_monitor.h"

namespace rosiwit_slam {

void ThresholdMonitor::reset() {
    m_level = MonitorLevel::INIT;
}

MonitorLevel ThresholdMonitor::update(const MonitorInput& in) {
    // 这里只做"当前帧健康与否"的分类；INIT/LOCALIZED -> LOST 的连续帧计数
    // 统一由 LocalizationPipeline 的 lost_patience 负责。
    if (!in.match.converged || in.match.fitness < m_cfg.lost_fitness) {
        m_level = MonitorLevel::DEGRADED;
        return m_level;
    }
    if (in.match.fitness >= m_cfg.credible_fitness) {
        m_level = MonitorLevel::CREDIBLE;
    } else {
        m_level = MonitorLevel::DEGRADED;
    }
    return m_level;
}

} // namespace rosiwit_slam
