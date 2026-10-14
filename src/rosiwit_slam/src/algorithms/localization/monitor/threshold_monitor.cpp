// src/algorithms/localization/monitor/threshold_monitor.cpp
#include "algorithms/localization/monitor/threshold_monitor.h"

namespace rosiwit_slam {

void ThresholdMonitor::reset() {
    m_low_count = 0;
    m_level = MonitorLevel::INIT;
}

MonitorLevel ThresholdMonitor::update(const MonitorInput& in) {
    if (!in.match.converged || in.match.fitness < m_cfg.lost_fitness) {
        ++m_low_count;
        if (m_low_count >= m_cfg.lost_patience_low) m_level = MonitorLevel::LOST;
        else m_level = MonitorLevel::DEGRADED;
        return m_level;
    }
    m_low_count = 0;
    m_level = (in.match.fitness >= m_cfg.credible_fitness)
                  ? MonitorLevel::CREDIBLE : MonitorLevel::DEGRADED;
    return m_level;
}

} // namespace rosiwit_slam
