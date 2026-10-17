// include/algorithms/localization/monitor/threshold_monitor.h
#pragma once
#include "slam_core/i_localization_monitor.h"

namespace rosiwit_slam {

/// 阈值监测: 仅按 fitness 阈值 + 连续低分计数判定, 实现简单稳健.
class ThresholdMonitor : public ILocalizationMonitor {
public:
    std::string name() const override { return "threshold"; }
    void configure(const MonitorConfig& cfg) override { m_cfg = cfg; }
    void reset() override;
    MonitorLevel update(const MonitorInput& in) override;

private:
    MonitorConfig m_cfg;
    int m_low_count = 0;
    int m_mediocre_count = 0;
    MonitorLevel m_level = MonitorLevel::INIT;
};

} // namespace rosiwit_slam
