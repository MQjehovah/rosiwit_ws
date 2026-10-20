// include/algorithms/localization/monitor/threshold_monitor.h
#pragma once
#include "slam_core/i_localization_monitor.h"

namespace rosiwit_slam {

/// 阈值监测: 仅做当前帧 fitness 健康分类；连续判丢由 pipeline 统一计数.
class ThresholdMonitor : public ILocalizationMonitor {
public:
    std::string name() const override { return "threshold"; }
    void configure(const MonitorConfig& cfg) override { m_cfg = cfg; }
    void reset() override;
    MonitorLevel update(const MonitorInput& in) override;

private:
    MonitorConfig m_cfg;
    MonitorLevel m_level = MonitorLevel::INIT;
};

} // namespace rosiwit_slam
