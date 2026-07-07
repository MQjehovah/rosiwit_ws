// include/algorithms/fast_lio2/fast_lio2_algorithm.h
#pragma once
#include <memory>
#include <string>
#include "slam_core/slam_base.h"
#include "commons.h"        // FastLIO2 专属: Config / IMUData / SyncPackage
#include "map_builder.h"    // MapBuilder
#include "ieskf.h"          // IESKF

namespace rosiwit_slam {

// FAST-LIO2 适配器: 把现有 MapBuilder + IESKF 核心封装为 ISlamAlgorithm。
// 持有算法内部状态(IESKF/MapBuilder/Config),对外只产 outSlamOutput。
class FastLio2Algorithm : public SlamBase {
public:
    bool        init(const std::string& config_path) override;
    std::string name() const override { return "fast_lio2"; }
    bool        getGlobalMap(PointVec& out_points) override;

protected:
    // SlamBase 同步完成后回调: 处理一帧已同步数据, 组装统一 SlamOutput
    bool onSyncedPackage(const SyncPackage& pkg, SlamOutput& out) override;

private:
    std::shared_ptr<IESKF>      m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    Config                      m_config;        // FastLIO2 专属配置
    bool m_inited = false;
};

} // namespace rosiwit_slam
