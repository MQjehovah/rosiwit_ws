// include/algorithms/localization/local_map/global_map_provider.h
#pragma once
#include "slam_core/i_local_map_provider.h"

namespace rosiwit_slam {

/// 全图提供者: 直接用整张全局地图(降采样后)作为配准目标. 基线实现.
class GlobalMapProvider : public ILocalMapProvider {
public:
    std::string name() const override { return "global"; }
    void configure(const LocalMapConfig& cfg) override { m_cfg = cfg; }
    void setMap(const std::string& map_path, const CloudType::Ptr& global_map) override;
    CloudType::Ptr getLocalMap(const PoseStamped& pose) override;

private:
    LocalMapConfig m_cfg;
    CloudType::Ptr m_map;
};

} // namespace rosiwit_slam
