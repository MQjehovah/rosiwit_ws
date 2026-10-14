// include/algorithms/localization/local_map/radius_submap_provider.h
#pragma once
#include <unordered_map>
#include <cstdint>
#include "slam_core/i_local_map_provider.h"

namespace rosiwit_slam {

/// 半径子图提供者: 把全局地图按 cell_size 分块建立体素哈希, 按当前位姿
/// 取半径内单元拼成局部地图. 相比全图大幅降低配准规模, 并限制错配范围.
class RadiusSubmapProvider : public ILocalMapProvider {
public:
    std::string name() const override { return "radius_submap"; }
    void configure(const LocalMapConfig& cfg) override { m_cfg = cfg; }
    void setMap(const std::string& map_path, const CloudType::Ptr& global_map) override;
    CloudType::Ptr getLocalMap(const PoseStamped& pose) override;

private:
    static int64_t cellKey(int cx, int cy, int cz);

    LocalMapConfig m_cfg;
    CloudType::Ptr m_map;
    double m_voxel = 0.3;         // 建块前降采样
    double m_cell = 10.0;         // 单元边长(m)
    std::unordered_map<int64_t, CloudType::Ptr> m_cells;
};

} // namespace rosiwit_slam
