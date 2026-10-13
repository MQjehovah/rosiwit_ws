// include/algorithms/grid_map/octomap_grid_map.h
#pragma once
#include "slam_core/i_grid_map_generator.h"

namespace rosiwit_slam {

/// OctoMap 概率占据法:
/// 用 octomap::OcTree 逐帧 insertPointCloud 累积 3D 概率占据(内含射线 free 更新),
/// 再把落在障碍高度带的占据叶节点投影为 2D occupied, 有 free 叶节点处标 free,
/// 其余未观测格 unknown. 作为独立的第三方占据建图基线与自研策略对照.
class OctomapGridMap : public IGridMapGenerator {
public:
    std::string name() const override { return "octomap"; }
    bool needsObservations() const override { return true; }

    bool build(const CloudType::Ptr& cloud,
               const Eigen::Vector3d& origin,
               const GridMapConfig& cfg,
               GridInfo& out) override;

    bool buildFromObservations(const std::vector<Observation>& obs,
                               const Eigen::Vector3d& origin,
                               const GridMapConfig& cfg,
                               GridInfo& out) override;
};

} // namespace rosiwit_slam
