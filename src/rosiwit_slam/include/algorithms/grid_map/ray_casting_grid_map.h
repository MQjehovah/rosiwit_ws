// include/algorithms/grid_map/ray_casting_grid_map.h
#pragma once
#include "slam_core/i_grid_map_generator.h"

namespace rosiwit_slam {

/// 射线投射法 (Bresenham/DDA):
/// 对每帧观测, 从传感器原点向每个扫描终点投射 2D 射线,
/// 途经格标记为 free, 终点(落在障碍高度带内)标记为 occupied,
/// 其余未观测格保持 unknown(-1). 相比高度投影可区分自由/未知空间,
/// 且障碍来自真实回波终点, 边界更贴合.
class RayCastingGridMap : public IGridMapGenerator {
public:
    std::string name() const override { return "ray_casting"; }
    bool needsObservations() const override { return true; }

    // 单帧入口: 视为原点为 origin 的一次观测
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
