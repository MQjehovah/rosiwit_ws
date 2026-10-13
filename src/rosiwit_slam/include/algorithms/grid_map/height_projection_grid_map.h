// include/algorithms/grid_map/height_projection_grid_map.h
#pragma once
#include "slam_core/i_grid_map_generator.h"

namespace rosiwit_slam {

/// 高度投影法: 将点云按高度带 [ground+min_height, ground+max_height] 投影到 2D,
/// 统计每格点数, 超过阈值判为占据; 再按点间距做圆形膨胀连通墙体.
/// 自适应地面检测 + 网格尺寸防溢出.
class HeightProjectionGridMap : public IGridMapGenerator {
public:
    std::string name() const override { return "height_projection"; }

    bool build(const CloudType::Ptr& cloud,
               const Eigen::Vector3d& origin,
               const GridMapConfig& cfg,
               GridInfo& out) override;
};

} // namespace rosiwit_slam
