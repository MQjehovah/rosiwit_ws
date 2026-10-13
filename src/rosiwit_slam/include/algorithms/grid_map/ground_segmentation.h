// include/algorithms/grid_map/ground_segmentation.h
#pragma once
#include <limits>
#include <string>
#include <Eigen/Eigen>
#include "slam_core/i_grid_map_generator.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 地面模型: 平面 n·p + d = 0 (n 指向地面上方). height() 返回点到地面的有符号高度.
struct GroundModel {
    Eigen::Vector3f n = Eigen::Vector3f::UnitZ();
    float d = 0.0f;
    bool valid = false;

    float height(const PointType& p) const {
        return n.x() * p.x + n.y() * p.y + n.z() * p.z + d;
    }
    float height(double x, double y, double z) const {
        return n.x() * static_cast<float>(x) + n.y() * static_cast<float>(y) +
               n.z() * static_cast<float>(z) + d;
    }
};

/// 估计地面: ground_z 非 NaN 时直接采用; 否则按 ground_method:
///   ransac     -> PCL 平面拟合(要求法向近竖直, 否则回退 percentile)
///   percentile -> z 低分位数(2%)
GroundModel fitGround(const CloudType::Ptr& cloud, const GridMapConfig& cfg);

/// 仅用 z 集合的分位数地面 (percentile 方法/兜底)
GroundModel fitGroundPercentile(const std::vector<double>& zs, const GridMapConfig& cfg);

} // namespace rosiwit_slam
