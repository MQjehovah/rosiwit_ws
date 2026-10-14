// include/algorithms/localization/localization_utils.h
#pragma once
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 体素降采样
CloudType::Ptr downsampleCloud(const CloudType::Ptr& in, double resolution);

/// 过滤非有限点
CloudType::Ptr filterFinite(const CloudType::Ptr& in);

/// 旋转矩阵 → yaw (rad)
double yawFromRot(const M3D& R);

/// 位姿差 (a ⊖ b): 返回 b^{-1} a 的平移与轴角
void poseDelta(const PoseStamped& a, const PoseStamped& b, V3D& dt, double& angle_rad);

} // namespace rosiwit_slam
