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

/// 用叠加层压制原图: 移除与 overlay 点距离 < radius 的 base 点 (处理结构被移除/
/// 替换 —— 叠加层作为权威, 原图同处旧点不再参与配准)
void suppressBaseByOverlay(CloudType::Ptr& cloud, const CloudType::Ptr& overlay,
                           double radius);

} // namespace rosiwit_slam
