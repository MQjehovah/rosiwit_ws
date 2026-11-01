// include/algorithms/localization/localization_utils.h
#pragma once
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <pcl/kdtree/kdtree_flann.h>
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

/// 位姿创新 (6 维, order=[rot(3)轴角, trans(3)]): match ⊖ pred
void poseInnovation(const PoseStamped& match, const PoseStamped& pred,
                    Eigen::Matrix<double, 6, 1>& e);

/// 用叠加层压制原图: 移除与 overlay 点距离 < radius 的 base 点 (处理结构被移除/
/// 替换 —— 叠加层作为权威, 原图同处旧点不再参与配准)
void suppressBaseByOverlay(CloudType::Ptr& cloud, const CloudType::Ptr& overlay,
                           double radius);


/// 以 center 为中心、radius 为半径裁剪局部子图 (KD-tree 半径搜索).
///
/// 全局重定位只需要候选附近的结构: 在 110 万点全图上给每个候选跑 GICP,
/// 单候选几百 ms~秒级且随地图线性变慢 —— 这是重定位 20~45 s 的主要来源。
/// 裁 25~30 m 后目标通常只剩 1~5 万点, 单候选 GICP 降到 10~50 ms。
CloudType::Ptr cropRadius(const CloudType::Ptr& cloud, const V3D& center, double radius);

/// 同上, 但复用调用方已建好的 KD-tree (整图只建一次树), 并限制返回点数上限。
CloudType::Ptr cropRadius(const pcl::KdTreeFLANN<PointType>& kdtree,
                          const CloudType::Ptr& cloud, const V3D& center, double radius,
                          double max_points = 60000);

} // namespace rosiwit_slam
