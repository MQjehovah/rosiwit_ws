// include/algorithms/localization/registration_metrics.h
#pragma once
#include <vector>
#include <Eigen/Eigen>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/slam_types.h"
#include "slam_core/i_map_store.h"

namespace rosiwit_slam {

/// 覆盖/内点统计 (跨配准器统一质量口径)
struct CoverageStats {
    double coverage = 0.0;    // inliers / total [0,1]
    double mean_dist = -1.0;  // 平均最近邻距离(m)
    int    inliers = 0;
    int    total = 0;
};

/// 将 scan 按 T 变换后, 统计到 target 的 1-NN 覆盖率.
CoverageStats evaluateCoverage(const pcl::KdTreeFLANN<PointType>& kdtree,
                               const CloudType& scan,
                               const Eigen::Matrix4d& T,
                               double inlier_dist,
                               int max_points = 3000);

/// 点到面平均残差(m): 对每个源点取 5-NN 拟合局部平面, 返回平均 |n·(p-c)|.
/// 点数不足返回 -1.
double evaluatePoint2PlaneResidual(const pcl::KdTreeFLANN<PointType>& kdtree,
                                   const CloudType& scan,
                                   const Eigen::Matrix4d& T,
                                   double corr_dist,
                                   int max_points = 2000,
                                   double plane_valid_thr = 0.10);

/// 动态/变化物剔除: 仅保留变换到地图系后与目标地图最近距离 <= max_dist 的源点.
/// (人来人往/叉车/未入图的新堆货 = 离地图远 -> 剔除, 定位以静态结构为准)
/// T 必须是 world<-source (通常是当前 guess); 不能拿原始雷达系点直接查世界地图.
CloudType::Ptr filterByMapProximity(const CloudType::Ptr& scan,
                                    const pcl::KdTreeFLANN<PointType>& kdtree,
                                    const Eigen::Matrix4d& T,
                                    double max_dist);

/// 方向可观测性 (gicp 等不提供 JtJ 的匹配器用):
/// 用"已配上的内点"的局部平面法向构造 6x6 信息矩阵 (order: [rot(3), trans(3)]):
///   J = [ ((p_w - t) x n)^T , n^T ]   ->   H = sum J J^T
/// 注意旋转项用"相对传感器"的点 (与 pp_icp 的 R*bp 同量纲); 用绝对世界点会让
/// 6D 特征值比恒为 ~1e-5, 把正常帧误判成严重退化.
/// 特征值升序; smallest_ratio = λmin/λmax; degenerate_dirs = λ/λmax < 1e-3 的个数.
/// 用途(2026-11-25): gicp 以前不填 has_hessian, 所以 degeneracy_gate /
/// degeneracy_partial_update / anchor 的 eig 判据一直是空转(eig 恒 1.0).
/// 本例只"上报"这组量用于诊断: 用本估计器算出的 λmin/λmax 在正常帧只有 ~4e-3,
/// 与 degeneracy_ratio_soft(0.05) 量纲不匹配, 直接把 has_hessian 置位会让
/// partial_update 把正常帧的修正压到 ~8% -> 实测 14% LOST. 要启用必须先按现场
/// 数据重新标定 soft 门限 (见 docs/HANDOVER_2026-11-25_device_sync.md).
/// 返回 false = 有效平面点不足(<30), 不可判定 (此时调用方应保持原行为).
bool evaluateObservability(const pcl::KdTreeFLANN<PointType>& kdtree,
                           const CloudType& scan,
                           const Eigen::Matrix4d& T,
                           double neigh_dist,
                           double plane_tol,
                           Eigen::Matrix<double, 6, 1>& eigvals,
                           Eigen::Matrix<double, 6, 6>& eigvecs,
                           double& smallest_ratio,
                           int& degenerate_dirs,
                           int max_points = 400,
                           double plane_valid_thr = 0.10);

// ---- 存储底座(IMapStore)版本 (P3: 存储与算法解耦) ----
CoverageStats evaluateCoverageStore(const IMapStore& store, const CloudType& scan,
                                    const Eigen::Matrix4d& T, double inlier_dist,
                                    int max_points = 3000);
double evaluatePoint2PlaneResidualStore(const IMapStore& store, const CloudType& scan,
                                        const Eigen::Matrix4d& T, double corr_dist,
                                        int max_points = 2000, double plane_valid_thr = 0.10);

} // namespace rosiwit_slam
