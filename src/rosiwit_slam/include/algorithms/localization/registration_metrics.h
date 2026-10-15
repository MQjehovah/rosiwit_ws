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

/// 动态/变化物剔除: 仅保留与目标地图最近距离 <= max_dist 的源点.
/// (人来人往/叉车/未入图的新堆货 = 离地图远 -> 剔除, 定位以静态结构为准)
CloudType::Ptr filterByMapProximity(const CloudType::Ptr& scan,
                                    const pcl::KdTreeFLANN<PointType>& kdtree,
                                    double max_dist);

// ---- 存储底座(IMapStore)版本 (P3: 存储与算法解耦) ----
CoverageStats evaluateCoverageStore(const IMapStore& store, const CloudType& scan,
                                    const Eigen::Matrix4d& T, double inlier_dist,
                                    int max_points = 3000);
double evaluatePoint2PlaneResidualStore(const IMapStore& store, const CloudType& scan,
                                        const Eigen::Matrix4d& T, double corr_dist,
                                        int max_points = 2000, double plane_valid_thr = 0.10);

} // namespace rosiwit_slam
