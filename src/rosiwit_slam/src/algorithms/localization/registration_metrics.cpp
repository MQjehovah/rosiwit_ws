// src/algorithms/localization/registration_metrics.cpp
#include "algorithms/localization/registration_metrics.h"
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {

namespace {
// 用 5 个邻域点做 PCA 平面拟合; 成功返回 true 并填充法向/质心.
bool fitPlanePCA(const std::vector<Eigen::Vector3d>& pts,
                 Eigen::Vector3d& normal, Eigen::Vector3d& centroid,
                 double valid_thr) {
    const int n = static_cast<int>(pts.size());
    if (n < 3) return false;
    centroid.setZero();
    for (const auto& p : pts) centroid += p;
    centroid /= n;

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts) {
        const Eigen::Vector3d d = p - centroid;
        cov += d * d.transpose();
    }
    cov /= n;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    normal = es.eigenvectors().col(0);  // 最小特征值方向
    // 平面有效性: 所有邻域点到平面距离需小于阈值
    for (const auto& p : pts) {
        if (std::fabs(normal.dot(p - centroid)) > valid_thr) return false;
    }
    return true;
}
} // namespace

CoverageStats evaluateCoverage(const pcl::KdTreeFLANN<PointType>& kdtree,
                               const CloudType& scan,
                               const Eigen::Matrix4d& T,
                               double inlier_dist,
                               int max_points) {
    CoverageStats s;
    const int n = static_cast<int>(scan.size());
    if (n == 0) return s;
    const int step = std::max(1, n / std::max(1, max_points));
    const double thr2 = inlier_dist * inlier_dist;

    std::vector<int> idx(1);
    std::vector<float> sq(1);
    const Eigen::Matrix3d R = T.block<3,3>(0,0);
    const Eigen::Vector3d t = T.block<3,1>(0,3);
    double sum = 0.0;
    int used = 0;
    for (int i = 0; i < n; i += step) {
        const auto& p = scan.points[i];
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointType qp;
        qp.x = static_cast<float>(q.x());
        qp.y = static_cast<float>(q.y());
        qp.z = static_cast<float>(q.z());
        if (kdtree.nearestKSearch(qp, 1, idx, sq) <= 0) continue;
        ++used;
        if (sq[0] <= thr2) { ++s.inliers; sum += std::sqrt(sq[0]); }
    }
    s.total = used;
    if (used > 0) {
        s.coverage = static_cast<double>(s.inliers) / used;
        if (s.inliers > 0) s.mean_dist = sum / s.inliers;
    }
    return s;
}

double evaluatePoint2PlaneResidual(const pcl::KdTreeFLANN<PointType>& kdtree,
                                   const CloudType& scan,
                                   const Eigen::Matrix4d& T,
                                   double corr_dist,
                                   int max_points,
                                   double plane_valid_thr) {
    const int n = static_cast<int>(scan.size());
    if (n == 0) return -1.0;
    const int step = std::max(1, n / std::max(1, max_points));
    const double corr2 = corr_dist * corr_dist;

    const Eigen::Matrix3d R = T.block<3,3>(0,0);
    const Eigen::Vector3d t = T.block<3,1>(0,3);

    std::vector<int> idx(5);
    std::vector<float> sq(5);
    std::vector<Eigen::Vector3d> nb(5);
    double sum = 0.0;
    int used = 0;
    for (int i = 0; i < n; i += step) {
        const auto& p = scan.points[i];
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointType qp;
        qp.x = static_cast<float>(q.x());
        qp.y = static_cast<float>(q.y());
        qp.z = static_cast<float>(q.z());
        if (kdtree.nearestKSearch(qp, 5, idx, sq) <= 0) continue;
        if (sq.back() > corr2) continue;
        for (int k = 0; k < 5; ++k) {
            const auto& tp = kdtree.getInputCloud()->points[idx[k]];
            nb[k] = Eigen::Vector3d(tp.x, tp.y, tp.z);
        }
        Eigen::Vector3d normal, centroid;
        if (!fitPlanePCA(nb, normal, centroid, plane_valid_thr)) continue;
        sum += std::fabs(normal.dot(q - centroid));
        ++used;
    }
    if (used < 10) return -1.0;
    return sum / used;
}

} // namespace rosiwit_slam
