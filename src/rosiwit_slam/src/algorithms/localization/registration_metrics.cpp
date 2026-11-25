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
    // P4: 目标点自带持久度 w (intensity), 用于加权锚定判据
    const CloudType::ConstPtr tcloud = kdtree.getInputCloud();
    double w_all = 0.0, w_in = 0.0;
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
        double tw = 128.0;                       // 目标点无 w 时按已确认计
        if (tcloud && idx[0] >= 0 && static_cast<size_t>(idx[0]) < tcloud->size())
            tw = std::max(0.0f, std::min(255.0f, tcloud->points[idx[0]].intensity));
        w_all += tw;
        ++used;
        if (sq[0] <= thr2) {
            ++s.inliers; sum += std::sqrt(sq[0]);
            w_in += tw;
            if (tw >= 128.0f) ++s.inl128;
        }
    }
    s.total = used;
    if (used > 0) {
        s.coverage = static_cast<double>(s.inliers) / used;
        if (s.inliers > 0) s.mean_dist = sum / s.inliers;
        if (w_all > 0.0) s.w_support = w_in / w_all;   // 位姿被已确认结构支撑的比例
    }
    return s;
}

CloudType::Ptr filterByMapProximity(const CloudType::Ptr& scan,
                                    const pcl::KdTreeFLANN<PointType>& kdtree,
                                    const Eigen::Matrix4d& T,
                                    double max_dist) {
    CloudType::Ptr out(new CloudType());
    if (!scan || scan->empty()) return out;
    const double d2 = max_dist * max_dist;
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    out->reserve(scan->size());
    for (const auto& p : scan->points) {
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointType qp;
        qp.x = static_cast<float>(q.x());
        qp.y = static_cast<float>(q.y());
        qp.z = static_cast<float>(q.z());
        if (kdtree.nearestKSearch(qp, 1, idx, sq) > 0 && sq[0] <= d2) out->push_back(p);
    }
    return out;
}

CoverageStats evaluateCoverageStore(const IMapStore& store, const CloudType& scan,
                                    const Eigen::Matrix4d& T, double inlier_dist, int max_points) {
    CoverageStats s;
    const int n = static_cast<int>(scan.size());
    if (n == 0) return s;
    const int step = std::max(1, n / std::max(1, max_points));
    const double thr2 = inlier_dist * inlier_dist;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    double sum = 0.0;
    int used = 0;
    for (int i = 0; i < n; i += step) {
        const auto& p = scan.points[i];
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointType qp; qp.x = static_cast<float>(q.x()); qp.y = static_cast<float>(q.y()); qp.z = static_cast<float>(q.z());
        const double d2 = store.nearestDist2(qp);
        if (d2 < 0) continue;
        ++used;
        if (d2 <= thr2) { ++s.inliers; sum += std::sqrt(d2); }
    }
    s.total = used;
    if (used > 0) {
        s.coverage = static_cast<double>(s.inliers) / used;
        if (s.inliers > 0) s.mean_dist = sum / s.inliers;
    }
    return s;
}

double evaluatePoint2PlaneResidualStore(const IMapStore& store, const CloudType& scan,
                                        const Eigen::Matrix4d& T, double corr_dist,
                                        int max_points, double plane_valid_thr) {
    const int n = static_cast<int>(scan.size());
    if (n == 0) return -1.0;
    const int step = std::max(1, n / std::max(1, max_points));
    const double corr2 = corr_dist * corr_dist;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);
    std::vector<PointType> nbpts;
    std::vector<float> sq;
    double sum = 0.0;
    int used = 0;
    for (int i = 0; i < n; i += step) {
        const auto& p = scan.points[i];
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointType qp; qp.x = static_cast<float>(q.x()); qp.y = static_cast<float>(q.y()); qp.z = static_cast<float>(q.z());
        if (store.nearestK(qp, 5, nbpts, sq) < 5) continue;
        if (sq.back() > corr2) continue;
        std::vector<Eigen::Vector3d> nb(5);
        for (int k = 0; k < 5; ++k) nb[k] = Eigen::Vector3d(nbpts[k].x, nbpts[k].y, nbpts[k].z);
        Eigen::Vector3d normal, centroid;
        if (!fitPlanePCA(nb, normal, centroid, plane_valid_thr)) continue;
        sum += std::fabs(normal.dot(q - centroid));
        ++used;
    }
    if (used < 10) return -1.0;
    return sum / used;
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

bool evaluateObservability(const pcl::KdTreeFLANN<PointType>& kdtree,
                           const CloudType& scan,
                           const Eigen::Matrix4d& T,
                           double neigh_dist,
                           double plane_tol,
                           Eigen::Matrix<double, 6, 1>& eigvals,
                           Eigen::Matrix<double, 6, 6>& eigvecs,
                           double& smallest_ratio,
                           int& degenerate_dirs,
                           int max_points,
                           double plane_valid_thr) {
    const int n = static_cast<int>(scan.size());
    if (n == 0) return false;
    const int step = std::max(1, n / std::max(1, max_points));
    const double neigh2 = neigh_dist * neigh_dist;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T.block<3, 1>(0, 3);

    std::vector<int> idx(5);
    std::vector<float> sq(5);
    std::vector<Eigen::Vector3d> nb(5);
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    int used = 0;
    for (int i = 0; i < n; i += step) {
        const auto& p = scan.points[i];
        const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
        PointType qp;
        qp.x = static_cast<float>(q.x());
        qp.y = static_cast<float>(q.y());
        qp.z = static_cast<float>(q.z());
        if (kdtree.nearestKSearch(qp, 5, idx, sq) <= 0) continue;
        if (sq.back() > neigh2) continue;
        for (int k = 0; k < 5; ++k) {
            const auto& tp = kdtree.getInputCloud()->points[idx[k]];
            nb[k] = Eigen::Vector3d(tp.x, tp.y, tp.z);
        }
        Eigen::Vector3d normal, centroid;
        if (!fitPlanePCA(nb, normal, centroid, plane_valid_thr)) continue;
        // 只统计"本轮确实配上"的点 (点到局部平面的距离)
        if (std::fabs(normal.dot(q - centroid)) > plane_tol) continue;
        Eigen::Matrix<double, 6, 1> J;
        // 旋转项必须用"相对传感器"的点 (p_w - t): 用绝对世界点会把 |p|~100m 的
        // 平移偏置带进旋转块, 让 6D 特征值比恒为 ~1e-5 -> 正常帧被误判成严重退化
        // (2026-11-25 回归 C: eig 全是 1.3e-5, partial_update 把修正清零 -> 98% LOST).
        // pp_icp 用的就是 R*bp (= p_w - t), 这里与它保持同一量纲.
        J.head<3>() = (q - t).cross(normal);
        J.tail<3>() = normal;            // d(trans): n
        H += J * J.transpose();
        ++used;
    }
    if (used < 30) return false;
    H /= static_cast<double>(used);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
    if (es.info() != Eigen::Success) return false;
    eigvals = es.eigenvalues();          // 升序
    eigvecs = es.eigenvectors();
    const double lmax = eigvals.maxCoeff();
    if (!(lmax > 1e-12)) return false;
    smallest_ratio = eigvals.minCoeff() / lmax;
    degenerate_dirs = 0;
    for (int k = 0; k < 6; ++k) {
        if (eigvals(k) / lmax < 1e-3) ++degenerate_dirs;
    }
    return true;
}

} // namespace rosiwit_slam
