// src/algorithms/localization/matcher/pp_icp_matcher.cpp
#include "algorithms/localization/matcher/pp_icp_matcher.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {

namespace {
constexpr double kPlaneValidThr = 0.15;  // 平面拟合有效性: 邻域点到平面最大距离(m)

inline Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
    Eigen::Matrix3d S;
    S <<     0.0, -a.z(),  a.y(),
         a.z(),      0.0, -a.x(),
        -a.y(),  a.x(),     0.0;
    return S;
}
} // namespace

double PpIcpMatcher::corrDistForIter(int iter) const {
    if (!m_cfg.use_corr_decay || m_cfg.max_iterations <= 1)
        return m_cfg.corr_dist_init;
    const double t = static_cast<double>(iter) / (m_cfg.max_iterations - 1);
    double d = m_cfg.corr_dist_init * (1.0 - t) + m_cfg.corr_dist_min * t;
    if (d < m_cfg.corr_dist_min) d = m_cfg.corr_dist_min;
    return d;
}

void PpIcpMatcher::setTarget(const CloudType::Ptr& target) {
    m_target = downsampleCloud(target, m_cfg.voxel_resolution);
    if (!m_target) m_target.reset(new CloudType());
    m_kdtree.setInputCloud(m_target);
}

bool PpIcpMatcher::match(const CloudType::Ptr& scan, const PoseStamped& guess,
                         MatchResult& out) {
    out = MatchResult();
    if (!m_target || m_target->size() < 5) return false;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!src || src->empty()) return false;
    if (m_cfg.dynamic_filter && m_cfg.dynamic_dist > 0) {
        src = filterByMapProximity(src, m_kdtree, m_cfg.dynamic_dist);
        if (!src || src->empty()) return false;
    }

    const int n = static_cast<int>(src->size());
    std::vector<Eigen::Vector3d> bp(n);
    for (int i = 0; i < n; ++i) {
        const auto& p = src->points[i];
        bp[i] = Eigen::Vector3d(p.x, p.y, p.z);
    }

    Eigen::Matrix3d R = guess.rot;
    Eigen::Vector3d t = guess.trans;

    bool solved = false, converged = false, any_degen = false;
    int max_degen = 0, iter_done = 0;
    double min_ratio = 1.0;
    Eigen::Matrix<double, 6, 6> Hlast = Eigen::Matrix<double, 6, 6>::Identity();

    for (int iter = 0; iter < m_cfg.max_iterations; ++iter) {
        iter_done = iter + 1;
        const double corr = corrDistForIter(iter);
        const double corr2 = corr * corr;
        const double gnc_mu = (m_cfg.robust_kernel == RobustKernel::GNC)
                                  ? m_cfg.gnc_mu0 * std::pow(m_cfg.gnc_decay, iter) : 1.0;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
        int used = 0;

        // 单点残差/雅可比累加 (线程局部; 供串行与 OpenMP 共用)
        auto accum = [&](int i,
                         Eigen::Matrix<double, 6, 6>& Hl, Eigen::Matrix<double, 6, 1>& gl, int& ul,
                         std::vector<int>& idx, std::vector<float>& sq,
                         std::vector<Eigen::Vector3d>& nb) {
            const Eigen::Vector3d q = R * bp[i] + t;
            PointType qp;
            qp.x = static_cast<float>(q.x());
            qp.y = static_cast<float>(q.y());
            qp.z = static_cast<float>(q.z());
            if (m_kdtree.nearestKSearch(qp, 5, idx, sq) < 5) return;
            if (sq[4] > corr2) return;

            for (int k = 0; k < 5; ++k) {
                const auto& tp = m_target->points[idx[k]];
                nb[k] = Eigen::Vector3d(tp.x, tp.y, tp.z);
            }

            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto& p : nb) centroid += p;
            centroid /= 5.0;
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (const auto& p : nb) { const Eigen::Vector3d d = p - centroid; cov += d * d.transpose(); }
            cov /= 5.0;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
            const Eigen::Vector3d ev = es.eigenvalues();
            const Eigen::Matrix3d evec = es.eigenvectors();

            Eigen::Vector3d nrm, c;
            const bool is_line = m_cfg.use_edge_feature && ev(2) > 1e-6 &&
                                 (ev(1) / ev(2)) < m_cfg.edge_line_ratio;
            if (is_line) {
                const Eigen::Vector3d line_dir = evec.col(2);
                if (std::fabs(line_dir.z()) < m_cfg.edge_vertical_cos) return;
                double tmin = 1e9, tmax = -1e9, max_perp = 0.0;
                for (const auto& p : nb) {
                    const double tp = line_dir.dot(p - centroid);
                    tmin = std::min(tmin, tp); tmax = std::max(tmax, tp);
                }
                for (const auto& p : nb) {
                    const double tp = line_dir.dot(p - centroid);
                    max_perp = std::max(max_perp, (p - centroid - tp * line_dir).norm());
                }
                if (tmax - tmin < m_cfg.edge_min_extent || max_perp > m_cfg.edge_valid_thr) return;
                c = centroid + line_dir * line_dir.dot(q - centroid);
                Eigen::Vector3d u = q - c;
                const double un = u.norm();
                if (un < 1e-6) return;
                nrm = u / un;
            } else {
                nrm = evec.col(0);
                c = centroid;
                double max_off = 0.0;
                for (const auto& p : nb)
                    max_off = std::max(max_off, std::fabs(nrm.dot(p - centroid)));
                if (max_off > kPlaneValidThr) return;
            }

            const double pd = nrm.dot(q - c);
            const double w = (m_cfg.robust_kernel == RobustKernel::GNC)
                                 ? gncWeight(pd, m_cfg.huber_delta, gnc_mu)
                                 : robustWeight(m_cfg.robust_kernel, pd, m_cfg.huber_delta);
            if (w <= 0.0) return;

            const Eigen::Vector3d a = R * bp[i];
            const Eigen::Matrix<double, 3, 1> j_rot = -skew(a).transpose() * nrm;
            Eigen::Matrix<double, 6, 1> J;
            J.head<3>() = j_rot;
            J.tail<3>() = nrm;

            Hl += w * (J * J.transpose());
            gl += w * J * pd;
            ++ul;
        };

#ifdef _OPENMP
#pragma omp parallel
#endif
        {
            Eigen::Matrix<double, 6, 6> Hl = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> gl = Eigen::Matrix<double, 6, 1>::Zero();
            int ul = 0;
            std::vector<int> idx(5);
            std::vector<float> sq(5);
            std::vector<Eigen::Vector3d> nb(5);
            #pragma omp for schedule(static)
            for (int i = 0; i < n; ++i) accum(i, Hl, gl, ul, idx, sq, nb);
            #pragma omp critical
            { H += Hl; g += gl; used += ul; }
        }

        if (used < m_cfg.min_correspondences) { solved = false; break; }
        solved = true;
        Hlast = H;

        // 退化门控: JtJ 特征分解, 不可观测方向修正置零
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> esH(H);
        const Eigen::Matrix<double, 6, 1> evals = esH.eigenvalues();
        const Eigen::Matrix<double, 6, 6> V = esH.eigenvectors();
        const double maxe = evals.maxCoeff();
        const double mine = evals.minCoeff();
        if (maxe > 1e-12) min_ratio = std::min(min_ratio, std::max(0.0, mine / maxe));

        Eigen::Matrix<double, 6, 1> x = -H.ldlt().solve(g);
        int nde = 0;
        if (m_cfg.degeneracy_gate && maxe > 1e-12) {
            const double cutoff = maxe * m_cfg.degeneracy_eig_ratio;
            Eigen::Matrix<double, 6, 1> coeff = V.transpose() * x;
            for (int k = 0; k < 6; ++k)
                if (evals(k) < cutoff) { coeff(k) = 0.0; ++nde; }
            if (nde > 0) x = V * coeff;
        }
        any_degen = any_degen || (nde > 0);
        max_degen = std::max(max_degen, nde);

        const Eigen::Vector3d dtheta = x.head<3>();
        const Eigen::Vector3d dt = x.tail<3>();
        const double ang = dtheta.norm();
        if (ang > 1e-12) R = Eigen::AngleAxisd(ang, dtheta / ang).toRotationMatrix() * R;
        t += dt;

        if (dtheta.norm() < m_cfg.transformation_epsilon &&
            dt.norm() < m_cfg.transformation_epsilon) { converged = true; break; }
    }

    if (!solved) return false;

    Eigen::Matrix4d Tf = Eigen::Matrix4d::Identity();
    Tf.block<3, 3>(0, 0) = R;
    Tf.block<3, 1>(0, 3) = t;
    out.pose = matrixToPose(Tf, guess.time);

    const CoverageStats cs = evaluateCoverage(m_kdtree, *src, Tf, m_cfg.inlier_dist);
    out.fitness = cs.coverage;
    out.rmse = cs.mean_dist;
    out.inliers = cs.inliers;
    out.correspondences = cs.total;
    out.residual = evaluatePoint2PlaneResidual(m_kdtree, *src, Tf,
                                               m_cfg.max_correspondence_dist);
    out.iterations = iter_done;
    out.degenerate = any_degen;
    out.degenerate_dirs = max_degen;
    out.smallest_eig_ratio = min_ratio;
    out.converged = true;

    // 协方差: JtJ 的伪逆 (奇异方向用大不确定度); 尺度按残差归一
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> esC(Hlast);
    const Eigen::Matrix<double, 6, 1> ce = esC.eigenvalues();
    const double cmax = ce.maxCoeff();
    Eigen::Matrix<double, 6, 1> inv = Eigen::Matrix<double, 6, 1>::Zero();
    const double floor_e = std::max(1e-9, cmax * 1e-6);
    for (int k = 0; k < 6; ++k) inv(k) = (ce(k) > floor_e) ? 1.0 / ce(k) : 1e3 / cmax;
    const double sigma2 = (out.residual > 0) ? out.residual * out.residual : 1.0;
    out.cov = esC.eigenvectors() * inv.asDiagonal() * esC.eigenvectors().transpose() * sigma2;
    return true;
}

double PpIcpMatcher::evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) {
    if (!m_target || m_target->size() < 5) return -1.0;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    return evaluatePoint2PlaneResidual(m_kdtree, *src, poseToMatrix(pose),
                                       m_cfg.max_correspondence_dist);
}

} // namespace rosiwit_slam
