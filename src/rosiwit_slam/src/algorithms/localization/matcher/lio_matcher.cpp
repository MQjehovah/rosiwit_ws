// src/algorithms/localization/matcher/lio_matcher.cpp
#include "algorithms/localization/matcher/lio_matcher.h"
#include <Eigen/Dense>
#include <cmath>

namespace rosiwit_slam {

void LioMatcher::configure(const ScanMatcherConfig& cfg) {
    GicpMatcher::configure(cfg);   // 无引擎, 只存 m_cfg
}

void LioMatcher::setTarget(const CloudType::Ptr& target) {
    GicpMatcher::setTarget(target);   // m_target (体素降采样) + m_kdtree
    m_normal.clear();
    m_center.clear();
    if (!m_target || m_target->empty()) return;
    const size_t n = m_target->size();
    m_normal.assign(n, Eigen::Vector3f::Zero());
    m_center.assign(n, Eigen::Vector3f::Zero());
    std::vector<int> idx(5);
    std::vector<float> sq(5);
    for (size_t i = 0; i < n; ++i) {
        const PointType& p = (*m_target)[i];
        const Eigen::Vector3f nf(p.normal_x, p.normal_y, p.normal_z);
        if (nf.squaredNorm() > 0.25f) {
            // 地图自带法向: 直接用, 平面过该点
            m_normal[i] = nf.normalized();
            m_center[i] = p.getVector3fMap();
            continue;
        }
        // 无法向: 5 近邻 PCA, 最小特征值方向 = 法向
        if (m_kdtree.nearestKSearch(p, 5, idx, sq) <= 0) continue;
        Eigen::Vector3d mu = Eigen::Vector3d::Zero();
        for (int k = 0; k < 5; ++k) mu += (*m_target)[idx[k]].getVector3fMap().cast<double>();
        mu /= 5.0;
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int k = 0; k < 5; ++k) {
            const Eigen::Vector3d d = (*m_target)[idx[k]].getVector3fMap().cast<double>() - mu;
            cov.noalias() += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        const Eigen::Vector3d lam = es.eigenvalues();   // 升序
        const double sum = lam.sum();
        if (!(sum > 1e-12) || lam(0) / sum > m_plane_max_curv) continue;   // 不是平面
        m_normal[i] = es.eigenvectors().col(0).cast<float>();
        m_center[i] = mu.cast<float>();
    }
}

bool LioMatcher::align(const CloudType::Ptr& src, const Eigen::Matrix4d& guess_T,
                       Eigen::Matrix4d& T_out, int& iterations) {
    if (!m_target || m_target->empty() || m_normal.size() != m_target->size()) return false;
    iterations = 0;
    const size_t n = src->size();
    if (n < 10) return false;
    // 对齐用源点: 步进抽样到 <= m_solve_max (评分仍用全量点, 见 match())
    const size_t stride =
        (m_solve_max > 0 && n > static_cast<size_t>(m_solve_max))
            ? (n + static_cast<size_t>(m_solve_max) - 1) / static_cast<size_t>(m_solve_max)
            : 1;
    const double s2 = m_sigma_r * m_sigma_r;
    const double max_corr2 =
        m_cfg.max_correspondence_dist > 0 ? m_cfg.max_correspondence_dist *
                                            m_cfg.max_correspondence_dist
                                          : 1e9;
    const double pr = 1.0 / (m_prior_rot * m_prior_rot);
    const double pt = 1.0 / (m_prior_trans * m_prior_trans);
    const Eigen::Matrix4d Tg_inv = guess_T.inverse();

    Eigen::Matrix4d T = guess_T;
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    for (int it = 0; it < m_max_iter; ++it) {
        ++iterations;
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const Eigen::Vector3d t = T.block<3, 1>(0, 3);
        // 先验: 当前累计修正量 δ_cur = Log(T · T_guess⁻¹) (iEKF: 先验始终约束总修正)
        const Eigen::Matrix4d D = T * Tg_inv;
        const Eigen::AngleAxisd aa(D.block<3, 3>(0, 0));
        const Eigen::Vector3d dth = aa.axis() * aa.angle();
        const Eigen::Vector3d dtr = D.block<3, 1>(0, 3);
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        H(0, 0) = H(1, 1) = H(2, 2) = pr;
        H(3, 3) = H(4, 4) = H(5, 5) = pt;
        b.head<3>() = -pr * dth;
        b.tail<3>() = -pt * dtr;
        for (size_t i = 0; i < n; i += stride) {
            const Eigen::Vector3d q =
                (R * (*src)[i].getVector3fMap().cast<double>() + t).eval();
            PointType qp;
            qp.x = static_cast<float>(q.x());
            qp.y = static_cast<float>(q.y());
            qp.z = static_cast<float>(q.z());
            if (m_kdtree.nearestKSearch(qp, 1, idx, sq) <= 0) continue;
            if (sq[0] > max_corr2) continue;
            const int j = idx[0];
            const Eigen::Vector3d nj = m_normal[j].cast<double>();
            if (nj.squaredNorm() < 0.5) continue;   // 无有效平面
            const double r = nj.dot(q - m_center[j].cast<double>());
            Eigen::Matrix<double, 6, 1> J;
            J.head<3>() = q.cross(nj);   // ∂r/∂δθ (左扰动: δq = δθ × q + δt)
            J.tail<3>() = nj;            // ∂r/∂δt
            const double w = 1.0 / (1.0 + r * r / s2);   // Cauchy
            H.noalias() += w * (J * J.transpose());
            b.noalias() -= w * r * J;
        }
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(H);
        if (ldlt.info() != Eigen::Success) return false;
        const Eigen::Matrix<double, 6, 1> dx = ldlt.solve(b);
        if (!dx.allFinite()) return false;
        const double dt = dx.tail<3>().norm();
        const double dr = dx.head<3>().norm();
        Eigen::Matrix4d U = Eigen::Matrix4d::Identity();   // T ← Exp(δx) · T
        if (dr > 1e-12) U.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(dr, dx.head<3>().normalized()).toRotationMatrix();
        U.block<3, 1>(0, 3) = dx.tail<3>();
        T = U * T;
        if (dt < m_conv_t && dr < m_conv_r) break;
    }
    T_out = T;
    return true;
}

} // namespace rosiwit_slam
