// src/algorithms/localization/matcher/vgicp_matcher.cpp
#include "algorithms/localization/matcher/vgicp_matcher.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/registration_metrics.h"
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {

namespace {
constexpr double kPlanarThr = 0.20;  // λmin/λmax 低于此值视为平面体素
inline Eigen::Matrix3d skew(const Eigen::Vector3d& a) {
    Eigen::Matrix3d S;
    S <<     0.0, -a.z(),  a.y(),
         a.z(),      0.0, -a.x(),
        -a.y(),  a.x(),     0.0;
    return S;
}
} // namespace

int64_t VgicpMatcher::voxelKey(int cx, int cy, int cz) {
    const int64_t ox = static_cast<int64_t>(cx) & 0x1FFFFF;
    const int64_t oy = static_cast<int64_t>(cy) & 0x1FFFFF;
    const int64_t oz = static_cast<int64_t>(cz) & 0x1FFFFF;
    return (ox << 42) | (oy << 21) | oz;
}

VgicpMatcher::Voxel* VgicpMatcher::findVoxel(const Eigen::Vector3d& p) {
    const int cx = static_cast<int>(std::floor(p.x() / m_leaf));
    const int cy = static_cast<int>(std::floor(p.y() / m_leaf));
    const int cz = static_cast<int>(std::floor(p.z() / m_leaf));
    auto it = m_voxels.find(voxelKey(cx, cy, cz));
    return (it == m_voxels.end()) ? nullptr : &it->second;
}

double VgicpMatcher::corrDistForIter(int iter) const {
    if (!m_cfg.use_corr_decay || m_cfg.max_iterations <= 1) return m_cfg.corr_dist_init;
    const double t = static_cast<double>(iter) / (m_cfg.max_iterations - 1);
    double d = m_cfg.corr_dist_init * (1.0 - t) + m_cfg.corr_dist_min * t;
    return std::max(d, m_cfg.corr_dist_min);
}

void VgicpMatcher::setTarget(const CloudType::Ptr& target) {
    // 体素叶片取配置分辨率的 1.5 倍, 保证每体素聚合足够点以稳定拟合平面
    m_leaf = (m_cfg.voxel_resolution > 0 ? m_cfg.voxel_resolution : 0.5) * 1.5;
    CloudType::Ptr full = filterFinite(target);
    // 覆盖率评估用与源同分辨率的降采样副本
    m_target = downsampleCloud(full, m_cfg.voxel_resolution > 0 ? m_cfg.voxel_resolution : 0.5);
    if (!m_target) m_target.reset(new CloudType());
    m_kdtree.setInputCloud(m_target);

    // 体素由全分辨率目标聚合; 每体素拟合一平面, 存均值 + 法向(最小特征向量)
    m_voxels.clear();
    for (const auto& p : full->points) {
        const Eigen::Vector3d v(p.x, p.y, p.z);
        const int cx = static_cast<int>(std::floor(p.x / m_leaf));
        const int cy = static_cast<int>(std::floor(p.y / m_leaf));
        const int cz = static_cast<int>(std::floor(p.z / m_leaf));
        Voxel& vox = m_voxels[voxelKey(cx, cy, cz)];
        vox.sum += v;
        vox.sum_sq += v * v.transpose();
        ++vox.n;
    }
    for (auto& kv : m_voxels) {
        Voxel& v = kv.second;
        if (v.n < 3) continue;
        v.mean = v.sum / v.n;
        Eigen::Matrix3d cov = v.sum_sq / v.n - v.mean * v.mean.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        const Eigen::Vector3d ev = es.eigenvalues();       // 升序
        if (ev(2) < 1e-8) continue;
        v.normal = es.eigenvectors().col(0);
        v.valid = (ev(0) / ev(2)) < kPlanarThr;   // 仅接受平面体素(法向可靠)
    }
}

bool VgicpMatcher::match(const CloudType::Ptr& scan, const PoseStamped& guess,
                         MatchResult& out) {
    out = MatchResult();
    if (m_voxels.empty()) return false;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    if (!src || src->empty()) return false;
    if (m_cfg.dynamic_filter && m_cfg.dynamic_dist > 0) {
        CloudType::Ptr f(new CloudType());
        f->reserve(src->size());
        const Eigen::Matrix4d guess_T = poseToMatrix(guess);
        const Eigen::Matrix3d R = guess_T.block<3, 3>(0, 0);
        const Eigen::Vector3d t = guess_T.block<3, 1>(0, 3);
        for (const auto& p : src->points) {
            const Eigen::Vector3d q = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
            Voxel* v = findVoxel(q);
            if (v && v->valid) f->push_back(p);
        }
        src = f;
        if (src->empty()) return false;
    }

    const int n = static_cast<int>(src->size());
    std::vector<Eigen::Vector3d> bp(n);
    for (int i = 0; i < n; ++i) {
        const auto& p = src->points[i];
        bp[i] = Eigen::Vector3d(p.x, p.y, p.z);
    }

    Eigen::Matrix3d R = guess.rot;
    Eigen::Vector3d t = guess.trans;

    bool solved = false, any_degen = false;
    int max_degen = 0, iter_done = 0;
    double min_ratio = 1.0;
    Eigen::Matrix<double, 6, 6> Hlast = Eigen::Matrix<double, 6, 6>::Identity();

    for (int iter = 0; iter < m_cfg.max_iterations; ++iter) {
        iter_done = iter + 1;
        const double corr = corrDistForIter(iter);
        const double corr2 = corr * corr;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
        int used = 0;

        auto accum = [&](int i, Eigen::Matrix<double, 6, 6>& Hl,
                         Eigen::Matrix<double, 6, 1>& gl, int& ul) {
            const Eigen::Vector3d q = R * bp[i] + t;
            Voxel* v = findVoxel(q);
            if (!v || !v->valid) return;
            const Eigen::Vector3d r = q - v->mean;
            if (r.squaredNorm() > corr2) return;

            const Eigen::Vector3d& nrm = v->normal;
            const double pd = nrm.dot(r);
            const double w = robustWeight(m_cfg.robust_kernel, pd, m_cfg.huber_delta);
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
            #pragma omp for schedule(static)
            for (int i = 0; i < n; ++i) accum(i, Hl, gl, ul);
            #pragma omp critical
            { H += Hl; g += gl; used += ul; }
        }
        if (used < m_cfg.min_correspondences) {
            if (!solved) return false;   // 从未获得足够对应, 判定失败
            break;                        // 后续对应塌缩: 保留上一次有效位姿
        }
        solved = true;
        Hlast = H;

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

        Eigen::Vector3d dtheta = x.head<3>();
        Eigen::Vector3d dt = x.tail<3>();
        const double max_trans = 1.0, max_rot = 0.5;   // 单步限幅, 防发散
        if (dt.norm() > max_trans) dt = dt.normalized() * max_trans;
        if (dtheta.norm() > max_rot) dtheta = dtheta.normalized() * max_rot;
        const double ang = dtheta.norm();
        if (ang > 1e-12) R = Eigen::AngleAxisd(ang, dtheta / ang).toRotationMatrix() * R;
        t += dt;

        if (dtheta.norm() < m_cfg.transformation_epsilon &&
            dt.norm() < m_cfg.transformation_epsilon) break;
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

double VgicpMatcher::evaluateResidual(const CloudType::Ptr& scan, const PoseStamped& pose) {
    if (m_voxels.empty()) return -1.0;
    CloudType::Ptr src = downsampleCloud(scan, m_cfg.voxel_resolution);
    return evaluatePoint2PlaneResidual(m_kdtree, *src, poseToMatrix(pose),
                                       m_cfg.max_correspondence_dist);
}

} // namespace rosiwit_slam
