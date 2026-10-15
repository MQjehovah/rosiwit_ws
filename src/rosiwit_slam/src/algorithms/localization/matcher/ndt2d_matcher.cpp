// src/algorithms/localization/matcher/ndt2d_matcher.cpp
#include "algorithms/localization/matcher/ndt2d_matcher.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {

int64_t Ndt2dMatcher::key(int cx, int cy) {
    const int64_t ix = static_cast<int64_t>(cx) & 0xFFFFFFFFLL;
    const int64_t iy = static_cast<int64_t>(cy) & 0xFFFFFFFFLL;
    return (ix << 32) | iy;
}

void Ndt2dMatcher::buildLevel(Level& lv, const std::vector<Eigen::Vector2d>& pts) {
    struct Acc { Eigen::Vector2d sum = Eigen::Vector2d::Zero();
                 Eigen::Matrix2d sq = Eigen::Matrix2d::Zero(); int n = 0; };
    std::unordered_map<int64_t, Acc> acc;
    for (const auto& p : pts) {
        const int cx = static_cast<int>(std::floor(p.x() / lv.cell));
        const int cy = static_cast<int>(std::floor(p.y() / lv.cell));
        Acc& a = acc[key(cx, cy)];
        a.sum += p;
        a.sq += p * p.transpose();
        ++a.n;
    }
    lv.cells.clear();
    for (auto& kv : acc) {
        if (kv.second.n < 1) continue;
        Cell c;
        c.mean = kv.second.sum / kv.second.n;
        Eigen::Matrix2d cov = kv.second.sq / kv.second.n - c.mean * c.mean.transpose();
        cov += Eigen::Matrix2d::Identity() * (0.25 * lv.cell * lv.cell);  // 随格尺寸正则
        c.cov_inv = cov.inverse();
        c.n = kv.second.n;
        lv.cells[kv.first] = c;
    }
}

const Ndt2dMatcher::Cell* Ndt2dMatcher::findCell(const Level& lv, double x, double y) const {
    const int cx = static_cast<int>(std::floor(x / lv.cell));
    const int cy = static_cast<int>(std::floor(y / lv.cell));
    auto it = lv.cells.find(key(cx, cy));
    return it == lv.cells.end() ? nullptr : &it->second;
}

void Ndt2dMatcher::setTarget(const CloudType::Ptr& target) {
    const double fine = m_cfg.ndt2d_cell > 0 ? m_cfg.ndt2d_cell : 0.5;
    m_zmin = m_cfg.ndt2d_z_min;
    m_zmax = m_cfg.ndt2d_z_max;
    m_levels.clear();

    CloudType::Ptr pts = filterFinite(target);
    if (!pts) return;
    std::vector<Eigen::Vector2d> p2;
    p2.reserve(pts->size());
    for (const auto& p : pts->points) {
        if (p.z < m_zmin || p.z > m_zmax) continue;
        p2.emplace_back(p.x, p.y);
    }
    if (p2.size() < 20) return;

    // 单层 (NDT 在周期性平面上多分辨率易被"格周期"误导; 保持与格同尺度)
    for (double f : {1.0}) {
        Level lv;
        lv.cell = fine * f;
        buildLevel(lv, p2);
        if (!lv.cells.empty()) m_levels.push_back(std::move(lv));
    }
}

bool Ndt2dMatcher::match(const CloudType::Ptr& scan, const PoseStamped& guess,
                         MatchResult& out) {
    out = MatchResult();
    if (m_levels.empty()) return false;
    const double fine = m_levels.back().cell;
    CloudType::Ptr src = downsampleCloud(filterFinite(scan), fine);
    if (!src || src->empty()) return false;

    std::vector<Eigen::Vector2d> sp;
    sp.reserve(src->size());
    for (const auto& p : src->points) {
        if (p.z < m_zmin || p.z > m_zmax) continue;
        sp.emplace_back(p.x, p.y);
    }
    if (sp.size() < 10) return false;

    double x = guess.trans.x(), y = guess.trans.y();
    const double yaw0 = std::atan2(guess.rot(1, 0), guess.rot(0, 0));
    double yaw = yaw0;

    // 由粗到细逐层 Gauss-Newton
    for (const Level& lv : m_levels) {
        for (int iter = 0; iter < m_cfg.max_iterations; ++iter) {
            const double c = std::cos(yaw), s = std::sin(yaw);
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            Eigen::Vector3d g = Eigen::Vector3d::Zero();
            int used = 0;
            for (const auto& p : sp) {
                const Eigen::Vector2d q(c * p.x() - s * p.y() + x, s * p.x() + c * p.y() + y);
                const Cell* cell = findCell(lv, q.x(), q.y());
                if (!cell) continue;
                const Eigen::Vector2d r = q - cell->mean;
                const double m2 = r.dot(cell->cov_inv * r);
                const double w = robustWeight(m_cfg.robust_kernel, std::sqrt(std::max(0.0, m2)),
                                              m_cfg.huber_delta);
                if (w <= 0.0) continue;
                Eigen::Matrix<double, 2, 3> J;
                J << 1.0, 0.0, -s * p.x() - c * p.y(),
                     0.0, 1.0,  c * p.x() - s * p.y();
                H += w * (J.transpose() * cell->cov_inv * J);
                g += w * (J.transpose() * cell->cov_inv * r);
                ++used;
            }
            if (used < m_cfg.min_correspondences) break;
            Eigen::Vector3d dx = -H.ldlt().solve(g);
            x += dx(0); y += dx(1); yaw += dx(2);
            if (std::fabs(dx(0)) < m_cfg.transformation_epsilon &&
                std::fabs(dx(1)) < m_cfg.transformation_epsilon &&
                std::fabs(dx(2)) < m_cfg.transformation_epsilon) break;
        }
    }

    // 覆盖率 (细层)
    const Level& fl = m_levels.back();
    const double c = std::cos(yaw), s = std::sin(yaw);
    int inl = 0, tot = 0;
    for (const auto& p : sp) {
        const Eigen::Vector2d q(c * p.x() - s * p.y() + x, s * p.x() + c * p.y() + y);
        const Cell* cell = findCell(fl, q.x(), q.y());
        ++tot;
        if (cell && (q - cell->mean).norm() <= m_cfg.inlier_dist) ++inl;
    }

    PoseStamped res = guess;
    res.trans.x() = x; res.trans.y() = y;
    const M3D Rz0 = Eigen::AngleAxisd(yaw0, V3D::UnitZ()).toRotationMatrix();
    const M3D Rrp = Rz0.transpose() * guess.rot;   // guess 的 roll/pitch
    res.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix() * Rrp;
    out.pose = res;
    out.fitness = tot > 0 ? static_cast<double>(inl) / tot : 0.0;
    out.inliers = inl;
    out.correspondences = tot;
    out.iterations = m_cfg.max_iterations;
    out.converged = true;
    return true;
}

} // namespace rosiwit_slam
