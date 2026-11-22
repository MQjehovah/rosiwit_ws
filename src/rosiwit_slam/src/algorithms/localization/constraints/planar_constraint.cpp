// src/algorithms/localization/constraints/planar_constraint.cpp
#include "algorithms/localization/constraints/planar_constraint.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace rosiwit_slam {

int64_t PlanarConstraint::key(int cx, int cy) {
    return (static_cast<int64_t>(cx) << 32) ^ (static_cast<unsigned int>(cy));
}

void PlanarConstraint::setMap(const CloudType::Ptr& base_map) {
    m_cells.clear();
    m_have_sensor_h = false;
    if (!base_map || base_map->empty()) return;
    m_cell = m_cfg.cell > 0 ? m_cfg.cell : 1.0;

    // 1) 每格最低 z
    std::unordered_map<int64_t, double> minz;
    minz.reserve(base_map->size());
    for (const auto& p : base_map->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int cx = static_cast<int>(std::floor(p.x / m_cell));
        const int cy = static_cast<int>(std::floor(p.y / m_cell));
        const int64_t k = key(cx, cy);
        auto it = minz.find(k);
        if (it == minz.end()) minz[k] = p.z;
        else if (p.z < it->second) it->second = p.z;
    }

    // 2) 收集每格地面带点, 求均值 z + 法向(PCA 最小特征向量)
    struct Acc { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); Eigen::Matrix3d sq = Eigen::Matrix3d::Zero(); int n = 0; };
    std::unordered_map<int64_t, Acc> acc;
    const double band = m_cfg.ground_band > 0 ? m_cfg.ground_band : 0.3;
    for (const auto& p : base_map->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int cx = static_cast<int>(std::floor(p.x / m_cell));
        const int cy = static_cast<int>(std::floor(p.y / m_cell));
        const int64_t k = key(cx, cy);
        auto mit = minz.find(k);
        if (mit == minz.end() || p.z > mit->second + band) continue;
        Acc& a = acc[k];
        const Eigen::Vector3d v(p.x, p.y, p.z);
        a.sum += v;
        a.sq += v * v.transpose();
        ++a.n;
    }

    for (auto& kv : acc) {
        Cell c;
        if (kv.second.n < 3) { c.ok = false; m_cells[kv.first] = c; continue; }
        const Eigen::Vector3d mean = kv.second.sum / kv.second.n;
        Eigen::Matrix3d cov = kv.second.sq / kv.second.n - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Vector3d n = es.eigenvectors().col(0);
        if (n.z() < 0) n = -n;               // 朝上
        c.z = mean.z();
        c.n = n;
        c.ok = (n.z() > std::cos(m_cfg.max_tilt_deg * M_PI / 180.0));
        m_cells[kv.first] = c;
    }
}

const PlanarConstraint::Cell* PlanarConstraint::query(double x, double y) const {
    const int cx = static_cast<int>(std::floor(x / m_cell));
    const int cy = static_cast<int>(std::floor(y / m_cell));
    // 就近搜索 3x3
    const Cell* best = nullptr;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
            auto it = m_cells.find(key(cx + dx, cy + dy));
            if (it == m_cells.end() || !it->second.ok) continue;
            if (!best) best = &it->second;
        }
    return best;
}

void PlanarConstraint::apply(PoseStamped& pose, const CloudType::Ptr& scan) {
    if (m_cfg.mode == "none") return;

    if (m_cfg.mode == "lock_zrp") {
        const double yaw = std::atan2(pose.rot(1, 0), pose.rot(0, 0));
        pose.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
        pose.trans.z() = m_cfg.z;
        return;
    }

    if (m_cfg.mode == "scan_plane") {
        applyScanPlane(pose, scan);
        return;
    }

    if (m_cfg.mode != "ground_plane") return;

    double gz = 0.0;
    Eigen::Vector3d gn = Eigen::Vector3d::UnitZ();
    bool have = false;
    if (m_store) {
        if (m_store->groundAt(pose.trans.x(), pose.trans.y(), gz, gn)) {
            if (gn.z() < std::cos(m_cfg.max_tilt_deg * M_PI / 180.0)) return;  // 倾角超限
            have = true;
        }
    }
    if (!have) {
        const Cell* c = query(pose.trans.x(), pose.trans.y());
        if (!c) return;
        gz = c->z; gn = c->n; have = true;
    }

    if (!m_have_sensor_h) {
        m_sensor_h = m_cfg.sensor_height > 0 ? m_cfg.sensor_height
                                             : (pose.trans.z() - gz);  // 首帧标定
        m_have_sensor_h = true;
    }
    pose.trans.z() = gz + m_sensor_h;

    // roll/pitch: 把机体上轴对齐到地面法向 (平地=竖直 -> 锁平; 坡道=法向倾斜 -> 跟随)
    const Eigen::Vector3d up = pose.rot * V3D::UnitZ();
    Eigen::Quaterniond qa;
    qa.setFromTwoVectors(up, gn);
    pose.rot = qa.toRotationMatrix() * pose.rot;
}

void PlanarConstraint::applyScanPlane(PoseStamped& pose, const CloudType::Ptr& scan) {
    if (!scan || scan->size() < 200) return;

    // 按 z 分位取地板/天花板带 (小房间里天花板通常比地板点更多)
    std::vector<float> zs;
    zs.reserve(scan->size());
    for (const auto& p : scan->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        zs.push_back(p.z);
    }
    if (zs.size() < 200) return;
    const size_t lo_i = zs.size() / 20;             // 5%
    const size_t hi_i = zs.size() * 19 / 20;        // 95%
    std::nth_element(zs.begin(), zs.begin() + lo_i, zs.end());
    const float zlo = zs[lo_i];
    std::nth_element(zs.begin(), zs.begin() + hi_i, zs.end());
    const float zhi = zs[hi_i];

    auto fitBand = [&](double lo, double hi, Eigen::Vector3d& n, int& cnt,
                       Eigen::Vector3d& centroid) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d sq = Eigen::Matrix3d::Zero();
        cnt = 0;
        for (const auto& p : scan->points) {
            if (!std::isfinite(p.z) || p.z < lo || p.z > hi) continue;
            const Eigen::Vector3d v(p.x, p.y, p.z);
            sum += v;
            sq += v * v.transpose();
            ++cnt;
        }
        if (cnt < 50) return false;
        const Eigen::Vector3d mean = sum / static_cast<double>(cnt);
        Eigen::Matrix3d cov = sq / static_cast<double>(cnt) - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        if (es.info() != Eigen::Success) return false;
        n = es.eigenvectors().col(0);
        if (n.z() < 0) n = -n;   // 统一朝上
        centroid = mean;
        return true;
    };

    Eigen::Vector3d nf, nc;
    Eigen::Vector3d meanf = Eigen::Vector3d::Zero(), meanc = Eigen::Vector3d::Zero();
    int cf = 0, cc = 0;
    const bool okf = fitBand(zlo - 0.2, zlo + 0.35, nf, cf, meanf);
    const bool okc = fitBand(zhi - 0.35, zhi + 0.2, nc, cc, meanc);
    Eigen::Vector3d up;
    const bool use_floor = okf && (!okc || cf + 100 >= cc);   // 地板更可靠时优先
    if (okc && !use_floor) up = nc;
    else if (okf) up = nf;
    else return;

    const double cos_tilt = std::cos(m_cfg.max_tilt_deg * M_PI / 180.0);
    if (up.z() < cos_tilt) return;   // 本身不是一个可靠的水平面

    // 位姿把它转到 map 系后, 还应该保持竖直
    const Eigen::Vector3d n_m = pose.rot * up;
    if (n_m.z() < cos_tilt) return;
    const double ang = std::acos(std::min(1.0, std::max(-1.0, n_m.z())));
    if (ang * 180.0 / M_PI > m_cfg.max_correction_deg) return;   // 跳变: 不信
    if (ang >= 1e-4) {
        Eigen::Vector3d axis = n_m.cross(Eigen::Vector3d::UnitZ());
        const double s = axis.norm();
        if (s < 1e-9) return;
        axis /= s;
        pose.rot = Eigen::AngleAxisd(ang, axis).toRotationMatrix() * pose.rot;
    }

    // z: 只要"地板可靠"就修, 与"转动用哪个面"无关 (小房间天花板常比地面点多)
    auto logZ = [&](const char* what, double gz, double F, double dz) {
        static int n = 0;
        if (++n % 10 != 0) return;
        std::cout.setf(std::ios::fixed);
        std::cout.precision(3);
        std::cout << "[planar] " << what
                  << " zlo=" << zlo << " zhi=" << zhi
                  << " cf=" << cf << " cc=" << cc
                  << " tilt=" << (up.z() >= cos_tilt ? "ok" : "bad") << " floor=" << (use_floor ? 1 : 0)
                  << " gz=" << gz << " F=" << F << " cur=" << pose.trans.z()
                  << " dz=" << dz << std::endl;
        std::cout.unsetf(std::ios::fixed);
    };

    if (!okf || cf < 50 || nf.z() < cos_tilt) {
        logZ("z-skip", 0.0, 0.0, 0.0);
        return;
    }
    double gz = 0.0;
    bool have_g = false;
    if (m_store) {
        Eigen::Vector3d gn;
        if (m_store->groundAt(pose.trans.x(), pose.trans.y(), gz, gn)) have_g = true;
    }
    if (!have_g) {
        const Cell* c = query(pose.trans.x(), pose.trans.y());
        if (c) { gz = c->z; have_g = true; }
    }
    if (!have_g) {
        logZ("z-noground", 0.0, 0.0, 0.0);
        return;
    }

    // 把地板平面转到 map 系, 求它在机器人 (x,y) 处的高度 F。
    // 用 F 与地图地面 gz 对齐, 不需要"传感器离地高"这种假设量(实机该量会随
    // 安装/外参变化, 且符号极易搞反): 点云地板落到地图地面上, z 就是对的。
    const Eigen::Vector3d c_m = pose.rot * meanf + pose.trans;
    const Eigen::Vector3d n_m_f = pose.rot * nf;
    if (n_m_f.z() < cos_tilt) return;   // 地板在 map 系也应水平
    const double F = c_m.z() - (n_m_f.x() * (pose.trans.x() - c_m.x()) +
                                n_m_f.y() * (pose.trans.y() - c_m.y())) / n_m_f.z();

    const double dz_target = gz - F;
    if (!std::isfinite(dz_target) || std::fabs(dz_target) > 5.0) {
        logZ("z-jump", gz, F, dz_target);
        return;   // 超过 5m 一定是数据异常(别把车甩飞)
    }
    const double max_dz = 0.3;
    double dz = dz_target;
    if (dz > max_dz) dz = max_dz;
    if (dz < -max_dz) dz = -max_dz;
    logZ("z-apply", gz, F, dz);
    pose.trans.z() += dz;
}

} // namespace rosiwit_slam
