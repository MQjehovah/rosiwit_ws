// src/algorithms/localization/constraints/planar_constraint.cpp
#include "algorithms/localization/constraints/planar_constraint.h"
#include <cmath>
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

void PlanarConstraint::apply(PoseStamped& pose) {
    if (m_cfg.mode == "none") return;

    if (m_cfg.mode == "lock_zrp") {
        const double yaw = std::atan2(pose.rot(1, 0), pose.rot(0, 0));
        pose.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix();
        pose.trans.z() = m_cfg.z;
        return;
    }

    if (m_cfg.mode != "ground_plane") return;
    const Cell* c = query(pose.trans.x(), pose.trans.y());
    if (!c) return;

    if (!m_have_sensor_h) {
        m_sensor_h = m_cfg.sensor_height > 0 ? m_cfg.sensor_height
                                             : (pose.trans.z() - c->z);  // 首帧标定
        m_have_sensor_h = true;
    }
    pose.trans.z() = c->z + m_sensor_h;

    // roll/pitch: 把机体上轴对齐到地图地面法向 (平地=竖直 -> 锁平; 坡道=法向倾斜 -> 跟随)
    const Eigen::Vector3d up = pose.rot * V3D::UnitZ();
    Eigen::Quaterniond qa;
    qa.setFromTwoVectors(up, c->n);
    pose.rot = qa.toRotationMatrix() * pose.rot;
}

} // namespace rosiwit_slam
