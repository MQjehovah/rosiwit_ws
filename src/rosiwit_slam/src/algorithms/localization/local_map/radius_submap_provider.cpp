// src/algorithms/localization/local_map/radius_submap_provider.cpp
#include "algorithms/localization/local_map/radius_submap_provider.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>

namespace rosiwit_slam {

int64_t RadiusSubmapProvider::cellKey(int cx, int cy, int cz) {
    // 21 位/轴 (约 ±1000 万格), 避免哈希冲突
    const int64_t ox = static_cast<int64_t>(cx) & 0x1FFFFF;
    const int64_t oy = static_cast<int64_t>(cy) & 0x1FFFFF;
    const int64_t oz = static_cast<int64_t>(cz) & 0x1FFFFF;
    return (ox << 42) | (oy << 21) | oz;
}

void RadiusSubmapProvider::setMap(const CloudType::Ptr& global_map) {
    m_voxel = m_cfg.voxel_resolution > 0 ? m_cfg.voxel_resolution : 0.3;
    m_cell  = m_cfg.cell_size > 0 ? m_cfg.cell_size : 10.0;
    m_cells.clear();
    m_map = downsampleCloud(filterFinite(global_map), m_voxel);
    if (!m_map) { m_map.reset(new CloudType()); return; }

    for (const auto& p : m_map->points) {
        const int cx = static_cast<int>(std::floor(p.x / m_cell));
        const int cy = static_cast<int>(std::floor(p.y / m_cell));
        const int cz = static_cast<int>(std::floor(p.z / m_cell));
        auto& cell = m_cells[cellKey(cx, cy, cz)];
        if (!cell) cell.reset(new CloudType());
        cell->push_back(p);
    }
}

CloudType::Ptr RadiusSubmapProvider::getLocalMap(const PoseStamped& pose) {
    CloudType::Ptr out(new CloudType());
    if (m_cells.empty()) return out;
    const double radius = m_cfg.radius > 0 ? m_cfg.radius : 60.0;
    const int span = static_cast<int>(std::ceil(radius / m_cell));
    const int cx = static_cast<int>(std::floor(pose.trans.x() / m_cell));
    const int cy = static_cast<int>(std::floor(pose.trans.y() / m_cell));
    const int cz = static_cast<int>(std::floor(pose.trans.z() / m_cell));
    const double r2 = radius * radius;
    const Eigen::Vector3d center = pose.trans;

    for (int dx = -span; dx <= span; ++dx)
        for (int dy = -span; dy <= span; ++dy)
            for (int dz = -span; dz <= span; ++dz) {
                auto it = m_cells.find(cellKey(cx + dx, cy + dy, cz + dz));
                if (it == m_cells.end() || !it->second) continue;
                for (const auto& p : it->second->points) {
                    const double ddx = p.x - center.x();
                    const double ddy = p.y - center.y();
                    const double ddz = p.z - center.z();
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) out->push_back(p);
                }
            }
    return out;
}

} // namespace rosiwit_slam
