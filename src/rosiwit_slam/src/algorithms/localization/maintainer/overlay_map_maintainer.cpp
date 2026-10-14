// src/algorithms/localization/maintainer/overlay_map_maintainer.cpp
#include "algorithms/localization/maintainer/overlay_map_maintainer.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <pcl/io/pcd_io.h>

namespace rosiwit_slam {

int64_t OverlayMapMaintainer::voxelKey(int cx, int cy, int cz) {
    const int64_t ox = static_cast<int64_t>(cx) & 0x1FFFFF;
    const int64_t oy = static_cast<int64_t>(cy) & 0x1FFFFF;
    const int64_t oz = static_cast<int64_t>(cz) & 0x1FFFFF;
    return (ox << 42) | (oy << 21) | oz;
}

void OverlayMapMaintainer::setBaseMap(const std::string& map_path, const CloudType::Ptr& base) {
    (void)base;
    m_map_path = map_path;
    m_voxel = m_cfg.voxel > 0 ? m_cfg.voxel : 0.3;
    m_cell = m_cfg.cell > 0 ? m_cfg.cell : 10.0;
    // 尝试载入已存在的叠加层 (原图不变, 叠加层单独落盘)
    if (!map_path.empty()) load(map_path + ".overlay.pcd");
}

void OverlayMapMaintainer::addPoint(const Eigen::Vector3d& p) {
    const int vx = static_cast<int>(std::floor(p.x() / m_voxel));
    const int vy = static_cast<int>(std::floor(p.y() / m_voxel));
    const int vz = static_cast<int>(std::floor(p.z() / m_voxel));
    const int64_t vk = voxelKey(vx, vy, vz);
    if (m_occupied.count(vk)) return;   // 体素去重
    m_occupied.insert(vk);

    const int cx = static_cast<int>(std::floor(p.x() / m_cell));
    const int cy = static_cast<int>(std::floor(p.y() / m_cell));
    const int cz = static_cast<int>(std::floor(p.z() / m_cell));
    auto& cell = m_cells[voxelKey(cx, cy, cz)];
    if (!cell) cell.reset(new CloudType());
    PointType q;
    q.x = static_cast<float>(p.x());
    q.y = static_cast<float>(p.y());
    q.z = static_cast<float>(p.z());
    cell->push_back(q);
    ++m_overlay_count;
}

void OverlayMapMaintainer::onLocalized(const PoseStamped& pose, const CloudType::Ptr& scan_body,
                                       const MatchResult& match) {
    if (!m_cfg.enabled || !scan_body || scan_body->empty()) return;
    ++m_frame;
    if (m_cfg.autosave_frames > 0 && !m_map_path.empty() &&
        m_frame % m_cfg.autosave_frames == 0) {
        save(m_map_path + ".overlay.pcd");
    }

    if (match.fitness >= m_cfg.fitness_trigger) { m_low_count = 0; return; }
    if (++m_low_count < m_cfg.min_confirm) return;

    const double moved = m_have_last ? (pose.trans - m_last_add).norm() : 1e9;
    if (moved < m_cfg.min_keyframe_dist) return;

    const int persist = std::max(1, m_cfg.persist_hits);
    for (const auto& p : scan_body->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d w = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        const int vx = static_cast<int>(std::floor(w.x() / m_voxel));
        const int vy = static_cast<int>(std::floor(w.y() / m_voxel));
        const int vz = static_cast<int>(std::floor(w.z() / m_voxel));
        const int64_t vk = voxelKey(vx, vy, vz);
        if (m_occupied.count(vk)) continue;                 // 已并入
        if (++m_hits[vk] >= persist) {                      // 达到持久化阈值才并入
            addPoint(w);
            m_hits.erase(vk);
        }
    }
    m_last_add = pose.trans;
    m_have_last = true;
}

CloudType::Ptr OverlayMapMaintainer::getOverlay(const PoseStamped& pose) {
    CloudType::Ptr out(new CloudType());
    if (m_cells.empty()) return out;
    const double radius = m_cfg.radius > 0 ? m_cfg.radius : 60.0;
    const int span = static_cast<int>(std::ceil(radius / m_cell));
    const int cx = static_cast<int>(std::floor(pose.trans.x() / m_cell));
    const int cy = static_cast<int>(std::floor(pose.trans.y() / m_cell));
    const int cz = static_cast<int>(std::floor(pose.trans.z() / m_cell));
    const double r2 = radius * radius;
    for (int dx = -span; dx <= span; ++dx)
        for (int dy = -span; dy <= span; ++dy)
            for (int dz = -span; dz <= span; ++dz) {
                auto it = m_cells.find(voxelKey(cx + dx, cy + dy, cz + dz));
                if (it == m_cells.end() || !it->second) continue;
                for (const auto& p : it->second->points) {
                    const double ddx = p.x - pose.trans.x();
                    const double ddy = p.y - pose.trans.y();
                    const double ddz = p.z - pose.trans.z();
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) out->push_back(p);
                }
            }
    return out;
}

bool OverlayMapMaintainer::save(const std::string& path) {
    CloudType all;
    for (const auto& kv : m_cells)
        if (kv.second) all += *kv.second;
    if (all.empty()) return false;
    return pcl::io::savePCDFileBinary(path, all) == 0;
}

bool OverlayMapMaintainer::load(const std::string& path) {
    CloudType in;
    if (pcl::io::loadPCDFile(path, in) != 0) return false;
    for (const auto& p : in.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        addPoint(Eigen::Vector3d(p.x, p.y, p.z));
    }
    return m_overlay_count > 0;
}

} // namespace rosiwit_slam
