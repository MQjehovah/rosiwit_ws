// src/algorithms/map_store/voxel_map_store.cpp
#include "algorithms/map_store/voxel_map_store.h"
#include <cmath>
#include <algorithm>
#include <pcl/io/pcd_io.h>

namespace rosiwit_slam {

int64_t VoxelMapStore::key(int cx, int cy, int cz) {
    const int64_t ix = static_cast<int64_t>(cx) & 0x1FFFFF;
    const int64_t iy = static_cast<int64_t>(cy) & 0x1FFFFF;
    const int64_t iz = static_cast<int64_t>(cz) & 0x1FFFFF;
    return (ix << 42) | (iy << 21) | iz;
}

int64_t VoxelMapStore::keyOf(double x, double y, double z) const {
    return key(static_cast<int>(std::floor(x / m_leaf)),
               static_cast<int>(std::floor(y / m_leaf)),
               static_cast<int>(std::floor(z / m_leaf)));
}

bool VoxelMapStore::buildFromCloud(const CloudType::Ptr& cloud) {
    m_cells.clear();
    m_size = 0;
    ++m_rev;
    insert(cloud);
    return !m_cells.empty();
}

void VoxelMapStore::insert(const CloudType::Ptr& world_points) {
    if (!world_points) return;
    for (const auto& p : world_points->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        auto& cell = m_cells[keyOf(p.x, p.y, p.z)];
        if (!cell) cell.reset(new CloudType());
        cell->push_back(p);
        ++m_size;
    }
    ++m_rev;
}

CloudType::Ptr VoxelMapStore::queryRadius(const Eigen::Vector3d& center, double radius) const {
    CloudType::Ptr out(new CloudType());
    if (m_cells.empty()) return out;
    const int span = static_cast<int>(std::ceil(radius / m_leaf)) + 1;
    const int cx = static_cast<int>(std::floor(center.x() / m_leaf));
    const int cy = static_cast<int>(std::floor(center.y() / m_leaf));
    const int cz = static_cast<int>(std::floor(center.z() / m_leaf));
    const double r2 = radius * radius;
    for (int dx = -span; dx <= span; ++dx)
        for (int dy = -span; dy <= span; ++dy)
            for (int dz = -span; dz <= span; ++dz) {
                auto it = m_cells.find(key(cx + dx, cy + dy, cz + dz));
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

int VoxelMapStore::nearestK(const PointType& query, int k,
                            std::vector<PointType>& points, std::vector<float>& sqdists) const {
    points.clear(); sqdists.clear();
    if (m_cells.empty() || k <= 0) return 0;
    const int qcx = static_cast<int>(std::floor(query.x / m_leaf));
    const int qcy = static_cast<int>(std::floor(query.y / m_leaf));
    const int qcz = static_cast<int>(std::floor(query.z / m_leaf));

    std::vector<std::pair<float, PointType>> found;
    const int max_ring = 6;
    for (int ring = 1; ring <= max_ring; ++ring) {
        found.clear();
        for (int dx = -ring; dx <= ring; ++dx)
            for (int dy = -ring; dy <= ring; ++dy)
                for (int dz = -ring; dz <= ring; ++dz) {
                    // 只统计新扩的一层, 避免重复; 简化: 每层重算外层壳即可, 这里全取
                    auto it = m_cells.find(key(qcx + dx, qcy + dy, qcz + dz));
                    if (it == m_cells.end() || !it->second) continue;
                    for (const auto& p : it->second->points) {
                        const float ddx = p.x - query.x, ddy = p.y - query.y, ddz = p.z - query.z;
                        found.emplace_back(ddx * ddx + ddy * ddy + ddz * ddz, p);
                    }
                }
        if (static_cast<int>(found.size()) >= k) {
            // 已找到 k 个; 再扩一圈确保边界无更近点
            std::sort(found.begin(), found.end(),
                      [](const std::pair<float, PointType>& a, const std::pair<float, PointType>& b) {
                          return a.first < b.first;
                      });
            const float kth = found[k - 1].first;
            const double shell_min = static_cast<double>(ring) * m_leaf;
            if (kth <= static_cast<float>(shell_min * shell_min)) break;  // 剩余未搜更远, 可停
        }
    }
    if (found.empty()) return 0;
    std::sort(found.begin(), found.end(),
              [](const std::pair<float, PointType>& a, const std::pair<float, PointType>& b) {
                  return a.first < b.first;
              });
    const int n = std::min<int>(k, static_cast<int>(found.size()));
    for (int i = 0; i < n; ++i) { points.push_back(found[i].second); sqdists.push_back(found[i].first); }
    return n;
}

double VoxelMapStore::nearestDist2(const PointType& query) const {
    std::vector<PointType> pts;
    std::vector<float> sq;
    if (nearestK(query, 1, pts, sq) < 1) return -1.0;
    return sq[0];
}

VoxelStats VoxelMapStore::statsAt(const Eigen::Vector3d& p, double radius) const {
    VoxelStats s;
    CloudType::Ptr nb = queryRadius(p, radius);
    if (nb->size() < 3) return s;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& q : nb->points) mean += Eigen::Vector3d(q.x, q.y, q.z);
    mean /= nb->size();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& q : nb->points) {
        const Eigen::Vector3d d = Eigen::Vector3d(q.x, q.y, q.z) - mean;
        cov += d * d.transpose();
    }
    cov /= nb->size();
    s.mean = mean; s.cov = cov; s.n = static_cast<int>(nb->size()); s.valid = true;
    return s;
}

void VoxelMapStore::erase(const CloudType::Ptr& world_points) {
    if (!world_points || world_points->empty()) return;
    for (const auto& p : world_points->points) {
        auto it = m_cells.find(keyOf(p.x, p.y, p.z));
        if (it == m_cells.end() || !it->second) continue;
        m_size -= it->second->size();
        m_cells.erase(it);
    }
    ++m_rev;
}

void VoxelMapStore::carve(const Eigen::Vector3d& origin, const CloudType::Ptr& endpoints) {
    if (!endpoints || endpoints->empty()) return;
    const double step = std::max(0.05, m_leaf * 0.5);
    std::unordered_map<int64_t, char> wiped;
    for (const auto& e : endpoints->points) {
        if (!std::isfinite(e.x) || !std::isfinite(e.y) || !std::isfinite(e.z)) continue;
        const Eigen::Vector3d d = Eigen::Vector3d(e.x, e.y, e.z) - origin;
        const double len = d.norm();
        if (len < step * 2) continue;
        const Eigen::Vector3d u = d / len;
        for (double t = step; t < len - m_leaf; t += step) {
            const Eigen::Vector3d q = origin + u * t;
            wiped[keyOf(q.x(), q.y(), q.z())] = 1;
        }
    }
    for (const auto& kv : wiped) {
        auto it = m_cells.find(kv.first);
        if (it != m_cells.end() && it->second) { m_size -= it->second->size(); m_cells.erase(it); }
    }
    if (!wiped.empty()) ++m_rev;
}

bool VoxelMapStore::save(const std::string& path) const {
    CloudType all;
    for (const auto& kv : m_cells)
        if (kv.second) all += *kv.second;
    if (all.empty()) return false;
    return pcl::io::savePCDFileBinary(path, all) == 0;
}

bool VoxelMapStore::load(const std::string& path) {
    CloudType c;
    if (pcl::io::loadPCDFile(path, c) != 0) return false;
    m_cells.clear();
    m_size = 0;
    insert(CloudType::Ptr(new CloudType(c)));
    return !m_cells.empty();
}

} // namespace rosiwit_slam
