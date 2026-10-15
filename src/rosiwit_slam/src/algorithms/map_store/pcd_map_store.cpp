// src/algorithms/map_store/pcd_map_store.cpp
#include "algorithms/map_store/pcd_map_store.h"
#include <cmath>
#include <pcl/io/pcd_io.h>

namespace rosiwit_slam {

int64_t PcdMapStore::voxelKey(const PointType& p) const {
    const double v = m_leaf > 0 ? m_leaf : 0.3;
    const int64_t ix = static_cast<int64_t>(std::floor(p.x / v)) & 0x1FFFFF;
    const int64_t iy = static_cast<int64_t>(std::floor(p.y / v)) & 0x1FFFFF;
    const int64_t iz = static_cast<int64_t>(std::floor(p.z / v)) & 0x1FFFFF;
    return (ix << 42) | (iy << 21) | iz;
}

bool PcdMapStore::buildFromCloud(const CloudType::Ptr& cloud) {
    m_cloud.reset(new CloudType());
    if (cloud) {
        m_cloud->reserve(cloud->size());
        for (const auto& p : cloud->points)
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) m_cloud->push_back(p);
    }
    m_dirty = true;
    ++m_rev;
    return !m_cloud->empty();
}

void PcdMapStore::ensureTree() const {
    if (m_dirty && m_cloud) { m_kdt.setInputCloud(m_cloud); m_dirty = false; }
}

CloudType::Ptr PcdMapStore::queryRadius(const Eigen::Vector3d& center, double radius) const {
    CloudType::Ptr out(new CloudType());
    if (!m_cloud || m_cloud->empty()) return out;
    ensureTree();
    PointType q;
    q.x = static_cast<float>(center.x());
    q.y = static_cast<float>(center.y());
    q.z = static_cast<float>(center.z());
    std::vector<int> idx;
    std::vector<float> sqd;
    if (m_kdt.radiusSearch(q, radius, idx, sqd) <= 0) return out;
    out->reserve(idx.size());
    for (int i : idx) out->push_back(m_cloud->points[i]);
    return out;
}

int PcdMapStore::nearestK(const PointType& query, int k,
                          std::vector<PointType>& points, std::vector<float>& sqdists) const {
    points.clear(); sqdists.clear();
    if (!m_cloud || m_cloud->empty()) return 0;
    ensureTree();
    std::vector<int> idx;
    std::vector<float> sq;
    const int n = m_kdt.nearestKSearch(query, k, idx, sq);
    points.reserve(n);
    for (int i = 0; i < n; ++i) points.push_back(m_cloud->points[idx[i]]);
    sqdists = sq;
    return n;
}

double PcdMapStore::nearestDist2(const PointType& query) const {
    std::vector<PointType> pts;
    std::vector<float> sq;
    if (nearestK(query, 1, pts, sq) < 1) return -1.0;
    return sq[0];
}

VoxelStats PcdMapStore::statsAt(const Eigen::Vector3d& p, double radius) const {
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

void PcdMapStore::insert(const CloudType::Ptr& world_points) {
    if (!m_cloud) m_cloud.reset(new CloudType());
    if (!world_points) return;
    for (const auto& p : world_points->points)
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) m_cloud->push_back(p);
    m_dirty = true;
    ++m_rev;
}

void PcdMapStore::erase(const CloudType::Ptr& world_points) {
    if (!m_cloud || !world_points || world_points->empty()) return;
    std::unordered_set<int64_t> kill;
    kill.reserve(world_points->size());
    for (const auto& p : world_points->points) kill.insert(voxelKey(p));
    CloudType::Ptr kept(new CloudType());
    kept->reserve(m_cloud->size());
    for (const auto& p : m_cloud->points)
        if (!kill.count(voxelKey(p))) kept->push_back(p);
    m_cloud = kept;
    m_dirty = true;
    ++m_rev;
}

bool PcdMapStore::save(const std::string& path) const {
    if (!m_cloud || m_cloud->empty()) return false;
    return pcl::io::savePCDFileBinary(path, *m_cloud) == 0;
}

bool PcdMapStore::load(const std::string& path) {
    CloudType::Ptr c(new CloudType());
    if (pcl::io::loadPCDFile(path, *c) != 0) return false;
    return buildFromCloud(c);
}

} // namespace rosiwit_slam
