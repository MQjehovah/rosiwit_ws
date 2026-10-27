// src/algorithms/map_store/voxel_map_store.cpp
#include "algorithms/map_store/voxel_map_store.h"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <unordered_set>
#include <pcl/io/pcd_io.h>

namespace rosiwit_slam {

namespace {
constexpr char kMagic[8] = {'V', 'X', 'M', 'A', 'P', '1', '\n', '1'};
} // namespace

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

// 列索引 (cx,cy) -> 该列所有 cell key, 供 groundAt 快速取某 (x,y) 的整列
static int64_t colKey(int cx, int cy) {
    const int64_t ix = static_cast<int64_t>(cx) & 0x1FFFFF;
    const int64_t iy = static_cast<int64_t>(cy) & 0x1FFFFF;
    return (ix << 21) | iy;
}

// 列索引维护 (与 m_cells 的增删保持一致)
using ColumnIndex = std::unordered_map<int64_t, std::vector<int64_t>>;

static void colAdd(ColumnIndex& cols, int64_t cell_key, int cx, int cy) {
    cols[colKey(cx, cy)].push_back(cell_key);
}

static void colDrop(ColumnIndex& cols, int64_t cell_key, int cx, int cy) {
    auto cit = cols.find(colKey(cx, cy));
    if (cit == cols.end()) return;
    auto& v = cit->second;
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (*it == cell_key) { v.erase(it); break; }
    }
    if (v.empty()) cols.erase(cit);
}

bool VoxelMapStore::buildFromCloud(const CloudType::Ptr& cloud) {
    m_cells.clear();
    m_columns.clear();
    m_size = 0;
    ++m_rev;
    insert(cloud);
    return !m_cells.empty();
}

void VoxelMapStore::insert(const CloudType::Ptr& world_points, double time) {
    if (!world_points) return;
    for (const auto& p : world_points->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int cx = static_cast<int>(std::floor(p.x / m_leaf));
        const int cy = static_cast<int>(std::floor(p.y / m_leaf));
        const int cz = static_cast<int>(std::floor(p.z / m_leaf));
        const int64_t k = key(cx, cy, cz);
        auto it = m_cells.find(k);
        if (it == m_cells.end()) {
            Cell c;
            c.pts.reset(new CloudType());
            c.cx = cx; c.cy = cy; c.cz = cz;
            it = m_cells.emplace(k, std::move(c)).first;
            colAdd(m_columns, k, cx, cy);
        }
        it->second.pts->push_back(p);
        if (time >= 0) it->second.t = time;
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
                if (it == m_cells.end()) continue;
                for (const auto& p : it->second.pts->points) {
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
                    auto it = m_cells.find(key(qcx + dx, qcy + dy, qcz + dz));
                    if (it == m_cells.end()) continue;
                    for (const auto& p : it->second.pts->points) {
                        const float ddx = p.x - query.x, ddy = p.y - query.y, ddz = p.z - query.z;
                        found.emplace_back(ddx * ddx + ddy * ddy + ddz * ddz, p);
                    }
                }
        if (static_cast<int>(found.size()) >= k) {
            std::sort(found.begin(), found.end(),
                      [](const std::pair<float, PointType>& a, const std::pair<float, PointType>& b) {
                          return a.first < b.first;
                      });
            const float kth = found[k - 1].first;
            const double shell_min = static_cast<double>(ring) * m_leaf;
            if (kth <= static_cast<float>(shell_min * shell_min)) break;
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

bool VoxelMapStore::groundAt(double x, double y, double& z, Eigen::Vector3d& normal) const {
    const int cx = static_cast<int>(std::floor(x / m_leaf));
    const int cy = static_cast<int>(std::floor(y / m_leaf));
    // 取 (x,y) 3x3 列的所有 cell 点: 走列索引, 与地图总大小无关
    std::vector<Eigen::Vector3d> pts;
    double minz = 1e18;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
            const auto cit = m_columns.find(colKey(cx + dx, cy + dy));
            if (cit == m_columns.end()) continue;
            for (const int64_t ck : cit->second) {
                const auto it = m_cells.find(ck);
                if (it == m_cells.end()) continue;
                for (const auto& p : it->second.pts->points) {
                    pts.emplace_back(p.x, p.y, p.z);
                    minz = std::min(minz, static_cast<double>(p.z));
                }
            }
        }
    if (pts.size() < 5) return false;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    int n = 0;
    for (const auto& p : pts) { if (p.z() <= minz + 0.3) { mean += p; ++n; } }
    if (n < 3) return false;
    mean /= n;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts) { if (p.z() <= minz + 0.3) { const Eigen::Vector3d d = p - mean; cov += d * d.transpose(); } }
    cov /= n;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d nm = es.eigenvectors().col(0);
    if (nm.z() < 0) nm = -nm;
    z = mean.z();
    normal = nm;
    return true;
}

void VoxelMapStore::erase(const CloudType::Ptr& world_points) {
    if (!world_points || world_points->empty()) return;
    for (const auto& p : world_points->points) {
        auto it = m_cells.find(keyOf(p.x, p.y, p.z));
        if (it == m_cells.end()) continue;
        m_size -= it->second.pts->size();
        colDrop(m_columns, it->first, it->second.cx, it->second.cy);
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
        if (it != m_cells.end()) {
            m_size -= it->second.pts->size();
            colDrop(m_columns, it->first, it->second.cx, it->second.cy);
            m_cells.erase(it);
        }
    }
    if (!wiped.empty()) ++m_rev;
}

void VoxelMapStore::decay(double now) {
    if (m_decay_window <= 0) return;
    bool changed = false;
    for (auto it = m_cells.begin(); it != m_cells.end();) {
        if (it->second.t >= 0 && now - it->second.t > m_decay_window) {
            m_size -= it->second.pts->size();
            colDrop(m_columns, it->first, it->second.cx, it->second.cy);
            it = m_cells.erase(it);
            changed = true;
        } else {
            ++it;
        }
    }
    if (changed) ++m_rev;
}

CloudType::Ptr VoxelMapStore::exportCloud() const {
    CloudType::Ptr out(new CloudType());
    out->reserve(m_size);
    for (const auto& kv : m_cells) {
        if (!kv.second.pts) continue;
        *out += *kv.second.pts;
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = false;
    return out;
}

bool VoxelMapStore::mergeFrom(const VoxelMapStore& other) {
    CloudType::Ptr cloud = other.exportCloud();
    if (!cloud || cloud->empty()) return false;
    insert(cloud, -1.0);
    return true;
}

bool VoxelMapStore::save(const std::string& path) const {
    if (m_cells.empty()) return false;
    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) return false;
    bool ok = true;
    ok = ok && std::fwrite(kMagic, 1, sizeof(kMagic), fp) == sizeof(kMagic);
    ok = ok && std::fwrite(&m_leaf, sizeof(double), 1, fp) == 1;
    const uint64_t nc = m_cells.size();
    ok = ok && std::fwrite(&nc, sizeof(uint64_t), 1, fp) == 1;
    for (const auto& kv : m_cells) {
        const Cell& c = kv.second;
        const uint32_t n = static_cast<uint32_t>(c.pts->size());
        const float t = static_cast<float>(c.t);
        const int32_t hdr[3] = {c.cx, c.cy, c.cz};
        ok = ok && std::fwrite(hdr, sizeof(int32_t), 3, fp) == 3;
        ok = ok && std::fwrite(&n, sizeof(uint32_t), 1, fp) == 1;
        ok = ok && std::fwrite(&t, sizeof(float), 1, fp) == 1;
        for (const auto& p : c.pts->points) {
            const float xyz[3] = {p.x, p.y, p.z};
            if (std::fwrite(xyz, sizeof(float), 3, fp) != 3) { ok = false; break; }
        }
        if (!ok) break;
    }
    std::fclose(fp);
    return ok;
}

bool VoxelMapStore::load(const std::string& path) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) return false;
    char magic[sizeof(kMagic)];
    if (std::fread(magic, 1, sizeof(magic), fp) != sizeof(magic) ||
        std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        std::fclose(fp);
        return false;
    }
    double leaf = 0.3;
    uint64_t nc = 0;
    std::fread(&leaf, sizeof(double), 1, fp);
    std::fread(&nc, sizeof(uint64_t), 1, fp);
    m_leaf = leaf > 0 ? leaf : 0.3;
    m_cells.clear();
    m_columns.clear();
    m_size = 0;
    for (uint64_t i = 0; i < nc; ++i) {
        int32_t hdr[3]; uint32_t n; float t;
        if (std::fread(hdr, sizeof(int32_t), 3, fp) != 3) break;
        if (std::fread(&n, sizeof(uint32_t), 1, fp) != 1) break;
        if (std::fread(&t, sizeof(float), 1, fp) != 1) break;
        const int64_t k = key(hdr[0], hdr[1], hdr[2]);
        Cell c;
        c.pts.reset(new CloudType());
        c.cx = hdr[0]; c.cy = hdr[1]; c.cz = hdr[2]; c.t = t;
        c.pts->reserve(n);
        for (uint32_t j = 0; j < n; ++j) {
            float xyz[3];
            if (std::fread(xyz, sizeof(float), 3, fp) != 3) break;
            PointType p; p.x = xyz[0]; p.y = xyz[1]; p.z = xyz[2];
            c.pts->push_back(p);
        }
        m_size += c.pts->size();
        const int ccx = hdr[0], ccy = hdr[1];
        m_cells.emplace(k, std::move(c));
        colAdd(m_columns, k, ccx, ccy);
    }
    std::fclose(fp);
    ++m_rev;
    return !m_cells.empty();
}

} // namespace rosiwit_slam
