// src/algorithms/localization/local_map/keyframe_submap_provider.cpp
#include "algorithms/localization/local_map/keyframe_submap_provider.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <unordered_map>
#include <algorithm>

namespace rosiwit_slam {

void KeyframeSubmapProvider::setMap(const std::string& map_path, const CloudType::Ptr& global_map) {
    (void)map_path;
    m_map = filterFinite(global_map);
    m_kfs.clear();
    m_pose_tree_ready = false;
    m_have_injected = false;
}

void KeyframeSubmapProvider::setKeyframes(const std::vector<LocalKeyframe>& keyframes) {
    m_kfs.clear();
    m_kfs.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
        if (!kf.cloud || kf.cloud->empty()) continue;
        Kf k;
        k.pose = kf.pose;
        k.world.reset(new CloudType());
        k.world->reserve(kf.cloud->size());
        for (const auto& p : kf.cloud->points) {
            const Eigen::Vector3d q = kf.pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + kf.pose.trans;
            PointType w;
            w.x = static_cast<float>(q.x());
            w.y = static_cast<float>(q.y());
            w.z = static_cast<float>(q.z());
            k.world->push_back(w);
        }
        m_kfs.push_back(std::move(k));
    }
    m_have_injected = true;
    buildPoseTree();
}

void KeyframeSubmapProvider::buildPoseTree() {
    m_pose_cloud.reset(new CloudType());
    m_pose_cloud->reserve(m_kfs.size());
    for (const auto& k : m_kfs) {
        PointType p;
        p.x = static_cast<float>(k.pose.trans.x());
        p.y = static_cast<float>(k.pose.trans.y());
        p.z = static_cast<float>(k.pose.trans.z());
        m_pose_cloud->push_back(p);
    }
    m_pose_kdtree.setInputCloud(m_pose_cloud);
    m_pose_tree_ready = !m_kfs.empty();
}

void KeyframeSubmapProvider::buildFromMapFallback() {
    // 无关键帧注入: 用全局地图按 XY 分块合成虚拟关键帧 (每组点=一帧, 质心=位姿)
    if (!m_map || m_map->empty()) return;
    const double cell = m_cfg.cell_size > 0 ? m_cfg.cell_size : 10.0;

    struct Acc { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); long n = 0; CloudType::Ptr cloud = nullptr; };
    std::unordered_map<long long, Acc> cells;
    auto key = [](long long cx, long long cy) {
        return static_cast<long long>((static_cast<unsigned long long>(cx) << 32) ^
                                      static_cast<unsigned int>(cy));
    };

    for (const auto& p : m_map->points) {
        const long long cx = static_cast<long long>(std::floor(p.x / cell));
        const long long cy = static_cast<long long>(std::floor(p.y / cell));
        Acc& a = cells[key(cx, cy)];
        if (!a.cloud) a.cloud.reset(new CloudType());
        a.cloud->push_back(p);
        a.sum += Eigen::Vector3d(p.x, p.y, p.z);
        ++a.n;
    }

    m_kfs.clear();
    m_kfs.reserve(cells.size());
    for (auto& kv : cells) {
        Acc& a = kv.second;
        if (!a.cloud || a.cloud->empty()) continue;
        Kf k;
        k.pose.trans = a.sum / static_cast<double>(a.n);
        k.pose.rot = M3D::Identity();
        k.world = a.cloud;
        m_kfs.push_back(std::move(k));
    }
    buildPoseTree();
}

CloudType::Ptr KeyframeSubmapProvider::getLocalMap(const PoseStamped& pose) {
    CloudType::Ptr out(new CloudType());
    if (!m_pose_tree_ready && !m_have_injected) buildFromMapFallback();
    if (m_kfs.empty() || !m_pose_tree_ready) return out;

    const double radius = m_cfg.radius > 0 ? m_cfg.radius : 60.0;
    const double r2 = radius * radius;
    const int k = std::min<int>(std::max(1, m_cfg.max_keyframes),
                                static_cast<int>(m_kfs.size()));

    PointType q;
    q.x = static_cast<float>(pose.trans.x());
    q.y = static_cast<float>(pose.trans.y());
    q.z = static_cast<float>(pose.trans.z());
    std::vector<int> idx(k);
    std::vector<float> sq(k);
    const int found = m_pose_kdtree.nearestKSearch(q, k, idx, sq);
    for (int i = 0; i < found; ++i) {
        if (sq[i] > r2) break;  // 升序, 后续更远
        if (m_kfs[idx[i]].world) *out += *m_kfs[idx[i]].world;
    }
    if (m_cfg.voxel_resolution > 0 && !out->empty())
        out = downsampleCloud(out, m_cfg.voxel_resolution);
    return out;
}

} // namespace rosiwit_slam
