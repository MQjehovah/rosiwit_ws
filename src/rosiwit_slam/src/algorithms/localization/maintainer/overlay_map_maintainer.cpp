// src/algorithms/localization/maintainer/overlay_map_maintainer.cpp
#include "algorithms/localization/maintainer/overlay_map_maintainer.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <pcl/io/pcd_io.h>

#ifdef USE_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#endif

namespace rosiwit_slam {

int64_t OverlayMapMaintainer::voxelKey(int cx, int cy, int cz) {
    const int64_t ox = static_cast<int64_t>(cx) & 0x1FFFFF;
    const int64_t oy = static_cast<int64_t>(cy) & 0x1FFFFF;
    const int64_t oz = static_cast<int64_t>(cz) & 0x1FFFFF;
    return (ox << 42) | (oy << 21) | oz;
}

void OverlayMapMaintainer::setBaseMap(const std::string& map_path, const CloudType::Ptr& base) {
    // 换图必须清空上一张图的叠加层/体素去重/增量缓存。否则旧 overlay 会
    // 继续叠加到新图局部目标上，造成"新图里混着上一张图的点" (切图干扰)。
    m_cells.clear();
    m_voxel_seen.clear();
    m_w.clear();
    m_last_world.reset();
    m_pending_store.reset();
    m_overlay_count = 0;
    ++m_revision;

    m_map_path = map_path;
    m_voxel = m_cfg.voxel > 0 ? m_cfg.voxel : 0.3;
    m_cell = m_cfg.cell > 0 ? m_cfg.cell : 10.0;
    // 懒建: 这里只存原始指针, 降采样+KD树推迟到首次真正用到(L147 吸收判据)再建。
    // 全图降采样+建树实测 ~15s(aarch64, DDD), 每次切图都白付 —— 切图(带初始位姿)
    // 根本不碰它 (用户语义: 有 initpose 不需要额外准备, 直接跟踪)。
    m_base_raw = base;
    m_base.reset();
    m_has_base = false;
    // 尝试载入已存在的叠加层 (原图不变, 叠加层单独落盘)
    if (!map_path.empty()) load(map_path + ".overlay.pcd");
}

void OverlayMapMaintainer::addPoint(const Eigen::Vector3d& p) {
    // 容量上限: 叠加层长期运行会无限增长 (实体素/单点只增不减). 超过上限时
    // 整体重置叠加层并告警 —— 底图不动, 叠加层只是"变化缓存", 重建代价可控,
    // 比无界内存增长/OOM 安全. yaml: maintainer.max_voxels
    if (m_cfg.max_voxels > 0 &&
        static_cast<int>(m_voxel_seen.size()) >= m_cfg.max_voxels) {
        m_cells.clear();
        m_voxel_seen.clear();
        m_w.clear();
        m_last_world.reset();
        m_pending_store.reset();
        m_hits.clear();
        m_overlay_count = 0;
        m_frame = 0;
        ++m_revision;
        std::cerr << "[OverlayMapMaintainer] overlay reset: reached max_voxels="
                  << m_cfg.max_voxels << std::endl;
    }
    const int vx = static_cast<int>(std::floor(p.x() / m_voxel));
    const int vy = static_cast<int>(std::floor(p.y() / m_voxel));
    const int vz = static_cast<int>(std::floor(p.z() / m_voxel));
    const int64_t vk = voxelKey(vx, vy, vz);
    if (m_voxel_seen.count(vk)) {   // 体素去重
        bumpWeight(vk, 8);          // A 阶段: 重复观测 -> 可信度累加
        return;
    }
    m_voxel_seen[vk] = m_frame;
    m_w[vk] = 0;                    // A 阶段: 新体素从 0 起, 本次观测即一次 hit
    bumpWeight(vk, 8);

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
    if (!m_pending_store) m_pending_store.reset(new CloudType());
    m_pending_store->push_back(q);   // 供 store 增量插入
    ++m_overlay_count;
    ++m_revision;                    // 目标缓存失效信号
}

void OverlayMapMaintainer::rayCarve(const PoseStamped& pose, const CloudType::Ptr& scan_body) {
    // 自由空间雕刻: 沿"传感器原点->扫描端点"射线前进, 途经的 overlay 体素判为自由并移除
    // (货物被发走/过道重新开阔 -> 射线穿过 -> 清除鬼影)
    if (scan_body->empty()) return;
    const double step = std::max(0.05, m_voxel);
    const int stride = std::max(1, m_cfg.carve_stride);
    const int n = static_cast<int>(scan_body->size());
    for (int i = 0; i < n; i += stride) {
        const auto& p = scan_body->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d end = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        const Eigen::Vector3d dir = end - pose.trans;
        const double len = dir.norm();
        if (len < step * 2) continue;
        const Eigen::Vector3d u = dir / len;
        for (double d = step; d < len - m_voxel; d += step) {
            const Eigen::Vector3d q = pose.trans + u * d;
            const int vx = static_cast<int>(std::floor(q.x() / m_voxel));
            const int vy = static_cast<int>(std::floor(q.y() / m_voxel));
            const int vz = static_cast<int>(std::floor(q.z() / m_voxel));
            const int64_t vk = voxelKey(vx, vy, vz);
            auto wit = m_w.find(vk);
            if (wit != m_w.end()) {
                // A 阶段: miss 只衰减, 衰减到 0 才从"可见集"删除 (结构重现时原体素还在)
                if (wit->second > 8) {
                    wit->second = static_cast<uint8_t>(wit->second - 8);
                } else {
                    m_w.erase(wit);
                    m_voxel_seen.erase(vk);
                }
            } else {
                m_voxel_seen.erase(vk);   // 无权重记录的体素: 保持原来的硬删除
            }
        }
    }
}

void OverlayMapMaintainer::bumpWeight(int64_t vk, int delta) {
    auto it = m_w.find(vk);
    if (it == m_w.end()) return;
    const int v = static_cast<int>(it->second) + delta;
    it->second = static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

void OverlayMapMaintainer::observeHits(const PoseStamped& pose, const CloudType::Ptr& scan_body) {
    // A 阶段: 每帧每个体素只加一次 w (扫描点按体素去重), 只累加"已经是变化体素"的那些,
    // 新体素仍走 commitKeyframe 的持久化门控 (persist_hits) —— w 只衡量持久结构被
    // 重复看到的次数, 不会把瞬时点云抬成高可信.
    if (!m_last_world) m_last_world.reset(new CloudType());
    else m_last_world->clear();
    if (!scan_body || scan_body->empty()) return;
    std::unordered_set<int64_t> seen;
    seen.reserve(scan_body->size());
    for (const auto& p : scan_body->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d w = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        const int vx = static_cast<int>(std::floor(w.x() / m_voxel));
        const int vy = static_cast<int>(std::floor(w.y() / m_voxel));
        const int vz = static_cast<int>(std::floor(w.z() / m_voxel));
        const int64_t vk = voxelKey(vx, vy, vz);
        if (!seen.insert(vk).second) continue;
        if (m_w.find(vk) == m_w.end()) continue;
        bumpWeight(vk, 8);
        m_voxel_seen[vk] = m_frame;   // 被再观测: 刷新存活时间
        PointType q;
        q.x = static_cast<float>(w.x());
        q.y = static_cast<float>(w.y());
        q.z = static_cast<float>(w.z());
        m_last_world->push_back(q);
    }
}

void OverlayMapMaintainer::onLocalized(const PoseStamped& pose, const CloudType::Ptr& scan_body,
                                       const MatchResult& match, bool pose_confident) {
    if (!m_cfg.enabled || !scan_body || scan_body->empty()) return;
    ++m_frame;
    if (m_cfg.autosave_frames > 0 && !m_map_path.empty() &&
        m_frame % m_cfg.autosave_frames == 0) {
        save(m_map_path + ".overlay.pcd");
    }

    // 过桥/漂移期(位姿不可信): 不写、不雕刻, 避免污染地图
    if (!pose_confident) { m_low_count = 0; return; }

    // 清除: 看到"本该有货的地方是空的" -> 移除该处 overlay
    rayCarve(pose, scan_body);
    // A 阶段: 观测命中 -> w += 8 (只统计, 不参与决策)
    observeHits(pose, scan_body);

    if (m_cfg.pose_graph) {
        // L13: 变化关键帧缓冲, fitness 恢复或达阈值时 flush (GTSAM 位姿图平滑)
        if (match.fitness >= m_cfg.fitness_trigger) {
            m_low_count = 0;
            if (!m_buf.empty()) flushBuffer();
        } else if (++m_low_count >= m_cfg.min_confirm) {
            const double moved = m_have_last ? (pose.trans - m_last_add).norm() : 1e9;
            if (moved >= m_cfg.min_keyframe_dist) {
                KfBuf k; k.pose = pose; k.body = scan_body;
                m_buf.push_back(std::move(k));
                m_last_add = pose.trans;
                m_have_last = true;
                if (static_cast<int>(m_buf.size()) >= std::max(2, m_cfg.pg_min_keyframes)) flushBuffer();
            }
        }
    } else {
        if (match.fitness >= m_cfg.fitness_trigger) { m_low_count = 0; }
        else if (++m_low_count >= m_cfg.min_confirm) {
            const double moved = m_have_last ? (pose.trans - m_last_add).norm() : 1e9;
            if (moved >= m_cfg.min_keyframe_dist) {
                commitKeyframe(pose, scan_body);
                m_last_add = pose.trans;
                m_have_last = true;
            }
        }
    }

    // 衰减兜底: 长期未再观测的叠加体素移除
    if (m_cfg.decay_frames > 0) {
        for (auto it = m_voxel_seen.begin(); it != m_voxel_seen.end();) {
            if (m_frame - it->second > m_cfg.decay_frames) it = m_voxel_seen.erase(it);
            else ++it;
        }
    }
}

void OverlayMapMaintainer::commitKeyframe(const PoseStamped& pose, const CloudType::Ptr& body) {
    if (!body) return;
    const int persist = std::max(1, m_cfg.persist_hits);
    const double dmin2 = m_cfg.delta_min * m_cfg.delta_min;
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    // 懒建 KD 树(首次吸收判据时): setBaseMap 只存了原始指针, 这里一次性补降采样+建树
    if (!m_has_base && m_base_raw && !m_base_raw->empty()) {
        m_base = downsampleCloud(filterFinite(m_base_raw), m_voxel);
        m_base_kdtree.setInputCloud(m_base);
        m_has_base = true;
    }
    for (const auto& p : body->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d w = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        if (m_has_base) {
            PointType q;
            q.x = static_cast<float>(w.x()); q.y = static_cast<float>(w.y()); q.z = static_cast<float>(w.z());
            if (m_base_kdtree.nearestKSearch(q, 1, idx, sq) > 0 && sq[0] <= dmin2) continue;
        }
        const int vx = static_cast<int>(std::floor(w.x() / m_voxel));
        const int vy = static_cast<int>(std::floor(w.y() / m_voxel));
        const int vz = static_cast<int>(std::floor(w.z() / m_voxel));
        const int64_t vk = voxelKey(vx, vy, vz);
        auto it = m_voxel_seen.find(vk);
        if (it != m_voxel_seen.end()) { it->second = m_frame; continue; }
        if (++m_hits[vk] >= persist) { addPoint(w); m_hits.erase(vk); }
    }
}

void OverlayMapMaintainer::flushBuffer() {
    if (m_buf.empty()) return;
    optimizePoses();
    for (auto& k : m_buf) commitKeyframe(k.pose, k.body);
    m_buf.clear();
}

void OverlayMapMaintainer::optimizePoses() {
#ifdef USE_GTSAM
    const int n = static_cast<int>(m_buf.size());
    if (n < 2) return;
    using gtsam::symbol_shorthand::X;
    auto toPose = [](const PoseStamped& p) {
        return gtsam::Pose3(gtsam::Rot3(p.rot), gtsam::Point3(p.trans.x(), p.trans.y(), p.trans.z()));
    };
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values init;
    for (int i = 0; i < n; ++i) init.insert(X(i), toPose(m_buf[i].pose));
    // 首帧中等先验(位置来自定位, 可信但不硬锁)
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector6() << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), toPose(m_buf[0].pose), prior_noise);
    // 相邻关键帧里程约束
    gtsam::Vector6 mv;
    mv << 4e-4, 4e-4, 4e-4, 2.5e-3, 2.5e-3, 2.5e-3;
    auto motion_noise = gtsam::noiseModel::Diagonal::Variances(mv);
    for (int i = 1; i < n; ++i) {
        const gtsam::Pose3 dp = toPose(m_buf[i - 1].pose).between(toPose(m_buf[i].pose));
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), dp, motion_noise);
    }
    try {
        gtsam::LevenbergMarquardtOptimizer opt(graph, init);
        gtsam::Values res = opt.optimize();
        for (int i = 0; i < n; ++i) {
            const gtsam::Pose3 g = res.at<gtsam::Pose3>(X(i));
            m_buf[i].pose.trans = g.translation();
            m_buf[i].pose.rot = g.rotation().matrix();
        }
    } catch (const std::exception&) { /* 失败则保留原缓冲位姿 */ }
#endif
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
                    const int vx = static_cast<int>(std::floor(p.x / m_voxel));
                    const int vy = static_cast<int>(std::floor(p.y / m_voxel));
                    const int vz = static_cast<int>(std::floor(p.z / m_voxel));
                    if (!m_voxel_seen.count(voxelKey(vx, vy, vz))) continue;  // 已衰减/被雕刻
                    const double ddx = p.x - pose.trans.x();
                    const double ddy = p.y - pose.trans.y();
                    const double ddz = p.z - pose.trans.z();
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) out->push_back(p);
                }
            }
    return out;
}

double OverlayMapMaintainer::staticSupport(const PoseStamped& pose,
                                          int* anchor_age, size_t* submap_pts) const {
    // A 阶段(影子): 统计"当前扫描点中落在 overlay(变化结构)上的点"里, w>=w_anchor 的占比.
    // 分母只含变化结构上的点; 纯底图区域不参与 (那里本来就是静态的, 不构成判断依据).
    // 只读, 不改变任何状态; 返回 -1 = 本帧没有可见的变化结构 (日志记 n/a).
    if (anchor_age) *anchor_age = 0;
    if (submap_pts) *submap_pts = 0;
    if (!m_last_world || m_last_world->empty()) return -1.0;
    const double radius = m_cfg.radius > 0 ? m_cfg.radius : 60.0;
    const double r2 = radius * radius;
    const int anchor = m_cfg.w_anchor > 0 ? m_cfg.w_anchor : 128;
    size_t total = 0;
    size_t trusted = 0;
    long oldest = -1;
    for (const auto& p : m_last_world->points) {
        const double dx = p.x - pose.trans.x();
        const double dy = p.y - pose.trans.y();
        const double dz = p.z - pose.trans.z();
        if (dx * dx + dy * dy + dz * dz > r2) continue;
        const int64_t vk = voxelKey(static_cast<int>(std::floor(p.x / m_voxel)),
                                    static_cast<int>(std::floor(p.y / m_voxel)),
                                    static_cast<int>(std::floor(p.z / m_voxel)));
        auto wit = m_w.find(vk);
        if (wit == m_w.end()) continue;
        ++total;
        if (static_cast<int>(wit->second) >= anchor) {
            ++trusted;
            auto sit = m_voxel_seen.find(vk);
            const long last = (sit == m_voxel_seen.end()) ? 0 : sit->second;
            if (oldest < 0 || last < oldest) oldest = last;
        }
    }
    if (submap_pts) *submap_pts = total;
    if (oldest >= 0) {
        const long age = m_frame - oldest;
        if (anchor_age) *anchor_age = static_cast<int>(age < 0 ? 0 : age);
    }
    if (total == 0) return -1.0;
    return static_cast<double>(trusted) / static_cast<double>(total);
}

void OverlayMapMaintainer::popOverlayDelta(CloudType::Ptr& out) {
    out = m_pending_store;
    m_pending_store.reset();
}

CloudType::Ptr OverlayMapMaintainer::snapshotOverlay() {
    CloudType::Ptr out(new CloudType());
    for (const auto& kv : m_cells) {
        if (!kv.second) continue;
        for (const auto& p : kv.second->points) {
            const int vx = static_cast<int>(std::floor(p.x / m_voxel));
            const int vy = static_cast<int>(std::floor(p.y / m_voxel));
            const int vz = static_cast<int>(std::floor(p.z / m_voxel));
            if (m_voxel_seen.count(voxelKey(vx, vy, vz))) out->push_back(p);
        }
    }
    return out;
}

bool OverlayMapMaintainer::save(const std::string& path) {
    CloudType::Ptr all = snapshotOverlay();
    if (!all || all->empty()) return false;
    return pcl::io::savePCDFileBinary(path, *all) == 0;
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
