// src/algorithms/localization/maintainer/overlay_map_maintainer.cpp
#include "algorithms/localization/maintainer/overlay_map_maintainer.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <fstream>
#include <iostream>
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

namespace {
} // namespace

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
    m_persistency.clear();
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
    m_base_voxels.clear();
    m_base_indexed = false;
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
        m_persistency.clear();
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
        hitVoxel(vk);               // 重复观测 -> 按计票单位累加
        return;
    }
    m_voxel_seen[vk] = m_frame;
    m_persistency[vk] = PersistencyVoxel{};             // 新体素从 0 起, 本次观测即一次 hit
    hitVoxel(vk);

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
            missVoxel(vk);   // 连续 persistency_miss_confirm 帧 free 证据才删(去抖)
        }
    }
}

void OverlayMapMaintainer::beginPersistencySession() {
    ++m_session_id;
    if (m_session_id == 0) m_session_id = 1;
    m_session_start_s = -1.0;   // 下一帧重新记起点
    std::cout << "[OverlayMapMaintainer] new session=" << m_session_id << std::endl;
}

uint32_t OverlayMapMaintainer::currentUnitKey() const {
    switch (m_cfg.persistency_unit) {
        case 0:  return static_cast<uint32_t>(m_frame);
        case 1:  return static_cast<uint32_t>(m_frame);
        case 2:  return m_session_id;
        default: return m_day_index;
    }
}

// 一个体素被"命中": 按计票单位决定这次算不算一票。
void OverlayMapMaintainer::hitVoxel(int64_t vk) {
    auto it = m_persistency.find(vk);
    if (it == m_persistency.end()) return;
    PersistencyVoxel& v = it->second;
    const uint32_t frame = static_cast<uint32_t>(m_frame);
    bool count = false;
    switch (m_cfg.persistency_unit) {
        case 0:  count = true; break;                              // frame: 每帧
        case 1:  // view: 首次看到也算一票; 之后再看到需要中间隔过帧(离开过视野)
                 count = (v.last_frame == 0) || (frame > v.last_frame + 1); break;
        case 2:  count = (v.last_key != m_session_id); break;       // session
        default: count = (v.last_key != m_day_index); break;        // day
    }
    v.last_frame = frame;
    if (count) {
        const int gain = m_cfg.persistency_hit_gain > 0 ? m_cfg.persistency_hit_gain : 1;
        const int nw = static_cast<int>(v.w) + gain;
        v.w = static_cast<uint8_t>(nw > 255 ? 255 : nw);
        if (m_cfg.persistency_unit >= 2) v.last_key = currentUnitKey();
    }
    v.miss = 0;   // 又看到了: 清空 free 累计
}

// 一个体素被"自由空间雕刻": 连续 persistency_miss_confirm 帧才生效(去抖)。
void OverlayMapMaintainer::missVoxel(int64_t vk) {
    auto it = m_persistency.find(vk);
    if (it == m_persistency.end()) {
        m_voxel_seen.erase(vk);   // 无权重记录的体素: 保持原来的硬删除语义
        return;
    }
    PersistencyVoxel& v = it->second;
    const int need = m_cfg.persistency_miss_confirm > 0 ? m_cfg.persistency_miss_confirm : 1;
    if (++v.miss < need) return;
    if (m_cfg.persistency_miss_action == 0) {
        m_persistency.erase(it);
        m_voxel_seen.erase(vk);
    } else {
        const int gain = m_cfg.persistency_hit_gain > 0 ? m_cfg.persistency_hit_gain : 1;
        v.w = (v.w > gain) ? static_cast<uint8_t>(v.w - gain) : 0;
        v.miss = 0;
        if (v.w == 0) {
            m_persistency.erase(it);
            m_voxel_seen.erase(vk);
        }
    }
}

int OverlayMapMaintainer::persistencyAt(const Eigen::Vector3d& p) const {
    const int64_t vk = voxelKey(static_cast<int>(std::floor(p.x() / m_voxel)),
                                static_cast<int>(std::floor(p.y() / m_voxel)),
                                static_cast<int>(std::floor(p.z() / m_voxel)));
    auto it = m_persistency.find(vk);
    return it == m_persistency.end() ? -1 : static_cast<int>(it->second.w);
}

void OverlayMapMaintainer::observeHits(const PoseStamped& pose, const CloudType::Ptr& scan_body) {
    // A 阶段: 每帧每个体素只加一次 w (扫描点按体素去重), 只累加"已经是变化体素"的那些,
    // 新体素仍走 collectNovel 的持久化门控 (按 persistency_unit 计票) —— w 只衡量持久结构被
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
        if (m_persistency.find(vk) == m_persistency.end()) continue;
        hitVoxel(vk);
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
    m_novel_votes = 0;
    m_novel_committed = 0;
    // 计票单位 = session/day 需要的记账
    if (pose.time > 0.0) {
        m_day_index = static_cast<uint32_t>(pose.time / 86400.0);
        if (m_session_start_s < 0.0) {
            m_session_start_s = pose.time;
        } else if (m_cfg.persistency_session_hours > 0 &&
                   pose.time - m_session_start_s > 3600.0 * m_cfg.persistency_session_hours) {
            beginPersistencySession();
            m_session_start_s = pose.time;
        }
    }
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
        // 新结构沉淀: 与 fitness 解耦. 新增点没有对应点, 不会进 fitness 的分子/分母,
        // 所以"环境变了"不一定让 fitness 下降; 反过来 low fitness 也可能只是稀疏/遮挡.
        // 真正的判据是几何证据: 离底图 > delta_min + 按计票单位重复确认 + 最小簇.
        collectNovel(pose, scan_body);
        m_low_count = 0;
    }

    // 衰减兜底: 长期未再观测的叠加体素移除
    if (m_cfg.decay_frames > 0) {
        for (auto it = m_voxel_seen.begin(); it != m_voxel_seen.end();) {
            if (m_frame - it->second > m_cfg.decay_frames) it = m_voxel_seen.erase(it);
            else ++it;
        }
    }
}

void OverlayMapMaintainer::ensureBaseIndex() {
    if (m_base_indexed) return;
    m_base_indexed = true;
    if (!m_base_raw || m_base_raw->empty()) return;
    if (!m_base) {
        m_base = downsampleCloud(filterFinite(m_base_raw), m_voxel);
        m_base_kdtree.setInputCloud(m_base);
    }
    if (!m_base || m_base->empty()) return;
    m_has_base = true;
    m_base_voxels.clear();
    m_base_voxels.reserve(m_base->size());
    for (const auto& p : m_base->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        m_base_voxels.insert(voxelKey(static_cast<int>(std::floor(p.x / m_voxel)),
                                      static_cast<int>(std::floor(p.y / m_voxel)),
                                      static_cast<int>(std::floor(p.z / m_voxel))));
    }
}

// 新结构沉淀: 与 fitness 解耦.
// 新增点没有对应点, 不进 fitness 的分子/分母, 所以"环境变了"不一定会让 fitness 下降;
// 反过来 fitness 低也可能只是稀疏/遮挡. 判据只看几何证据:
//   1) 离底图 > delta_min (防"双层墙"), 底图同体素直接跳过(不解 KD);
//   2) 单帧同体素只算一个候选点;
//   3) 26 邻域最小簇 novel_min_cluster, 滤掉孤立噪点;
//   4) 按 persistency_unit 计票, 够 persist_hits 才写入.
void OverlayMapMaintainer::collectNovel(const PoseStamped& pose, const CloudType::Ptr& body) {
    if (!body || body->empty()) return;
    ensureBaseIndex();
    const int persist = std::max(1, m_cfg.persist_hits);
    const int min_cluster = m_cfg.novel_min_cluster;
    const double dmin2 = m_cfg.delta_min * m_cfg.delta_min;

    // 候选表只增不减会让长期运行内存无界: 每 600 帧清一次"很久没再看到"的候选.
    if (m_frame % 600 == 0 && !m_hits.empty()) {
        for (auto it = m_hits.begin(); it != m_hits.end();) {
            if (m_frame - static_cast<long>(it->second.last_frame) > 600) it = m_hits.erase(it);
            else ++it;
        }
    }

    struct Novel {
        PointType p;
        int cx = 0, cy = 0, cz = 0;
    };
    std::unordered_map<int64_t, Novel> novel;
    novel.reserve(body->size() / 4 + 16);
    // 整帧扫描的体素集合: 最小簇按它算. 已经沉淀/被底图覆盖的部分同样是"邻居",
    // 否则新长出来的一小块会因为邻居都被算成"非新体素"而误判成孤立噪点.
    std::unordered_set<int64_t> scan_voxels;
    scan_voxels.reserve(body->size());
    m_dbg_scan = 0; m_dbg_seen = 0; m_dbg_base = 0; m_dbg_kd = 0; m_dbg_cluster = 0;
    std::vector<int> idx(1);
    std::vector<float> sq(1);
    for (const auto& p : body->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        ++m_dbg_scan;
        const Eigen::Vector3d w = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        const int cx = static_cast<int>(std::floor(w.x() / m_voxel));
        const int cy = static_cast<int>(std::floor(w.y() / m_voxel));
        const int cz = static_cast<int>(std::floor(w.z() / m_voxel));
        const int64_t vk = voxelKey(cx, cy, cz);
        scan_voxels.insert(vk);
        auto sit = m_voxel_seen.find(vk);
        if (sit != m_voxel_seen.end()) { sit->second = m_frame; ++m_dbg_seen; continue; }  // 已沉淀: 刷新存活
        if (novel.find(vk) != novel.end()) continue;                         // 同帧同体素只留一个点
        if (m_base_voxels.count(vk)) { ++m_dbg_base; continue; }   // 底图同一体素已有观测: 不是新结构
        PointType q;
        q.x = static_cast<float>(w.x());
        q.y = static_cast<float>(w.y());
        q.z = static_cast<float>(w.z());
        if (m_has_base && m_base_kdtree.nearestKSearch(q, 1, idx, sq) > 0 &&
            sq[0] <= dmin2) {
            ++m_dbg_kd;
            continue;   // 离底图太近: 位姿误差/重采样差异, 不是新结构
        }
        Novel n;
        n.p = q;
        n.cx = cx;
        n.cy = cy;
        n.cz = cz;
        novel.emplace(vk, n);
    }
    if (novel.empty()) return;

    const uint32_t frame = static_cast<uint32_t>(m_frame);
    for (const auto& kv : novel) {
        const Novel& n = kv.second;
        if (min_cluster > 1) {
            int cnt = 0;
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dz = -1; dz <= 1; ++dz)
                        if (scan_voxels.count(voxelKey(n.cx + dx, n.cy + dy, n.cz + dz))) ++cnt;
            if (cnt < min_cluster) { ++m_dbg_cluster; continue; }   // 孤立点/散点: 不是结构
        }
        CandidateVoxel& cand = m_hits[kv.first];
        bool count = false;
        switch (m_cfg.persistency_unit) {
            case 0:  count = true; break;   // frame: 每帧
            case 1:  // view: 连续看到只算一票, 离开过视野再回来才 +1
                     count = (cand.last_frame == 0) || (frame > cand.last_frame + 1); break;
            case 2:  count = (cand.last_key != m_session_id); break;   // session
            default: count = (cand.last_key != m_day_index); break;    // day
        }
        cand.last_frame = frame;
        if (count) {
            if (cand.votes < 0xFFFF) ++cand.votes;
            if (m_cfg.persistency_unit >= 2) cand.last_key = currentUnitKey();
            ++m_novel_votes;
        }
        if (cand.votes >= persist) {
            addPoint(Eigen::Vector3d(n.p.x, n.p.y, n.p.z));
            m_hits.erase(kv.first);
            ++m_novel_committed;
        }
    }
    // [诊断] 每 100 帧打一次分支计数 (定位"新结构不沉淀")
    if (m_frame % 100 == 0) {
        std::cout << "[maint] scan=" << m_dbg_scan
                  << " seen=" << m_dbg_seen
                  << " base=" << m_dbg_base
                  << " kd=" << m_dbg_kd
                  << " cand=" << novel.size()
                  << " cluster_rej=" << m_dbg_cluster
                  << " votes=" << m_novel_votes
                  << " commits=" << m_novel_committed << std::endl;
    }
}

void OverlayMapMaintainer::flushBuffer() {
    if (m_buf.empty()) return;
    optimizePoses();
    for (auto& k : m_buf) collectNovel(k.pose, k.body);
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

double OverlayMapMaintainer::persistencyCoverage(const PoseStamped& pose,
                                          int* anchor_age, size_t* session_map_pts) const {
    // A 阶段(影子): 统计"当前扫描点中落在 overlay(变化结构)上的点"里, w>=persistency_anchor 的占比.
    // 分母只含变化结构上的点; 纯底图区域不参与 (那里本来就是静态的, 不构成判断依据).
    // 只读, 不改变任何状态; 返回 -1 = 本帧没有可见的变化结构 (日志记 n/a).
    if (anchor_age) *anchor_age = 0;
    if (session_map_pts) *session_map_pts = 0;
    if (!m_last_world || m_last_world->empty()) return -1.0;
    const double radius = m_cfg.radius > 0 ? m_cfg.radius : 60.0;
    const double r2 = radius * radius;
    const int anchor = m_cfg.persistency_anchor > 0 ? m_cfg.persistency_anchor : 128;
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
        auto wit = m_persistency.find(vk);
        if (wit == m_persistency.end()) continue;
        ++total;
        if (static_cast<int>(wit->second.w) >= anchor) {
            ++trusted;
            auto sit = m_voxel_seen.find(vk);
            const long last = (sit == m_voxel_seen.end()) ? 0 : sit->second;
            if (oldest < 0 || last < oldest) oldest = last;
        }
    }
    if (session_map_pts) *session_map_pts = total;
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
            const int64_t vk = voxelKey(vx, vy, vz);
            if (!m_voxel_seen.count(vk)) continue;
            // intensity 带 A 阶段持久度 w(0~255): 0=无记录/刚提交, >=persistency_anchor
            // (默认 128) 才算"高可信静态几何". 调试可视化按它上色, 落盘 PCD 也带这个值.
            PointType q = p;
            auto wit = m_persistency.find(vk);
            q.intensity = (wit == m_persistency.end())
                              ? 0.0f : static_cast<float>(wit->second.w);
            out->push_back(q);
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
        // 恢复持久度: 新版 PCD 把 A 阶段 w 存在 intensity 里 (旧文件是 0 -> 保持本次记的 1 票).
        // 不恢复的话 w 只活在内存里, 每次重启都从 1 票开始, "跨会话确认"永远攒不到 anchor.
        if (p.intensity > 0.0f) {
            const int64_t vk = voxelKey(static_cast<int>(std::floor(p.x / m_voxel)),
                                        static_cast<int>(std::floor(p.y / m_voxel)),
                                        static_cast<int>(std::floor(p.z / m_voxel)));
            auto it = m_persistency.find(vk);
            if (it != m_persistency.end()) {
                it->second.w = static_cast<uint8_t>(std::min(255.0f, p.intensity));
            }
        }
    }
    return m_overlay_count > 0;
}

} // namespace rosiwit_slam
