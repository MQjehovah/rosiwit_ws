// src/algorithms/localization/maintainer/overlay_map_maintainer.cpp
#include "algorithms/localization/maintainer/overlay_map_maintainer.h"
#include "algorithms/localization/localization_utils.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

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
// ---- ELite (Gil et al., ICRA 2025, "Ephemerality meets LiDAR-based Lifelong Mapping") 式 5/6 ----
constexpr double kEliteAlpha = 0.5;
constexpr double kEliteBeta = 0.1;
constexpr double kFullLogit = 2.1972245773362196;   // logit(0.9) = ln 9: x = 0 时的一票 (满强度)
// 式 5: f(x) = min(α(1 − e^(−x²/σ²)) + β, α) —— 观测到地图点的距离为 x 时, 这个观测给出的
//       "瞬态"概率。x = 0 -> 0.1 (强占据证据); x ≥ 1.27σ -> 0.5 (中性: 不提供任何信息)。
// 式 6: ε' = fε / (fε + (1−f)(1−ε)) —— 贝叶斯更新, 在对数几率域里就是加 logit(1 − f)。
// 映射到 w (0~255, 128 = 中性): 满强度一票 (x = 0) = persistency_hit_gain。
inline int evidenceStep(double x, double sigma, int gain) {
    if (!(sigma > 0.0) || !(x >= 0.0) || !(x < 1.2 * sigma)) return 0;
    const double u2 = (x / sigma) * (x / sigma);
    const double f = kEliteAlpha * (1.0 - std::exp(-u2)) + kEliteBeta;
    if (f >= kEliteAlpha) return 0;
    return static_cast<int>(std::lround(gain * std::log((1.0 - f) / f) / kFullLogit));
}
constexpr int kBlkShift = 3;                         // 占用位图: 每块 8^3 体素
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

    m_occ.clear();                 // 账本的派生索引, 随账本一起清
    m_occ_cache_key = -1;
    m_occ_cache_words = nullptr;
    m_map_path = map_path;
    m_voxel = m_cfg.voxel > 0 ? m_cfg.voxel : 0.3;
    m_cell = m_cfg.cell > 0 ? m_cfg.cell : 10.0;
    resolveSigmas();
    // 懒建: 这里只存原始指针, 降采样+KD树推迟到首次真正用到(L147 吸收判据)再建。
    // 全图降采样+建树实测 ~15s(aarch64, DDD), 每次切图都白付 —— 切图(带初始位姿)
    // 根本不碰它 (用户语义: 有 initpose 不需要额外准备, 直接跟踪)。
    m_base_raw = base;
    m_base.reset();
    m_has_base = false;
    m_base_voxels.clear();
    m_base_indexed = false;
    m_dead_base.clear();
    // ---- 统一工作图 (Phase A): 底图体素进同一套 w 账本 ----
    // 底图点不复制进 m_cells (几何仍走底图云的 KD 树, 性能不变), 账本只记 w:
    //   射线穿过 -> w 递减, 到 0 -> 进 m_dead_base -> 匹配目标里剔除对应的底图点
    //   (墙挪了/拆了会被自由空间证据自然吸收 = 方案 §3 的"底图小错可修")
    //   被观测命中 -> w 累积 (固定结构权重随时间变大 = T2 的"固定物体权重更大")
    // 底图与 overlay 同一体素网格 (都是 0.3m), 账本 ≈ 1:1 对应底图点, 不是额外一份几何。
    // 工作图文件 (map.overlay.pcd) 里 intensity 就是 w (w==0 的点落盘时已丢弃);
    // 原图 map.pcd 的 intensity 是反射率, 不能当 w -> 此时全部按 base_voxel_w 播种。
    if (m_base_raw && !m_base_raw->empty()) {
        const int w0 = std::min(255, m_cfg.base_voxel_w > 0 ? m_cfg.base_voxel_w : 128);
        const double v = m_voxel;
        size_t seeded = 0, dead = 0;
        for (const auto& p : m_base_raw->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const int vx = static_cast<int>(std::floor(p.x / v));
            const int vy = static_cast<int>(std::floor(p.y / v));
            const int vz = static_cast<int>(std::floor(p.z / v));
            const int64_t vk = voxelKey(vx, vy, vz);
            if (m_persistency.find(vk) != m_persistency.end()) continue;
            PersistencyVoxel pv;
            pv.from_base = true;
            if (m_base_embedded) {
                const int w = static_cast<int>(std::min(255.0f, std::max(0.0f, p.intensity)));
                if (w <= 0) { ++dead; continue; }   // 理论上不会: 落盘已剔除
                pv.w = static_cast<uint8_t>(w);
            } else {
                pv.w = static_cast<uint8_t>(w0);
            }
            setRep(pv, Eigen::Vector3d(p.x, p.y, p.z), vx, vy, vz);   // 代表点 = 该体素的地图点
            m_persistency.emplace(vk, pv);
            occSet(vx, vy, vz);
            ++seeded;
        }
        std::cout << "[overlay] base voxels in w ledger: " << seeded
                  << " (dead=" << dead << ", ledger=" << m_persistency.size()
                  << ", embedded_weights=" << (m_base_embedded ? 1 : 0) << ")" << std::endl;
    }
    // 没有增量日志: 只有一个全量工作图文件 (map.overlay.pcd, 1h 落盘),
    // 加载时它整体就是底图 (intensity = w)。这里不再读旧的 overlay 增量文件。
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
        m_occ.clear();
        m_occ_cache_key = -1;
        m_occ_cache_words = nullptr;
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
        voteHit(vk, 0.0);           // 重复观测 -> 按计票单位累加
        return;
    }
    m_voxel_seen[vk] = m_frame;
    PersistencyVoxel& pv = m_persistency[vk];           // 新体素从 0 起, 本次观测即一次 hit
    setRep(pv, p, vx, vy, vz);                         // 代表点 = 提交时的扫描端点
    occSet(vx, vy, vz);
    voteHit(vk, 0.0);

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
    // ★w-map (2026-11-26): 提交即升格进工作图云 (intensity=当前 w) —— 目标从
    // 工作图分块生成, 新结构当帧即可参与匹配; 不再有"增量层"与"全量图"的时差。
    if (m_base_raw) {
        PointType bp;
        bp.x = q.x; bp.y = q.y; bp.z = q.z;
        bp.intensity = static_cast<float>(m_persistency[vk].w);
        m_base_raw->push_back(bp);
    }
    if (m_commit_sink) m_commit_sink(p, static_cast<float>(m_persistency[vk].w));
}

void OverlayMapMaintainer::rayCarve(const PoseStamped& pose, const CloudType::Ptr& scan_body) {
    // 自由空间证据 (ELite 式 5 的自由项): 沿"传感器原点 -> 扫描端点"的射线前进, 途经的
    // 账本体素若**离射线足够近** (≤ r_free = 1.2σ_free, 0.3 体素下 6cm) 才记一票自由证据。
    // 旧实现是"采样点落在体素里就扣满 8": 0.3m 体素等于把 ELite 的 σ (2.5cm) 放大十倍,
    // 任何从墙前 30cm 内掠过的射线都给真实墙面记一票 —— 底图被侵蚀的直接原因。
    // 射线末端前一个体素内不采样 (d < len − m_voxel): 端点所在表面附近是"无证据区"
    // (ELite 同样只在射线长度的 20%~90% 之间取自由采样点)。
    if (!scan_body || scan_body->empty()) return;
    // 采样间距 = voxel/3: 必须与 σ_free 同量级 —— 间距 0.3m 时"贴着地图点经过"这件事会被
    // 采样量化吃掉 (鬼影删除慢一个数量级, 仿真验证); 位图预筛让密采样不再付哈希查找。
    const double step = std::max(0.02, m_voxel / 3.0);
    const int stride = std::max(1, m_cfg.carve_stride);
    const double inv = 1.0 / m_voxel;
    const double r2 = m_r_free * m_r_free;
    const uint32_t frame = static_cast<uint32_t>(m_frame);
    m_free_touch.clear();
    const int n = static_cast<int>(scan_body->size());
    // 抽样相位逐帧轮转: 按点序每 stride 个取一条射线, 而扫描点序有周期结构 (环 x 方位)。
    // 周期与 stride 不互素时, 固定相位会永远跳过某些方向 (合成扫描 24 排 / stride 3 ->
    // 只有 z 序号 ≡ 0 的排被采样, 经过目标物的那一排从不采样 —— 鬼影一票证据都收不到)。
    const int phase = stride > 1 ? static_cast<int>(m_frame % stride) : 0;
    // 下标绕回 (i % n): 每帧采样的射线条数与相位无关。直接从 i = phase 起步会在点数少于
    // phase 时 (单测里的单点扫描) 一条都不采。
    for (int i = phase; i < n + phase; i += stride) {
        const auto& p = scan_body->points[i % n];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d end = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        const Eigen::Vector3d dir = end - pose.trans;
        const double len = dir.norm();
        if (len < step * 2) continue;
        const Eigen::Vector3d du = dir * (step / len);
        const double d_end = len - m_voxel;
        Eigen::Vector3d q = pose.trans + du;
        for (double d = step; d < d_end; d += step, q += du) {
            const int vx = static_cast<int>(std::floor(q.x() * inv));
            const int vy = static_cast<int>(std::floor(q.y() * inv));
            const int vz = static_cast<int>(std::floor(q.z() * inv));
            // 空体素一次位测试排除。不变量: m_voxel_seen 的键 ⊆ 账本的键 (两处同增同删),
            // 所以账本里没有的体素也不在 m_voxel_seen 里, 旧的"硬删除"分支无事可做。
            if (!occupied(vx, vy, vz)) continue;
            auto it = m_persistency.find(voxelKey(vx, vy, vz));
            if (it == m_persistency.end()) continue;   // 位图与账本不同步时以账本为准
            PersistencyVoxel& v = it->second;
            const Eigen::Vector3d pp = repPoint(v, vx, vy, vz);
            const double x2 = (q - pp).squaredNorm();
            if (!(x2 < r2)) continue;
            const uint16_t xm = static_cast<uint16_t>(std::sqrt(x2) * 1000.0 + 0.5);
            if (v.last_miss_frame != frame) {          // 本帧第一次: 挂到投票清单
                v.last_miss_frame = frame;
                v.free_x = xm;
                m_free_touch.emplace_back(voxelKey(vx, vy, vz), &v);
            } else if (xm < v.free_x) {                // 同一帧更近的观测: 只取最强的一次
                v.free_x = xm;
            }
        }
    }
    applyFreeVotes();
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

// ============================================================================
// w 证据模型 (2026-11-27, 移植 ELite 式 5/6; docs/SCHEME_PERSISTENT_MAP.md §3)
//
// 旧模型: 命中 = 扫描端点**落在体素里** +8; free = 射线采样点**落在体素里** -8。
// 体素 0.3m 下这两条硬边界一个太松一个太紧: free 票对从墙前 30cm 内掠过的射线全记满,
// 命中票却只认"端点与地图点同体素" —— 今天的扫描与地图差几厘米 (跨体素) 就拿不到命中。
// 实机 (2026-11-27 02:39 起): 沿走廊跑 3 分钟判死 1.06 万个底图点, 23 分钟 1.6 万个,
// fitness 持续下降; 而 cov_base (对未剔除的底图) 始终是平的 —— 不是地图变差, 是自更新
// 的"删除"侧在过度删除。
//
// ELite (Gil et al., ICRA 2025) 的做法 (见文件头 evidenceStep): 证据强度随"观测到地图点
// 的距离"指数衰减, 远了趋于中性; 关联用最近邻而不是体素包含。移植要点:
//   占据证据 (扫描端点):  端点附近 r_hit = 1.2σ_hit 内的**每个**账本体素各记一次
//                         (ELite 的 k 近邻)。σ_hit 的默认值让证据恰好在 inlier_dist 处
//                         归零: 账本的"命中"与匹配器的"内点"是同一个距离判据。
//   自由证据 (射线采样点): 采样点必须从地图点 r_free = 1.2σ_free (voxel/6 -> 6cm) 内经过。
//                         σ 小, free 证据就够不着真实墙面 —— ELite 用 2.5cm 也是这个道理
//                         (我们是在线、每视角一票, 直接照搬 2.5cm 则鬼影删不掉: 仿真验证)。
//   累加: 对数几率域 (式 6); w=128 即 ELite 的 ε 初值 0.5。
//   每个计票单位 (frame/view/session/day) 每个体素最多一票, 同一帧取最强的那次观测。
// 二维仿真 (走廊, 行驶 40m, 200 个视角): 7 种 ±15cm 偏移下墙体判死 0 (旧模型每种偏移
// 死 168~175 个); 墙前 0.5m 的鬼影在第 54~189 个视角被删。
// ============================================================================

void OverlayMapMaintainer::resolveSigmas() {
    const double v = m_cfg.voxel > 0 ? m_cfg.voxel : 0.3;
    // σ_hit: 1.2σ = voxel (= inlier_dist) -> 命中判据与匹配器内点判据一致
    m_sigma_hit = m_cfg.persistency_sigma_hit > 0 ? m_cfg.persistency_sigma_hit : v * (5.0 / 6.0);
    // σ_free: voxel/6 -> 只有贴着地图点经过的射线才算自由证据
    m_sigma_free = m_cfg.persistency_sigma_free > 0 ? m_cfg.persistency_sigma_free : v / 6.0;
    m_r_hit = 1.2 * m_sigma_hit;
    m_r_free = 1.2 * m_sigma_free;
}

namespace {
inline int floorDiv8(int v) { return (v - (v & 7)) >> 3; }   // 负坐标也按向下取整
inline int signExtend21(int64_t v) { return static_cast<int>(v & 0x100000 ? v - 0x200000 : v); }
} // namespace

void OverlayMapMaintainer::occSet(int vx, int vy, int vz) {
    std::array<uint64_t, 8>& w = m_occ[voxelKey(floorDiv8(vx), floorDiv8(vy), floorDiv8(vz))];
    const int idx = ((vx & 7) * 8 + (vy & 7)) * 8 + (vz & 7);
    w[idx >> 6] |= (uint64_t(1) << (idx & 63));
    m_occ_cache_key = -1;          // m_occ 可能刚 rehash: 之前缓存的块指针作废
    m_occ_cache_words = nullptr;
}

void OverlayMapMaintainer::occClear(int vx, int vy, int vz) {
    const int64_t bk = voxelKey(floorDiv8(vx), floorDiv8(vy), floorDiv8(vz));
    auto it = m_occ.find(bk);
    if (it == m_occ.end()) return;
    const int idx = ((vx & 7) * 8 + (vy & 7)) * 8 + (vz & 7);
    it->second[idx >> 6] &= ~(uint64_t(1) << (idx & 63));
    m_occ_cache_key = -1;
    m_occ_cache_words = nullptr;
}

bool OverlayMapMaintainer::occupied(int vx, int vy, int vz) {
    const int64_t bk = voxelKey(floorDiv8(vx), floorDiv8(vy), floorDiv8(vz));
    if (bk != m_occ_cache_key) {
        auto it = m_occ.find(bk);
        m_occ_cache_key = bk;
        m_occ_cache_words = (it == m_occ.end()) ? nullptr : it->second.data();
    }
    if (!m_occ_cache_words) return false;
    const int idx = ((vx & 7) * 8 + (vy & 7)) * 8 + (vz & 7);
    return (m_occ_cache_words[idx >> 6] >> (idx & 63)) & 1u;
}

void OverlayMapMaintainer::setRep(PersistencyVoxel& v, const Eigen::Vector3d& p,
                                  int vx, int vy, int vz) const {
    auto mm = [this](double d) {
        return static_cast<uint16_t>(std::max(0L, std::min(65535L, std::lround(d * 1000.0))));
    };
    v.rx = mm(p.x() - vx * m_voxel);
    v.ry = mm(p.y() - vy * m_voxel);
    v.rz = mm(p.z() - vz * m_voxel);
}

Eigen::Vector3d OverlayMapMaintainer::repPoint(const PersistencyVoxel& v,
                                               int vx, int vy, int vz) const {
    return Eigen::Vector3d(vx * m_voxel + 0.001 * v.rx,
                           vy * m_voxel + 0.001 * v.ry,
                           vz * m_voxel + 0.001 * v.rz);
}

bool OverlayMapMaintainer::hitUnitAllows(const PersistencyVoxel& v) const {
    switch (m_cfg.persistency_unit) {
        case 0:  return true;                              // frame: 每帧
        case 1:  return v.last_view != m_view_id;          // view: 位姿移动超过阈值才算新视角。
                                                         // 静止时只有一个视角, 每体素只计一票。
        case 2:  return v.last_key != m_session_id;        // session
        default: return v.last_key != m_day_index;         // day
    }
}

bool OverlayMapMaintainer::freeUnitAllows(const PersistencyVoxel& v) const {
    switch (m_cfg.persistency_unit) {
        case 0:  return true;
        case 1:  return v.last_miss_view != m_view_id;     // 与 hit 完全对称
        case 2:  return v.last_miss_key != m_session_id;
        default: return v.last_miss_key != m_day_index;
    }
}

void OverlayMapMaintainer::voteHit(int64_t vk, double x) {
    auto it = m_persistency.find(vk);
    if (it == m_persistency.end()) return;
    PersistencyVoxel& v = it->second;
    const int gain = m_cfg.persistency_hit_gain > 0 ? m_cfg.persistency_hit_gain : 1;
    const int d = evidenceStep(x, m_sigma_hit, gain);
    v.last_frame = static_cast<uint32_t>(m_frame);
    v.miss = 0;   // 又看到了: 清空 free 累计
    if (d <= 0 || !hitUnitAllows(v)) return;
    v.last_view = m_view_id;
    if (m_cfg.persistency_unit >= 2) v.last_key = currentUnitKey();
    const int nw = static_cast<int>(v.w) + d;
    v.w = static_cast<uint8_t>(nw > 255 ? 255 : nw);
}

void OverlayMapMaintainer::applyFreeVotes() {
    const int gain = m_cfg.persistency_hit_gain > 0 ? m_cfg.persistency_hit_gain : 1;
    const uint32_t frame = static_cast<uint32_t>(m_frame);
    for (auto& kv : m_free_touch) {
        PersistencyVoxel& v = *kv.second;
        const int d = evidenceStep(static_cast<double>(v.free_x) * 0.001, m_sigma_free, gain);
        if (d <= 0 || !freeUnitAllows(v)) continue;
        // ★占据证据优先 (ELite 跨会话关联的规则: 新会话在该点关联距离内有点 -> 共存, 不删)。
        //   同一计票单位内已有扫描端点落在 r_hit (inlier_dist) 内的体素, 不叠加自由证据。
        //   删除的充要条件于是变成"没有扫描落在它附近, 且射线从它旁边穿过":
        //   - 今天的墙面与地图差 10cm (地图点在真实表面后面 10cm, 射线确实从它的位置穿过):
        //     仍有端点落在它 inlier_dist 内 -> 不衰减。否则每趟经过都掉 ~90, 第二趟就死,
        //     而新结构门控 (delta_min 0.5m) 又不允许在 10cm 外补点 -> 整面墙丢光。
        //   - 挪走的货物 / 挪了 0.5m 的墙: 附近没有端点 -> 自由证据照常衰减 -> 删除。
        //   (旧实现里同一帧的命中 +8 与 free -8 互相抵消, 等价于"命中优先"只在 hit/free 恰好
        //    同帧时成立; 掠射的 free 票常常出现在没有端点落进**同体素**的帧里, 于是净扣。)
        const bool hit_now = (m_cfg.persistency_unit == 0) ? (v.last_frame == frame)
                                                           : !hitUnitAllows(v);
        if (hit_now) continue;
        v.last_miss_view = m_view_id;
        if (m_cfg.persistency_unit >= 2) v.last_miss_key = currentUnitKey();
        const bool from_base = v.from_base;
        v.w = (v.w > d) ? static_cast<uint8_t>(v.w - d) : 0;
        if (v.w == 0) {
            m_persistency.erase(kv.first);
            m_voxel_seen.erase(kv.first);
            const int64_t k = kv.first;
            occClear(signExtend21(k >> 42), signExtend21((k >> 21) & 0x1FFFFF), signExtend21(k & 0x1FFFFF));
            if (from_base) m_dead_base.insert(kv.first);   // 底图体素: 目标里剔除
            else m_pending_remove.insert(kv.first);        // overlay 体素: 点从 m_cells 回收
        }
    }
    m_free_touch.clear();
}

void OverlayMapMaintainer::compactCells() {
    // 批量回收: 把已删除体素的点从 m_cells 移除, 空分块删除。
    // 只在 pending 达到阈值时调用 (摊还成本), 语义 = "@w 降到 0 就去除"。
    if (m_pending_remove.empty()) return;
    size_t removed = 0;
    for (auto it = m_cells.begin(); it != m_cells.end();) {
        auto& cell = it->second;
        if (!cell) { it = m_cells.erase(it); continue; }
        auto& pts = cell->points;
        size_t keep = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const auto& p = pts[i];
            const int vx = static_cast<int>(std::floor(p.x / m_voxel));
            const int vy = static_cast<int>(std::floor(p.y / m_voxel));
            const int vz = static_cast<int>(std::floor(p.z / m_voxel));
            if (m_pending_remove.count(voxelKey(vx, vy, vz))) { ++removed; continue; }
            pts[keep++] = pts[i];
        }
        pts.resize(keep);
        cell->width = static_cast<uint32_t>(keep);
        it = (keep == 0) ? m_cells.erase(it) : std::next(it);
    }
    if (m_overlay_count >= removed) m_overlay_count -= removed;
    else m_overlay_count = 0;
    m_pending_remove.clear();
    if (removed > 0) ++m_revision;
}

int OverlayMapMaintainer::persistencyAt(const Eigen::Vector3d& p) const {
    const int64_t vk = voxelKey(static_cast<int>(std::floor(p.x() / m_voxel)),
                                static_cast<int>(std::floor(p.y() / m_voxel)),
                                static_cast<int>(std::floor(p.z() / m_voxel)));
    auto it = m_persistency.find(vk);
    return it == m_persistency.end() ? -1 : static_cast<int>(it->second.w);
}

void OverlayMapMaintainer::observeHits(const PoseStamped& pose, const CloudType::Ptr& scan_body) {
    // A 阶段 (ELite 式 5/6): 占据证据 = 扫描端点, 记给**端点附近**的每个账本体素 (r_hit 内),
    // 强度随距离衰减 (见上方"w 证据模型"说明)。必须在雕刻之前: 同一帧里被命中的体素先拿到
    // 占据证据。每个计票单位每体素最多一票, 同一帧取最强的那次观测。
    //
    // 代价: 端点附近 27 个邻格里的账本体素都是候选。逐端点查 27 次哈希 = 8000×27×150ns
    // ≈ 32ms/帧 (实机 260 万条账本), 不可接受; 而 27 格里绝大多数是空的。所以
    //   1) 先按扫描体素分组 (同一扫描体素里的端点共用一组候选), 每组只收集一次候选;
    //   2) 候选收集用占用位图预筛, 只有确实在账本里的邻格才做哈希查找。
    // 新结构仍走 collectNovel 的持久化门控 (按 persistency_unit 计票) —— w 只衡量持久结构
    // 被重复看到的次数, 不会把瞬时点云抬成高可信。
    if (!m_last_world) m_last_world.reset(new CloudType());
    else m_last_world->clear();
    m_groups.clear();
    m_cand.clear();
    if (!scan_body || scan_body->empty()) return;
    const double inv = 1.0 / m_voxel;
    const double r2 = m_r_hit * m_r_hit;
    for (const auto& p : scan_body->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3d e = pose.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose.trans;
        const int vx = static_cast<int>(std::floor(e.x() * inv));
        const int vy = static_cast<int>(std::floor(e.y() * inv));
        const int vz = static_cast<int>(std::floor(e.z() * inv));
        const int64_t vk = voxelKey(vx, vy, vz);
        Group& g = m_groups[vk];
        if (!g.scanned) {
            g.scanned = true;
            g.off = static_cast<uint32_t>(m_cand.size());
            g.n = 0;
            for (int a = -1; a <= 1; ++a)
                for (int b = -1; b <= 1; ++b)
                    for (int c = -1; c <= 1; ++c) {
                        const int nx = vx + a, ny = vy + b, nz = vz + c;
                        if (!occupied(nx, ny, nz)) continue;
                        const int64_t nk = voxelKey(nx, ny, nz);
                        auto it = m_persistency.find(nk);
                        if (it == m_persistency.end()) continue;   // 位图与账本不同步时以账本为准
                        Cand cd;
                        cd.key = nk;
                        cd.v = &it->second;
                        cd.p = repPoint(it->second, nx, ny, nz);
                        cd.best2 = r2;
                        m_cand.push_back(cd);
                        ++g.n;
                        if (a == 0 && b == 0 && c == 0) g.own = true;
                    }
            if (g.own) {   // 这个扫描体素自己就是账本体素: 每帧记一个点, 供 persistencyCoverage 统计
                m_voxel_seen[vk] = m_frame;   // 被再观测: 刷新存活时间
                PointType q;
                q.x = static_cast<float>(e.x());
                q.y = static_cast<float>(e.y());
                q.z = static_cast<float>(e.z());
                m_last_world->push_back(q);
            }
        }
        // 本帧最近的观测 (强度最高): 同一扫描体素里的端点共用这组候选, 只有距离计算
        for (uint32_t i = g.off; i < g.off + g.n; ++i) {
            Cand& cd = m_cand[i];
            const double dx = e.x() - cd.p.x(), dy = e.y() - cd.p.y(), dz = e.z() - cd.p.z();
            const double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < cd.best2) {
                cd.best2 = d2;
                cd.has = true;
            }
        }
    }
    // 帧末统一投票
    const int gain = m_cfg.persistency_hit_gain > 0 ? m_cfg.persistency_hit_gain : 1;
    for (auto& cd : m_cand) {
        if (!cd.has) continue;
        PersistencyVoxel& v = *cd.v;
        const int d = evidenceStep(std::sqrt(cd.best2), m_sigma_hit, gain);
        v.last_frame = static_cast<uint32_t>(m_frame);
        v.miss = 0;
        if (d <= 0 || !hitUnitAllows(v)) continue;
        v.last_view = m_view_id;
        if (m_cfg.persistency_unit >= 2) v.last_key = currentUnitKey();
        const int nw = static_cast<int>(v.w) + d;
        v.w = static_cast<uint8_t>(nw > 255 ? 255 : nw);
        m_voxel_seen[cd.key] = m_frame;   // 被再观测 (含邻近体素): 刷新存活时间
    }
    // Cand 里是指向账本条目的指针: 下一步 rayCarve 可能删掉条目 —— 用完立即清, 不留悬空指针。
    m_groups.clear();
    m_cand.clear();
}

void OverlayMapMaintainer::onLocalizedLocked(const PoseStamped& pose, const CloudType::Ptr& scan_body,
                                       const MatchResult& match, bool pose_confident) {
    if (!m_cfg.enabled || !scan_body || scan_body->empty()) return;
    ++m_frame;
    // 内存回收 (#6) 交给落盘时刻 (1h 一次, 全量扫一遍): 逐条回收太贵 (见 reclaim())。
    // 这里只留一个极大上限的安全阀, 防止长期不落盘时清单无限增长 (只有 key, 很小)。
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
    // 落盘只有一条路径: 管道在 1h 定时器里调用 reclaim() 并写全量工作图 (map.overlay.pcd)。

    // 过桥/漂移期(位姿不可信): 不写、不雕刻, 避免污染地图
    if (!pose_confident) { m_low_count = 0; return; }

    // 视角推进 (view 计票单位): 位姿移动 > 0.2m 或 > 2° 才算新视角。
    if (!m_view_valid) {
        m_view_pose = pose;
        m_view_valid = true;
    } else {
        const double dxy = (pose.trans - m_view_pose.trans).norm();
        const Eigen::Matrix3d dR = pose.rot * m_view_pose.rot.transpose();
        const double dang =
            std::acos(std::max(-1.0, std::min(1.0, (dR.trace() - 1.0) * 0.5))) *
            57.2957795;
        if (dxy > 0.2 || dang > 2.0) {
            ++m_view_id;
            if (m_view_id == 0) m_view_id = 1;
            m_view_pose = pose;
        }
    }
    // A 阶段: 观测命中 -> w += 8。必须先于雕刻: 同一帧里被命中的体素
    // 立即更新上次视角, 雕刻的 view 门随即把它排除 (不被同帧自由票误伤)。
    observeHits(pose, scan_body);
    // 清除: 看到"本该有货的地方是空的" -> 移除该处 overlay
    rayCarve(pose, scan_body);

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
    // [诊断] 每 100 帧打一次分支计数 (定位"新结构不沉淀")。是全局状态, 不依赖本帧有没有新结构候选
    // (下面 novel.empty() 会提前返回), 所以先定义、两条路径都调。
    auto log_status = [&]() {
        if (m_frame % 100 != 0) return;
        std::cout << "[maint] scan=" << m_dbg_scan
                  << " seen=" << m_dbg_seen
                  << " base=" << m_dbg_base
                  << " kd=" << m_dbg_kd
                  << " cand=" << novel.size()
                  << " cluster_rej=" << m_dbg_cluster
                  << " votes=" << m_novel_votes
                  << " commits=" << m_novel_committed
                  // 判死的底图体素总数 (全局, 精确): 账本是否在侵蚀底图的直接度量
                  // (tools/drive_report.py)。行驶几分钟就从 0 涨到上万 = 在删真实的墙。
                  << " dead_base=" << m_dead_base.size() << std::endl;
    };
    if (novel.empty()) {
        log_status();
        return;
    }

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
    log_status();
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

CloudType::Ptr OverlayMapMaintainer::getOverlayLocked(const PoseStamped& pose) {
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
                    const int64_t vk = voxelKey(vx, vy, vz);
                    if (!m_voxel_seen.count(vk)) continue;      // 已衰减/被雕刻
                    // 只让"高可信度"的 overlay 点参与匹配 (w>=match_w_min);
                    // 低于门槛的是虚影/单次观测, 只留在调试云里, 不进目标。
                    if (m_cfg.match_w_min > 0) {
                        auto pit = m_persistency.find(vk);
                        if (pit == m_persistency.end() ||
                            static_cast<int>(pit->second.w) < m_cfg.match_w_min) continue;
                    }
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

void OverlayMapMaintainer::popOverlayDeltaLocked(CloudType::Ptr& out) {
    out = m_pending_store;
    m_pending_store.reset();
}

// w-map 段级融合 (走出子图覆盖时的一次性记账):
//   命中票  -> 该段观测到的地图体素 w += gain (确认/成熟)
//   自由票  -> 射线穿过但段内无命中的底图体素 w -= gain (挪走/消失的衰减)
//   新结构  -> 离底图 > delta_min 的点按 persist_hits 提交 (融进工作图)
// 每种证据均与命中/雕刻同源, 只是打包一次性处理。


// w-map 段级融合 (走出子图覆盖时的一次性记账):
//   命中票  -> 该段观测到的地图体素 w += gain (确认/成熟)
//   自由票  -> 射线穿过但段内无命中的底图体素 w -= gain (挪走/消失的衰减)
//   新结构  -> 离底图 > delta_min 的点按 persist_hits 提交 (融进工作图)
// 每种证据均与命中/雕刻同源, 只是打包一次性处理。
void OverlayMapMaintainer::mergeChunkLocked(const PoseStamped& pose, const CloudType::Ptr& world) {
    if (!world || world->empty()) return;
    // world -> body (用该块自己的注册位姿逆变换), 之后走与普通帧完全相同的账目
    Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity();
    Ti.block<3, 3>(0, 0) = pose.rot.transpose();
    Ti.block<3, 1>(0, 3) = -pose.rot.transpose() * pose.trans;
    CloudType::Ptr body(new CloudType());
    pcl::transformPointCloud(*world, *body, Ti);
    observeHits(pose, body);      // 命中票 (w += gain, view 门控)
    rayCarve(pose, body);         // 自由票 (w -= gain, 对称计票)
    collectNovel(pose, body);     // 新结构提交 (persist_hits 门控)
}

CloudType::Ptr OverlayMapMaintainer::snapshotOverlayLocked() {
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

bool OverlayMapMaintainer::saveLocked(const std::string& path) {
    CloudType::Ptr all = snapshotOverlay();
    if (!all || all->empty()) return false;
    // 只存 x/y/z/intensity(=A 阶段持久度 w), 与调试话题字段一致:
    // normal_*/curvature 在叠加层里恒为 0 (从不算法向量), 存了只是误导 + 文件翻倍.
    pcl::PointCloud<pcl::PointXYZI> out;
    out.reserve(all->size());
    for (const auto& p : all->points) {
        pcl::PointXYZI q;
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
        out.push_back(q);
    }
    out.width = static_cast<uint32_t>(out.size());
    out.height = 1;
    out.is_dense = false;
    return pcl::io::savePCDFileBinary(path, out) == 0;
}

bool OverlayMapMaintainer::loadLocked(const std::string& path) {
    // 兼容旧文件: 旧的 8 字段 PCD (含 normal_*/curvature) 也能按 xyzi 读出来.
    pcl::PointCloud<pcl::PointXYZI> in;
    if (pcl::io::loadPCDFile(path, in) != 0) return false;
    for (const auto& p : in.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int64_t vk0 = voxelKey(static_cast<int>(std::floor(p.x / m_voxel)),
                                     static_cast<int>(std::floor(p.y / m_voxel)),
                                     static_cast<int>(std::floor(p.z / m_voxel)));
        // 统一工作图: 该体素已在底图账本里 (from_base) = 该结构已经烘焙进工作图文件,
        // 不再作为 overlay 重复加入 (否则同一结构两套点, 目标里双层冗余)。
        auto bit = m_persistency.find(vk0);
        if (bit != m_persistency.end() && bit->second.from_base) continue;
        addPoint(Eigen::Vector3d(p.x, p.y, p.z));
        // 恢复持久度: PCD 的 intensity 就是 A 阶段 w (旧文件是 0 -> 保持本次记的 1 票).
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

// ---- 统一工作图: 一个文件同时装几何和权重 (intensity = w) ----
// 落盘前调用: 给底图点写上它所在体素的 w, 丢弃 w==0 的点 -> 跨会话"删除"生效,
// 不需要额外的 sidecar 文件 (w 就在 PCD 里, 和 overlay 的表示完全一致)。
void OverlayMapMaintainer::annotateBaseWeightsRange(CloudType& cloud, size_t begin,
                                                    size_t end) const {
    // 只写 intensity, 不改点数: 调用方可以把它切成有界的小片分多次调用 (工作图落盘:
    // 账本只有跟踪线程在写, 需要账本的这一步就由跟踪线程在自己的帧预算内分片完成,
    // 见 localization_pipeline.cpp mapSaveWorker 上方注释)。持锁范围 = 调用方给的这一片。
    if (m_voxel <= 0.0) return;
    const int w_default = m_cfg.base_voxel_w > 0 ? m_cfg.base_voxel_w : 128;
    std::lock_guard<std::recursive_mutex> lk(m_mu);
    auto& pts = cloud.points;
    end = std::min(end, pts.size());
    for (size_t i = begin; i < end; ++i) {
        PointType& p = pts[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            p.intensity = kAnnotateDrop;
            continue;
        }
        const int64_t vk = voxelKey(static_cast<int>(std::floor(p.x / m_voxel)),
                                    static_cast<int>(std::floor(p.y / m_voxel)),
                                    static_cast<int>(std::floor(p.z / m_voxel)));
        int w = w_default;
        auto it = m_persistency.find(vk);
        if (it != m_persistency.end()) w = it->second.w;
        p.intensity = (w <= 0) ? kAnnotateDrop                      // 已删除: 不写进工作图
                               : static_cast<float>(std::min(255, w));
    }
}

// 匹配目标用: 剔除 w 已降到 0 的底图点 (墙挪了/拆了 -> 射线证据判死)。
// 无死亡体素时原样返回, 不复制。
CloudType::Ptr OverlayMapMaintainer::filterDeadBaseLocked(const CloudType::Ptr& cloud) const {
    if (!cloud || m_dead_base.empty() || m_voxel <= 0.0) return cloud;
    CloudType::Ptr out(new CloudType());
    out->reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int64_t vk = voxelKey(static_cast<int>(std::floor(p.x / m_voxel)),
                                    static_cast<int>(std::floor(p.y / m_voxel)),
                                    static_cast<int>(std::floor(p.z / m_voxel)));
        if (m_dead_base.count(vk)) continue;
        out->push_back(p);
    }
    return out;
}

} // namespace rosiwit_slam
