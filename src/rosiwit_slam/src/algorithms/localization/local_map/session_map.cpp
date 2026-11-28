// ============================================================================
// session_map.cpp — v2 (ROLL 式段级临时建图), 见 session_map.h 顶部说明
// ============================================================================
#include "algorithms/localization/local_map/session_map.h"

#include <cmath>
#include <iostream>

namespace rosiwit_slam {

static int64_t vkey(double x, double y, double z, double v) {
    const int64_t ix = static_cast<int64_t>(std::floor(x / v));
    const int64_t iy = static_cast<int64_t>(std::floor(y / v));
    const int64_t iz = static_cast<int64_t>(std::floor(z / v));
    return ((ix & 0x1FFFFF) << 42) | ((iy & 0x1FFFFF) << 21) | (iz & 0x1FFFFF);
}

// vkey 把坐标截成 21 位有符号; 反解时补回符号位 (只用于按距离裁剪命中表)。
static int64_t signExtend21(int64_t v) {
    v &= 0x1FFFFF;
    return (v & 0x100000) ? (v | ~int64_t(0x1FFFFF)) : v;
}

void SessionMap::reset(const PoseStamped& robot, double now_s) {
    anchor_ = robot;               // 兼容字段: 开图位姿
    robot_trans_ = robot.trans;    // 兼容旧成员语义
    created_s_ = now_s;
    active_ = true;
    have_robot_ = true;
    capacity_warned_ = false;
    m_kfs.clear();
    m_frames_since_kf_ = 0;
    m_frame = 0;
    kf_hits_.clear();
    view_.reset();
    view_dirty_ = true;
    // 开图即建第一个关键帧 (锚定在当前估计位姿)
    EpKf k;
    k.pose = robot;
    k.world.reset(new CloudType());
    m_kfs.push_back(std::move(k));
    std::cout << "[submap] v2 opened: anchor=(" << robot.trans.x() << ", "
              << robot.trans.y() << ") radius=" << cfg_.radius << std::endl;
}

void SessionMap::deactivate() {
    active_ = false;
    m_kfs.clear();
    m_frame = 0;
    kf_hits_.clear();
    view_.reset();
    view_dirty_ = true;
}

void SessionMap::pruneKfs() {
    // 关键帧级裁剪: 质心超出 radius×1.2 的关键帧整体丢弃 (内存恒定)
    const double r = cfg_.radius * 1.2;
    const double r2 = r * r;
    Eigen::Vector3d ref = m_kfs.back().centroid;   // 以最新关键帧为参考
    if (have_robot_) ref = robot_trans_;
    // ★先判有没有要裁的帧, 没有就原样返回、绝不碰 m_kfs。
    //   不能先 move 再按条件 swap: 没裁剪时 nk.size()==m_kfs.size() 不 swap,
    //   m_kfs 里全是 move 后的空壳 (world==nullptr), 下一帧 update() 里
    //   push_back 直接 SIGSEGV (2026-11-26 实机: 子图一开必崩, 根因即此)。
    bool any_prune = false;
    for (const auto& k : m_kfs) {
        // centroid 在 update() 里由世界系点累加得到 (acc += w), 不要再乘 kf.pose:
        // 旧写法把世界系质心又当成本体系变换了一次, 机器人离地图原点 >radius 时
        // (c ≈ 2*位置) 会把全部关键帧裁掉 -> 子图在远处永远是空的 (2026-11-28 发现)。
        const Eigen::Vector3d c = k.centroid;
        if ((c - ref).squaredNorm() > r2) { any_prune = true; break; }
    }
    if (!any_prune) return;
    std::vector<EpKf> nk;
    nk.reserve(m_kfs.size());
    for (auto& k : m_kfs) {
        const Eigen::Vector3d c = k.centroid;
        if ((c - ref).squaredNorm() > r2) continue;
        nk.push_back(std::move(k));
    }
    m_kfs.swap(nk);
    view_dirty_ = true;
}

void SessionMap::update(const CloudType& scan_body, const PoseStamped& pose_world) {
    if (!active_ || scan_body.empty()) return;
    robot_trans_ = pose_world.trans;
    robot_rot_ = pose_world.rot;
    have_robot_ = true;
    ++m_frame;

    // 当前关键帧: 不存在/移动超距/帧数超限 → 新建
    bool new_kf = m_kfs.empty();
    if (!new_kf) {
        const Eigen::Vector3d dp = pose_world.trans - m_kfs.back().pose.trans;
        new_kf = (dp.norm() >= cfg_.kf_min_dist) ||
                 (m_frames_since_kf_ >= cfg_.kf_min_frames);
    }
    if (new_kf) {
        EpKf k;
        k.pose = pose_world;
        k.world.reset(new CloudType());
        m_kfs.push_back(std::move(k));
        m_frames_since_kf_ = 0;
        view_dirty_ = true;
    }
    // ★一个关键帧只存一次扫描 (2026-11-26 实机: 子图"全是糊的"):
    //   旧实现把**每一帧**的扫描都按当时的世界位姿堆进同一个关键帧。位姿每帧
    //   有零点几度的抖动/融合滞后, 几百帧叠加就在同一面墙上形成"毛边图层",
    //   厚度随距离放大 —— 子图作为匹配目标完全不可用。
    //   ROLL 式临时建图: 关键帧只在其创建时记录一帧扫描, 帧间不再堆积;
    //   子图 = 少量关键帧扫描的并集, 每个关键帧内部自洽 (无跨帧位姿抖动)。
    if (new_kf) {
        EpKf& kf = m_kfs.back();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        size_t npts = 0;
        // 动态点剔除: 统计每个体素在多少个**关键帧**里出现过 (同一关键帧内先按体素去重,
        // 否则一帧里同一个行人几十个点就把票数堆满了)。缺失/少于阈值 => 瞬态, 不进视图。
        const bool count_hits = (cfg_.min_kf_hits > 1);
        std::unordered_set<int64_t> seen;
        if (count_hits) seen.reserve(scan_body.size() * 2 + 1);
        for (const auto& p : scan_body.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const Eigen::Vector3d w =
                pose_world.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose_world.trans;
            if (count_hits) {
                const int64_t vk = vkey(w.x(), w.y(), w.z(), cfg_.voxel);
                if (seen.insert(vk).second) {
                    uint8_t& h = kf_hits_[vk];
                    if (h < 255) ++h;
                }
            }
            PointType q;
            q.x = static_cast<float>(w.x());
            q.y = static_cast<float>(w.y());
            q.z = static_cast<float>(w.z());
            kf.world->push_back(q);
            acc += w;
            ++npts;
        }
        if (npts) kf.centroid = acc / static_cast<double>(npts);
    }
    view_dirty_ = true;

    // 关键帧级裁剪: 质心超出 radius×1.2 的关键帧整体丢弃
    pruneKfs();
    maybePruneHits();
}

size_t SessionMap::applyExitCorrection(const PoseStamped& reliable) {
    if (m_kfs.empty()) return 0;
    // ROLL Fig.3 简化闭式解: 里程计因子把关键帧串联, 两端各一个全局匹配因子。
    // 当只有末端一个可靠位姿时, 最优解 = 把末端残差按序号线性分摊到全部关键帧
    // (漂移平滑假设下的线性摊平 ≈ GTSAM 解, 免依赖)。
    const size_t n = m_kfs.size();
    const Eigen::Vector3d d_last = reliable.trans - m_kfs.back().pose.trans;
    const Eigen::Matrix3d dR_last = reliable.rot * m_kfs.back().pose.rot.transpose();
    Eigen::Quaterniond qd(dR_last);
    qd.normalize();
    for (size_t i = 0; i < n; ++i) {
        const double a = (n > 1) ? static_cast<double>(i) / static_cast<double>(n - 1) : 1.0;
        m_kfs[i].pose.trans += a * d_last;
        Eigen::Quaterniond q0(m_kfs[i].pose.rot);
        q0 = q0.slerp(a, qd).normalized();   // Eigen 3.3: a.slerp(t, b)
        m_kfs[i].pose.rot = q0.toRotationMatrix();
    }
    view_dirty_ = true;
    return n;
}

void SessionMap::applyCorrection(const Eigen::Matrix4d& C) {
    if (!active_) return;
    for (auto& k : m_kfs) {
        k.pose.rot = C.block<3, 3>(0, 0) * k.pose.rot;
        k.pose.trans = C.block<3, 3>(0, 0) * k.pose.trans + C.block<3, 1>(0, 3);
    }
    view_dirty_ = true;
}



std::vector<SessionMap::EpKf> SessionMap::popLeavingKfs(const Eigen::Vector3d& cur_xy) {
    // 走出覆盖: 关键帧质心距当前机器人位置 > radius×1.2 → 整帧退出。
    // 退出前管道会对每帧做一次「融合+雕刻」(w 账本记账), 然后这些帧不再属于子图。
    std::vector<EpKf> out;
    const double r2 = (cfg_.radius * 1.2) * (cfg_.radius * 1.2);
    // ★先判有没有要退出的帧, 没有就原样返回、绝不碰 m_kfs。
    //   不能先 move 再按条件 swap: 没有退出帧时 m_kfs 会剩下 move 后的空壳
    //   (world==nullptr), 下一帧 update() 里 push_back 直接 SIGSEGV。
    bool any_leaving = false;
    for (const auto& k : m_kfs) {
        const Eigen::Vector3d c = k.centroid;   // 世界系质心, 见 pruneKfs 注释
        if ((c.head<2>() - cur_xy.head<2>()).squaredNorm() > r2) { any_leaving = true; break; }
    }
    if (!any_leaving) return out;
    std::vector<EpKf> keep;
    keep.reserve(m_kfs.size());
    for (auto& k : m_kfs) {
        const Eigen::Vector3d c = k.centroid;
        if ((c.head<2>() - cur_xy.head<2>()).squaredNorm() > r2) out.push_back(std::move(k));
        else keep.push_back(std::move(k));
    }
    m_kfs.swap(keep);
    view_dirty_ = true;
    // 动态点剔除: 送出融合前剔掉"只被 1 个关键帧看到"的瞬态点, 否则行人/门会被
    // mergeChunk 当成新结构写进 w 账本 (先污染再靠雕刻慢慢删)。
    if (cfg_.min_kf_hits > 1) {
        size_t dropped = 0;
        for (auto& k : out) {
            if (!k.world || k.world->empty()) continue;
            CloudType::Ptr kept(new CloudType());
            kept->reserve(k.world->size());
            for (const auto& p : k.world->points) {
                if (voxelTransient(vkey(p.x, p.y, p.z, cfg_.voxel))) { ++dropped; continue; }
                kept->push_back(p);
            }
            kept->width = static_cast<uint32_t>(kept->size());
            kept->height = 1;
            k.world.swap(kept);
        }
        if (dropped)
            std::cout << "[submap] drop " << dropped << " transient pts before merge" << std::endl;
    }
    std::cout << "[submap] leaving chunk: " << out.size() << " kfs merged out (remained "
              << m_kfs.size() << ")" << std::endl;
    return out;
}

CloudType::Ptr SessionMap::pointsWorld() const {
    CloudType::Ptr out(new CloudType());
    if (!active_ || m_kfs.empty()) return out;
    if (view_ && !view_dirty_) return view_;
    // 合并视图: 全体关键帧点 → 体素去重 → 动态瞬态体素跳过 (最新关键帧例外)
    const double vv = std::max(0.05, cfg_.voxel);
    const bool filter_transient = (cfg_.min_kf_hits > 1);
    const size_t newest = m_kfs.size() - 1;   // 最新关键帧不过滤: 子图刚开/刚移动时视图不为空
    std::unordered_set<int64_t> emitted;
    emitted.reserve(m_kfs.size() * 4);
    out->reserve(m_kfs.size() * 8);
    for (size_t ki = 0; ki < m_kfs.size(); ++ki) {
        const auto& k = m_kfs[ki];
        if (!k.world) continue;
        const bool is_newest = (ki == newest);
        for (const auto& p : k.world->points) {
            const int64_t vk = vkey(p.x, p.y, p.z, vv);
            if (!emitted.insert(vk).second) continue;
            if (filter_transient && !is_newest &&
                voxelTransient(vkey(p.x, p.y, p.z, cfg_.voxel))) continue;
            PointType q;
            q.x = p.x; q.y = p.y; q.z = p.z;
            q.intensity = 0.0f;
            out->push_back(q);
        }
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = false;
    if (out->size() > cfg_.max_points) {
        if (!capacity_warned_) {
            std::cout << "[submap] view cap reached (" << cfg_.max_points
                      << "), keep newest" << std::endl;
            capacity_warned_ = true;
        }
        CloudType::Ptr keep(new CloudType());
        const size_t total = out->size();
        const size_t st = std::max<size_t>(1, total / cfg_.max_points);
        for (size_t i = 0; i < total; i += st) keep->push_back(out->points[i]);
        out = keep;
    }
    view_ = out;
    view_dirty_ = false;
    return view_;
}

CloudType::Ptr SessionMap::pointsWorld(const Eigen::Vector3d& center, double radius) const {
    CloudType::Ptr full = pointsWorld();
    CloudType::Ptr out(new CloudType());
    if (!full || full->empty()) return out;
    const double r2 = radius * radius;
    for (const auto& p : full->points) {
        const double dx = p.x - center.x(), dy = p.y - center.y();
        if (dx * dx + dy * dy <= r2) out->push_back(p);
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    return out;
}

bool SessionMap::voxelTransient(int64_t vk) const {
    if (cfg_.min_kf_hits <= 1) return false;
    auto it = kf_hits_.find(vk);
    // 缺记录 = 已按距离裁掉/从未统计 -> 按"支持"处理, 宁可保留也不误删。
    return it != kf_hits_.end() && it->second < cfg_.min_kf_hits;
}

void SessionMap::maybePruneHits() {
    if (cfg_.min_kf_hits <= 1) {
        if (!kf_hits_.empty()) kf_hits_.clear();
        return;
    }
    // 条目数 = 走过区域的体素数 (关键帧裁剪不会回收它们)。超过上限时按距离裁:
    // 只删机器人 radius×1.5 以外的条目; 被裁掉 = missing -> 按支持处理, 不会误删
    // 仍在保留关键帧里的点。
    const size_t cap = std::max<size_t>(200000, cfg_.max_points * 2);
    if (kf_hits_.size() <= cap) return;
    const double r = cfg_.radius * 1.5;
    const double r2 = r * r;
    const double v = cfg_.voxel;
    const double rx = have_robot_ ? robot_trans_.x() : 0.0;
    const double ry = have_robot_ ? robot_trans_.y() : 0.0;
    size_t erased = 0;
    for (auto it = kf_hits_.begin(); it != kf_hits_.end();) {
        const int64_t k = it->first;
        const double x = (static_cast<double>(signExtend21(k >> 42)) + 0.5) * v;
        const double y = (static_cast<double>(signExtend21((k >> 21) & 0x1FFFFF)) + 0.5) * v;
        const double dx = x - rx, dy = y - ry;
        if (dx * dx + dy * dy > r2) {
            it = kf_hits_.erase(it);
            ++erased;
        } else {
            ++it;
        }
    }
    std::cout << "[submap] prune hit table: -" << erased << " -> " << kf_hits_.size()
              << " (transient filter)" << std::endl;
}

}  // namespace rosiwit_slam
