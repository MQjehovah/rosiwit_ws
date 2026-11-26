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
        const Eigen::Vector3d c = k.pose.rot * k.centroid + k.pose.trans;
        if ((c - ref).squaredNorm() > r2) { any_prune = true; break; }
    }
    if (!any_prune) return;
    std::vector<EpKf> nk;
    nk.reserve(m_kfs.size());
    for (auto& k : m_kfs) {
        const Eigen::Vector3d c = k.pose.rot * k.centroid + k.pose.trans;
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
    const uint32_t frame = static_cast<uint32_t>(m_frame);

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
        for (const auto& p : scan_body.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const Eigen::Vector3d w =
                pose_world.rot * Eigen::Vector3d(p.x, p.y, p.z) + pose_world.trans;
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
        const Eigen::Vector3d c = k.pose.rot * k.centroid + k.pose.trans;
        if ((c.head<2>() - cur_xy.head<2>()).squaredNorm() > r2) { any_leaving = true; break; }
    }
    if (!any_leaving) return out;
    std::vector<EpKf> keep;
    keep.reserve(m_kfs.size());
    for (auto& k : m_kfs) {
        const Eigen::Vector3d c = k.pose.rot * k.centroid + k.pose.trans;
        if ((c.head<2>() - cur_xy.head<2>()).squaredNorm() > r2) out.push_back(std::move(k));
        else keep.push_back(std::move(k));
    }
    m_kfs.swap(keep);
    view_dirty_ = true;
    std::cout << "[submap] leaving chunk: " << out.size() << " kfs merged out (remained "
              << m_kfs.size() << ")" << std::endl;
    return out;
}

CloudType::Ptr SessionMap::pointsWorld() const {
    CloudType::Ptr out(new CloudType());
    if (!active_ || m_kfs.empty()) return out;
    if (view_ && !view_dirty_) return view_;
    // 合并视图: 全体关键帧点 → 0.3 体素去重 → free>=free_votes_to_clear 跳过
    const double vv = std::max(0.05, cfg_.voxel);
    std::unordered_set<int64_t> emitted;
    emitted.reserve(m_kfs.size() * 4);
    out->reserve(m_kfs.size() * 8);
    for (const auto& k : m_kfs) {
        if (!k.world) continue;
        for (const auto& p : k.world->points) {
            const int64_t vk = vkey(p.x, p.y, p.z, vv);
            if (!emitted.insert(vk).second) continue;
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



}  // namespace rosiwit_slam
