// src/algorithms/visual/visual_constraint_source.cpp
#include "algorithms/visual/visual_constraint_source.h"
#include <cmath>

namespace rosiwit_slam {

void VisualConstraintSource::addVisualOdometry(double time, const PoseStamped& vo_pose) {
    Sample s;
    s.time = time;
    s.pose = vo_pose;
    s.pose.time = time;
    m_vo.push_back(s);
    // 限制缓存 (只需覆盖最近若干关键帧)
    while (m_vo.size() > 500) m_vo.pop_front();
}

void VisualConstraintSource::addKeyframe(const std::string& id, double time) {
    m_kfs.emplace_back(id, time);

    if (m_kfs.size() >= 2) {
        const auto& a = m_kfs[m_kfs.size() - 2];
        const auto& b = m_kfs.back();
        PoseStamped va, vb;
        if (nearestVO(a.second, va) && nearestVO(b.second, vb)) {
            Constraint c;
            c.from_kf = a.first;
            c.to_kf = b.first;
            c.relative_pose.time = b.second;
            c.relative_pose.rot = va.rot.transpose() * vb.rot;
            c.relative_pose.trans = va.rot.transpose() * (vb.trans - va.trans);
            c.cov = m_cfg.vo_trans_noise;
            m_pending.push_back(c);
        }
    }
    while (m_kfs.size() > 500) m_kfs.pop_front();
}

bool VisualConstraintSource::nearestVO(double t, PoseStamped& out) const {
    double best_dt = m_cfg.assoc_dt;
    bool found = false;
    for (const auto& s : m_vo) {
        const double dt = std::fabs(s.time - t);
        if (dt <= best_dt) { best_dt = dt; out = s.pose; found = true; }
    }
    return found;
}

std::vector<Constraint> VisualConstraintSource::popBetweenConstraints() {
    std::vector<Constraint> out;
    out.swap(m_pending);
    return out;
}

Constraint VisualConstraintSource::makeLoopConstraint(const std::string& from_id,
                                                      const std::string& to_id,
                                                      const PoseStamped& rel, double cov) {
    Constraint c;
    c.from_kf = from_id;
    c.to_kf = to_id;
    c.relative_pose = rel;
    c.cov = cov;
    return c;
}

} // namespace rosiwit_slam
