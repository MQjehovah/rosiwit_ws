// src/algorithms/localization/hypothesis/hypothesis_manager.cpp
#include "algorithms/localization/hypothesis/hypothesis_manager.h"

namespace rosiwit_slam {

void HypothesisManager::reset(const PoseStamped& p) {
    m_h.clear();
    PoseHypothesis h;
    h.pose = p;
    h.weight = 1.0;
    m_h.push_back(h);
}

int HypothesisManager::seed(const std::vector<PoseStamped>& poses,
                            const std::vector<double>& scores) {
    m_h.clear();
    const int n = static_cast<int>(std::min(poses.size(), scores.size()));
    for (int i = 0; i < n && static_cast<int>(m_h.size()) < m_k; ++i) {
        // 去重: 与已有假设位置过近则跳过
        bool dup = false;
        for (const auto& e : m_h)
            if ((e.pose.trans - poses[i].trans).norm() < 0.5) { dup = true; break; }
        if (dup) continue;
        PoseHypothesis h;
        h.pose = poses[i];
        h.weight = scores[i];
        h.fitness = scores[i];
        m_h.push_back(h);
    }
    if (m_h.empty() && n > 0) { PoseHypothesis h; h.pose = poses[0]; h.weight = scores[0]; m_h.push_back(h); }
    return static_cast<int>(m_h.size());
}

void HypothesisManager::update(int i, const MatchResult& r) {
    if (i < 0 || i >= static_cast<int>(m_h.size())) return;
    m_h[i].fitness = r.fitness;
    if (r.converged) m_h[i].pose = r.pose;
}

int HypothesisManager::best() const {
    int bi = -1;
    double bf = -1.0;
    for (int i = 0; i < static_cast<int>(m_h.size()); ++i)
        if (m_h[i].fitness > bf) { bf = m_h[i].fitness; bi = i; }
    return bi;
}

void HypothesisManager::prune(double margin, int stale_limit) {
    const int bi = best();
    if (bi < 0) return;
    const double bf = m_h[bi].fitness;
    std::vector<PoseHypothesis> keep;
    keep.reserve(m_h.size());
    for (int i = 0; i < static_cast<int>(m_h.size()); ++i) {
        if (i == bi) { keep.push_back(m_h[i]); continue; }
        if (m_h[i].fitness < (1.0 - margin) * bf) {
            m_h[i].stale++;
            if (m_h[i].stale >= stale_limit) continue;  // 淘汰
        } else {
            m_h[i].stale = 0;
        }
        keep.push_back(m_h[i]);
    }
    m_h.swap(keep);
}

} // namespace rosiwit_slam
