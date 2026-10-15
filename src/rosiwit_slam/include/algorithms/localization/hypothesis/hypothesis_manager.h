// include/algorithms/localization/hypothesis/hypothesis_manager.h
#pragma once
#include <vector>
#include <algorithm>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

struct PoseHypothesis {
    PoseStamped pose;
    double weight = 1.0;
    double fitness = -1.0;
    int stale = 0;
};

/// 多假设跟踪: 应对工厂重复货架/通道的错配(aliasing).
/// 重定位给出多候选时保留 Top-K 假设; 每帧对每个假设做局部配准并评分,
/// 落后假设连续多帧劣于最优则淘汰; 仅剩一个时退化为单假设.
class HypothesisManager {
public:
    void setMax(int k) { m_k = std::max(1, k); }
    void reset(const PoseStamped& p);

    /// 用重定位候选初始化 (poses 与 scores 等长); 返回假设数
    int seed(const std::vector<PoseStamped>& poses, const std::vector<double>& scores);

    int size() const { return static_cast<int>(m_h.size()); }
    bool ambiguous() const { return m_h.size() > 1; }
    std::vector<PoseHypothesis>& all() { return m_h; }
    const std::vector<PoseHypothesis>& all() const { return m_h; }

    void update(int i, const MatchResult& r);
    int best() const;

    /// 淘汰: 适应度低于 (1-margin)*best 的假设 stale++, 超 stale_limit 移除
    void prune(double margin, int stale_limit);

private:
    std::vector<PoseHypothesis> m_h;
    int m_k = 4;
};

} // namespace rosiwit_slam
