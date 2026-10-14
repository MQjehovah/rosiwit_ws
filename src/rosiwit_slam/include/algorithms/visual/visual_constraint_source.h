// include/algorithms/visual/visual_constraint_source.h
#pragma once
#include <string>
#include <vector>
#include <deque>
#include "slam_core/pipeline_types.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

struct VisualConstraintConfig {
    bool   enabled = false;
    double assoc_dt = 0.05;        // VO 与关键帧时间关联容差(s)
    double vo_trans_noise = 0.05;  // VO between 因子平移噪声(std)
    double vo_rot_noise = 0.02;    // VO between 因子旋转噪声(std)
};

/// 视觉松耦合因子源 (route b): 把外部 VO 库的输出(绝对里程计)与视觉回环,
/// 转换为后端可用的相对位姿约束 (Constraint). 输出不依赖视觉/帧间坐标系:
/// 关键帧间相对位姿 = VO(t_i)^{-1} ∘ VO(t_j), 天然在机体系.
///
/// 用法: 关键帧产生时 addKeyframe(id,time); VO 回调 addVisualOdometry(time,pose);
/// 随后 popBetweenConstraints() 取相邻关键帧的 between 因子喂给 GTSAM 后端.
class VisualConstraintSource {
public:
    void configure(const VisualConstraintConfig& cfg) { m_cfg = cfg; }

    void addVisualOdometry(double time, const PoseStamped& vo_pose);
    void addKeyframe(const std::string& id, double time);

    /// 取出并清空待用的相邻关键帧 between 因子
    std::vector<Constraint> popBetweenConstraints();

    /// 由视觉回环构造约束 (from->to 相对位姿)
    static Constraint makeLoopConstraint(const std::string& from_id, const std::string& to_id,
                                         const PoseStamped& rel, double cov);

    size_t voCount() const { return m_vo.size(); }

private:
    struct Sample { double time; PoseStamped pose; };
    bool nearestVO(double t, PoseStamped& out) const;

    VisualConstraintConfig m_cfg;
    std::deque<Sample> m_vo;
    std::deque<std::pair<std::string, double>> m_kfs;
    std::vector<Constraint> m_pending;
};

} // namespace rosiwit_slam
