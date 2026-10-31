// include/algorithms/localization/fusion/direct_pose_fusion.h
#pragma once
#include <string>
#include "slam_core/i_pose_fusion.h"

namespace rosiwit_slam {

/// direct: 配准结果直接作为位姿 —— 与引入融合层之前的行为**逐位一致**。
/// 保留它是为了 (a) 可回退, (b) 作为 A/B 基线 (同一套上游, 只差这一层)。
class DirectPoseFusion : public IPoseFusion {
public:
    std::string name() const override { return "direct"; }

    void reset(const PoseStamped& pose) override {
        m_pose = pose;
        m_valid = true;
        m_diag = FusionDiag{};
    }

    void onMatch(const PoseStamped& pose,
                 const Eigen::Matrix<double, 6, 6>& /*cov*/,
                 const MatchResult& /*res*/) override {
        m_pose = pose;
        m_valid = true;
        ++m_diag.accepted;
    }

    PoseStamped predict(double t) const override {
        PoseStamped p = m_pose;
        p.time = t;
        return p;
    }

    FusionDiag diag() const override { return m_diag; }

private:
    PoseStamped m_pose;
    bool m_valid = false;
    FusionDiag m_diag;
};

}  // namespace rosiwit_slam
