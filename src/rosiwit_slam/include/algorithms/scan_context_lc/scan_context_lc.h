#pragma once
#include <string>
#include <vector>
#include "slam_core/i_loop_closure.h"

namespace rosiwit_slam {

/// 基于距离阈值的简单回环检测。
///
/// @note 这是简化的距离式回环检测, 并非真正的 ScanContext 描述子匹配。
///       适合小场景快速验证, 大场景建图建议替换为基于描述子的回环检测
///       (如 ScanContext / BoW / NetVLAD)。
///
/// 检测逻辑: 当最新关键帧与任一历史关键帧的平移距离 < loop_radius_ 时,
/// 视为回环候选。不处理旋转不变性 / 场景外观相似性。
class ScanContextLC : public ILoopClosure {
public:
    ScanContextLC();
    ~ScanContextLC() override = default;

    bool init(const std::string& config_path) override;
    void addKeyFrame(const KeyFrame& kf) override;
    bool detect(PoseStamped& relative_pose, double& cov) override;

private:
    std::vector<KeyFrame> keyframes_;
    /// 回环检测半径 (米): 当前 KF 与历史 KF 平移距离小于此值视为回环候选
    double loop_radius_ = 15.0;
    /// 最小关键帧间隔: 至少隔多少个 KF 才做回环检测 (避免连续帧误检)
    int min_keyframe_gap_ = 30;
};

} // namespace rosiwit_slam
