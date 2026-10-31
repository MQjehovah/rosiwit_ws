// include/slam_core/i_pose_fusion.h
#pragma once
#include <Eigen/Eigen>
#include <string>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 位姿融合诊断 (供 /nav_diagnostics 与离线指标使用).
struct FusionDiag {
    int    accepted = 0;          // 接受并应用的匹配数
    int    rejected_chi2 = 0;     // 新息卡方门控拒收
    int    rejected_in_place = 0; // 原地旋转时丢弃平移分量
    double last_chi2 = -1.0;      // 最近一次观测的 χ²
    double last_gain_trans = 0.0; // 最近一次平移增益 (0~1)
    double last_gain_rot = 0.0;
    double applied_trans = 0.0;   // 最近一次实际应用的平移修正 (m)
    double applied_rot_deg = 0.0; // 最近一次实际应用的旋转修正 (deg)
    // ---- 相对位姿观测 (LIO) ----
    int    relative_accepted = 0;      // 相对观测被采用的次数
    double last_rel_residual = 0.0;    // 最近一次相对观测残差 (m); 也作为 LIO/S2M 分歧代理
};

/// 位姿融合层: 把"配准结果(观测)"变成"位姿(状态)".
///
/// 两种实现:
///   direct —— 配准结果直接作为位姿 (改造前行为, 默认)
///   eskf   —— IMU/odom 推进 + 带协方差的匹配观测做增益修正 (单帧修正有界)
///
/// 配置: localization.pose_fusion (direct | eskf) + localization.fusion.*
class IPoseFusion {
public:
    virtual ~IPoseFusion() = default;

    virtual std::string name() const = 0;

    virtual void configure(const PoseFusionConfig& /*cfg*/) {}

    /// 初始化/重定位后锚定 (清速度, 重置协方差)
    virtual void reset(const PoseStamped& pose) = 0;

    /// 高频 IMU: 用于姿态推进与协方差扩散
    virtual void onImu(const IMUSample& /*s*/) {}

    /// 轮式里程计位姿 (frame=odom, 车体姿态): 速度观测, 实现自行门控
    virtual void onOdom(const PoseStamped& /*odom*/) {}

    /// 相对位姿观测 (LIO 里程计): 只表达"相对上一帧走了多少", 不含绝对位置。
    /// 默认忽略 —— direct 实现保持"配准结果直通"的旧语义。
    virtual void onRelative(const PoseStamped& /*pose*/,
                            const Eigen::Matrix<double, 6, 6>& /*cov*/) {}

    /// 配准观测 (带协方差). 是否采信由实现决定 (chi2 / 原地旋转 / 限幅).
    virtual void onMatch(const PoseStamped& pose,
                         const Eigen::Matrix<double, 6, 6>& cov,
                         const MatchResult& res) = 0;

    /// t 时刻位姿 (推进 + 外推)
    virtual PoseStamped predict(double t) const = 0;

    /// 当前位姿协方差 (order: [rot(3), trans(3)]); 不支持时返回 false
    virtual bool getCov(Eigen::Matrix<double, 6, 6>& /*out*/) const { return false; }

    virtual FusionDiag diag() const { return FusionDiag{}; }
};

}  // namespace rosiwit_slam
