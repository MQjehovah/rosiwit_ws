// include/slam_core/i_motion_prior.h
#pragma once
#include <string>
#include "slam_core/localization_stage_types.h"

namespace rosiwit_slam {

/// 运动先验/预测器: 在两帧配准之间给出位姿猜测(初值).
/// 可基于 IMU 航迹推算 / 常速模型 / 外部里程计.
class IMotionPrior {
public:
    virtual ~IMotionPrior() = default;

    virtual std::string name() const = 0;

    virtual bool init(const std::string& /*config_path*/) { return true; }

    /// 初始化/重定位后重置积分基准 (含速度清零)
    virtual void reset(const PoseStamped& pose) = 0;

    /// 喂入 IMU (仅 IMU 类先验使用)
    virtual void onImu(const IMUSample& /*s*/) {}

    /// 喂入轮式里程计 (可选; 打滑时由实现自行门控)
    virtual void onOdom(const PoseStamped& /*odom*/) {}

    /// 预测 t 时刻位姿
    virtual PoseStamped predict(double t) = 0;

    /// 配准/重定位观测反馈, 校正先验状态
    virtual void onMatch(const PoseStamped& pose) = 0;

    virtual void configure(const MotionPriorConfig& /*cfg*/) {}
};

} // namespace rosiwit_slam
