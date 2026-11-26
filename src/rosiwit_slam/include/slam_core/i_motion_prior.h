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

    /// 重定位/位姿注入专用: 重置运动学状态, 保留传感器零偏/标定 (默认退回 reset)。
    /// 零偏是传感器属性 —— 清掉会让重定位后 yaw 以真实零偏速率自转 (实机 1.5°/s)。
    virtual void resetPose(const PoseStamped& pose) { reset(pose); }

    /// 当前陀螺 z 零偏估计 (rad/s). 供融合器等下游使用 —— 融合器的 yaw 积分
    /// 必须减掉它, 否则会以原始零偏速率自转 (2026-11-26 实机 1.5°/s 自转根因)。
    virtual double gyroBiasZ() const { return 0.0; }

    /// 喂入 IMU (仅 IMU 类先验使用)
    virtual void onImu(const IMUSample& /*s*/) {}

    /// 喂入轮式里程计 (可选; 打滑时由实现自行门控)
    virtual void onOdom(const PoseStamped& /*odom*/) {}

    /// 预测 t 时刻位姿
    virtual PoseStamped predict(double t) = 0;

    /// 配准/重定位观测反馈, 校正先验状态
    virtual void onMatch(const PoseStamped& pose) = 0;

    /// 陀螺自由积分的帧内相对旋转: dq = q(t0)⁻¹·q(t1) (传感器系, 不受匹配锚定影响).
    /// deskew 用它做帧内插值 —— 位姿链滞后时不污染去畸变. false = 未实现/时间超界.
    virtual bool gyroDelta(double /*t0*/, double /*t1*/, Eigen::Quaterniond& /*dq*/) { return false; }

    virtual void configure(const MotionPriorConfig& /*cfg*/) {}

    /// 先验的等效不确定度 (米 / 弧度). <0 表示不可用 (实现未提供).
    /// 下游按方差加权, 而不是"用/不用"地二选一.
    virtual double priorTransSigma() const { return -1.0; }
    virtual double priorYawSigma() const { return -1.0; }
};

} // namespace rosiwit_slam
