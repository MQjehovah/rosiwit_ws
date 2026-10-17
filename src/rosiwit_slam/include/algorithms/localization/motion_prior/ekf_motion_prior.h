// include/algorithms/localization/motion_prior/ekf_motion_prior.h
#pragma once
#include <deque>
#include "slam_core/i_motion_prior.h"

namespace rosiwit_slam {

/// odom + IMU 误差状态 EKF 运动先验 (2026-10-17, 替代 ImuMotionPrior 的启发式融合).
///
/// 状态 x = [vx, vy] (体系二维速度), P 2x2; 陀螺零偏沿用 L11 静止标定 + 匹配航向
/// 误差在线微调 (与 ImuMotionPrior 同款, 该部分已被实机验证, 不重造)。
///   预测 @IMU 199Hz:  p += R(θ)·v·dt ; θ += (gz - b)·dt ; P += q_v·dt·I
///   更新 @odom:       z = R(θ)ᵀ·Δp_odom/dt (体系速度), 卡方门控(χ²)后标准更新
///   更新 @match:      同结构, 大测量噪声(激光位移是低频弱观测); 并锚定绝对位姿
///
/// 对比 ImuMotionPrior (常速外推 + odom 锚定偏移 + 速度上限硬阈值) 的三个实质提升:
///   ① 体系二维速度 -> **被侧推可测**(单前向速度模型表达不了横向推行)
///   ② 打滑判断 = 卡方新息检验(协方差自适应), 替代 max_odom_speed 硬阈值的"一刀切"
///   ③ 协方差 P 可导出 -> 后续给重播种半径/接受判据做原则性门控(预留)
///
/// 已知局限 (v1, 文档 §17):
///   轮子抱死被推行时 odom=0 而机器人真在动 —— 新息检验无法分辨"真停车"与"抱死滑行"
///   (v 会收敛到 0, 预测冻结, 与旧行为一致, 仍靠每帧 GICP 修正兜底)。
class EkfMotionPrior : public IMotionPrior {
public:
    std::string name() const override { return "imu_ekf"; }
    void configure(const MotionPriorConfig& cfg) override { m_cfg = cfg; }
    void reset(const PoseStamped& pose) override;
    void onImu(const IMUSample& s) override;
    void onOdom(const PoseStamped& odom) override;
    PoseStamped predict(double t) override;
    void onMatch(const PoseStamped& pose) override;

    // 测试/诊断
    Eigen::Vector2d velocity() const { return m_v; }
    Eigen::Matrix2d velocityCov() const { return m_P; }
    int odomRejected() const { return m_odom_rejected; }
    V3D gyroBias() const { return m_gyro_bias; }

private:
    void velocityUpdate(const Eigen::Vector2d& z, double r_var);
    MotionPriorConfig m_cfg;
    PoseStamped m_pose;

    // EKF: 体系速度 [vx, vy]
    Eigen::Vector2d m_v = Eigen::Vector2d::Zero();
    Eigen::Matrix2d m_P = Eigen::Matrix2d::Identity();
    int m_odom_rejected = 0;

    // IMU 积分
    double m_last_imu_time = -1.0;
    bool m_first_imu = false;

    // odom 缓存
    PoseStamped m_odom;
    double m_odom_time = -1.0;
    bool m_have_odom = false;
    double m_odom_yaw_rate = 0.0;
    double m_gyro_yaw_rate = 0.0;

    // match 间隔 (速度弱观测)
    double m_last_match_time = -1.0;
    V3D m_last_match_trans = V3D::Zero();

    // L11 静止标定 + 重力姿态校正 (与 ImuMotionPrior 同款实现)
    V3D m_gyro_bias = V3D::Zero();
    V3D m_acc_bias = V3D::Zero();
    V3D m_up_body = V3D::UnitZ();
    bool m_calibrated = false;
    std::deque<V3D> m_cal_gyro, m_cal_acc;
};

}   // namespace rosiwit_slam
