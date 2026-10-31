// include/algorithms/localization/fusion/ekf_pose_fusion.h
#pragma once
#include <Eigen/Eigen>
#include <string>
#include "slam_core/i_pose_fusion.h"

namespace rosiwit_slam {

/// eskf: IMU/odom 推进 + 带协方差的匹配观测做增益修正。
///
/// v1 状态 (9): p(3) / v_body(2) / θ(3 小角度)；协方差对角维护。
///   - 推进: onImu 用陀螺积分姿态、用 v 推进位置
///   - odom: z = R(θ_odom)ᵀ·Δp_odom/dt (体体系, 与 imu_ekf 先验同一套写法)
///   - 匹配: χ² 门控 + 增益 K = P/(P+R) + 单帧修正限幅
///
/// 设计目标不是"更准", 而是**让一次错配不可能造成瞬移**: 1 m 的错配最多
/// 走 max_correction_trans (默认 0.15 m), 并计入 rejected/chi2 诊断。
class EkfPoseFusion : public IPoseFusion {
public:
    std::string name() const override { return "eskf"; }

    void configure(const PoseFusionConfig& cfg) override { m_cfg = cfg; }

    void reset(const PoseStamped& pose) override;

    void onImu(const IMUSample& s) override;
    void onOdom(const PoseStamped& odom) override;

    void onMatch(const PoseStamped& pose,
                 const Eigen::Matrix<double, 6, 6>& cov,
                 const MatchResult& res) override;

    PoseStamped predict(double t) const override;

    bool getCov(Eigen::Matrix<double, 6, 6>& out) const override;

    FusionDiag diag() const override { return m_diag; }

private:
    static void yawOf(const Eigen::Matrix3d& R, double& yaw);
    static Eigen::Matrix3d yawToRot(double yaw);

    PoseFusionConfig m_cfg;

    bool   m_valid = false;
    double m_time = -1.0;

    Eigen::Vector3d m_p = Eigen::Vector3d::Zero();   // world 系位置
    Eigen::Vector3d m_theta = Eigen::Vector3d::Zero(); // world 系姿态 (rpy 近似)
    Eigen::Matrix3d m_R = Eigen::Matrix3d::Identity();

    Eigen::Vector2d m_v = Eigen::Vector2d::Zero();   // 体体系速度 [vx, vy]
    Eigen::Matrix2d m_v_cov = Eigen::Matrix2d::Identity();

    // 位姿协方差 (对角): 0..2 = 姿态(rpy, rad^2), 3..5 = 位置(m^2)
    Eigen::Matrix<double, 6, 1> m_P = Eigen::Matrix<double, 6, 1>::Constant(0.1);

    double m_last_imu_time = -1.0;
    bool   m_have_imu = false;
    double m_gyro_z = 0.0;
    double m_gyro_bias_z = 0.0;

    PoseStamped m_last_odom;
    double m_last_odom_time = -1.0;
    bool   m_have_odom = false;
    double m_odom_yaw = 0.0;

    // 最近一次被采信的匹配, 用于原地旋转判定 (本帧航向变化)
    double m_last_match_yaw = 0.0;
    bool   m_have_last_match = false;

    FusionDiag m_diag;
};

}  // namespace rosiwit_slam
