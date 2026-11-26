// include/algorithms/localization/fusion/ekf_pose_fusion.h
#pragma once
#include <Eigen/Eigen>
#include <deque>
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
    void resetRelative() override;

    void onImu(const IMUSample& s) override;
    void setGyroBias(const V3D& b) override { m_gyro_bias = b; }
    // 重定位语义: 注入位姿只是假设, 注入后的第一次高匹配才是地图一致的解 ——
    // 该帧 yaw 全量采纳 (不是门控, 是"锚定第一观测"), 之后恢复互补增益防棘轮。
    // 否则 5% 增益需要 1~2s 收敛, 期间的写图会把几度烙进子图 (实机 1.25° 偏移).
    void anchorYawOnNextMatch() { m_anchor_yaw_next = true; }
    void onOdom(const PoseStamped& odom) override;
    void onRelative(const PoseStamped& pose,
                    const Eigen::Matrix<double, 6, 6>& cov) override;

    void onMatch(const PoseStamped& pose,
                 const Eigen::Matrix<double, 6, 6>& cov,
                 const MatchResult& res) override;

    PoseStamped predict(double t) const override;

    bool getCov(Eigen::Matrix<double, 6, 6>& out) const override;

    FusionDiag diag() const override { return m_diag; }

private:
    static void yawOf(const Eigen::Matrix3d& R, double& yaw);
    static Eigen::Matrix3d yawToRot(double yaw);
    /// 延迟观测用: 历史插值, t ∈ [front, back] 时返回该时刻的滤波器状态
    bool histAt(double t, PoseStamped& out) const;

    PoseFusionConfig m_cfg;

    // 状态历史 (~3s @200Hz): 匹配观测属于帧末时刻 t_a, 到达时 IMU 已积到 now.
    // 延迟观测必须先把状态退回 t_a (2026-11-23, 见 onMatch)
    std::deque<std::pair<double, PoseStamped>> m_hist;

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
    // 传感器系 3D 陀螺零偏 (运动先验同步)。与角速度一起先去零偏、再按 up_body 投影;
    // 杆臂轨道项用的 m_w_s 同样去零偏 (与 EkfMotionPrior 同一口径)。
    Eigen::Vector3d m_gyro_bias = Eigen::Vector3d::Zero();
    bool m_anchor_yaw_next = true;   // 下一帧 yaw 全量采纳 (注入/初始化后)
    Eigen::Vector3d m_w_s = Eigen::Vector3d::Zero();  // 传感器系角速度 (杆臂轨道项用)
    // 机体 up 方向 (IMU 系). 本车 IMU 随雷达俯装 ~38°, gyro.z **不是**偏航角速度,
    // 必须投影: yaw_rate = ω·up_body. 直接用 gyro.z 会让转弯转速少算 ~22%
    // (转 180° 丢 38°) —— 2026-11-23 实机"一转弯就丢定位"的根因.
    Eigen::Vector3d m_up_body = Eigen::Vector3d::UnitZ();
    bool m_up_valid = false;
    double m_diag_yaw_int = 0.0;   // 诊断: 融合层投影积分累计(rad)

    PoseStamped m_last_odom;
    double m_last_odom_time = -1.0;
    bool   m_have_odom = false;
    double m_odom_yaw = 0.0;

    // 最近一次被采信的匹配, 用于原地旋转判定 (本帧航向变化)
    double m_last_match_yaw = 0.0;
    bool   m_have_last_match = false;
    // 激光位移速度观测的锚点 (与 odom 无关, odom_mode=none 时是唯一速度来源)
    bool        m_lv_have_prev = false;
    PoseStamped m_lv_prev_pose;
    double      m_last_match_time = -1.0;   // 平移限幅随间隔放宽用

    // ---- 相对位姿观测 (LIO) 的锚点 ----
    bool        m_have_rel_prev = false;
    PoseStamped m_rel_prev_lio;                              // 上一帧 LIO 位姿
    Eigen::Vector3d m_rel_prev_state_p = Eigen::Vector3d::Zero();  // 同一时刻滤波器位置
    Eigen::Matrix3d m_rel_prev_state_R = Eigen::Matrix3d::Identity();  // 同一时刻滤波器姿态

    // ---- odom 打滑门控的锚点 ----
    // plan 语义: 最近一次被接受的匹配位姿/时刻
    bool        m_have_slip_ref = false;
    PoseStamped m_slip_ref_odom;                              // 锚点时刻的 odom 位姿
    Eigen::Vector3d m_slip_ref_p = Eigen::Vector3d::Zero();   // 锚点时刻的融合位置
    // 独立对照: 两次匹配之间的位移 vs 同区间 odom 位移 (不被滤波器自证)
    bool        m_have_prev_match = false;
    PoseStamped m_prev_match_pose;                            // 上一次被接受的匹配位姿
    PoseStamped m_prev_match_odom;                            // 上一次匹配时刻的 odom
    double      m_odom_slip_weight = 1.0;                     // 由匹配区间算出的 odom 权重

    FusionDiag m_diag;
};

}  // namespace rosiwit_slam
