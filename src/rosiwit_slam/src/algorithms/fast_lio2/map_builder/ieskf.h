#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "commons.h"

using M12D = Eigen::Matrix<double, 12, 12>;
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using M21X12D = Eigen::Matrix<double, 21, 12>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    static double gravity;
    static double accel_bias_max;   // 加计零偏限幅 (m/s^2), 由配置设置
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    V3D v = V3D::Zero();
    V3D bg = V3D::Zero();
    V3D ba = V3D::Zero();
    V3D g = V3D(0.0, 0.0, -9.81);

    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    void operator+=(const V21D &delta);

    /// 物理限幅: 每次 EKF 迭代/预测后都调用, 防止一次坏匹配把 v/z/ba/bg
    /// 推到 1e30, 进而让 PCL VoxelGrid 整数索引溢出并破坏堆 (服务器实测).
    void clampPhysical();

    V21D operator-(const State &other) const;

    friend std::ostream &operator<<(std::ostream &os, const State &state);
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const V21D &)>;

class IESKF
{
public:
    IESKF() = default;
    void setMaxIter(size_t iter) { m_max_iter = iter; }
    /// 方向性退化阈值: LiDAR 信息矩阵 λ_min/λ_max 低于它的方向被视为退化,
    /// 注入正则让该方向回落到 IMU/odom 先验 (窄通道沿轴无约束的场景).
    void setDegenerateEigenRatio(double r) { m_degenerate_eigen_ratio = std::max(1e-6, r); }
    /// 弱方向自适应降权 (0 = 关闭): gain_k = clamp((r_k/thr)^0.25, gain, 1).
    /// 没有删除路径, 该方向至少保留 gain 的信息量.
    void setDegenerateGain(double g) { m_degenerate_gain = g; }
    void setAccelBiasMax(double v) { State::accel_bias_max = std::max(0.01, v); }
    // 状态速度物理上限(m/s); <=0 关闭。坏配准把速度打爆后的最后一道刹车(见 update)。
    void setMaxVel(double v) { m_max_vel = v; }
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const M12D &Q);

    void update();
    /// 轮速相对位移约束: 使"窗口内 EKF 位移"与"odom 位移"一致.
    /// 只约束相对量, 不需要 map<->odom 对齐, 因此不会把 odom 的慢漂灌进地图.
    /// @param d_meas odom 在该窗口内的水平位移
    /// @param p_ref  窗口起点的 EKF 水平位置
    void updateOdomPos(const V2D &d_meas, const V2D &p_ref, double sigma,
                       double p_floor = 0.0);
    /// 垂直速度软约束: 车体在坡上 v.z 仍然是小量, 长时间累积会变成 z 漂移.
    /// 每帧把 v.z 轻拉向 0, sigma 取 0.5m/s, 不影响正常爬坡分量.
    /// 窗口内相对转角约束: yaw(now) - yaw(窗口起点) 应等于 dpsi.
    /// 比"用瞬时 yaw 率"稳得多: 轮速 odom 的 twist.angular.z 在 50Hz 上抖动 ±8 deg/s,
    /// 但它的**位姿 yaw** 在 1s 窗口上与陀螺积分相关系数 0.999 (实测 300 个窗口).
    /// @param dpsi    轮速 odom 在窗口内的转角 (rad, 已去 wrap)
    /// @param yaw_ref 窗口起点的 EKF yaw (rad)
    void updateYawDelta(double dpsi, double yaw_ref, double sigma,
                        double p_floor = 0.0);
    void updateVerticalVelocity(double target_vz, double sigma);
    /// 地面 3-DoF 量测 (LeGO-LOAM 式): 用地平面法向约束 roll/pitch, 用地面高度约束 z.
    /// n_world 是世界系地面法向; z_target 是传感器高度期望 (地面 + h0).
    void updateGround(const V3D &n_world, double z_target, double sigma_tilt, double sigma_z);

    /// 位置级 z 弱先验: 把 t_wi.z 软拉向 z_target (sigma 米), 不动 roll/pitch/yaw.
    /// 用于地面高度里程计兜住 z 的慢漂, 避免自由积分.
    void updateZ(double z_target, double sigma);

    /// 平面约束 (平地假设): z 锁到 z_ref, v.z=0; lock_roll_pitch 时把 roll/pitch
    /// 对齐到 r_flat 但保留当前 yaw. 与 odom 模式无关, 坡道上由调用方按坡度解锁.
    void updatePlanar(double z_ref, const M3D &r_flat, bool lock_roll_pitch);

    State &x() { return m_x; }

    M21D &P() { return m_P; }

private:
    size_t m_max_iter = 10;
    double m_degenerate_eigen_ratio = 0.02;
    /// 弱方向自适应降权增益: 0 = 关闭(只检测); (0,1] = 降权下限, 无删除路径.
    double m_degenerate_gain = 0.0;
    double m_max_vel = 3.0;
    double m_last_dt = 0.1;          // 上次 predict 的 dt (s), 供单帧位姿限幅使用
    double m_max_step_factor = 2.0;  // 单帧位移上限 = factor * max_vel * dt
    double m_max_rot_step_deg = 30.0; // 单帧转角上限 (deg)
    State m_x;
    M21D m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    M21D m_F;
    M21X12D m_G;
};
