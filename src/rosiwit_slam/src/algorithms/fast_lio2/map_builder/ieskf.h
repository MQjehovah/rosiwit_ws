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
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const M12D &Q);

    void update();
    /// 轮式 odom 量测 (fuse): 车体 x 前进速度(带符号) + 车体 y 侧向速度(≈0) +
    /// 世界系 yaw 角速度, 3 维标准 EKF 更新. 比原来只用速度模长多利用了符号与侧向约束.
    void updateOdom(double v_fwd, double v_lat, double yaw_rate, const V3D &gyro,
                    double sigma_v, double sigma_lat, double sigma_w);
    /// 垂直速度软约束: 车体在坡上 v.z 仍然是小量, 长时间累积会变成 z 漂移.
    /// 每帧把 v.z 轻拉向 0, sigma 取 0.5m/s, 不影响正常爬坡分量.
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
    State m_x;
    M21D m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    M21D m_F;
    M21X12D m_G;
};
