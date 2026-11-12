#include "ieskf.h"

#include <cmath>
#include <type_traits>
#include <utility>
#include <Eigen/Geometry>

double State::gravity = 9.81;

namespace
{
// ===========================================================================
// Sophus 版本兼容垫片
//   Sophus::SO3::leftJacobian / leftJacobianInverse 是较新版本才提供的静态成员
//   (melodic/18.04 的 sophus 1.0.1 没有)。这里编译期探测: 有就走 Sophus 原实现
//   (保证 Humble 等新环境的行为逐位不变), 没有就走下面的闭式解, 两者数学等价。
// ===========================================================================
template <typename T, typename = void>
struct has_left_jacobian : std::false_type
{
};

template <typename T>
struct has_left_jacobian<T, decltype(void(T::leftJacobian(std::declval<V3D>())))> : std::true_type
{
};

template <typename T, typename = void>
struct has_left_jacobian_inverse : std::false_type
{
};

template <typename T>
struct has_left_jacobian_inverse<T, decltype(void(T::leftJacobianInverse(std::declval<V3D>())))>
    : std::true_type
{
};

// SO(3) 左雅可比闭式解 (与 Sophus::SO3::leftJacobian(phi) 等价):
//   J_l(phi) = I + (1-cos t)/t^2 * phi^ + (t-sin t)/t^3 * phi^^2,   t = |phi|
// t -> 0 时取级数极限以避免除零: (1-cos t)/t^2 -> 1/2 - t^2/24,
//                               (t-sin t)/t^3 -> 1/6 - t^2/120
M3D so3LeftJacobian(const V3D &phi)
{
    const double t2 = phi.squaredNorm();
    const double t = std::sqrt(t2);
    const M3D phi_hat = Sophus::SO3d::hat(phi);
    const M3D phi_hat2 = phi_hat * phi_hat;

    double a; // (1 - cos t) / t^2
    double b; // (t - sin t) / t^3
    if (t > 1e-4)
    {
        a = (1.0 - std::cos(t)) / t2;
        b = (t - std::sin(t)) / (t2 * t);
    }
    else
    {
        a = 0.5 - t2 / 24.0;
        b = 1.0 / 6.0 - t2 / 120.0;
    }
    return M3D::Identity() + a * phi_hat + b * phi_hat2;
}

// SO(3) 左雅可比之逆闭式解 (与 Sophus::SO3::leftJacobianInverse(phi) 等价):
//   J_l^-1(phi) = I - 1/2 * phi^ + (1/t^2 - (1+cos t)/(2 t sin t)) * phi^^2
// t -> 0 时上式中括号 -> 1/12 + t^2/720
M3D so3LeftJacobianInverse(const V3D &phi)
{
    const double t2 = phi.squaredNorm();
    const double t = std::sqrt(t2);
    const M3D phi_hat = Sophus::SO3d::hat(phi);
    const M3D phi_hat2 = phi_hat * phi_hat;

    double c; // 1/t^2 - (1 + cos t) / (2 t sin t)
    if (t > 1e-4)
    {
        c = 1.0 / t2 - (1.0 + std::cos(t)) / (2.0 * t * std::sin(t));
    }
    else
    {
        c = 1.0 / 12.0 + t2 / 720.0;
    }
    return M3D::Identity() - 0.5 * phi_hat + c * phi_hat2;
}

// 右雅可比 = 左雅可比的转置
template <typename T>
M3D jacobianRight(const V3D &inp)
{
    if constexpr (has_left_jacobian<T>::value)
        return T::leftJacobian(inp).transpose();
    else
        return so3LeftJacobian(inp).transpose();
}

template <typename T>
M3D jacobianRightInverse(const V3D &inp)
{
    if constexpr (has_left_jacobian_inverse<T>::value)
        return T::leftJacobianInverse(inp).transpose();
    else
        return so3LeftJacobianInverse(inp).transpose();
}

// SO3 对数: 旋转矩阵经过大量乘法更新后会有 ~1e-8 的正交性漂移, 而
// Sophus::SO3d(Matrix3d) 的 ensure 容差是 1e-12, 会直接 abort
// (实机前倾 43 度雷达外参下必现). 先转成归一化四元数再构造.
V3D so3LogSafe(const M3D &R)
{
    Eigen::Quaterniond q(R);
    q.normalize();
    return Sophus::SO3d(q).log();
}
} // namespace

M3D Jr(const V3D &inp)
{
    return jacobianRight<Sophus::SO3d>(inp);
}
M3D JrInv(const V3D &inp)
{
    return jacobianRightInverse<Sophus::SO3d>(inp);
}

void State::operator+=(const V21D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

void State::clampPhysical()
{
    if (!t_wi.allFinite()) { t_wi.setZero(); v.setZero(); }
    if (!v.allFinite()) v.setZero();
    if (!bg.allFinite()) bg.setZero();
    if (!ba.allFinite()) ba.setZero();
    if (!r_wi.allFinite()) r_wi = M3D::Identity();
    if (std::fabs(t_wi.z()) > 100.0) {
        t_wi.z() = std::copysign(100.0, t_wi.z());
        v.z() = 0.0;
    }
    const double vn = v.norm();
    if (vn > 3.0) v *= 3.0 / vn;
    v.z() = std::max(-0.8, std::min(0.8, v.z()));
    bg = bg.cwiseMax(-0.10).cwiseMin(0.10);
    ba = ba.cwiseMax(-1.0).cwiseMin(1.0);
}

V21D State::operator-(const State &other) const
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = so3LogSafe(other.r_wi.transpose() * r_wi);
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = so3LogSafe(other.r_il.transpose() * r_il);
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_il: " << state.t_il.transpose() << std::endl;
    os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V21D delta = V21D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_x.clampPhysical();
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

void IESKF::updateOdom(double speed, double yaw_rate, const V3D &gyro,
                       double sigma_v, double sigma_w)
{
    Eigen::Matrix<double, 2, 21> A = Eigen::Matrix<double, 2, 21>::Zero();
    Eigen::Vector2d r = Eigen::Vector2d::Zero();
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

    // 只用水平速度模长; 轮式 odom 不约束 z 方向速度.
    const double vh = std::hypot(m_x.v.x(), m_x.v.y());
    if (vh > 0.2) {
        A(0, 12) = m_x.v.x() / vh;
        A(0, 13) = m_x.v.y() / vh;
        r(0) = speed - vh;
        R(0, 0) = sigma_v * sigma_v;
    } else {
        R(0, 0) = 1.0;   // 静止/低速: 不约束速度模长
    }

    const V3D w = gyro - m_x.bg;
    A.block<1, 3>(1, 0) = -(m_x.r_wi * Sophus::SO3d::hat(w)).row(2);
    A.block<1, 3>(1, 15) = -m_x.r_wi.row(2);
    r(1) = yaw_rate - (m_x.r_wi * w).z();
    R(1, 1) = sigma_w * sigma_w;

    const Eigen::Matrix2d S = A * m_P * A.transpose() + R;
    const Eigen::Matrix<double, 21, 2> K = m_P * A.transpose() * S.inverse();
    const V21D dx = K * r;
    m_x += dx;
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKA = I - K * A;
    m_P = IKA * m_P * IKA.transpose() + K * R * K.transpose();
}

void IESKF::updateVerticalVelocity(double target_vz, double sigma)
{
    // 软约束 v.z 到 target_vz: 平地 target=0, 坡道 target=轮速*sin(pitch).
    // 只抑制 z 零偏长期积分, 不会把真实坡道压平.
    Eigen::Matrix<double, 1, 21> H = Eigen::Matrix<double, 1, 21>::Zero();
    H(0, 14) = 1.0;
    const Eigen::Matrix<double, 1, 1> R =
        Eigen::Matrix<double, 1, 1>::Identity() * (sigma * sigma);
    const Eigen::Matrix<double, 1, 1> r =
        Eigen::Matrix<double, 1, 1>::Identity() * (target_vz - m_x.v.z());
    const Eigen::Matrix<double, 1, 1> S = H * m_P * H.transpose() + R;
    const Eigen::Matrix<double, 21, 1> K = m_P * H.transpose() * S.inverse();
    m_x += K * r(0, 0);
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKH = I - K * H;
    m_P = IKH * m_P * IKH.transpose() + K * R * K.transpose();
}

void IESKF::updateGround(const V3D &n_world, double z_target,
                        double sigma_tilt, double sigma_z)
{
    // 3 维量测: 绕 x/y 的姿态误差 (把地面法向拉回竖直) + z 位置误差.
    // 不做位姿硬写, 走标准 EKF (H/K/P 更新), 与 FAST-LIO 的其它量测同构.
    Eigen::Matrix<double, 3, 21> H = Eigen::Matrix<double, 3, 21>::Zero();
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = sigma_tilt * sigma_tilt;
    R(1, 1) = sigma_tilt * sigma_tilt;
    R(2, 2) = sigma_z * sigma_z;

    // 世界系下把 n 转到 +z 的小角度旋转向量
    const V3D e_w = V3D(n_world.y(), -n_world.x(), 0.0);
    const V3D e_b = m_x.r_wi.transpose() * e_w;   // 转到 body 系
    r(0) = e_b.x();
    r(1) = e_b.y();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    r(2) = z_target - m_x.t_wi.z();
    H(2, 5) = 1.0;

    const Eigen::Matrix3d S = H * m_P * H.transpose() + R;
    const Eigen::Matrix<double, 21, 3> K = m_P * H.transpose() * S.inverse();
    const V21D dx = K * r;
    m_x += dx;
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKH = I - K * H;
    m_P = IKH * m_P * IKH.transpose() + K * R * K.transpose();
}

void IESKF::updateZ(double z_target, double sigma)
{
    // 1 维软量测, 只作用于 t_wi.z (索引 5).
    Eigen::Matrix<double, 1, 21> H = Eigen::Matrix<double, 1, 21>::Zero();
    H(0, 5) = 1.0;
    const double R = std::max(sigma * sigma, 1e-6);
    const double r = z_target - m_x.t_wi.z();
    const double S = (H * m_P * H.transpose())(0, 0) + R;
    const Eigen::Matrix<double, 21, 1> K = m_P * H.transpose() / S;
    m_x += K * r;
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKH = I - K * H;
    m_P = IKH * m_P * IKH.transpose() + K * (K.transpose() * R);
}

void IESKF::updatePlanar(double z_ref, const M3D &r_flat, bool lock_roll_pitch)
{
    // 平面约束: 平地假设下 z 不变、v.z=0、roll/pitch 不变化. 与 odom 模式无关,
    // 也不引入任何绝对坐标 —— z_ref 由调用方给 (建图起始高度或回锁时的高度).
    m_x.t_wi.z() = z_ref;
    m_x.v.z() = 0.0;

    if (lock_roll_pitch)
    {
        // 保留当前 yaw, 只把 roll/pitch 换回平地参考姿态.
        const double yaw_cur = std::atan2(m_x.r_wi(1, 0), m_x.r_wi(0, 0));
        const double yaw_flat = std::atan2(r_flat(1, 0), r_flat(0, 0));
        const M3D rz = Eigen::AngleAxisd(yaw_cur - yaw_flat, V3D::UnitZ()).toRotationMatrix();
        m_x.r_wi = rz * r_flat;
        // roll / pitch 是强假设: 收紧到极小协方差, yaw (索引 2) 不动.
        m_P(0, 0) = std::min(m_P(0, 0), 1e-6);
        m_P(1, 1) = std::min(m_P(1, 1), 1e-6);
        m_P(0, 1) = m_P(1, 0) = 0.0;
    }
    // z 与 v.z 收紧 (索引 5 = t_wi.z, 14 = v.z).
    m_P(5, 5) = std::min(m_P(5, 5), 1e-4);
    m_P(14, 14) = std::min(m_P(14, 14), 1e-4);
    m_x.clampPhysical();
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V21D delta = V21D::Zero();
    M21D H = M21D::Identity();
    V21D b = V21D::Zero();

    bool any_valid_iter = false;
    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        any_valid_iter = true;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        M21D J = M21D::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        J.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
        H += J.transpose() * m_P.inverse() * J;
        b += J.transpose() * m_P.inverse() * delta;

        H.block<12, 12>(0, 0) += shared_data.H;
        b.block<12, 1>(0, 0) += shared_data.b;

        delta = -H.ldlt().solve(b);

        m_x += delta;
        m_x.clampPhysical();
        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
    }

    // 一帧都没得到有效约束时不要重写协方差: 否则 H=I、delta=0 会把 P 直接刷成单位阵,
    // 之后所有帧都会被"过度信任"(一次空白墙/遮挡就能造成位姿跳变)。
    if (!any_valid_iter) { return; }
    M21D L = M21D::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    // L.block<3, 3>(6, 6) = JrInv(delta.segment<3>(6));
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
    m_P = L * H.ldlt().solve(M21D::Identity()) * L.transpose();
}