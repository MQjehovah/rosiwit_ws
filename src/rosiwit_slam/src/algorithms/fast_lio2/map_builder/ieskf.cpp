#include "ieskf.h"

#include <cmath>
#include <type_traits>
#include <utility>
#include <Eigen/Geometry>

double State::gravity = 9.81;
double State::accel_bias_max = 1.0;

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
    ba = ba.cwiseMax(-accel_bias_max).cwiseMin(accel_bias_max);
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
    m_last_dt = dt;   // 供 update() 里的单帧位姿限幅使用
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

void IESKF::updateOdomPos(const V2D &d_meas, const V2D &p_ref, double sigma,
                          double p_floor)
{
    // 2 维量测: 窗口内水平位移应与 odom 一致. 只约束相对量, 无绝对位置约束.
    Eigen::Matrix<double, 2, 21> A = Eigen::Matrix<double, 2, 21>::Zero();
    A(0, 3) = 1.0;   // t_wi.x
    A(1, 4) = 1.0;   // t_wi.y
    Eigen::Vector2d r;
    r(0) = d_meas.x() - (m_x.t_wi.x() - p_ref.x());
    r(1) = d_meas.y() - (m_x.t_wi.y() - p_ref.y());
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (sigma * sigma);
    // 协方差下限: FAST-LIO 每帧被上千个点面约束压到亚毫米级 (实测 Pxx≈1.4e-7),
    // 此时任何 1Hz 的位移量测增益只有 1e-4 量级, 等于没约束. 先把位置协方差
    // 抬到不低于 p_floor^2, 轮速位移才真正能拉住 (常见做法: covariance flooring).
    if (p_floor > 0.0) {
        const double f2 = p_floor * p_floor;
        if (m_P(3, 3) < f2) m_P(3, 3) = f2;
        if (m_P(4, 4) < f2) m_P(4, 4) = f2;
    }
    const Eigen::Matrix2d S = A * m_P * A.transpose() + R;
    const Eigen::Matrix<double, 21, 2> K = m_P * A.transpose() * S.inverse();
    m_x += K * r;
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKA = I - K * A;
    m_P = IKA * m_P * IKA.transpose() + K * R * K.transpose();
}

void IESKF::updateYawDelta(double dpsi, double yaw_ref, double sigma,
                           double p_floor)
{
    // 1 维: (yaw_now - yaw_ref) 应等于 dpsi. 姿态扰动的世界系 z 分量对 yaw 的
    // 雅可比为 1 (小角度), 所以 A 只在旋转块的第 3 个分量上取 1.
    const M3D &R = m_x.r_wi;
    const double yaw_now = std::atan2(R(1, 0), R(0, 0));
    const double d_now = std::atan2(std::sin(yaw_now - yaw_ref),
                                    std::cos(yaw_now - yaw_ref));
    Eigen::Matrix<double, 1, 21> A = Eigen::Matrix<double, 1, 21>::Zero();
    A(0, 2) = 1.0;
    const double r = dpsi - d_now;
    if (std::fabs(r) > 1.0) return;      // 超过 57deg 视为野值, 直接丢
    // 协方差下限: 与位置同理, FAST-LIO 的姿态协方差也被雷达压到极小
    // (实测 P_yaw≈5e-9, 即 sigma=0.004deg), 量测增益只有 5e-5, 角度约束进不来.
    if (p_floor > 0.0) {
        const double f2 = p_floor * p_floor;
        if (m_P(2, 2) < f2) m_P(2, 2) = f2;
    }
    const double Rm = sigma * sigma;
    const double S = (A * m_P * A.transpose())(0, 0) + Rm;
    const Eigen::Matrix<double, 21, 1> K = m_P * A.transpose() / S;
    m_x += K * r;
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKA = I - K * A;
    m_P = IKA * m_P * IKA.transpose() + K * Rm * K.transpose();
}

void IESKF::updateForwardSpeed(double v_meas, double sigma)
{
    // 1 维量测: 车头方向的水平速度. ĥ 取当前 yaw, 只对 v_x/v_y 求导
    // (yaw 本身的误差由姿态通道负责, 这里不引入 yaw 量测).
    const double psi = std::atan2(m_x.r_wi(1, 0), m_x.r_wi(0, 0));
    const double c = std::cos(psi), s = std::sin(psi);
    Eigen::Matrix<double, 1, 21> H = Eigen::Matrix<double, 1, 21>::Zero();
    H(0, 12) = c;
    H(0, 13) = s;
    const double R = std::max(sigma * sigma, 1e-6);
    const double r = v_meas - (c * m_x.v.x() + s * m_x.v.y());
    const double S = (H * m_P * H.transpose())(0, 0) + R;
    const Eigen::Matrix<double, 21, 1> K = m_P * H.transpose() / S;
    m_x += K * r;
    m_x.clampPhysical();
    const M21D I = M21D::Identity();
    const M21D IKH = I - K * H;
    m_P = IKH * m_P * IKH.transpose() + K * (K.transpose() * R);
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

        // 方向性退化 (窄通道/走廊): 沿走廊轴向的信息量趋近 0, 直接求解会让位姿沿轴滑动.
        // 这里把"检测"和"处理"分开:
        //   检测: 对 H 的 6x6 位姿块做特征分解, λ/λmax < degenerate_eigen_ratio 的方向
        //         记为退化. 无论是否处理都执行, 便于用它来定量观测现场退化程度.
        //   处理: degenerate_remove_weak=true 时, 把弱方向的量测信息从 H 里**移除**,
        //         该方向回落到先验 (IMU/odom). 绝不能反过来给弱方向加信息(等于宣称量得
        //         很准, 实测把滤波焊死, z 掉到 -101).
        // 只处理"水平平移主导"的弱方向: z/roll/pitch 对这个 42deg 斜装雷达本来就弱
        // (ratio 常年 0.0002~0.002), 由地面因子负责, 不在这里动.
        {
            Eigen::Matrix<double, 6, 6> Hpose = shared_data.H.block<6, 6>(0, 0);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(Hpose);
            if (es.info() == Eigen::Success) {
                const Eigen::Matrix<double, 6, 6> V = es.eigenvectors();
                const Eigen::Matrix<double, 6, 1> ev = es.eigenvalues();
                const double ev_max = std::max(ev(5), 1e-9);
                int n_deg = 0;
                Eigen::Matrix<double, 6, 1> d0 = Eigen::Matrix<double, 6, 1>::Zero();
                Eigen::Matrix<double, 6, 6> weak_dirs = Eigen::Matrix<double, 6, 6>::Zero();
                Eigen::Matrix<double, 6, 1> weak_lam = Eigen::Matrix<double, 6, 1>::Zero();
                for (int k = 0; k < 6; ++k) {
                    if (ev(k) >= m_degenerate_eigen_ratio * ev_max) continue;
                    const Eigen::Matrix<double, 6, 1> d = V.col(k);
                    const double horiz = std::hypot(d(3), d(4));
                    if (horiz < 0.5 || std::fabs(d(5)) > 0.5) continue;   // 只管水平滑动
                    weak_dirs.col(n_deg) = d;
                    weak_lam(n_deg) = ev(k);
                    if (n_deg == 0) d0 = d;
                    ++n_deg;
                }
                // 观测日志: 与是否处理无关, 始终上报 (每 500 次迭代).
                // gmin 记录本帧实际施加的最小增益, 便于确认自适应是否在起作用.
                const double kShapeP = 0.25;   // 自适应指数 (固定): gain_k = (r_k/thr)^p
                double gmin = -1.0;
                if (n_deg > 0 && m_degenerate_gain > 0.0) {
                    const double thr = std::max(m_degenerate_eigen_ratio, 1e-12);
                    for (int j = 0; j < n_deg; ++j) {
                        const double rr = std::max(weak_lam(j) / ev_max, 1e-30);
                        double g = std::pow(rr / thr, kShapeP);
                        g = std::min(std::max(g, m_degenerate_gain), 1.0);
                        if (gmin < 0.0 || g < gmin) gmin = g;
                    }
                }
                static int deg_probe_log = 0;
                if (++deg_probe_log % 500 == 0) {
                    std::cout << "[FLIO2] degen probe ratio=" << (ev(0) / ev_max)
                              << " thr=" << m_degenerate_eigen_ratio
                              << " n_weak=" << n_deg
                              << " gain=" << m_degenerate_gain
                              << " gmin=" << gmin
                              << " axis=(" << d0(3) << "," << d0(4) << "," << d0(5) << ")"
                              << std::endl;
                }
                // 处理: 只做自适应降权 (无删除路径). 沿该方向从 H 与 b 各扣掉 (1-gain).
                // gain 有下限 m_degenerate_gain>0, 所以该方向永远保留部分约束.
                if (n_deg > 0 && m_degenerate_gain > 0.0) {
                    const double thr = std::max(m_degenerate_eigen_ratio, 1e-12);
                    const Eigen::Matrix<double, 6, 1> bpose = b.segment<6>(0);
                    for (int j = 0; j < n_deg; ++j) {
                        const Eigen::Matrix<double, 6, 1> d = weak_dirs.col(j);
                        const double lam = weak_lam(j);
                        const double rr = std::max(lam / ev_max, 1e-30);
                        double g = std::pow(rr / thr, kShapeP);
                        g = std::min(std::max(g, m_degenerate_gain), 1.0);
                        const double a = 1.0 - g;
                        if (a <= 0.0) continue;
                        H.block<6, 6>(0, 0) -= a * lam * d * d.transpose();
                        b.segment<6>(0) -= a * d.dot(bpose) * d;
                    }
                }
            }
        }

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

    // 物理速度限幅 (安全网): 状态速度超物理上限(清洁机器人 ~1m/s, 默认 3m/s 留余量)
    // 视为被坏更新污染 -> 截断。防止一次错配把速度打爆后位姿狂奔
    // (2026-11 窄区事故形态: 位姿 5-10m/s 且持续加速)。
    if (m_max_vel > 0.0) {
        const double vn = m_x.v.norm();
        if (vn > m_max_vel) m_x.v *= (m_max_vel / vn);
    }

    // 单帧位姿增量限幅 (位置安全网):
    //   速度限幅只管状态速度; 位置是量测更新直接改的, 一次坏匹配能在单帧里把位置
    //   整块拉走 (实测整包里有单帧跳 68m / 140m 的情况, 直接撕开地图).
    //   这里把「本次更新相对预测值的增量」按物理上限回缩: 位移 ≤ factor*max_vel*dt,
    //   转角 ≤ max_rot_step_deg. 正常运动 (≤2.5m/s, ≤15°/帧) 不会被触发.
    if (m_max_vel > 0.0 && m_last_dt > 0.0) {
        const double max_step = m_max_step_factor * m_max_vel * m_last_dt;
        const V3D dp = m_x.t_wi - predict_x.t_wi;
        const double dn = dp.norm();
        if (dn > max_step && dn > 1e-9) {
            m_x.t_wi = predict_x.t_wi + dp * (max_step / dn);
        }
        const M3D dR = predict_x.r_wi.transpose() * m_x.r_wi;
        const Eigen::AngleAxisd aa(dR);
        const double max_ang = m_max_rot_step_deg * std::acos(-1.0) / 180.0;
        if (aa.angle() > max_ang) {
            m_x.r_wi = predict_x.r_wi *
                       Eigen::AngleAxisd(max_ang, aa.axis()).toRotationMatrix();
        }
    }
}