/**
 * @file sophus_se3.hpp
 * @brief 简化的Sophus SE3实现 (基于Eigen)
 * @author AI Development Team
 * @date 2026-04-24
 *
 * 这是一个简化版的Sophus SE3实现，仅包含FAST-LIO2 SLAM所需的基本功能
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Sophus {

/**
 * @brief SO3 - 3D旋转群
 */
template<typename Scalar>
class SO3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Quaternion = Eigen::Quaternion<Scalar>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    SO3() : q_(Quaternion::Identity()) {}
    explicit SO3(const Quaternion& q) : q_(q.normalized()) {}
    SO3(const Matrix3& R) : q_(R) {}
    SO3(const Vector3& axis_angle) {
        Scalar angle = axis_angle.norm();
        if (angle < 1e-10) {
            q_ = Quaternion::Identity();
        } else {
            q_ = Quaternion(Eigen::AngleAxis<Scalar>(angle, axis_angle / angle));
        }
    }
    SO3(const Eigen::AngleAxis<Scalar>& angle_axis) : q_(angle_axis) {}

    // 静态构造函数
    static SO3 Identity() { return SO3(); }

    // 赋值运算符
    SO3& operator=(const SO3& other) { q_ = other.q_; return *this; }
    static SO3 rotX(Scalar angle) { return SO3(Eigen::AngleAxis<Scalar>(angle, Vector3::UnitX())); }
    static SO3 rotY(Scalar angle) { return SO3(Eigen::AngleAxis<Scalar>(angle, Vector3::UnitY())); }
    static SO3 rotZ(Scalar angle) { return SO3(Eigen::AngleAxis<Scalar>(angle, Vector3::UnitZ())); }

    // 访问器
    const Quaternion& unit_quaternion() const { return q_; }
    Matrix3 matrix() const { return q_.toRotationMatrix(); }
    Matrix3 rotationMatrix() const { return q_.toRotationMatrix(); }

    // 逆
    SO3 inverse() const { return SO3(q_.inverse()); }

    // 对数映射 (返回轴角)
    Vector3 log() const {
        Scalar angle = 2.0 * std::acos(std::abs(q_.w()));
        if (angle < 1e-10) {
            return Vector3::Zero();
        }
        Vector3 axis = q_.vec() / q_.vec().norm();
        return axis * angle;
    }

    // 李代数hat运算
    static Matrix3 hat(const Vector3& omega) {
        Matrix3 Omega;
        Omega << 0, -omega(2), omega(1),
                omega(2), 0, -omega(0),
                -omega(1), omega(0), 0;
        return Omega;
    }

    // 李代数vee运算
    static Vector3 vee(const Matrix3& Omega) {
        return Vector3(Omega(2, 1), Omega(0, 2), Omega(1, 0));
    }

    // 指数映射
    static SO3 exp(const Vector3& omega) {
        Scalar theta = omega.norm();
        if (theta < 1e-10) {
            return SO3();
        }
        Vector3 axis = omega / theta;
        return SO3(Eigen::AngleAxis<Scalar>(theta, axis));
    }

    // 乘法
    SO3 operator*(const SO3& other) const { return SO3(q_ * other.q_); }
    Vector3 operator*(const Vector3& v) const { return q_ * v; }

    // 流形上的加法 (右乘)
    SO3& operator*=(const SO3& other) { q_ *= other.q_; return *this; }

    // 伴随矩阵
    Matrix3 adj() const { return matrix(); }

    // 内部四元数访问
    Quaternion& quaternion() { return q_; }
    const Quaternion& quaternion() const { return q_; }

private:
    Quaternion q_;
};

/**
 * @brief SE3 - 3D刚体运动群
 */
template<typename Scalar>
class SE3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SO3Type = SO3<Scalar>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
    using Vector6 = Eigen::Matrix<Scalar, 6, 1>;

    SE3() : translation_(Vector3::Zero()), rotation_(SO3Type::Identity()) {}

    SE3(const SO3Type& rotation, const Vector3& translation)
        : translation_(translation), rotation_(rotation) {}

    SE3(const Quaternion& q, const Vector3& translation)
        : translation_(translation), rotation_(q) {}

    SE3(const Matrix3& R, const Vector3& translation)
        : translation_(translation), rotation_(R) {}

    SE3(const Matrix4& T)
        : translation_((Vector3)T.template block<3, 1>(0, 3)),
          rotation_((Matrix3)T.template block<3, 3>(0, 0)) {}

    // 拷贝构造和赋值
    SE3(const SE3& other) : translation_(other.translation_), rotation_(other.rotation_) {}
    SE3& operator=(const SE3& other) {
        translation_ = other.translation_;
        rotation_ = other.rotation_;
        return *this;
    }

    // 静态构造函数
    static SE3 Identity() { return SE3(); }

    static SE3 trans(Scalar x, Scalar y, Scalar z) {
        return SE3(SO3Type::Identity(), Vector3(x, y, z));
    }

    static SE3 trans(const Vector3& t) {
        return SE3(SO3Type::Identity(), t);
    }

    static SE3 rotX(Scalar angle) {
        return SE3(SO3Type::rotX(angle), Vector3::Zero());
    }

    static SE3 rotY(Scalar angle) {
        return SE3(SO3Type::rotY(angle), Vector3::Zero());
    }

    static SE3 rotZ(Scalar angle) {
        return SE3(SO3Type::rotZ(angle), Vector3::Zero());
    }

    // 访问器
    const Vector3& translation() const { return translation_; }
    Vector3& translation() { return translation_; }

    const SO3Type& so3() const { return rotation_; }
    SO3Type& so3() { return rotation_; }

    const SO3Type& rotation() const { return rotation_; }
    SO3Type& rotation() { return rotation_; }

    // 四元数访问器 (兼容Sophus API)
    const typename SO3Type::Quaternion& unit_quaternion() const {
        return rotation_.unit_quaternion();
    }

    // 矩阵表示
    Matrix4 matrix() const {
        Matrix4 T = Matrix4::Identity();
        T.template block<3, 3>(0, 0) = rotation_.matrix();
        T.template block<3, 1>(0, 3) = translation_;
        return T;
    }

    Matrix4 homogeneousMatrix() const { return matrix(); }

    // 逆变换
    SE3 inverse() const {
        SO3Type R_inv = rotation_.inverse();
        return SE3(R_inv, -(R_inv * translation_));
    }

    // 乘法
    SE3 operator*(const SE3& other) const {
        return SE3(rotation_ * other.rotation_,
                    rotation_ * other.translation_ + translation_);
    }

    Vector3 operator*(const Vector3& p) const {
        return rotation_ * p + translation_;
    }

    // 复合赋值
    SE3& operator*=(const SE3& other) {
        translation_ = rotation_ * other.translation_ + translation_;
        rotation_ = rotation_ * other.rotation_;
        return *this;
    }

    // 对数映射 (返回6维李代数)
    Vector6 log() const {
        Vector6 xi;
        Vector3 omega = rotation_.log();
        Scalar theta = omega.norm();

        xi.template tail<3>() = omega;

        if (theta < 1e-10) {
            xi.template head<3>() = translation_;
        } else {
            // SE3对数映射公式
            Matrix3 Omega = SO3Type::hat(omega);
            Matrix3 Omega2 = Omega * Omega;
            Scalar theta2 = theta * theta;

            Matrix3 V_inv = Matrix3::Identity() - 0.5 * Omega +
                            (1.0 - theta * std::cos(theta / 2.0) / (2.0 * std::sin(theta / 2.0))) /
                            theta2 * Omega2;

            xi.template head<3>() = V_inv * translation_;
        }

        return xi;
    }

    // 指数映射 (从6维李代数构造SE3)
    static SE3 exp(const Vector6& xi) {
        Vector3 omega = xi.template tail<3>();
        Vector3 upsilon = xi.template head<3>();

        Scalar theta = omega.norm();

        if (theta < 1e-10) {
            return SE3(SO3Type::Identity(), upsilon);
        }

        Vector3 axis = omega / theta;
        SO3Type R = SO3Type::exp(omega);

        Matrix3 Omega = SO3Type::hat(omega);
        Matrix3 Omega2 = Omega * Omega;

        // SE3指数映射公式
        Matrix3 V = Matrix3::Identity() +
                    (1.0 - std::cos(theta)) / (theta * theta) * Omega +
                    (theta - std::sin(theta)) / (theta * theta * theta) * Omega2;

        Vector3 t = V * upsilon;

        return SE3(R, t);
    }

    // 李代数hat运算 (6维向量 -> 4x4矩阵)
    static Matrix4 hat(const Vector6& xi) {
        Matrix4 Xi = Matrix4::Zero();
        Xi.template block<3, 3>(0, 0) = SO3Type::hat(xi.template tail<3>());
        Xi.template block<3, 1>(0, 3) = xi.template head<3>();
        return Xi;
    }

    // 李代数vee运算 (4x4矩阵 -> 6维向量)
    static Vector6 vee(const Matrix4& Xi) {
        Vector6 xi;
        xi.template head<3>() = Xi.template block<3, 1>(0, 3);
        xi.template tail<3>() = SO3Type::vee(Xi.template block<3, 3>(0, 0));
        return xi;
    }

    // 伴随矩阵
    Matrix4 adj() const {
        Matrix4 Adj = Matrix4::Zero();
        Adj.block<3, 3>(0, 0) = rotation_.matrix();
        Adj.block<3, 3>(0, 3) = SO3Type::hat(translation_) * rotation_.matrix();
        Adj.block<3, 3>(3, 3) = rotation_.matrix();
        return Adj;
    }

    // 设置函数
    void setTranslation(const Vector3& t) { translation_ = t; }
    void setRotation(const SO3Type& R) { rotation_ = R; }
    void setQuaternion(const Quaternion& q) { rotation_ = SO3Type(q); }
    void setMatrix(const Matrix4& T) {
        rotation_ = SO3Type(T.block<3, 3>(0, 0));
        translation_ = T.block<3, 1>(0, 3);
    }

private:
    Vector3 translation_;
    SO3Type rotation_;
};

// 常用类型别名
using SE3d = SE3<double>;
using SE3f = SE3<float>;
using SO3d = SO3<double>;
using SO3f = SO3<float>;

}  // namespace Sophus