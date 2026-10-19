// ROS1 接口层专用: base_link <-> 核心 body(IMU) 的刚体变换.
// 只在本包使用, 不与 ROS2 共享实现 (两套 TF API 不同).
#pragma once
#include <Eigen/Eigen>
#include <pcl/common/transforms.h>

#include "slam_core/slam_types.h"

namespace rosiwit_slam_ros1 {

/// T_target_source: p_target = R * p_source + t
struct RigidTransform {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    bool valid = false;

    void set(const Eigen::Matrix3d& R_in, const Eigen::Vector3d& t_in) {
        R = R_in;
        t = t_in;
        valid = true;
    }
};

/// T_parent_new = T_parent_old * T_old_new: 输入位姿在 old 系, 输出在 new 系.
inline rosiwit_slam::PoseStamped composeRight(const rosiwit_slam::PoseStamped& p_old,
                                              const RigidTransform& old_new) {
    rosiwit_slam::PoseStamped out = p_old;
    out.rot = p_old.rot * old_new.R;
    out.trans = p_old.rot * old_new.t + p_old.trans;
    return out;
}

/// T_parent_old = T_parent_new * T_new_old: 输入位姿在 new 系, 输出在 old 系.
inline rosiwit_slam::PoseStamped composeRightInv(const rosiwit_slam::PoseStamped& p_new,
                                                 const RigidTransform& old_new) {
    rosiwit_slam::PoseStamped out = p_new;
    const Eigen::Matrix3d R_new_old = old_new.R.transpose();
    const Eigen::Vector3d t_new_old = -R_new_old * old_new.t;
    out.rot = p_new.rot * R_new_old;
    out.trans = p_new.rot * t_new_old + p_new.trans;
    return out;
}

/// 点云从 source 系换到 target 系 (原地).
inline void transformCloud(rosiwit_slam::CloudType& cloud, const RigidTransform& target_source) {
    if (cloud.empty()) return;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = target_source.R.cast<float>();
    T.block<3, 1>(0, 3) = target_source.t.cast<float>();
    pcl::transformPointCloud(cloud, cloud, T);
}

/// 协方差 (order [rot(3), trans(3)]) 从 source 系换到 target 系.
inline Eigen::Matrix<double, 6, 6> transformCovariance(
    const Eigen::Matrix<double, 6, 6>& cov, const Eigen::Matrix3d& R_target_source) {
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    A.block<3, 3>(0, 0) = R_target_source;
    A.block<3, 3>(3, 3) = R_target_source;
    return A * cov * A.transpose();
}

}  // namespace rosiwit_slam_ros1
