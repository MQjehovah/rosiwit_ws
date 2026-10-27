// ============================================================================
// 外参/TF 解析 (base_link <- imu/lidar, 快照发布) — SlamAlgorithm 的一个职责切片 (S1 拆分, 行为不变)
// ============================================================================
#include "slam_algorithm.h"

#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include "slam_core/diagnostics.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "slam_core/slam_factory.h"
#include "slam_core/slam_pipeline.h"

namespace rosiwit_slam_ros1 {

using namespace rosiwit_slam;

bool SlamAlgorithm::computeBaseFrames(bool blocking) {
    // 三条线程都会调: tick (每 20ms, 非阻塞)、FE/服务 (imuFromChild)、初始位姿 (阻塞 3s)。
    // 解析过程串行, 结果发布成快照; 非阻塞调用者拿不到锁就按"还没解析好"处理,
    // 绝不能因为别人正在等 TF 就把 tick 线程卡住。
    if (blocking) {
        std::lock_guard<std::timed_mutex> lk(mount_mu_);
        return computeBaseFramesLocked(true);
    }
    std::unique_lock<std::timed_mutex> lk(mount_mu_, std::try_to_lock);
    if (!lk.owns_lock()) return hasBaseImu();
    return computeBaseFramesLocked(false);
}

bool SlamAlgorithm::computeBaseFramesLocked(bool blocking) {
    // 行业标准: 算法估计 IMU body; ROS1 接口层负责 base_link <-> IMU 的输出/输入换算.
    // base_lidar 只给 BBS 的车体系投影用 (输入仍是原始 lidar 点云).
    RigidTransform base_imu = baseImu();
    RigidTransform base_lidar = baseLidar();
    if (base_imu.valid) return true;   // 输出/初始位姿只需要 base_link <- imu
    // 失败后**限频重试**(每 2s 最多一次):
    //   - 每帧重试: lookupTransform 最多阻塞 3s, 会把 100Hz 定时器拖死(实测 /slam/info 停发);
    //   - 只试一次: 启动瞬间 robot_state_publisher 还没发 /tf_static 就永远失败。
    const ros::Time now_try = ros::Time::now();
    if (!mount_last_try_.isZero() && (now_try - mount_last_try_).toSec() < 2.0) return false;
    mount_last_try_ = now_try;
    if (!tf_buffer_) tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    if (!tf_listener_) tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pnh_.param<std::string>("lidar_frame", mount_lidar_frame_, mount_lidar_frame_);
    pnh_.param<std::string>("imu_frame", mount_imu_frame_, mount_imu_frame_);
    pnh_.param<std::string>("base_frame", mount_base_frame_, mount_base_frame_);

    auto lookup = [&](const std::string& target, const std::string& source,
                      RigidTransform& out, const char* tag) -> bool {
        try {
            const geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
                target, source, ros::Time(0), ros::Duration(0.1));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
            q.normalize();
            out.set(q.toRotationMatrix(),
                    Eigen::Vector3d(tf.transform.translation.x,
                                    tf.transform.translation.y,
                                    tf.transform.translation.z));
            const Eigen::Vector3d rpy = out.R.eulerAngles(2, 1, 0);
            char b[320];
            std::snprintf(b, sizeof(b),
                          "[slam_algo] %s mount from TF ('%s' <- '%s'): rpy(deg)=[%.2f %.2f %.2f] xyz=[%.3f %.3f %.3f]",
                          tag, target.c_str(), source.c_str(),
                          rpy.z() * 180 / M_PI, rpy.y() * 180 / M_PI, rpy.x() * 180 / M_PI,
                          out.t.x(), out.t.y(), out.t.z());
            std::cout << b << std::endl;
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    };

    // blocking=true (启动/初始位姿): 最多 ~3s; false: 只试一次, 2s 节流后重试.
    const int attempts = blocking ? 30 : 1;
    for (int i = 0; i < attempts; ++i) {
        bool ok_imu = base_imu.valid;
        bool ok_lidar = base_lidar.valid;
        if (!ok_imu) {
            if (mount_imu_frame_ == mount_base_frame_) {
                base_imu.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                ok_imu = true;
            } else {
                ok_imu = lookup(mount_base_frame_, mount_imu_frame_, base_imu, "imu");
            }
        }
        if (!ok_lidar) {
            if (mount_lidar_frame_ == mount_base_frame_) {
                base_lidar.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                ok_lidar = true;
            } else if (mount_lidar_frame_ == mount_imu_frame_ && base_imu.valid) {
                base_lidar = base_imu;
                ok_lidar = true;
            } else {
                ok_lidar = lookup(mount_base_frame_, mount_lidar_frame_, base_lidar, "lidar");
            }
        }
        if (ok_imu && ok_lidar) {
            base_imu_slot_.publish(std::make_shared<const RigidTransform>(base_imu));
            base_lidar_slot_.publish(std::make_shared<const RigidTransform>(base_lidar));
            return true;
        }
        if (i + 1 < attempts) ros::Duration(0.05).sleep();
    }
    // 部分成功也要发布 (例如只有 base<-imu 拿到), 调用方按 .valid 判断
    if (base_imu.valid) {
        base_imu_slot_.publish(std::make_shared<const RigidTransform>(base_imu));
    }
    if (base_lidar.valid) {
        base_lidar_slot_.publish(std::make_shared<const RigidTransform>(base_lidar));
    }
    if (!base_imu.valid) {
        ROS_WARN_STREAM("[slam_algo] TF base_link <- imu unavailable; retry in 2s");
    }
    return base_imu.valid;
}

bool SlamAlgorithm::imuFromChild(const std::string& child_frame, RigidTransform& out) {
    // 核心 body 是 IMU: T_world_imu = T_world_child * T_child_imu.
    if (child_frame.empty() || child_frame == mount_imu_frame_) {
        out.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        return true;
    }
    const RigidTransform base_imu = baseImu();
    if (child_frame == mount_base_frame_ && base_imu.valid) {
        out = base_imu;   // T_base_link_imu
        return true;
    }
    {
        std::lock_guard<std::mutex> lk(child_mu_);
        auto it = child_to_imu_.find(child_frame);
        if (it != child_to_imu_.end()) {
            out = it->second;
            return true;
        }
    }
    // TF 查询要串行; 拿到锁再建 buffer/listener (它们只在 mount_mu_ 保护下使用)
    std::lock_guard<std::timed_mutex> mount_lk(mount_mu_);
    if (!tf_buffer_) tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    if (!tf_listener_) tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    try {
        const geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
            child_frame, mount_imu_frame_, ros::Time(0), ros::Duration(0.1));
        Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z);
        q.normalize();
        out.set(q.toRotationMatrix(),
                Eigen::Vector3d(tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z));
        {
            std::lock_guard<std::mutex> lk(child_mu_);
            child_to_imu_[child_frame] = out;
        }
        return true;
    } catch (const tf2::TransformException&) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[slam_algo] TF imu <- " << child_frame
                                 << " unavailable; drop this measurement");
        return false;
    }
}

} // namespace rosiwit_slam_ros1
