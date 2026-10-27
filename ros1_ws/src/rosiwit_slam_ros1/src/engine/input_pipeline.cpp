// ============================================================================
// 输入流水线 (入队、统计、IMU/LiDAR/odom/vo/absolute 处理) — SlamAlgorithm 的一个职责切片 (S1 拆分, 行为不变)
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

void SlamAlgorithm::enqueueInput(InputItem item) {
    // 有界队列 + 丢旧保新。计数只统计**真正入队**的帧:
    // 以前在 push 之前自增, 关停后所有被拒的帧也算进去了 (I11)。
    const bool is_imu = (item.kind == InputKind::IMU);
    if (input_.push(std::move(item))) {
        if (is_imu) { ++input_imu_in_; } else { ++input_lidar_in_; }
    }
}

void SlamAlgorithm::logInputStats() {
    // 每秒一条, 打印**增量**: 丢弃/拒绝/峰值都能直接看到 (I11)。
    // 以前每 1000 帧才打一条累计值, 关停后丢帧(rejected_closed)完全不可见。
    static auto last_log = std::chrono::steady_clock::now();
    static rosiwit_common::ChannelStats prev;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count() < 1000) {
        return;
    }
    last_log = now;
    const auto st = input_.stats();
    ROS_INFO("[slam_algo] input 1s: imu+%lu lidar+%lu | dropped_oldest+%lu dropped_newest+%lu "
             "cleared+%lu rejected+%lu | pending=%zu peak=%zu",
             static_cast<unsigned long>(input_imu_in_.load()),
             static_cast<unsigned long>(input_lidar_in_.load()),
             static_cast<unsigned long>(st.dropped_oldest - prev.dropped_oldest),
             static_cast<unsigned long>(st.dropped_newest - prev.dropped_newest),
             static_cast<unsigned long>(st.cleared - prev.cleared),
             static_cast<unsigned long>(st.rejected_closed - prev.rejected_closed),
             input_.size(), st.peak);
    prev = st;
}

void SlamAlgorithm::processImu(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> core_lk(core_mu_);
    if (online_calib_enabled_) {
        std::lock_guard<std::mutex> lk(calib_mutex_);
        online_time_calib_.addImu(msg->header.stamp.toSec(), msg->angular_velocity.z);
        const auto & q = msg->orientation;
        if (std::abs(q.w) + std::abs(q.x) + std::abs(q.y) + std::abs(q.z) > 0.5) {
            last_imu_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            last_imu_stamp_ = msg->header.stamp.toSec();
            have_imu_yaw_ = true;
        }
    }
    IMUSample s;
    s.acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    s.gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.time = getSec(msg->header.stamp) + imu_time_offset_.load();
    if (input_seen_hook_) input_seen_hook_(false);
    if (algo_) algo_->onImu(s);
}

void SlamAlgorithm::processLidar(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> core_lk(core_mu_);
    if (!algo_) return;

    // FREE/idle 不跑算法，也不做昂贵的 PointCloud2 -> PCL 转换。
    // 仍调用 input_seen_hook_，保持旧接口的雷达频率统计/告警。
    if (algo_raw_ && algo_raw_->getPipelineMode() == "idle") {
        if (input_seen_hook_) input_seen_hook_(true);
        return;
    }

    LidarFrame f;
    f.cloud = ros2Pcl(msg, cfg_.lidar_filter_num, cfg_.lidar_min_range, cfg_.lidar_max_range,
                      cfg_.scan_period_ms);
    if (!f.cloud || f.cloud->empty()) return;
    applySelfFilter(f.cloud);
    if (f.cloud->empty()) return;
    f.start_time = getSec(msg->header.stamp);
    f.end_time = f.start_time + cfg_.scan_period_ms / 1000.0;
    if (input_seen_hook_) input_seen_hook_(true);
    if (raw_lidar_hook_) raw_lidar_hook_(*f.cloud, msg->header.stamp);
    algo_->onLidar(f);
}

void SlamAlgorithm::onImuMsg(const sensor_msgs::Imu::ConstPtr& msg) {
    enqueueInput(InputItem{InputKind::IMU, msg, nullptr});
}

void SlamAlgorithm::onLidarMsg(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    enqueueInput(InputItem{InputKind::LIDAR, nullptr, msg});
}

void SlamAlgorithm::onOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> core_lk(core_mu_);
    if (!algo_) return;
    PoseStamped p;
    p.time = getSec(msg->header.stamp);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    RigidTransform imu_child;
    if (!imuFromChild(msg->child_frame_id, imu_child)) return;
    algo_->onOdom(composeRight(p, imu_child));
}

void SlamAlgorithm::voCB(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!algo_) return;
    PoseStamped p;
    p.time = getSec(msg->header.stamp);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    RigidTransform imu_child;
    if (!imuFromChild(msg->child_frame_id, imu_child)) return;
    algo_->onVisualOdometry(p.time, composeRight(p, imu_child));
}

void SlamAlgorithm::absoluteCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!algo_) return;
    PoseStamped p;
    p.time = getSec(msg->header.stamp);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e2;
    auto mi = [](int r) { return r < 3 ? r + 3 : r - 3; };
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            cov(r, c) = msg->pose.covariance[mi(r) * 6 + mi(c)];
    const RigidTransform base_imu = baseImu();
    if (!computeBaseFrames(/*blocking=*/false) || !base_imu.valid) return;
    p = composeRight(p, base_imu);   // map<-base -> map<-imu
    cov = transformCovariance(cov, base_imu.R);
    algo_->onAbsolute(p, cov);
}

void SlamAlgorithm::onInitialPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!algo_raw_ || algo_raw_->getPipelineMode() != "localization") return;
    // 与 /switch_map、/relocation 共用同一份实现 (换算只写一次)
    applyInitPose(*msg, /*zero_if_insane=*/false, /*force_z_zero=*/false);
}

} // namespace rosiwit_slam_ros1
