// ============================================================================
// outputs.cpp — 兼容层之外的补充输出
//
// 只做一件事: 把核心输出的位姿/点云变成导航要的两路话题 (/lio_odom、/world_cloud)。
// 老接口 (xz_3d_slam 兼容层) 负责 App 看到的一切; 这里不再重复。
// ============================================================================
#include "outputs.h"
#include "map_adapter.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <sstream>

#include <yaml-cpp/yaml.h>

using namespace rosiwit_slam;

namespace rosiwit_slam_ros1 {

RosiwitInterface::RosiwitInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                   SlamAlgorithm& engine)
    : nh_(nh), pnh_(pnh), engine_(engine) {
    // ---- 发布 ----
    // App 只调老接口; 这里只发导航消费的两路输出。TF 由老接口独占 (重复发会打架)。
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 1000);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10000);

    // 不再用 ros::Timer (主线程): 整图/栅格在 cpu lane 上算, tick 线程发布。
    engine_.addTickHook([this] { onTick(); });
}

void RosiwitInterface::onTick() {
    SlamOutput out;
    if (engine_.latestOutput(out) && out.has_new_pose) publishPose(out);
    // 在线标定的估算更新 (原来挂在 2s 定时器上, 保持 2s 节奏)
    const ros::Time now = ros::Time::now();
    // 在线标定的估算更新 (原来挂在 2s 定时器上, 保持 2s 节奏)。
    // 只更新内部状态, 不再额外发一条 slam_calibration 话题 (App 不看, 老接口也没有)。
    if (last_calib_update_.isZero() || (now - last_calib_update_).toSec() >= 2.0) {
        last_calib_update_ = now;
        engine_.updateCalibration();
    }
}

void RosiwitInterface::publishPose(const SlamOutput& out) {
    const PreparedPose p = preparePose(out);
    publishOdometry(p);
    publishClouds(out, p);
}

RosiwitInterface::PreparedPose RosiwitInterface::preparePose(const SlamOutput& out) {
    PreparedPose p;
    p.new_pose = out.pose.time > last_pose_time_ + 1e-6;
    if (p.new_pose) last_pose_time_ = out.pose.time;

    p.pose = out.pose;
    // body_cloud 已不再发布 (App 用老接口的 curr_scan_point), 这里不再深拷贝点云。
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        p.pose = composeRightInv(out.pose, engine_.baseImu());
    }

    p.stamp = (p.pose.time > 0) ? ros::Time(p.pose.time) : ros::Time::now();

    // 输出平滑: alpha-beta 因果滤波. 只影响发布的位姿/轨迹/TF, 不改核心状态.
    if (out_filter_alpha_ > 1e-6) {
        const double t = p.stamp.toSec();
        if (!out_filter_.initialized()) {
            out_filter_.configure(out_filter_alpha_, out_filter_beta_, out_filter_extrap_s_);
            out_filter_.reset(p.pose);
        } else if (p.new_pose) {
            out_filter_.update(t, p.pose);
        }
        PoseStamped smoothed;
        if (out_filter_.poseAt(t, smoothed)) {
            smoothed.time = p.pose.time;
            smoothed.vel = p.pose.vel;
            p.pose = smoothed;
        }
    }
    return p;
}

void RosiwitInterface::publishOdometry(const PreparedPose& p) {
    const Eigen::Quaterniond q(p.pose.rot);

    nav_msgs::Odometry odom;
    odom.header.frame_id = engine_.worldFrame();
    odom.child_frame_id = engine_.bodyFrame();
    odom.header.stamp = p.stamp;
    odom.pose.pose.position.x = p.pose.trans.x();
    odom.pose.pose.position.y = p.pose.trans.y();
    odom.pose.pose.position.z = p.pose.trans.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // 位姿协方差 (x,y,z,roll,pitch,yaw): 之前全零 = 对外宣称零不确定度,
    // 下游 (nav/robot_localization) 无法据此降级. 按定位状态/匹配度给保守估计.
    {
        double sig_t = 0.30, sig_r = 0.15;   // 默认: 建图/LIO 里程计精度
        if (engine_.pipelineMode() == "localization") {
            const auto st = engine_.localizationStatus();
            if (st == rosiwit_slam::ILocalization::LOST) {
                sig_t = 1.0; sig_r = 0.5;
            } else if (st == rosiwit_slam::ILocalization::INIT) {
                sig_t = 0.3; sig_r = 0.15;
            } else {
                sig_t = 0.05; sig_r = 0.02;
                const double m = engine_.getMatchingDegree();
                if (m < 0.60) { sig_t = 0.40; sig_r = 0.20; }
                else if (m < 0.75) { sig_t = 0.12; sig_r = 0.05; }
            }
        }
        odom.pose.covariance[0]  = sig_t * sig_t;
        odom.pose.covariance[7]  = sig_t * sig_t;
        odom.pose.covariance[14] = 0.05 * 0.05;
        odom.pose.covariance[21] = sig_r * sig_r;
        odom.pose.covariance[28] = sig_r * sig_r;
        odom.pose.covariance[35] = sig_r * sig_r;
    }
    const Eigen::Vector3d vel_body = q.conjugate() * p.pose.vel;
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    // 只发 /lio_odom; 机体的 /odom 由底盘发布, 不再重复一份 (S2)
    odom_pub_.publish(odom);
}

void RosiwitInterface::publishClouds(const rosiwit_slam::SlamOutput& out,
                                   const PreparedPose& /*p*/) {
    if (!out.world_cloud || out.world_cloud->empty()) return;
    if (world_cloud_pub_.getNumSubscribers() == 0) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*out.world_cloud, msg);
    msg.header.frame_id = engine_.worldFrame();
    msg.header.stamp = (out.pose.time > 0) ? ros::Time(out.pose.time) : ros::Time::now();
    world_cloud_pub_.publish(msg);
}

} // namespace rosiwit_slam_ros1
