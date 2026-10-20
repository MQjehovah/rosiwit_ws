// ============================================================================
// Ros1SlamInterface 位姿/状态/TF 发布：curr_localization_pose、slam/info、
// 高频重发以及 map->odom。
// ============================================================================

#include "ros1_slam_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "convert.h"
#include "internal.h"
#include "map_adapter.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {
void Ros1SlamInterface::timerPublishLocalizationPose(const ros::TimerEvent&) {
    publishCurrPose();   // legacy.cpp:826-827
}
void Ros1SlamInterface::publishCurrPose() {
    updateModeFromCore();

    // 切图期间核心已 idle、旧输出快照已清空；这里再兜底，任何残留的旧图输出都不发布。
    if (slam_mode_ == SlamMode::LOCALIZATION && map_switch_busy_.load()) return;

    SlamOutput out;
    const bool have_out = engine_.latestOutput(out) && out.has_new_pose;

    // [PubProbe] 发布心跳：区分“发布链路被锁住”和“核心没有新位姿”。
    {
        static int pub_n = 0;
        if (++pub_n % 1000 == 0) {
            std::cout << "[PubProbe] tick=" << pub_n << " mode=" << modeName(slam_mode_)
                      << " have_new_pose=" << (have_out ? 1 : 0)
                      << " pose_time=" << out.pose.time << std::endl;
        }
    }

    // 核心输出 IMU body；对外发布前统一换到 base_link。
    if (have_out) {
        computeLidarToBase();
        if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
            out.pose = composeRightInv(out.pose, engine_.baseImu());
        }
    }

    bool publish_info = true;
    if (slam_mode_ == SlamMode::LOCALIZATION) {
        const PoseTickResult r = publishLocalizationPose(out, have_out);
        if (r == PoseTickResult::DropTick) return;
        publish_info = (r == PoseTickResult::Info);
    } else if (slam_mode_ == SlamMode::MAPPING) {
        publishMappingPose(out, have_out);
    }

    if (publish_info) publishSlamInfoThrottled();

    // 建图时 TF 跟随位姿定时器；定位时由高频发布块统一节流。
    if (slam_mode_ == SlamMode::MAPPING && use_tf_) sendTransform();

    if (slam_mode_ == SlamMode::LOCALIZATION) {
        publishHighRateLocalizationPose(publish_info);
        publishStaticMapFallback();
    }
}

Ros1SlamInterface::PoseTickResult Ros1SlamInterface::publishLocalizationPose(
    const SlamOutput& out, bool have_out) {
    // 旧定位分支有同帧去重：重复帧不更新位姿，也不发 slam/info (legacy.cpp:1121-1125)。
    if (!have_out) return PoseTickResult::NoInfo;

    const double t = out.pose.time;
    const bool dup = (t - last_pose_stamp_ < 1e-4);
    last_pose_stamp_ = t;
    if (dup) return PoseTickResult::NoInfo;

    const double yaw = stdYawFromRot(out.pose.rot);
    if (!std::isfinite(out.pose.trans.x()) || !std::isfinite(out.pose.trans.y()) ||
        !std::isfinite(yaw)) {
        std::cout << "XZSlamInterFace::publishCurrPose(), curr_pose has NaN, drop this frame"
                  << std::endl;
        return PoseTickResult::DropTick;
    }

    curr_pose_.x = out.pose.trans.x();
    curr_pose_.y = out.pose.trans.y();
    curr_pose_.z = out.pose.trans.z();
    curr_pose_.yaw = yaw;
    curr_pose_.q = Eigen::Quaterniond(out.pose.rot);
    curr_pose_.q.normalize();

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.header.stamp = (t > 0) ? ros::Time(t) : ros::Time::now();
    ps.pose.position.x = curr_pose_.x;
    ps.pose.position.y = curr_pose_.y;
    ps.pose.position.z = curr_pose_.z;
    ps.pose.orientation.x = curr_pose_.q.x();
    ps.pose.orientation.y = curr_pose_.q.y();
    ps.pose.orientation.z = curr_pose_.q.z();
    ps.pose.orientation.w = curr_pose_.q.w();
    pose_puber_.publish(ps);

    // LOST 的 loose output 只用于保持最后位置，不算“定位成功”。
    if (out.state != rosiwit_slam::SlamState::LOST) {
        curr_pose_valid_ = true;
        last_new_pose_ros_ = ros::Time::now();
        lost_monitor_.onLocalizedPose();
    }

    // confidently-wrong 防线：匹配分高但位姿锁错时，激光位移 vs 轮速位移是唯一信号。
    if (last_odom_t_ > 0.0) {
        odom_mon_.onPose(t, curr_pose_.x, curr_pose_.y, last_odom_x_, last_odom_y_);
        const bool wrong = odom_mon_.wrongness();
        if (wrong && !odom_mon_wrong_prev_) {
            addWarningCode(20003505);
            std::cout << "[ros1_slam] pose-odom DEVIATION " << odom_mon_.deviation()
                      << "m -> warning 20003505" << std::endl;
        }
        odom_mon_wrong_prev_ = wrong;
    }
    return PoseTickResult::Info;
}

void Ros1SlamInterface::publishMappingPose(const SlamOutput& out, bool have_out) {
    if (!have_out) return;

    last_pose_stamp_ = out.pose.time;
    curr_pose_.x = out.pose.trans.x();
    curr_pose_.y = out.pose.trans.y();
    curr_pose_.z = out.pose.trans.z();
    curr_pose_.yaw = stdYawFromRot(out.pose.rot);
    curr_pose_.q = Eigen::Quaterniond(out.pose.rot);
    curr_pose_.q.normalize();

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.header.stamp = (out.pose.time > 0) ? ros::Time(out.pose.time) : ros::Time::now();
    ps.pose.position.x = curr_pose_.x;
    ps.pose.position.y = curr_pose_.y;
    ps.pose.position.z = curr_pose_.z;
    ps.pose.orientation.x = curr_pose_.q.x();
    ps.pose.orientation.y = curr_pose_.q.y();
    ps.pose.orientation.z = curr_pose_.q.z();
    ps.pose.orientation.w = curr_pose_.q.w();
    pose_puber_.publish(ps);

    if (out.pose.time <= last_mapping_path_time_ + 1e-6) return;
    last_mapping_path_time_ = out.pose.time;
    mapping_path_msg_.poses.push_back(ps);
    if (mapping_path_msg_.poses.size() > kMaxMappingPathPoses) {
        const size_t drop = kMaxMappingPathPoses / 10;
        mapping_path_msg_.poses.erase(mapping_path_msg_.poses.begin(),
                                      mapping_path_msg_.poses.begin() + drop);
    }
}

void Ros1SlamInterface::publishSlamInfoThrottled() {
    const ros::Time now = ros::Time::now();
    if (!last_location_info_pub_.isZero() &&
        (now - last_location_info_pub_).toSec() < 0.1) return;
    publishLocationInfo();
    last_location_info_pub_ = now;
}

void Ros1SlamInterface::publishHighRateLocalizationPose(bool info_published) {
    if (!curr_pose_valid_) return;
    const ros::Time now = ros::Time::now();
    const bool fresh = !last_new_pose_ros_.isZero() &&
                       (now - last_new_pose_ros_).toSec() <= 0.30;
    const bool due = last_high_rate_pub_.isZero() ||
                     (now - last_high_rate_pub_).toSec() >= 0.02;
    if (!fresh || !due) return;

    if (!info_published) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = kMapFrame;
        ps.header.stamp = now;
        ps.pose.position.x = curr_pose_.x;
        ps.pose.position.y = curr_pose_.y;
        ps.pose.position.z = curr_pose_.z;
        ps.pose.orientation.x = curr_pose_.q.x();
        ps.pose.orientation.y = curr_pose_.q.y();
        ps.pose.orientation.z = curr_pose_.q.z();
        ps.pose.orientation.w = curr_pose_.q.w();
        pose_puber_.publish(ps);
        if (use_tf_ && (last_high_rate_tf_.isZero() ||
                        (now - last_high_rate_tf_).toSec() >= 0.05)) {
            sendTransform();
            last_high_rate_tf_ = now;
        }
    } else if (use_tf_ && (last_high_rate_tf_.isZero() ||
                           (now - last_high_rate_tf_).toSec() >= 0.05)) {
        sendTransform();
        last_high_rate_tf_ = now;
    }
    last_high_rate_pub_ = now;
}

void Ros1SlamInterface::publishStaticMapFallback() {
    if (!static_map_valid_ || realtime_grid_map_.data.empty()) return;
    const ros::Time now = ros::Time::now();
    if (!last_static_map_pub_.isZero() && (now - last_static_map_pub_).toSec() < 1.0) return;
    realtime_grid_map_puber_.publish(realtime_grid_map_);
    last_static_map_pub_ = now;
}

void Ros1SlamInterface::publishZeroPoseInfo() {
    curr_pose_ = Xz3dPose();
    last_location_info_pub_ = ros::Time::now();
    slam_info_puber_.publish(
        buildSlamInfo(/*slam_state=*/1, /*location_status=*/1, curr_map_name_,
                      0.0, 0.0, 0.0,
                      static_cast<float>(realtime_grid_map_area_), 0.0f));
}

void Ros1SlamInterface::publishLocationInfo() {
    Xz3dPose p = curr_pose_;
    // legacy.cpp:1826-1830: 任一 NaN 则三者归零
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
        p.x = 0.0; p.y = 0.0; p.yaw = 0.0;
    }

    int8_t slam_state = 2;
    int8_t location_status = 0;
    float map_area = 0.0f;
    // 定位匹配度: 旧实现在 LOCALIZATION 且**不在重定位中**时取 localiza.getFitness()
    // (legacy.cpp:1866, 在 status 的 if/else **之后**, 所以 status=2 时也会带上真值 ——
    //  实机实测正是 location_status=2 且 matching_degree=0.831)。其它模式留 0。
    float matching_degree = 0.0f;

    switch (slam_mode_) {
        case SlamMode::LOCALIZATION: {
            slam_state = 1;   // legacy.cpp:1842
            map_area = static_cast<float>(realtime_grid_map_area_);
            // 旧实现: relocating_ = isRelocalization(); 重定位中 -> 1,
            // 否则 getLocalizationStatus() ∈{0,1} -> 0, 其它 -> 2。
            // 核心不暴露 relocalization/status, 用 T3 的 LostMonitor 等价表达 0/1/2 (设计文档 §5.11 B)
            PoseStamped pose;
            Eigen::Matrix<double, 6, 6> cov;
            const bool cov_ok = engine_.getPoseCov(pose, cov);
            const LocStatus st = lost_monitor_.update(cov_ok);
            location_status = static_cast<int8_t>(st);
            relocating_ = (st == LocStatus::RECOVERING);
            if (!relocating_) {
                matching_degree = static_cast<float>(engine_.getMatchingDegree());
            }
            break;
        }
        case SlamMode::MAPPING:   // legacy.cpp:1873-1882
            slam_state = 0;
            location_status = 0;
            map_area = static_cast<float>(realtime_grid_map_area_);
            break;
        case SlamMode::FREE:      // legacy.cpp:1883-1886
            slam_state = 1;
            location_status = 1;
            break;
        default:                  // legacy.cpp:1887-1889
            slam_state = 2;
            break;
    }
    // 最终覆盖 (legacy.cpp:1891-1894): 在 switch **之后**, 会改写 FREE/MAPPING 的结果
    if (relocating_) {
        slam_state = 1;
        location_status = 1;
    }

    // [T11] 丢失边沿 -> 20003504 (pose lost, level=FAULT, 取自 conf/warning_code.json)。
    // 与 update() 同一 tick 消费(同线程), 所以真实的丢失不会被漏报;
    // 被作废的只是"丢失后又立刻恢复、发布方还没看过"的瞬时抖动 (T3 的设计)。
    // 核心不暴露告警码通道, 旧系统的 20003504 是定位模块自己 push 的
    // (nano_gicp_localization.cpp:9403) —— 这里是我们**补上**。
    if (lost_monitor_.consumeLostEdge()) {
        addWarningCode(20003504);
        std::cout << "[ros1_slam] localization LOST -> warning 20003504 (pose lost)" << std::endl;
    }

    slam_info_puber_.publish(
        buildSlamInfo(slam_state, location_status, curr_map_name_, p.x, p.y, p.yaw, map_area,
                      matching_degree));
}

// ---------------------------------------------------------------- [T7] 告警
void Ros1SlamInterface::sendTransform() {
    // legacy.cpp:1899-1936 逐行:
    //   Tfo = lookupTransform("base_footprint", "odom", Time(0))     = T(base_footprint <- odom)
    //   Tmf = (origin=(x, y, -0.07), rot=AngleAxis(-yaw, Z))         = T(map <- base_footprint)
    //        其中 yaw = atan2(R(0,1), R(0,0))  (即 xzYawFromRot)
    //   Tmo = Tmf * Tfo                                             = T(map <- odom)
    //   broadcast(map -> odom), stamp = now
    geometry_msgs::TransformStamped tfo_msg;
    try {
        tfo_msg = tf_buffer_->lookupTransform("base_footprint", "odom", ros::Time(0));
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    if (!std::isfinite(curr_pose_.x) || !std::isfinite(curr_pose_.y) ||
        !std::isfinite(curr_pose_.yaw)) {
        return;
    }

    // 用 Eigen 直接做刚体复合: T(a<-b) * T(b<-c) = T(a<-c), 与 tf2::Transform 的 A*B 同义。
    // 这样不必为一次乘法引入 tf2_geometry_msgs 依赖。
    // 注意这里必须用 xzYawFromRot(q)(= -标准航向) 再取负, 与旧实现成对出现 ——
    // 直接用 curr_pose_.yaw 会把符号弄反。
    const double yaw =
        xzYawFromRot(curr_pose_.q.normalized().toRotationMatrix());
    const Eigen::Matrix3d R_f =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d t_f(curr_pose_.x, curr_pose_.y, -0.07);

    const Eigen::Quaterniond q_o(tfo_msg.transform.rotation.w, tfo_msg.transform.rotation.x,
                                 tfo_msg.transform.rotation.y, tfo_msg.transform.rotation.z);
    const Eigen::Matrix3d R_o = q_o.toRotationMatrix();
    const Eigen::Vector3d t_o(tfo_msg.transform.translation.x, tfo_msg.transform.translation.y,
                              tfo_msg.transform.translation.z);

    const Eigen::Matrix3d R_mo = R_f * R_o;
    const Eigen::Vector3d t_mo = R_f * t_o + t_f;
    Eigen::Quaterniond q_mo(R_mo);
    q_mo.normalize();

    geometry_msgs::TransformStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";
    out.child_frame_id = "odom";
    out.transform.translation.x = t_mo.x();
    out.transform.translation.y = t_mo.y();
    out.transform.translation.z = t_mo.z();
    out.transform.rotation.x = q_mo.x();
    out.transform.rotation.y = q_mo.y();
    out.transform.rotation.z = q_mo.z();
    out.transform.rotation.w = q_mo.w();
    tf_broadcaster_->sendTransform(out);
}


} // namespace rosiwit_slam_ros1
