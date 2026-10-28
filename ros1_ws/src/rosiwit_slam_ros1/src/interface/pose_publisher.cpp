// ============================================================================
// pose_publisher.cpp — 兼容层的位姿 / 健康 / TF 发布
//
// 单一职责: 按 legacy 语义发布"这一帧在哪里、可不可信":
//   curr_localization_pose / slam/info / /localization/state / TF map->odom,
//   以及建图/定位两种模式的位姿分支与节流。
// 不在这里做的事: 2D 平面约束算法 (planar_constraint.cpp, 它只产出修正量)。
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
#include "xzrobot_ros_dependency/LocalizationState.h"

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

    // 核心状态变化不一定伴随新位姿（例如 seed 首帧被拒 -> LOST），
    // 因此状态变化必须独立于 pose 输出立即上报。
    publishCoreStatusIfChanged();

    // C2b: 2D 激光约束 (只在 DEGRADED/LOST 且曾定位成功过时介入, 内部自带节流).
    planarConstraintStep();

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
        // 即使没有新位姿（全局重定位/GICP 占用 worker 时），也按 10Hz 节流
        // 重发 slam/info，让界面持续看到 INIT/LOST，而不是认为节点卡死。
        publish_info = true;
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

    {
        // tick 线程写; 存图服务等其它线程通过 currentPoseSnapshot() 读
        std::lock_guard<std::mutex> lk(pose_mu_);
        curr_pose_.x = out.pose.trans.x();
        curr_pose_.y = out.pose.trans.y();
        curr_pose_.z = out.pose.trans.z();
        curr_pose_.yaw = yaw;
        curr_pose_.q = Eigen::Quaterniond(out.pose.rot);
        curr_pose_.q.normalize();
    }

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
    // curr_pose_ 由 tick 线程写、存图服务经 currentPoseSnapshot() 读 (持 pose_mu_);
    // 写侧以前完全不加锁 -> 可能读到 (x,y,yaw) 与四元数不同源的撕裂值。
    const double px = out.pose.trans.x();
    const double py = out.pose.trans.y();
    const double pz = out.pose.trans.z();
    const double pyaw = stdYawFromRot(out.pose.rot);
    Eigen::Quaterniond pq(out.pose.rot);
    pq.normalize();
    {
        std::lock_guard<std::mutex> lk(pose_mu_);
        curr_pose_.x = px;
        curr_pose_.y = py;
        curr_pose_.z = pz;
        curr_pose_.yaw = pyaw;
        curr_pose_.q = pq;
    }

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = kMapFrame;
    ps.header.stamp = (out.pose.time > 0) ? ros::Time(out.pose.time) : ros::Time::now();
    ps.pose.position.x = px;
    ps.pose.position.y = py;
    ps.pose.position.z = pz;
    ps.pose.orientation.x = pq.x();
    ps.pose.orientation.y = pq.y();
    ps.pose.orientation.z = pq.z();
    ps.pose.orientation.w = pq.w();
    pose_puber_.publish(ps);

    if (out.pose.time <= last_mapping_path_time_ + 1e-6) return;
    last_mapping_path_time_ = out.pose.time;
    {
        // 发布线程按 1Hz 整条序列化; 这里 push_back/erase 会让 vector 扩容 ->
        // 与 publish 并发时是 UB (畸形 Path 甚至崩溃)。
        std::lock_guard<std::mutex> lk(path_mu_);
        mapping_path_msg_.poses.push_back(ps);
        if (mapping_path_msg_.poses.size() > kMaxMappingPathPoses) {
            const size_t drop = kMaxMappingPathPoses / 10;
            mapping_path_msg_.poses.erase(mapping_path_msg_.poses.begin(),
                                          mapping_path_msg_.poses.begin() + drop);
        }
    }
}

void Ros1SlamInterface::publishCoreStatusIfChanged() {
    if (slam_mode_ != SlamMode::LOCALIZATION) return;
    const auto st = engine_.localizationStatus();
    if (st == last_core_loc_status_) return;
    publishLocationInfo();
}

void Ros1SlamInterface::publishSlamInfoThrottled() {
    const ros::Time now = ros::Time::now();
    if (!last_location_info_pub_.isZero() &&
        (now - last_location_info_pub_).toSec() < 0.1) return;
    publishLocationInfo();
    last_location_info_pub_ = now;
}

void Ros1SlamInterface::publishHighRateLocalizationPose(bool info_published) {
    if (!curr_pose_valid_) {
        broadcastMapOdom();
        return;
    }
    const ros::Time now = ros::Time::now();
    const bool fresh = !last_new_pose_ros_.isZero() &&
                       (now - last_new_pose_ros_).toSec() <= 0.30;
    const bool due = last_high_rate_pub_.isZero() ||
                     (now - last_high_rate_pub_).toSec() >= 0.02;
    if (!fresh || !due) {
        broadcastMapOdom();
        return;
    }

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
    if (!static_map_valid_) return;
    const ros::Time now = ros::Time::now();
    if (!last_static_map_pub_.isZero() && (now - last_static_map_pub_).toSec() < 1.0) return;
    // 拷贝快照: 这张图由主线程的 1Hz timer / beginMapSwitch 更新, 这里在 tick 线程读
    nav_msgs::OccupancyGrid snap = staticMapSnapshot();
    if (snap.data.empty()) return;
    realtime_grid_map_puber_.publish(snap);
    last_static_map_pub_ = now;
}

void Ros1SlamInterface::publishZeroPoseInfo() {
    // 切图后旧图的 map->odom 不能再沿用，先回到 identity，定位成功后再更新。
    have_map_odom_ = false;
    last_map_odom_pub_ = ros::Time();
    {
        std::lock_guard<std::mutex> lk(pose_mu_);
        curr_pose_ = Xz3dPose();
    }
    last_location_info_pub_ = ros::Time::now();
    slam_info_puber_.publish(
        buildSlamInfo(/*slam_state=*/1, /*location_status=*/1, mapName(),
                      0.0, 0.0, 0.0,
                      static_cast<float>(realtime_grid_map_area_), 0.0f));
    // 切图后到定位成功之前一律 INIT: 位姿不可信, 消费端不得据此导航.
    publishLocalizationState(
        xzrobot_ros_dependency::LocalizationState::HEALTH_INIT, 0.0f);
    // 新地图上还没定位成功过: 2D 纠偏不能用旧图的"曾经锁定"状态.
    planar_ever_locked_ = false;
    planar_field_.clear();
    planar_pending_ = false;
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

            // 状态唯一来源：核心 localizationStatus()。
            // ROS 层只做协议映射：LOCALIZED=0, INIT=1, LOST=2。
            const auto st = engine_.localizationStatus();
            switch (st) {
                case rosiwit_slam::ILocalization::LOCALIZED:  location_status = 0; break;
                case rosiwit_slam::ILocalization::INIT:       location_status = 1; break;
                case rosiwit_slam::ILocalization::LOST:        location_status = 2; break;
            }
            relocating_ = (st == rosiwit_slam::ILocalization::INIT);
            if (st == rosiwit_slam::ILocalization::LOCALIZED) {
                matching_degree = static_cast<float>(engine_.getMatchingDegree());
                core_ever_localized_ = true;
                planar_ever_locked_ = true;
            }

            // LOST 边沿 -> 20003504。状态本身由核心给出，这里只负责告警边沿。
            if (st == rosiwit_slam::ILocalization::LOST &&
                last_core_loc_status_ != rosiwit_slam::ILocalization::LOST &&
                core_ever_localized_) {
                addWarningCode(20003504);
                std::cout << "[ros1_slam] core LOST -> warning 20003504 (pose lost)" << std::endl;
            }
            last_core_loc_status_ = st;
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

    slam_info_puber_.publish(
        buildSlamInfo(slam_state, location_status, curr_map_name_, p.x, p.y, p.yaw, map_area,
                      matching_degree));

    // ---- LocalizationServer 契约: /localization/state ----
    // 状态语义: INIT(不可信) -> TRACKING <-> DEGRADED(质量下降) -> LOST(必须停车).
    using LocState = xzrobot_ros_dependency::LocalizationState;
    int8_t health = LocState::HEALTH_LOST;
    if (slam_mode_ == SlamMode::LOCALIZATION) {
        if (location_status == 0) {          // LOCALIZED
            health = (matching_degree >= loc_degrade_matching_)
                         ? LocState::HEALTH_TRACKING
                         : LocState::HEALTH_DEGRADED;
        } else if (location_status == 1) {   // INIT / 重定位中
            health = LocState::HEALTH_INIT;
        } else {                             // LOST
            health = LocState::HEALTH_LOST;
        }
    } else if (slam_mode_ == SlamMode::MAPPING) {
        // 建图没有地图匹配度, 但有连续位姿; 对外按 TRACKING 处理.
        health = LocState::HEALTH_TRACKING;
    } else if (slam_mode_ == SlamMode::FREE) {
        health = LocState::HEALTH_INIT;
    }
    publishLocalizationState(health, matching_degree);
    double prior_xy = 0.0, prior_yaw = 0.0;
    const bool has_prior = engine_.priorUncertainty(prior_xy, prior_yaw);
    ROS_INFO_THROTTLE(2.0,
        "[locstate] mode=%s st=%d matching=%.3f pos=(%.3f,%.3f,%.1fdeg) health=%d "
        "relocating=%d planar=%d prior=(%s%.3f,%.3f)",
        modeName(slam_mode_), static_cast<int>(engine_.localizationStatus()),
        static_cast<double>(matching_degree), p.x, p.y, p.yaw * 57.2957795,
        static_cast<int>(health), relocating_.load() ? 1 : 0,
        planar_constraint_enabled_ ? 1 : 0, has_prior ? "" : "n/a ",
        has_prior ? prior_xy : 0.0, has_prior ? prior_yaw : 0.0);
}

void Ros1SlamInterface::publishLocalizationState(int8_t health, float matching) {
    using LocState = xzrobot_ros_dependency::LocalizationState;
    LocState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = kMapFrame;
    const Xz3dPose p = curr_pose_;
    const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw) &&
                        std::isfinite(p.q.w());
    msg.pose.pose.position.x = finite ? p.x : 0.0;
    msg.pose.pose.position.y = finite ? p.y : 0.0;
    msg.pose.pose.position.z = finite ? p.z : 0.0;
    msg.pose.pose.orientation.x = finite ? p.q.x() : 0.0;
    msg.pose.pose.orientation.y = finite ? p.q.y() : 0.0;
    msg.pose.pose.orientation.z = finite ? p.q.z() : 0.0;
    msg.pose.pose.orientation.w = finite ? p.q.w() : 1.0;

    // C1: 不确定度来自 2D 平面匹配 (C2) 或质量分档值.
    // 明确"不可用"时给大值, 避免消费端把 0 协方差当成"绝对准确".
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    const bool planar_fresh = !planar_cov_stamp_.isZero() &&
        (ros::Time::now() - planar_cov_stamp_).toSec() < 1.0;
    if (health == LocState::HEALTH_DEGRADED && planar_fresh) {
        // 真实可观性: 退化方向的方差自然大, 交叉项按对角处理.
        msg.pose.covariance[0] = planar_cov_xx_;
        msg.pose.covariance[7] = planar_cov_yy_;
        msg.pose.covariance[35] = planar_cov_yaw_;
    } else {
        double sxy = loc_sigma_xy_;
        double syaw = loc_sigma_yaw_;
        // 估计器给的运动先验不确定度 (打滑时为 odom 降权后膨胀的值).
        // 与分档值取大: 先验越不确定, 对外协方差越保守.
        double prior_xy = 0.0, prior_yaw = 0.0;
        if (engine_.priorUncertainty(prior_xy, prior_yaw)) {
            sxy = std::max(sxy, prior_xy);
            syaw = std::max(syaw, prior_yaw);
        }
        if (health == LocState::HEALTH_DEGRADED) { sxy *= 3.0; syaw *= 3.0; }
        if (health == LocState::HEALTH_LOST || health == LocState::HEALTH_INIT) {
            sxy = 1.0e3;
            syaw = 1.0e3;
        }
        msg.pose.covariance[0] = sxy * sxy;
        msg.pose.covariance[7] = sxy * sxy;
        msg.pose.covariance[35] = syaw * syaw;
    }

    msg.health = health;
    msg.matching = health == LocState::HEALTH_LOST ? 0.0f : matching;
    msg.map_id = mapName();
    msg.map_valid = hasStaticMap();
    msg.relocalize_count = engine_.poseJumpSeq();   // 单源: SlamAlgorithm 的跳变序号
    localization_state_puber_.publish(msg);
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
    last_map_odom_ = out;
    have_map_odom_ = true;
    last_map_odom_pub_ = out.header.stamp;
    tf_broadcaster_->sendTransform(out);
}

void Ros1SlamInterface::broadcastMapOdom() {
    if (!use_tf_ || !tf_broadcaster_) return;
    const ros::Time now = ros::Time::now();
    if (!last_map_odom_pub_.isZero() && (now - last_map_odom_pub_).toSec() < 0.05) return;
    if (!have_map_odom_) {
        // 首次定位前 / 切图后没有有效校正：先发 identity，保证 TF 树完整。
        last_map_odom_ = geometry_msgs::TransformStamped();
        last_map_odom_.header.frame_id = "map";
        last_map_odom_.child_frame_id = "odom";
        last_map_odom_.transform.rotation.w = 1.0;
        have_map_odom_ = true;
    }
    last_map_odom_.header.stamp = now;
    tf_broadcaster_->sendTransform(last_map_odom_);
    last_map_odom_pub_ = now;
}


} // namespace rosiwit_slam_ros1
