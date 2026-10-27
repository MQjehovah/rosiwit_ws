// ============================================================================
// outputs.cpp — 兼容接口之外的补充输出
//
// 单一职责: 只发兼容层没有、且有人消费的输出:
//   /lio_odom (导航) 、/world_cloud (导航体素层);
//   /body_cloud、/lio_path、/slam_calibration 暂无消费者, 保留。
// 与兼容层重复的话题 (odom/map/cloud_map/grid_map) 已删除。
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
    // 只发"兼容接口没有"的输出: /lio_odom 与 /world_cloud 是导航消费的;
    // body_cloud / lio_path / slam_calibration 暂无消费者但不是兼容接口的重复项。
    // 重复项 (odom / map / cloud_map / grid_map) 已删除 (S2)。
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 1000);
    path_pub_ = nh_.advertise<nav_msgs::Path>("lio_path", 1000);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10000);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10000);
    calib_pub_ = nh_.advertise<std_msgs::String>("slam_calibration", 1, true);

    // ---- TF ----
    // 动态 map->odom 与静态 TF 都由兼容接口独占 (重复发会打架), 这里不发任何 TF。
    // ---- 服务 ----
    srv_save_map_ = nh_.advertiseService("save_map", &RosiwitInterface::handleSaveMap, this);
    srv_load_map_ = nh_.advertiseService("load_map", &RosiwitInterface::handleLoadMap, this);
    srv_merge_map_ = nh_.advertiseService("merge_map", &RosiwitInterface::handleMergeMap, this);
    srv_save_grid_ = nh_.advertiseService("save_grid_map", &RosiwitInterface::handleSaveGridMap, this);
    srv_set_mode_ = nh_.advertiseService("set_slam_mode", &RosiwitInterface::handleSetMode, this);

    // 不再用 ros::Timer (主线程): 整图/栅格在 cpu lane 上算, tick 线程发布。
    engine_.addTickHook([this] { onTick(); });
}

void RosiwitInterface::onTick() {
    SlamOutput out;
    if (engine_.latestOutput(out) && out.has_new_pose) publishPose(out);
    // 在线标定的估算更新 (原来挂在 2s 定时器上, 保持 2s 节奏)
    const ros::Time now = ros::Time::now();
    if (last_calib_pub_.isZero() || (now - last_calib_pub_).toSec() >= 2.0) {
        last_calib_pub_ = now;
        const auto decision = engine_.updateCalibration();
        if (calib_pub_) {
            std::ostringstream os;
            os << "{\"state\":\"" << decision.state << "\""
               << ",\"applied\":" << (decision.applied ? "true" : "false")
               << ",\"rolled_back\":" << (decision.rolled_back ? "true" : "false")
               << ",\"time_offset\":" << decision.time_offset
               << ",\"yaw_offset\":" << decision.yaw_offset << "}";
            std_msgs::String msg;
            msg.data = os.str();
            calib_pub_.publish(msg);
        }
    }
}

void RosiwitInterface::publishPose(const SlamOutput& out) {
    const PreparedPose p = preparePose(out);
    publishOdometry(p);
    publishClouds(out, p);
    publishPath(p);
}

RosiwitInterface::PreparedPose RosiwitInterface::preparePose(const SlamOutput& out) {
    PreparedPose p;
    p.new_pose = out.pose.time > last_pose_time_ + 1e-6;
    if (p.new_pose) last_pose_time_ = out.pose.time;

    p.pose = out.pose;
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        p.pose = composeRightInv(out.pose, engine_.baseImu());
        if (p.new_pose && out.body_cloud && body_cloud_pub_.getNumSubscribers() > 0) {
            p.body_cloud.reset(new CloudType(*out.body_cloud));
            transformCloud(*p.body_cloud, engine_.baseImu());
        }
    } else if (p.new_pose && out.body_cloud && body_cloud_pub_.getNumSubscribers() > 0) {
        p.body_cloud = out.body_cloud;
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

void RosiwitInterface::publishClouds(const SlamOutput& out, const PreparedPose& p) {
    if (!p.new_pose) return;
    auto pub_cloud = [&](ros::Publisher& pub, const CloudType::Ptr& cloud,
                         const std::string& frame) {
        if (pub.getNumSubscribers() <= 0 || !cloud) return;
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = frame;
        msg.header.stamp = p.stamp;
        pub.publish(msg);
    };
    if (p.body_cloud) pub_cloud(body_cloud_pub_, p.body_cloud, engine_.bodyFrame());
    if (out.world_cloud) pub_cloud(world_cloud_pub_, out.world_cloud, engine_.worldFrame());
}

void RosiwitInterface::publishPath(const PreparedPose& p) {
    if (!p.new_pose || path_pub_.getNumSubscribers() <= 0) return;

    const Eigen::Quaterniond q(p.pose.rot);
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = engine_.worldFrame();
    ps.header.stamp = p.stamp;
    ps.pose.position.x = p.pose.trans.x();
    ps.pose.position.y = p.pose.trans.y();
    ps.pose.position.z = p.pose.trans.z();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    path_.poses.push_back(ps);
    if (path_.poses.size() > kMaxPathPoses) {
        const size_t drop = kMaxPathPoses / 10;
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + drop);
    }
    path_.header.stamp = p.stamp;
    if (!last_path_pub_.isZero() &&
        (ros::Time::now() - last_path_pub_).toSec() < 1.0) return;
    path_pub_.publish(path_);
    last_path_pub_ = ros::Time::now();
}

bool RosiwitInterface::handleSaveMap(rosiwit_slam_ros1::SaveMap::Request& req,
                                     rosiwit_slam_ros1::SaveMap::Response& res) {
    // 导出整图 + 写盘是秒级操作: 丢 disk_write lane, 服务等结果 (主线程不被占)
    bool done_ok = false;
    const bool ran = engine_.runOnLaneAndWait(
        "disk_write", "save_map", [this, &req, &done_ok](rosiwit_common::CancelToken) {
            rosiwit_slam::MapData data;
            if (!engine_.exportCoreMapData(data)) return;
            map_adapter::MapIoGuard guard(/*wait_ms=*/5000);
            if (!guard.ok()) return;
            done_ok = map_adapter::saveMapData(data, req.path);
        });
    res.success = ran && done_ok;
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleLoadMap(rosiwit_slam_ros1::LoadMap::Request& req,
                                     rosiwit_slam_ros1::LoadMap::Response& res) {
    // Plain PCD goes through the core's loadMap(), matching the ROS2 service:
    // PcdMapManager loads it and the configured localization module (e.g.
    // gicp_localization) receives the PCD path directly. The MapData branch is
    // for the legacy xzmap (keyframe) map format.
    const bool is_pcd = req.path.size() >= 4 &&
                        req.path.compare(req.path.size() - 4, 4, ".pcd") == 0;
    // 载图同样是秒级 (解析 + 装配): disk_read lane, 服务等结果
    bool done_ok = false;
    const bool ran = engine_.runOnLaneAndWait(
        "disk_read", "load_map", [this, is_pcd, &req, &done_ok](rosiwit_common::CancelToken) {
            if (is_pcd) {
                done_ok = engine_.loadCoreMap(req.path);
                return;
            }
            rosiwit_slam::MapData data;
            const uint64_t gen = engine_.coreGeneration();
            done_ok = map_adapter::loadMapData(req.path, data) &&
                      engine_.loadLocalizationMapData(data, gen);
        });
    res.success = ran && done_ok;
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleMergeMap(rosiwit_slam_ros1::LoadMap::Request& req,
                                      rosiwit_slam_ros1::LoadMap::Response& res) {
    // 融合会重建栅格 (秒级): 丢 disk_read lane, 服务等结果
    bool done_ok = false;
    const bool ran = engine_.runOnLaneAndWait(
        "disk_read", "merge_map",
        [this, &req, &done_ok](rosiwit_common::CancelToken) {
            map_adapter::MapIoGuard guard(/*wait_ms=*/5000);
            if (!guard.ok()) return;
            done_ok = engine_.appendCoreMap(req.path);
        });
    res.success = ran && done_ok;
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleSaveGridMap(rosiwit_slam_ros1::SaveGridMap::Request& req,
                                         rosiwit_slam_ros1::SaveGridMap::Response& res) {
    if (!engine_.algo()) { res.success = false; res.message = "failed"; return true; }
    bool done_ok = false;
    const bool ran = engine_.runOnLaneAndWait(
        "disk_write", "save_grid_map", [this, &req, &done_ok](rosiwit_common::CancelToken) {
            // 请求里带了 generator 就临时切到该 2D 栅格策略再落盘 (空则沿用当前策略)
            if (!req.generator.empty() && !engine_.setCoreGridMapGenerator(req.generator)) {
                ROS_WARN_STREAM("[rosiwit_interface] save_grid_map: unknown grid_map_generator '"
                                << req.generator << "', using current generator");
            }
            map_adapter::MapIoGuard guard(/*wait_ms=*/5000);
            if (!guard.ok()) return;
            done_ok = engine_.saveCoreGridMap(req.pgm_path, req.yaml_path, req.resolution);
        });
    res.success = ran && done_ok;
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleSetMode(rosiwit_slam_ros1::SetSlamMode::Request& req,
                                     rosiwit_slam_ros1::SetSlamMode::Response& res) {
    if (req.mode == "idle") engine_.cancelCoreMapLoad();
    res.success = engine_.setCorePipelineMode(req.mode);
    res.message = res.success ? "ok" : "unsupported";
    return true;
}

} // namespace rosiwit_slam_ros1
