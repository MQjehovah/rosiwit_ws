// ============================================================================
// RosiwitInterface 实现 —— 逐字搬自原 rosiwit_slam_ros1_node.cpp 的
// 发布器 / 服务 / TF 部分, 仅按 M5/M3 加 profile 抑制。
// ============================================================================
#include "rosiwit_interface.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <sstream>

using namespace rosiwit_slam;

namespace rosiwit_slam_ros1 {

RosiwitInterface::RosiwitInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamEngine& engine,
                                   bool suppress_conflicts)
    : nh_(nh), pnh_(pnh), engine_(engine),
      legacy_(suppress_conflicts || engine.legacyProfile()) {
    // ---- 发布 ----
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 1000);
    if (!legacy_) {
        // M5: legacy profile 下不发 `odom` 别名 —— 连 advertise 都不做,
        // 免得 `rosnode info` 里多出一条永远不出数据的发布者 (旧系统里
        // /odom 由 robot_localization 提供)。
        odom_nav_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    }
    path_pub_ = nh_.advertise<nav_msgs::Path>("lio_path", 1000);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10000);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10000);
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 10, true);
    grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
    calib_pub_ = nh_.advertise<std_msgs::String>("slam_calibration", 1, true);
    if (!legacy_) {
        // legacy profile 下 /map 由旧接口独占 (M3)
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    }

    // ---- TF ----
    // 非 legacy 才发: legacy 要自己发动态 map->odom (旧实现在 legacy.cpp:1899-1936),
    // 静态同边会打架; odom->base_link 在旧系统里由 robot_localization 提供。
    if (!legacy_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
        geometry_msgs::TransformStamped stf;
        stf.header.stamp = ros::Time::now();
        stf.header.frame_id = "map"; stf.child_frame_id = "odom";
        stf.transform.rotation.w = 1.0;
        static_tf_->sendTransform(stf);
    }

    path_.header.frame_id = engine_.worldFrame();

    // ---- 服务 ----
    srv_save_map_ = nh_.advertiseService("save_map", &RosiwitInterface::handleSaveMap, this);
    srv_load_map_ = nh_.advertiseService("load_map", &RosiwitInterface::handleLoadMap, this);
    srv_merge_map_ = nh_.advertiseService("merge_map", &RosiwitInterface::handleMergeMap, this);
    srv_save_grid_ = nh_.advertiseService("save_grid_map", &RosiwitInterface::handleSaveGridMap, this);
    srv_set_mode_ = nh_.advertiseService("set_slam_mode", &RosiwitInterface::handleSetMode, this);

    // ---- 定时器 / 泵 ----
    map_timer_ = nh_.createTimer(ros::Duration(2.0), &RosiwitInterface::mapCB, this);
    engine_.addTickHook([this] { onTick(); });
}

void RosiwitInterface::onTick() {
    SlamOutput out;
    if (engine_.latestOutput(out) && out.has_new_pose) publishPose(out);
}

void RosiwitInterface::publishPose(const SlamOutput& out) {
    // 行业标准: 核心估计 IMU body; 所有对外位姿/TF/body_cloud 在 ROS1 层转 base_link.
    // world_cloud 已在 map 系, 不参与变换。
    PoseStamped pose = out.pose;
    CloudType::Ptr body_cloud = out.body_cloud;
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        // 核心输出/body_cloud 在 IMU body; 对外转 base_link.
        pose = composeRightInv(out.pose, engine_.baseImu());
        if (out.body_cloud) {
            body_cloud.reset(new CloudType(*out.body_cloud));
            transformCloud(*body_cloud, engine_.baseImu());
        }
    }

    ros::Time stamp;
    if (pose.time > 0) stamp = ros::Time(pose.time); else stamp = ros::Time::now();
    Eigen::Quaterniond q(pose.rot);

    nav_msgs::Odometry odom;
    odom.header.frame_id = engine_.worldFrame();
    odom.child_frame_id = engine_.bodyFrame();
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = pose.trans.x();
    odom.pose.pose.position.y = pose.trans.y();
    odom.pose.pose.position.z = pose.trans.z();
    odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
    const Eigen::Vector3d vel_body = q.conjugate() * pose.vel;
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    odom_pub_.publish(odom);
    // M5: legacy profile 下不发 `odom` 别名
    if (!legacy_ && odom_nav_pub_.getNumSubscribers() > 0) odom_nav_pub_.publish(odom);

    // M5: legacy profile 下不发任何新 TF
    if (!legacy_) {
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = engine_.worldFrame(); tf.child_frame_id = engine_.bodyFrame();
        tf.header.stamp = stamp;
        tf.transform.translation.x = pose.trans.x();
        tf.transform.translation.y = pose.trans.y();
        tf.transform.translation.z = pose.trans.z();
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);
    }

    auto pub_cloud = [&](ros::Publisher& p, const CloudType::Ptr& c, const std::string& fid) {
        if (p.getNumSubscribers() <= 0 || !c) return;
        sensor_msgs::PointCloud2 m; pcl::toROSMsg(*c, m);
        m.header.frame_id = fid; m.header.stamp = stamp; p.publish(m);
    };
    pub_cloud(body_cloud_pub_, body_cloud, engine_.bodyFrame());
    pub_cloud(world_cloud_pub_, out.world_cloud, engine_.worldFrame());

    if (path_pub_.getNumSubscribers() > 0) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = engine_.worldFrame(); ps.header.stamp = stamp;
        ps.pose.position.x = pose.trans.x();
        ps.pose.position.y = pose.trans.y();
        ps.pose.position.z = pose.trans.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        path_.poses.push_back(ps);
        path_.header.stamp = stamp;
        path_pub_.publish(path_);
    }
}

void RosiwitInterface::mapCB(const ros::TimerEvent&) {
    auto* algo = engine_.algo();
    if (!algo) return;

    const std::string mode = engine_.pipelineMode();
    const bool is_loc = (mode == "localization");
    // cloud_map
    if (is_loc || global_map_pub_.getNumSubscribers() > 0) {
        PointVec pts;
        if (engine_.getGlobalMap(pts) && !pts.empty()) {
            CloudType cloud; cloud.reserve(pts.size());
            for (auto& p : pts) cloud.push_back(p);
            cloud.width = cloud.size(); cloud.height = 1;
            sensor_msgs::PointCloud2 m; pcl::toROSMsg(cloud, m);
            m.header.frame_id = engine_.worldFrame(); m.header.stamp = ros::Time::now();
            global_map_pub_.publish(m);
        }
    }
    // grid_map (+ map, 仅非冲突模式)
    const bool want_grid = is_loc || grid_map_pub_.getNumSubscribers() > 0 ||
                           (!legacy_ && map_pub_.getNumSubscribers() > 0);
    if (want_grid) {
        GridInfo gi;
        // 必须走 getRealtimeGrid, 它会按当前观测重建栅格; 只读 getGridInfo
        // 会在建图过程中一直发旧缓存. 核心内部按 revision 缓存, 不重复重建.
        if (engine_.getRealtimeGrid(0.05, gi) && gi.width > 0 && gi.height > 0) {
            nav_msgs::OccupancyGrid grid;
            grid.header.frame_id = engine_.worldFrame();
            grid.header.stamp = ros::Time::now();
            grid.info.resolution = gi.resolution;
            grid.info.width = gi.width;
            grid.info.height = gi.height;
            grid.info.origin.position.x = gi.origin_x;
            grid.info.origin.position.y = gi.origin_y;
            grid.info.origin.orientation.w = 1.0;
            // GridInfo 内部 row0 = 最小 y, 与 ROS OccupancyGrid 一致.
            // 旧 PGM 的图像坐标翻转只在存盘/读盘层做, 不能在这里翻.
            grid.data = gi.data;
            grid_map_pub_.publish(grid);
            if (!legacy_) map_pub_.publish(grid);
            grid_published_ = true;
        }
    }

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

bool RosiwitInterface::handleSaveMap(rosiwit_slam_ros1::SaveMap::Request& req,
                                     rosiwit_slam_ros1::SaveMap::Response& res) {
    res.success = engine_.algo() && engine_.algo()->saveMapXzmapOnly(req.path);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleLoadMap(rosiwit_slam_ros1::LoadMap::Request& req,
                                     rosiwit_slam_ros1::LoadMap::Response& res) {
    res.success = engine_.algo() && engine_.algo()->loadMapXzmapOnly(req.path);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleMergeMap(rosiwit_slam_ros1::LoadMap::Request& req,
                                      rosiwit_slam_ros1::LoadMap::Response& res) {
    res.success = engine_.algo() && engine_.algo()->appendMap(req.path);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleSaveGridMap(rosiwit_slam_ros1::SaveGridMap::Request& req,
                                         rosiwit_slam_ros1::SaveGridMap::Response& res) {
    if (!engine_.algo()) { res.success = false; res.message = "failed"; return true; }
    // 请求里带了 generator 就临时切到该 2D 栅格策略再落盘 (空则沿用当前策略)
    if (!req.generator.empty() && !engine_.algo()->setGridMapGenerator(req.generator)) {
        ROS_WARN_STREAM("[rosiwit_interface] save_grid_map: unknown grid_map_generator '"
                        << req.generator << "', using current generator");
    }
    res.success = engine_.algo()->saveGridMap(req.pgm_path, req.yaml_path, req.resolution);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleSetMode(rosiwit_slam_ros1::SetSlamMode::Request& req,
                                     rosiwit_slam_ros1::SetSlamMode::Response& res) {
    if (engine_.algo() && req.mode == "idle") engine_.algo()->cancelMapLoad();
    res.success = engine_.algo() && engine_.algo()->setPipelineMode(req.mode);
    res.message = res.success ? "ok" : "unsupported";
    return true;
}

} // namespace rosiwit_slam_ros1
