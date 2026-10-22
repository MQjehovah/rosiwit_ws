// ============================================================================
// RosiwitInterface 实现 —— 逐字搬自原 rosiwit_slam_ros1_node.cpp 的
// 发布器 / 服务 / TF 部分, 仅抑制兼容接口已经负责的冲突输出。
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

RosiwitInterface::RosiwitInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamEngine& engine,
                                   bool suppress_conflicts)
    : nh_(nh), pnh_(pnh), engine_(engine),
      suppress_conflicts_(suppress_conflicts) {
    // ---- 发布 ----
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 1000);
    if (!suppress_conflicts_) {
        // 兼容接口已经发布 `odom`, 这里不再发别名 —— 连 advertise 都不做,
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
    if (!suppress_conflicts_) {
        // /map 由兼容接口独占
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    }

    // ---- TF ----
    // 兼容接口负责动态 map->odom, 这里不再发同边静态 TF,
    // 静态同边会打架; odom->base_link 在旧系统里由 robot_localization 提供。
    if (!suppress_conflicts_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        static_tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
        geometry_msgs::TransformStamped stf;
        stf.header.stamp = ros::Time::now();
        stf.header.frame_id = "map"; stf.child_frame_id = "odom";
        stf.transform.rotation.w = 1.0;
        static_tf_->sendTransform(stf);
    }

    path_.header.frame_id = engine_.worldFrame();

    // 实时栅格分辨率: mapping.grid.realtime_resolution
    try {
        const std::string config_path = engine_.configPath();
        if (!config_path.empty()) {
            const YAML::Node cfg = YAML::LoadFile(config_path);
            if (cfg["mapping"] && cfg["mapping"]["grid"] &&
                cfg["mapping"]["grid"]["realtime_resolution"]) {
                realtime_grid_resolution_ =
                    cfg["mapping"]["grid"]["realtime_resolution"].as<double>();
            }
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[rosiwit_interface] realtime grid resolution parse failed: " << e.what());
    }

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
    const PreparedPose p = preparePose(out);
    publishOdometry(p);
    publishTransform(p);
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
    const Eigen::Vector3d vel_body = q.conjugate() * p.pose.vel;
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    odom_pub_.publish(odom);
    if (!suppress_conflicts_ && odom_nav_pub_.getNumSubscribers() > 0) {
        odom_nav_pub_.publish(odom);
    }
}

void RosiwitInterface::publishTransform(const PreparedPose& p) {
    if (suppress_conflicts_) return;
    const Eigen::Quaterniond q(p.pose.rot);
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = engine_.worldFrame();
    tf.child_frame_id = engine_.bodyFrame();
    tf.header.stamp = p.stamp;
    tf.transform.translation.x = p.pose.trans.x();
    tf.transform.translation.y = p.pose.trans.y();
    tf.transform.translation.z = p.pose.trans.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
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

void RosiwitInterface::mapCB(const ros::TimerEvent&) {
    auto* algo = engine_.algo();
    if (!algo) return;

    // cloud_map / grid_map 都是"有人订阅才发"。以前 is_loc 强制每 2s
    // getGlobalMap() + getRealtimeGrid()，即使无人订阅也复制整张地图/栅格。
    if (global_map_pub_.getNumSubscribers() > 0) {
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
    const bool want_grid = grid_map_pub_.getNumSubscribers() > 0 ||
                           (!suppress_conflicts_ && map_pub_.getNumSubscribers() > 0);
    if (want_grid) {
        GridInfo gi;
        // 必须走 getRealtimeGrid, 它会按当前观测重建栅格; 只读 getGridInfo
        // 会在建图过程中一直发旧缓存. 核心内部按 revision 缓存, 不重复重建.
        // 实时图用 mapping.grid.realtime_resolution; 最终图才用 ray_casting 0.05.
        if (engine_.getRealtimeGrid(realtime_grid_resolution_, gi) && gi.width > 0 && gi.height > 0) {
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
            if (!suppress_conflicts_) map_pub_.publish(grid);
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
    rosiwit_slam::MapData data;
    res.success = engine_.algo() && engine_.algo()->exportMapData(data) &&
                  map_adapter::saveMapData(data, req.path);
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
    if (is_pcd) {
        res.success = engine_.algo() && engine_.algo()->loadMap(req.path);
    } else {
        rosiwit_slam::MapData data;
        res.success = engine_.algo() && map_adapter::loadMapData(req.path, data) &&
                      engine_.algo()->loadLocalizationMapData(data);
    }
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
