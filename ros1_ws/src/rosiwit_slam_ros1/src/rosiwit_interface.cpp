// ============================================================================
// RosiwitInterface 实现 —— 逐字搬自原 rosiwit_slam_ros1_node.cpp 的
// 发布器 / 服务 / TF 部分, 仅按 M5/M3 加 profile 抑制。
// ============================================================================
#include "rosiwit_interface.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>

using namespace rosiwit_slam;

namespace rosiwit_slam_ros1 {

RosiwitInterface::RosiwitInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, SlamEngine& engine)
    : nh_(nh), pnh_(pnh), engine_(engine), legacy_(engine.legacyProfile()) {
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
    ros::Time stamp;
    if (out.pose.time > 0) stamp = ros::Time(out.pose.time); else stamp = ros::Time::now();
    Eigen::Quaterniond q(out.pose.rot);

    nav_msgs::Odometry odom;
    odom.header.frame_id = engine_.worldFrame();
    odom.child_frame_id = engine_.bodyFrame();
    odom.header.stamp = stamp;
    odom.pose.pose.position.x = out.pose.trans.x();
    odom.pose.pose.position.y = out.pose.trans.y();
    odom.pose.pose.position.z = out.pose.trans.z();
    odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = out.pose.vel.x();
    odom.twist.twist.linear.y = out.pose.vel.y();
    odom.twist.twist.linear.z = out.pose.vel.z();
    odom_pub_.publish(odom);
    // M5: legacy profile 下不发 `odom` 别名
    if (!legacy_ && odom_nav_pub_.getNumSubscribers() > 0) odom_nav_pub_.publish(odom);

    // M5: legacy profile 下不发任何新 TF
    if (!legacy_) {
        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = engine_.worldFrame(); tf.child_frame_id = engine_.bodyFrame();
        tf.header.stamp = stamp;
        tf.transform.translation.x = out.pose.trans.x();
        tf.transform.translation.y = out.pose.trans.y();
        tf.transform.translation.z = out.pose.trans.z();
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);
    }

    auto pub_cloud = [&](ros::Publisher& p, const CloudType::Ptr& c, const std::string& fid) {
        if (p.getNumSubscribers() <= 0 || !c) return;
        sensor_msgs::PointCloud2 m; pcl::toROSMsg(*c, m);
        m.header.frame_id = fid; m.header.stamp = stamp; p.publish(m);
    };
    pub_cloud(body_cloud_pub_, out.body_cloud, engine_.bodyFrame());
    pub_cloud(world_cloud_pub_, out.world_cloud, engine_.worldFrame());

    if (path_pub_.getNumSubscribers() > 0) {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = engine_.worldFrame(); ps.header.stamp = stamp;
        ps.pose.position.x = out.pose.trans.x();
        ps.pose.position.y = out.pose.trans.y();
        ps.pose.position.z = out.pose.trans.z();
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
    // grid_map (+ map, 仅 rosiwit profile)
    const bool want_grid = is_loc || grid_map_pub_.getNumSubscribers() > 0 ||
                           (!legacy_ && map_pub_.getNumSubscribers() > 0);
    if (!grid_published_ && want_grid) {
        GridInfo gi;
        if (engine_.getGridInfo(gi) && gi.width > 0 && gi.height > 0) {
            nav_msgs::OccupancyGrid grid;
            grid.header.frame_id = engine_.worldFrame();
            grid.header.stamp = ros::Time::now();
            grid.info.resolution = gi.resolution;
            grid.info.width = gi.width;
            grid.info.height = gi.height;
            grid.info.origin.position.x = gi.origin_x;
            grid.info.origin.position.y = gi.origin_y;
            grid.info.origin.orientation.w = 1.0;
            const int w = gi.width, h = gi.height;
            grid.data.resize(static_cast<size_t>(w) * h);
            for (int row = 0; row < h; ++row) {
                const int src = h - 1 - row;   // 竖直翻转: 首行=最大 y
                for (int col = 0; col < w; ++col)
                    grid.data[static_cast<size_t>(row) * w + col] =
                        gi.data[static_cast<size_t>(src) * w + col];
            }
            grid_map_pub_.publish(grid);
            if (!legacy_) map_pub_.publish(grid);
            grid_published_ = true;
        }
    }
}

bool RosiwitInterface::handleSaveMap(rosiwit_slam_ros1::SaveMap::Request& req,
                                     rosiwit_slam_ros1::SaveMap::Response& res) {
    res.success = engine_.algo() && engine_.algo()->saveMap(req.path);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleLoadMap(rosiwit_slam_ros1::LoadMap::Request& req,
                                     rosiwit_slam_ros1::LoadMap::Response& res) {
    res.success = engine_.algo() && engine_.algo()->loadMap(req.path);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleSaveGridMap(rosiwit_slam_ros1::SaveGridMap::Request& req,
                                         rosiwit_slam_ros1::SaveGridMap::Response& res) {
    res.success = engine_.algo() && engine_.algo()->saveGridMap(req.pgm_path, req.yaml_path, req.resolution);
    res.message = res.success ? "ok" : "failed";
    return true;
}

bool RosiwitInterface::handleSetMode(rosiwit_slam_ros1::SetSlamMode::Request& req,
                                     rosiwit_slam_ros1::SetSlamMode::Response& res) {
    res.success = engine_.algo() && engine_.algo()->setPipelineMode(req.mode);
    res.message = res.success ? "ok" : "unsupported";
    return true;
}

} // namespace rosiwit_slam_ros1
