// src/ros_interface/slam_node.cpp
#include "ros_interface/slam_node.h"
#include "slam_core/slam_factory.h"
#include "slam_core/slam_base.h"
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <functional>
#include "ros_interface/ros_utils.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rosiwit_slam {

SlamNode::SlamNode(const rclcpp::NodeOptions& options) : rclcpp::Node("rosiwit_slam", options) {
    loadParameters();
    std::string algo_name = "fast_lio2";
    this->declare_parameter<std::string>("slam_algorithm", algo_name);
    this->get_parameter("slam_algorithm", algo_name);
    if (algo_name == "fast_lio2" || algo_name.empty()) algo_name = "fast_lio2_pipeline";

    std::string config_path;
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("config_path", "");
    this->get_parameter("config_file", config_path);
    if (config_path.empty()) this->get_parameter("config_path", config_path);

    m_algo = SlamFactory::create(algo_name);
    if (!m_algo) { RCLCPP_FATAL(this->get_logger(), "Unknown algo: %s", algo_name.c_str()); throw std::runtime_error("unknown algo"); }
    if (!config_path.empty()) {
        YAML::Node cfg = YAML::LoadFile(config_path);
        if (cfg) {
            m_cfg.lidar_filter_num = cfg["lidar_filter_num"].as<int>(m_cfg.lidar_filter_num);
            m_cfg.lidar_min_range  = cfg["lidar_min_range"].as<double>(m_cfg.lidar_min_range);
            m_cfg.lidar_max_range  = cfg["lidar_max_range"].as<double>(m_cfg.lidar_max_range);
        }
    }
    if (!m_algo->init(config_path)) { RCLCPP_FATAL(this->get_logger(), "Init failed"); throw std::runtime_error("init failed"); }
    m_algo->setOutputCallback([this](const SlamOutput& o){ onOutput(o); });

    m_imu_sub  = create_subscription<sensor_msgs::msg::Imu>(m_cfg.imu_topic, 10, std::bind(&SlamNode::imuCB, this, _1));
    m_lidar_sub= create_subscription<sensor_msgs::msg::PointCloud2>(m_cfg.lidar_topic, 10, std::bind(&SlamNode::lidarCB, this, _1));
    m_init_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&SlamNode::initialPoseCB, this, _1));

    m_body_cloud_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
    m_world_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
    m_odom_pub        = create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
    m_path_pub        = create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
    m_global_map_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 10);
    m_grid_map_pub    = create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 1);
    m_map_pub         = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    m_odom_nav_pub    = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    m_tf = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    m_timer     = create_wall_timer(20ms, std::bind(&SlamNode::timerCB, this));
    m_map_timer = create_wall_timer(2s,   std::bind(&SlamNode::mapTimerCB, this));
    m_path.header.frame_id = m_cfg.world_frame;

    m_srv_save_map = create_service<rosiwit_slam::srv::SaveMap>("save_map", std::bind(&SlamNode::handleSaveMap, this, _1, _2, _3));
    m_srv_load_map = create_service<rosiwit_slam::srv::LoadMap>("load_map", std::bind(&SlamNode::handleLoadMap, this, _1, _2, _3));
    m_srv_save_grid_map = create_service<rosiwit_slam::srv::SaveGridMap>("save_grid_map", std::bind(&SlamNode::handleSaveGridMap, this, _1, _2, _3));
    m_srv_set_slam_mode = create_service<rosiwit_slam::srv::SetSlamMode>("set_slam_mode", std::bind(&SlamNode::handleSetSlamMode, this, _1, _2, _3));

    auto pipe = getPipeline();
    std::string mode_str = "idle";
    if (pipe) { auto m = pipe->getMode(); mode_str = (m==PipelineMode::MAPPING)?"mapping":(m==PipelineMode::LOCALIZATION)?"localization":"idle"; }
    RCLCPP_INFO(get_logger(), "SlamNode ready: algo=%s mode=%s imu=%s lidar=%s", algo_name.c_str(), mode_str.c_str(), m_cfg.imu_topic.c_str(), m_cfg.lidar_topic.c_str());
}

void SlamNode::loadParameters() {
    this->declare_parameter<std::string>("imu_topic", m_cfg.imu_topic);
    this->declare_parameter<std::string>("lidar_topic", m_cfg.lidar_topic);
    this->declare_parameter<std::string>("body_frame", m_cfg.body_frame);
    this->declare_parameter<std::string>("world_frame", m_cfg.world_frame);
    this->get_parameter("imu_topic", m_cfg.imu_topic);
    this->get_parameter("lidar_topic", m_cfg.lidar_topic);
    this->get_parameter("body_frame", m_cfg.body_frame);
    this->get_parameter("world_frame", m_cfg.world_frame);
}

void SlamNode::imuCB(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMUSample s;
    s.acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    s.gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.time = RosUtils::getSec(msg->header);
    m_algo->onImu(s);
}

void SlamNode::lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    LidarFrame f;
    f.cloud = RosUtils::ros2PCL(msg, m_cfg.lidar_filter_num, m_cfg.lidar_min_range, m_cfg.lidar_max_range);
    std::sort(f.cloud->points.begin(), f.cloud->points.end(), [](const PointType& a, const PointType& b){ return a.curvature < b.curvature; });
    f.start_time = RosUtils::getSec(msg->header);
    f.end_time = f.start_time + (f.cloud->points.empty() ? 0.0 : f.cloud->points.back().curvature / 1000.0);
    m_algo->onLidar(f);
}

void SlamNode::onOutput(const SlamOutput& out) { std::lock_guard<std::mutex> lock(m_out_mutex); m_latest = out; m_have_output = true; }

void SlamNode::timerCB() {
    auto* base = dynamic_cast<SlamBase*>(m_algo.get());
    if (base) base->tryPopAndProcess();
    SlamOutput out; bool have;
    { std::lock_guard<std::mutex> lock(m_out_mutex); have = m_have_output; out = m_latest; }
    if (have && out.has_new_pose) publish(out);
}

void SlamNode::publish(const SlamOutput& out) {
    builtin_interfaces::msg::Time stamp;
    if (out.pose.time > 0) stamp = toRosTime(out.pose.time);
    else stamp = now();
    Eigen::Quaterniond q(out.pose.rot);
    if (m_odom_pub->get_subscription_count() > 0) {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = m_cfg.world_frame; odom.child_frame_id = m_cfg.body_frame; odom.header.stamp = stamp;
        odom.pose.pose.position.x = out.pose.trans.x(); odom.pose.pose.position.y = out.pose.trans.y(); odom.pose.pose.position.z = out.pose.trans.z();
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y(); odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = out.pose.vel.x(); odom.twist.twist.linear.y = out.pose.vel.y(); odom.twist.twist.linear.z = out.pose.vel.z();
        m_odom_pub->publish(odom);
        if (m_odom_nav_pub->get_subscription_count() > 0) m_odom_nav_pub->publish(odom);
    }
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = m_cfg.world_frame; tf.child_frame_id = m_cfg.body_frame; tf.header.stamp = stamp;
        tf.transform.translation.x = out.pose.trans.x(); tf.transform.translation.y = out.pose.trans.y(); tf.transform.translation.z = out.pose.trans.z();
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y(); tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        m_tf->sendTransform(tf);
    }
    auto pub_cloud = [&](const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& p, const CloudType::Ptr& c, const std::string& fid){
        if (!p || p->get_subscription_count() <= 0 || !c) return;
        sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(*c, m); m.header.frame_id = fid; m.header.stamp = stamp; p->publish(m);
    };
    pub_cloud(m_body_cloud_pub, out.body_cloud, m_cfg.body_frame);
    pub_cloud(m_world_cloud_pub, out.world_cloud, m_cfg.world_frame);
    if (m_path_pub->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseStamped ps; ps.header.frame_id = m_cfg.world_frame; ps.header.stamp = stamp;
        ps.pose.position.x = out.pose.trans.x(); ps.pose.position.y = out.pose.trans.y(); ps.pose.position.z = out.pose.trans.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        m_path.poses.push_back(ps); m_path_pub->publish(m_path);
    }
}

void SlamNode::mapTimerCB() {
    auto pipe = getPipeline();
    bool always_pub = (pipe && pipe->getMode() == PipelineMode::LOCALIZATION);

    // 1. cloud_map
    if (always_pub || m_global_map_pub->get_subscription_count() > 0) {
        PointVec pts;
        if (m_algo->getGlobalMap(pts) && !pts.empty()) {
            CloudType cloud; for (auto& p : pts) cloud.push_back(p);
            cloud.width = cloud.size(); cloud.height = 1;
            sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(cloud, m);
            m.header.frame_id = m_cfg.world_frame; m.header.stamp = now(); m_global_map_pub->publish(m);
        }
    }

    // 2. grid_map (nav_msgs/OccupancyGrid) — 定位模式下发布
    if (pipe && pipe->getMode() == PipelineMode::LOCALIZATION && !m_grid_map_published) {
        auto* pcd = dynamic_cast<PcdMapManager*>(pipe->m_map_mgr.get());
        if (pcd && pcd->getGridWidth() > 0 && pcd->getGridHeight() > 0) {
            nav_msgs::msg::OccupancyGrid grid;
            grid.header.frame_id = m_cfg.world_frame;
            grid.header.stamp = now();
            grid.info.resolution = pcd->getGridRes();
            grid.info.width = pcd->getGridWidth();
            grid.info.height = pcd->getGridHeight();
            grid.info.origin.position.x = pcd->getGridOriginX();
            grid.info.origin.position.y = pcd->getGridOriginY();
            grid.info.origin.position.z = 0.0;
            grid.info.origin.orientation.w = 1.0;
            grid.data = pcd->getGridData();
            m_grid_map_pub->publish(grid);
            if (m_map_pub) m_map_pub->publish(grid);
            m_grid_map_published = true;
            RCLCPP_INFO(get_logger(), "Published grid_map (%d x %d, res=%.3f)", grid.info.width, grid.info.height, grid.info.resolution);
        }
    }
}

void SlamNode::initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    auto pipe = getPipeline();
    if (!pipe || pipe->getMode() != PipelineMode::LOCALIZATION || !pipe->m_localization) return;
    PoseStamped pose;
    pose.trans.x() = msg->pose.pose.position.x; pose.trans.y() = msg->pose.pose.position.y; pose.trans.z() = msg->pose.pose.position.z;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    pose.rot = q.toRotationMatrix();
    pipe->m_localization->setInitPose(pose);
    RCLCPP_INFO(get_logger(), "Set init pose: %.2f %.2f %.2f", pose.trans.x(), pose.trans.y(), pose.trans.z());
}

builtin_interfaces::msg::Time SlamNode::toRosTime(double sec) { return RosUtils::getTime(sec); }
SlamPipeline* SlamNode::getPipeline() { return dynamic_cast<SlamPipeline*>(m_algo.get()); }

void SlamNode::handleSaveMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::SaveMap::Request> req, std::shared_ptr<rosiwit_slam::srv::SaveMap::Response> res) {
    auto pipe = getPipeline();
    if (!pipe || !pipe->m_map_mgr) { res->success = false; res->message = "No map manager"; return; }
    res->success = pipe->m_map_mgr->saveMap(req->path);
    res->message = res->success ? "Saved" : "Failed";
}

void SlamNode::handleLoadMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::LoadMap::Request> req, std::shared_ptr<rosiwit_slam::srv::LoadMap::Response> res) {
    auto pipe = getPipeline();
    if (!pipe || !pipe->m_map_mgr) { res->success = false; res->message = "No map manager"; return; }
    PoseStamped init; init.trans = V3D::Zero(); init.rot = M3D::Identity();
    res->success = pipe->loadMapForLocalization(req->path, init);
    if (res->success) {
        auto* pcd = dynamic_cast<PcdMapManager*>(pipe->m_map_mgr.get());
        if (pcd) pcd->generateGridMap(0.05);
    }
    res->message = res->success ? "Loaded" : "Failed";
}

void SlamNode::handleSaveGridMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::SaveGridMap::Request> req, std::shared_ptr<rosiwit_slam::srv::SaveGridMap::Response> res) {
    auto pipe = getPipeline();
    if (!pipe) { res->success = false; res->message = "No pipeline"; return; }
    auto* pcd = dynamic_cast<PcdMapManager*>(pipe->m_map_mgr.get());
    if (!pcd) { res->success = false; res->message = "No PcdMapManager"; return; }
    res->success = pcd->saveGridMap(req->pgm_path, req->yaml_path, req->resolution);
    res->message = res->success ? "Saved" : "Failed";
}

void SlamNode::handleSetSlamMode(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam::srv::SetSlamMode::Request> req, std::shared_ptr<rosiwit_slam::srv::SetSlamMode::Response> res) {
    auto pipe = getPipeline();
    if (!pipe) { res->success = false; res->message = "No pipeline"; return; }
    if (req->mode == "mapping")         { pipe->setMode(PipelineMode::MAPPING); m_have_output = false; m_path.poses.clear(); }
    else if (req->mode == "localization") { pipe->setMode(PipelineMode::LOCALIZATION); m_have_output = false; m_path.poses.clear(); }
    else if (req->mode == "idle")         { pipe->setMode(PipelineMode::IDLE); m_have_output = false; }
    else { res->success = false; res->message = "Unknown: " + req->mode; return; }
    res->success = true; res->message = "Mode -> " + req->mode;
}

} // namespace rosiwit_slam
