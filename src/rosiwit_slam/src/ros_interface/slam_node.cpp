// src/ros_interface/slam_node.cpp
#include "ros_interface/slam_node.h"
#include "slam_core/slam_factory.h"
#include "slam_core/slam_base.h"   // dynamic_cast to call tryPopAndProcess
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include "utils.h"                 // Utils::ros2PCL / getSec / getTime (map_builder)

using namespace std::chrono_literals;

namespace rosiwit_slam {

SlamNode::SlamNode(const rclcpp::NodeOptions& options) : rclcpp::Node("rosiwit_slam", options) {
    loadParameters();

    std::string algo_name = "fast_lio2";
    this->declare_parameter<std::string>("slam_algorithm", algo_name);
    this->get_parameter("slam_algorithm", algo_name);

    std::string config_path;
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("config_path", "");
    this->get_parameter("config_file", config_path);
    if (config_path.empty()) this->get_parameter("config_path", config_path);

    m_algo = SlamFactory::create(algo_name);
    if (!m_algo) {
        RCLCPP_FATAL(this->get_logger(), "Unknown SLAM algorithm: %s", algo_name.c_str());
        throw std::runtime_error("unknown slam algorithm");
    }
    // 从 config yaml 读取 lidar_filter_num / range, 用于入参时的点云预处理
    if (!config_path.empty()) {
        YAML::Node cfg = YAML::LoadFile(config_path);
        if (cfg) {
            m_cfg.lidar_filter_num = cfg["lidar_filter_num"].as<int>(m_cfg.lidar_filter_num);
            m_cfg.lidar_min_range  = cfg["lidar_min_range"].as<double>(m_cfg.lidar_min_range);
            m_cfg.lidar_max_range  = cfg["lidar_max_range"].as<double>(m_cfg.lidar_max_range);
            m_cfg.print_time_cost  = cfg["print_time_cost"].as<bool>(m_cfg.print_time_cost);
        }
    }
    if (!m_algo->init(config_path)) {
        RCLCPP_FATAL(this->get_logger(), "Algorithm init failed for config: %s", config_path.c_str());
        throw std::runtime_error("slam init failed");
    }
    m_algo->setOutputCallback([this](const SlamOutput& o){ onOutput(o); });

    m_imu_sub  = create_subscription<sensor_msgs::msg::Imu>(m_cfg.imu_topic, 10,
                  std::bind(&SlamNode::imuCB, this, std::placeholders::_1));
    m_lidar_sub= create_subscription<sensor_msgs::msg::PointCloud2>(m_cfg.lidar_topic, 10,
                  std::bind(&SlamNode::lidarCB, this, std::placeholders::_1));
    m_body_cloud_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
    m_world_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
    m_odom_pub        = create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
    m_path_pub        = create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
    m_global_map_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 10);
    m_tf = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    m_timer     = create_wall_timer(20ms, std::bind(&SlamNode::timerCB, this));
    m_map_timer = create_wall_timer(2s,   std::bind(&SlamNode::mapTimerCB, this));
    m_path.header.frame_id = m_cfg.world_frame;
    RCLCPP_INFO(get_logger(), "SlamNode ready: algo=%s imu=%s lidar=%s",
                algo_name.c_str(), m_cfg.imu_topic.c_str(), m_cfg.lidar_topic.c_str());
}

void SlamNode::loadParameters() {
    this->declare_parameter<std::string>("imu_topic",   m_cfg.imu_topic);
    this->declare_parameter<std::string>("lidar_topic", m_cfg.lidar_topic);
    this->declare_parameter<std::string>("body_frame",  m_cfg.body_frame);
    this->declare_parameter<std::string>("world_frame", m_cfg.world_frame);
    this->get_parameter("imu_topic",   m_cfg.imu_topic);
    this->get_parameter("lidar_topic", m_cfg.lidar_topic);
    this->get_parameter("body_frame",  m_cfg.body_frame);
    this->get_parameter("world_frame", m_cfg.world_frame);
}

void SlamNode::imuCB(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMUSample s;
    s.acc  = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    s.gyro = V3D(msg->angular_velocity.x,    msg->angular_velocity.y,    msg->angular_velocity.z);
    s.time = Utils::getSec(msg->header);
    m_algo->onImu(s);
}

void SlamNode::lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    LidarFrame f;
    f.cloud = Utils::ros2PCL(msg, m_cfg.lidar_filter_num, m_cfg.lidar_min_range, m_cfg.lidar_max_range);
    // 沿用现状: 按点时间(curvature 存偏移)排序
    std::sort(f.cloud->points.begin(), f.cloud->points.end(),
              [](const PointType& a, const PointType& b){ return a.curvature < b.curvature; });
    f.start_time = Utils::getSec(msg->header);
    f.end_time   = f.start_time + (f.cloud->points.empty() ? 0.0 : f.cloud->points.back().curvature / 1000.0);
    m_algo->onLidar(f);
}

void SlamNode::onOutput(const SlamOutput& out) {
    std::lock_guard<std::mutex> lock(m_out_mutex);
    m_latest = out;
    m_have_output = true;
}

void SlamNode::timerCB() {
    // 轮询触发同步处理 (SlamBase fix b)
    auto* base = dynamic_cast<SlamBase*>(m_algo.get());
    if (base) {
        do { } while (base->tryPopAndProcess());   // 排空已同步的包
    }

    SlamOutput out; bool have;
    { std::lock_guard<std::mutex> lock(m_out_mutex); have = m_have_output; out = m_latest; }
    if (have && out.has_new_pose) publish(out);
}

void SlamNode::publish(const SlamOutput& out) {
    auto stamp = toRosTime(out.pose.time);
    Eigen::Quaterniond q(out.pose.rot);

    if (m_odom_pub->get_subscription_count() > 0) {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = m_cfg.world_frame; odom.child_frame_id = m_cfg.body_frame;
        odom.header.stamp = stamp;
        odom.pose.pose.position.x = out.pose.trans.x();
        odom.pose.pose.position.y = out.pose.trans.y();
        odom.pose.pose.position.z = out.pose.trans.z();
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = out.pose.vel.x();
        odom.twist.twist.linear.y = out.pose.vel.y();
        odom.twist.twist.linear.z = out.pose.vel.z();
        m_odom_pub->publish(odom);
    }

    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = m_cfg.world_frame; tf.child_frame_id = m_cfg.body_frame;
        tf.header.stamp = stamp;
        tf.transform.translation.x = out.pose.trans.x();
        tf.transform.translation.y = out.pose.trans.y();
        tf.transform.translation.z = out.pose.trans.z();
        tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
        m_tf->sendTransform(tf);
    }

    auto pub_cloud = [&](const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& p,
                         const CloudType::Ptr& c, const std::string& fid){
        if (!p || p->get_subscription_count() <= 0 || !c) return;
        sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(*c, m);
        m.header.frame_id = fid; m.header.stamp = stamp; p->publish(m);
    };
    pub_cloud(m_body_cloud_pub,  out.body_cloud,  m_cfg.body_frame);
    pub_cloud(m_world_cloud_pub, out.world_cloud, m_cfg.world_frame);

    if (m_path_pub->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = m_cfg.world_frame; ps.header.stamp = stamp;
        ps.pose.position.x = out.pose.trans.x(); ps.pose.position.y = out.pose.trans.y();
        ps.pose.position.z = out.pose.trans.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        m_path.poses.push_back(ps);
        m_path_pub->publish(m_path);
    }
}

void SlamNode::mapTimerCB() {
    if (m_global_map_pub->get_subscription_count() <= 0) return;
    PointVec pts;
    if (!m_algo->getGlobalMap(pts) || pts.empty()) return;
    CloudType cloud; for (auto& p : pts) cloud.push_back(p);
    cloud.width = cloud.size(); cloud.height = 1;
    sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(cloud, m);
    m.header.frame_id = m_cfg.world_frame; m.header.stamp = now();
    m_global_map_pub->publish(m);
}

builtin_interfaces::msg::Time SlamNode::toRosTime(double sec) { return Utils::getTime(sec); }

} // namespace rosiwit_slam
