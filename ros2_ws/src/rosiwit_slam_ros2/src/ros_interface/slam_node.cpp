// src/ros_interface/slam_node.cpp
#include "ros_interface/slam_node.h"
#include "slam_core/slam_factory.h"
#include "slam_core/slam_base.h"
#include "slam_core/slam_pipeline.h"
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <tf2/time.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <functional>
#include <cmath>
#include "ros_interface/ros_utils.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rosiwit_slam {

using rosiwit_slam_ros2::composeRight;
using rosiwit_slam_ros2::composeRightInv;
using rosiwit_slam_ros2::transformCloud;
using rosiwit_slam_ros2::transformCovariance;

SlamNode::~SlamNode() {
    m_process_timer.reset();
    m_map_timer.reset();
    m_mount_timer.reset();
}

bool SlamNode::lookupMount(const std::string& target, const std::string& source,
                           RigidTransform& out, double timeout_sec) {
    if (source.empty() || target.empty() || target == source) {
        out.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        return true;
    }
    if (!m_tf_buffer) return false;
    try {
        const auto tf = m_tf_buffer->lookupTransform(
            target, source, tf2::TimePointZero, tf2::durationFromSec(timeout_sec));
        const auto& q_msg = tf.transform.rotation;
        Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        q.normalize();
        const auto& t_msg = tf.transform.translation;
        out.set(q.toRotationMatrix(), Eigen::Vector3d(t_msg.x, t_msg.y, t_msg.z));
        return true;
    } catch (const tf2::TransformException&) {
        return false;
    }
}

bool SlamNode::ensureBaseFrames() {
    // 可能由 mount timer (work 组) 与 /initialpose (sensor 组) 并发调用.
    std::lock_guard<std::mutex> mount_lk(m_mount_mu);
    if (m_base_imu.valid) return true;
    if (lookupMount(m_cfg.body_frame, m_cfg.imu_frame, m_base_imu, 0.05)) {
        if (!lookupMount(m_cfg.body_frame, m_cfg.lidar_frame, m_base_lidar, 0.05)) {
            if (m_cfg.lidar_frame == m_cfg.imu_frame) m_base_lidar = m_base_imu;
        }
        const Eigen::Vector3d rpy = m_base_imu.R.eulerAngles(2, 1, 0);
        RCLCPP_INFO(get_logger(),
                    "ROS frame: T_%s_%s from TF rpy(deg)=[%.2f %.2f %.2f] xyz=[%.3f %.3f %.3f]",
                    m_cfg.body_frame.c_str(), m_cfg.imu_frame.c_str(),
                    rpy.z() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI,
                    rpy.x() * 180.0 / M_PI, m_base_imu.t.x(), m_base_imu.t.y(),
                    m_base_imu.t.z());
        if (m_mount_timer) m_mount_timer->cancel();
        return true;
    }
    if (!m_mount_warned) {
        RCLCPP_WARN(get_logger(),
                    "ROS frame: TF '%s' <- '%s' unavailable; output stays in core frame",
                    m_cfg.body_frame.c_str(), m_cfg.imu_frame.c_str());
        m_mount_warned = true;
    }
    return false;
}

bool SlamNode::imuFromChild(const std::string& child_frame, RigidTransform& out) {
    if (child_frame.empty() || child_frame == m_cfg.imu_frame) {
        out.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        return true;
    }
    if (child_frame == m_cfg.body_frame) {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        if (!m_base_imu.valid) return false;
        out = m_base_imu;
        return true;
    }
    {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        auto it = m_child_to_imu.find(child_frame);
        if (it != m_child_to_imu.end()) {
            out = it->second;
            return true;
        }
    }
    if (!lookupMount(child_frame, m_cfg.imu_frame, out, 0.1)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF imu <- %s unavailable; drop measurement",
                             child_frame.c_str());
        return false;
    }
    std::lock_guard<std::mutex> lk(m_mount_mu);
    m_child_to_imu[child_frame] = out;
    return true;
}

std::string SlamNode::writeExtrinsicOverride(const std::string& base_yaml,
                                             const RigidTransform& imu_lidar,
                                             const RigidTransform& base_lidar,
                                             const RigidTransform& base_imu) {
    if (base_yaml.empty()) return base_yaml;
    const std::string out_path = "/tmp/rosiwit_slam_ros2_core_extrinsic.yaml";
    std::ifstream in(base_yaml);
    std::ofstream out(out_path, std::ios::trunc);
    if (!out.good()) return base_yaml;
    std::string line;
    while (std::getline(in, line)) {
        const size_t b = line.find_first_not_of(" \t");
        if (b != std::string::npos) {
            const std::string t = line.substr(b);
            if (t.rfind("r_il:", 0) == 0 || t.rfind("t_il:", 0) == 0 ||
                t.rfind("esti_il:", 0) == 0 || t.rfind("bbs_mount_r:", 0) == 0 ||
                t.rfind("init_z:", 0) == 0) continue;
        }
        out << line << "\n";
    }
    out << "esti_il: false   # ROS layer feeds IMU body; never estimate extrinsic online\n";
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "\n# --- r_il/t_il from TF (IMU<-lidar), injected by ROS2 layer ---\n"
                  "r_il: [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n"
                  "t_il: [%.8f, %.8f, %.8f]\n",
                  imu_lidar.R(0,0), imu_lidar.R(0,1), imu_lidar.R(0,2),
                  imu_lidar.R(1,0), imu_lidar.R(1,1), imu_lidar.R(1,2),
                  imu_lidar.R(2,0), imu_lidar.R(2,1), imu_lidar.R(2,2),
                  imu_lidar.t.x(), imu_lidar.t.y(), imu_lidar.t.z());
    out << buf;
    std::snprintf(buf, sizeof(buf),
                  "init_z: %.8f   # IMU body 世界 z 初值, 使 base_footprint 在地面 z=0\n",
                  base_imu.valid ? base_imu.t.z() : 0.0);
    out << buf;
    if (base_lidar.valid) {
        std::snprintf(buf, sizeof(buf),
                      "\n# base<-lidar rotation for BBS car-frame projection\n"
                      "bbs_mount_r: [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n",
                      base_lidar.R(0,0), base_lidar.R(0,1), base_lidar.R(0,2),
                      base_lidar.R(1,0), base_lidar.R(1,1), base_lidar.R(1,2),
                      base_lidar.R(2,0), base_lidar.R(2,1), base_lidar.R(2,2));
        out << buf;
    }
    return out_path;
}

void SlamNode::resetPublishState() {
    std::lock_guard<std::mutex> lock(m_out_mutex);
    m_have_output = false;
    m_path.poses.clear();
    m_grid_map_published = false;
    m_last_map_revision = 0;
    m_last_cloud_map_revision = 0;
    m_has_smoothed = false;
    m_has_cov = false;
}

SlamNode::SlamNode(const rclcpp::NodeOptions& options) : rclcpp::Node("rosiwit_slam", options) {
    std::string algo_name = "fast_lio2";
    this->declare_parameter<std::string>("slam_algorithm", algo_name);
    this->get_parameter("slam_algorithm", algo_name);
    if (algo_name.empty()) algo_name = "fast_lio2_pipeline";

    std::string config_path;
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<std::string>("config_path", "");
    this->get_parameter("config_file", config_path);
    if (config_path.empty()) this->get_parameter("config_path", config_path);

    m_algo = SlamFactory::create(algo_name);
    if (!m_algo) { RCLCPP_FATAL(this->get_logger(), "Unknown algo: %s", algo_name.c_str()); throw std::runtime_error("unknown algo"); }
    m_algo_raw = m_algo.get();
    if (!config_path.empty()) {
        YAML::Node cfg = YAML::LoadFile(config_path);
        if (cfg) {
            m_cfg.imu_topic        = cfg["imu_topic"].as<std::string>(m_cfg.imu_topic);
            m_cfg.lidar_topic      = cfg["lidar_topic"].as<std::string>(m_cfg.lidar_topic);
            m_cfg.lidar_frame      = cfg["lidar_frame"].as<std::string>(m_cfg.lidar_frame);
            m_cfg.imu_frame        = cfg["imu_frame"].as<std::string>(m_cfg.imu_frame);
            m_cfg.body_frame       = cfg["body_frame"].as<std::string>(m_cfg.body_frame);
            m_cfg.world_frame      = cfg["world_frame"].as<std::string>(m_cfg.world_frame);
            m_cfg.map_frame        = cfg["map_frame"].as<std::string>(m_cfg.map_frame);
            m_cfg.odom_frame       = cfg["odom_frame"].as<std::string>(m_cfg.odom_frame);
            m_cfg.odom_mode        = cfg["odom_mode"].as<std::string>(m_cfg.odom_mode);
            m_cfg.lidar_filter_num = cfg["lidar_filter_num"].as<int>(m_cfg.lidar_filter_num);
            m_cfg.lidar_min_range  = cfg["lidar_min_range"].as<double>(m_cfg.lidar_min_range);
            m_cfg.lidar_max_range  = cfg["lidar_max_range"].as<double>(m_cfg.lidar_max_range);
            m_cfg.scan_period_ms   = cfg["scan_period_ms"].as<float>(m_cfg.scan_period_ms);
            m_cfg.odom_enabled     = cfg["enable_odom"].as<bool>(m_cfg.odom_enabled);
            m_cfg.odom_topic       = cfg["odom_topic"].as<std::string>(m_cfg.odom_topic);
            m_cfg.output_rate_hz   = cfg["output_rate_hz"].as<double>(m_cfg.output_rate_hz);
            m_cfg.output_filter_alpha = cfg["output_filter_alpha"].as<double>(m_cfg.output_filter_alpha);
            m_cfg.output_filter_beta  = cfg["output_filter_beta"].as<double>(m_cfg.output_filter_beta);
            m_cfg.output_extrap_max_s = cfg["output_extrap_max_s"].as<double>(m_cfg.output_extrap_max_s);
            m_cfg.grid_map_resolution = cfg["map_resolution"].as<double>(m_cfg.grid_map_resolution);
            m_cfg.map_publish_period = cfg["map_publish_period"].as<double>(m_cfg.map_publish_period);
            m_cfg.absolute_enabled = cfg["absolute_enabled"].as<bool>(m_cfg.absolute_enabled);
            m_cfg.absolute_topic   = cfg["absolute_topic"].as<std::string>(m_cfg.absolute_topic);
            m_cfg.datum_x = cfg["absolute_datum_x"].as<double>(m_cfg.datum_x);
            m_cfg.datum_y = cfg["absolute_datum_y"].as<double>(m_cfg.datum_y);
            m_cfg.datum_z = cfg["absolute_datum_z"].as<double>(m_cfg.datum_z);
            m_cfg.datum_yaw_deg = cfg["absolute_datum_yaw_deg"].as<double>(m_cfg.datum_yaw_deg);
            if (cfg["vision"]) {
                m_cfg.vision_enabled = cfg["vision"]["enabled"].as<bool>(m_cfg.vision_enabled);
                m_cfg.vo_topic       = cfg["vision"]["vo_topic"].as<std::string>(m_cfg.vo_topic);
            }
            if (cfg["output_smoothing_alpha"])
                m_ema_alpha = cfg["output_smoothing_alpha"].as<double>();
        }
    }
    // 行业标准: 核心估计 IMU body; r_il/t_il 必须由 ROS2 层从 TF 推导后注入.
    // TF listener 自带线程, 可在主 node spin 前收到 /tf_static.
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    RigidTransform imu_lidar;
    if (!lookupMount(m_cfg.imu_frame, m_cfg.lidar_frame, imu_lidar, 3.0)) {
        RCLCPP_FATAL(get_logger(),
                     "cannot determine T_%s_%s from TF; provide TF or set imu_frame==lidar_frame",
                     m_cfg.imu_frame.c_str(), m_cfg.lidar_frame.c_str());
        throw std::runtime_error("missing imu<-lidar TF");
    }
    RigidTransform base_imu, base_lidar;
    const bool have_base_imu = lookupMount(m_cfg.body_frame, m_cfg.imu_frame, base_imu, 3.0);
    const bool have_base_lidar = lookupMount(m_cfg.body_frame, m_cfg.lidar_frame, base_lidar, 0.2);
    {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        if (have_base_imu) m_base_imu = base_imu;
        if (have_base_lidar) m_base_lidar = base_lidar;
        else if (m_cfg.lidar_frame == m_cfg.imu_frame && have_base_imu) m_base_lidar = base_imu;
    }
    if (have_base_imu) {
        const Eigen::Vector3d rpy = base_imu.R.eulerAngles(2, 1, 0);
        RCLCPP_INFO(get_logger(),
                    "ROS frame: T_%s_%s rpy(deg)=[%.2f %.2f %.2f] xyz=[%.3f %.3f %.3f]",
                    m_cfg.body_frame.c_str(), m_cfg.imu_frame.c_str(),
                    rpy.z() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI,
                    rpy.x() * 180.0 / M_PI, base_imu.t.x(), base_imu.t.y(), base_imu.t.z());
    } else {
        RCLCPP_WARN(get_logger(),
                    "TF %s <- %s unavailable at startup; output will stay in core frame until it appears",
                    m_cfg.body_frame.c_str(), m_cfg.imu_frame.c_str());
    }
    // 同帧时 TF 只能给出 identity, 若仍覆写会把配置文件里真实的
    // t_il/r_il (例如 IMU 在 LiDAR 上方 9cm) 清零. 同帧明确表示
    // "核心已配置外参", 保留 YAML.
    if (m_cfg.imu_frame == m_cfg.lidar_frame) {
        RCLCPP_INFO(get_logger(),
            "IMU/LiDAR share frame '%s'; keeping configured r_il/t_il",
            m_cfg.imu_frame.c_str());
    } else {
        config_path = writeExtrinsicOverride(
            config_path, imu_lidar, m_base_lidar,
            have_base_imu ? base_imu : RigidTransform{});
    }

    if (!m_algo->init(config_path)) { RCLCPP_FATAL(this->get_logger(), "Init failed"); throw std::runtime_error("init failed"); }
    m_algo->setOutputCallback([this](const SlamOutput& o){ onOutput(o); });

    // 回调分组: 传感器订阅独立于"重处理"(定时器/服务)分组。配合多线程执行器,
    // 即使建图/回环/保存地图耗时, 收包回调仍能在另一线程及时入队, 避免 DDS 队列溢出丢帧。
    m_sensor_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_work_cb_group   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_map_cb_group    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_output_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sensor_sub_opts;
    sensor_sub_opts.callback_group = m_sensor_cb_group;

    // 真实 LiDAR/IMU 驱动通常 best-effort; 默认 reliable 订阅会直接匹配不上.
    // reliable 发布者可以被 best-effort 订阅者接收, 仿真/实机都兼容.
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort();
    m_imu_sub  = create_subscription<sensor_msgs::msg::Imu>(m_cfg.imu_topic, sensor_qos, std::bind(&SlamNode::imuCB, this, _1), sensor_sub_opts);
    m_lidar_sub= create_subscription<sensor_msgs::msg::PointCloud2>(m_cfg.lidar_topic, sensor_qos, std::bind(&SlamNode::lidarCB, this, _1), sensor_sub_opts);
    m_init_pose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&SlamNode::initialPoseCB, this, _1), sensor_sub_opts);
    // 视觉里程计 (可选): 由 vision.enabled 开关控制, 话题名可配置
    if (m_cfg.vision_enabled) {
        m_vo_sub = create_subscription<nav_msgs::msg::Odometry>(
            m_cfg.vo_topic, 100, std::bind(&SlamNode::voCB, this, _1), sensor_sub_opts);
    }
    if (m_cfg.odom_enabled) {
        // SLAM 自己发布 /odom; 若输入也配成 /odom 会形成自订阅反馈.
        // 真实轮式里程计应通过 odom_topic 指向 /wheel_odom 等独立话题.
        if (m_cfg.odom_topic == "/odom" || m_cfg.odom_topic == "odom") {
            RCLCPP_WARN(get_logger(),
                "odom_enabled=true but odom_topic=%s is the SLAM output topic; "
                "disable wheel-odom prior to avoid self-subscription",
                m_cfg.odom_topic.c_str());
            m_cfg.odom_enabled = false;
        }
    }
    if (m_cfg.odom_mode == "wheel") {
        if (!m_cfg.odom_enabled) {
            RCLCPP_WARN(get_logger(),
                "odom_mode=wheel requires enable_odom and a distinct odom_topic; "
                "falling back to unified map==odom");
            m_cfg.odom_mode = "unified";
        } else if (m_cfg.world_frame != m_cfg.map_frame) {
            RCLCPP_WARN(get_logger(),
                "odom_mode=wheel: world_frame '%s' -> '%s' (must equal map_frame)",
                m_cfg.world_frame.c_str(), m_cfg.map_frame.c_str());
            m_cfg.world_frame = m_cfg.map_frame;
        }
    }
    if (m_cfg.odom_enabled) {
        m_odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            m_cfg.odom_topic, 100, std::bind(&SlamNode::odomCB, this, _1), sensor_sub_opts);
    }
    if (m_cfg.absolute_enabled) {
        m_absolute_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            m_cfg.absolute_topic, 100, std::bind(&SlamNode::absoluteCB, this, _1), sensor_sub_opts);
    }

    m_body_cloud_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
    m_world_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
    m_odom_pub        = create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
    m_path_pub        = create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
    m_global_map_pub  = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 10);
    m_grid_map_pub    = create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map", 1);
    m_map_pub         = create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
    m_odom_nav_pub    = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    m_tf = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    m_static_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    // unified: 发布 map -> odom 静态 identity (map==odom 单世界模式).
    // wheel: map->odom 由 publishOdomTf() 动态计算, 不再发静态 identity.
    if (m_cfg.odom_mode != "wheel") {
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = now();
        static_tf.header.frame_id = m_cfg.map_frame;
        static_tf.child_frame_id = m_cfg.odom_frame;
        static_tf.transform.rotation.w = 1.0;
        m_static_tf->sendTransform(static_tf);
    } else {
        RCLCPP_INFO(get_logger(),
            "odom_mode=wheel: publishing dynamic %s -> %s from wheel odom",
            m_cfg.map_frame.c_str(), m_cfg.odom_frame.c_str());
    }
    // 处理定时器(work 分组): 只做重处理, 与传感器收包分组互不阻塞。
    // 缩短周期以降低入队->处理的延迟; 每 tick 把已同步的包一次性排空, 避免积压。
    m_process_timer = create_wall_timer(20ms, std::bind(&SlamNode::timerCB, this), m_work_cb_group);
    m_map_timer = create_wall_timer(
        std::chrono::duration<double>(std::max(0.05, m_cfg.map_publish_period)),
        std::bind(&SlamNode::mapTimerCB, this), m_map_cb_group);
    // base_link <- IMU 输出换算; 失败时 1Hz 限频重试, 不阻塞 100Hz 传感器回调.
    m_mount_timer = create_wall_timer(1s, std::bind(&SlamNode::ensureBaseFrames, this),
                                      m_work_cb_group);
    // L12 高频输出: 定位模式下按 output_rate_hz 发布平滑位姿(IMU 传播 + alpha-beta)
    m_out_filter.configure(m_cfg.output_filter_alpha, m_cfg.output_filter_beta, m_cfg.output_extrap_max_s);
    if (m_cfg.output_rate_hz > 0.0)
        m_output_timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / m_cfg.output_rate_hz),
            std::bind(&SlamNode::highRateCB, this), m_output_cb_group);
    m_path.header.frame_id = m_cfg.world_frame;

    // 服务与处理定时器同组, 避免保存地图等操作与建图重处理并发(扫描上下文/地图非线程安全)
    m_srv_save_map = create_service<rosiwit_slam_ros2::srv::SaveMap>("save_map", std::bind(&SlamNode::handleSaveMap, this, _1, _2, _3), rmw_qos_profile_services_default, m_work_cb_group);
    m_srv_load_map = create_service<rosiwit_slam_ros2::srv::LoadMap>("load_map", std::bind(&SlamNode::handleLoadMap, this, _1, _2, _3), rmw_qos_profile_services_default, m_work_cb_group);
    m_srv_save_grid_map = create_service<rosiwit_slam_ros2::srv::SaveGridMap>("save_grid_map", std::bind(&SlamNode::handleSaveGridMap, this, _1, _2, _3), rmw_qos_profile_services_default, m_work_cb_group);
    m_srv_set_slam_mode = create_service<rosiwit_slam_ros2::srv::SetSlamMode>("set_slam_mode", std::bind(&SlamNode::handleSetSlamMode, this, _1, _2, _3), rmw_qos_profile_services_default, m_work_cb_group);

    std::string mode_str = m_algo_raw->getPipelineMode();
    RCLCPP_INFO(get_logger(), "SlamNode ready: algo=%s mode=%s imu=%s lidar=%s",
                algo_name.c_str(), mode_str.c_str(), m_cfg.imu_topic.c_str(), m_cfg.lidar_topic.c_str());
    if (m_cfg.vision_enabled)
        RCLCPP_INFO(get_logger(), "Vision loose-coupling ON: vo_topic=%s", m_cfg.vo_topic.c_str());
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
    f.cloud = RosUtils::ros2PCL(msg, m_cfg.lidar_filter_num, m_cfg.lidar_min_range, m_cfg.lidar_max_range, m_cfg.scan_period_ms);
    std::sort(f.cloud->points.begin(), f.cloud->points.end(), [](const PointType& a, const PointType& b){ return a.curvature < b.curvature; });
    f.start_time = RosUtils::getSec(msg->header);
    f.end_time = f.start_time + (f.cloud->points.empty() ? 0.0 : f.cloud->points.back().curvature / 1000.0);
    m_algo->onLidar(f);
    // 只入队, 重处理交给 work 分组定时器, 保证本回调尽快返回、不阻塞后续收包
}

void SlamNode::voCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    PoseStamped p;
    p.time = RosUtils::getSec(msg->header);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const auto& q = msg->pose.pose.orientation;
    p.rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    RigidTransform imu_child;
    if (!imuFromChild(msg->child_frame_id, imu_child)) return;
    m_algo->onVisualOdometry(p.time, composeRight(p, imu_child));
}

void SlamNode::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    PoseStamped p;
    p.time = RosUtils::getSec(msg->header);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const auto& q = msg->pose.pose.orientation;
    p.rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    // wheel_odom 模式: 保存 odom->body, 用于计算 map->odom.
    if (m_cfg.odom_mode == "wheel" && msg->child_frame_id == m_cfg.body_frame) {
        std::lock_guard<std::mutex> lk(m_wheel_odom_mutex);
        m_wheel_odom = p;
        m_wheel_odom_valid = true;
        m_wheel_odom_sec = p.time;
    } else if (m_cfg.odom_mode == "wheel") {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "wheel odom child '%s' != body_frame '%s'; cannot compute map->odom",
            msg->child_frame_id.c_str(), m_cfg.body_frame.c_str());
    }
    RigidTransform imu_child;
    if (!imuFromChild(msg->child_frame_id, imu_child)) return;
    m_algo->onOdom(composeRight(p, imu_child));
}

void SlamNode::absoluteCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    PoseStamped meas;
    meas.time = RosUtils::getSec(msg->header);
    meas.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    const auto& q = msg->pose.pose.orientation;
    meas.rot = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    // datum 变换: map_T_abs ∘ 测量
    const M3D Rd = Eigen::AngleAxisd(m_cfg.datum_yaw_deg * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix();
    PoseStamped pose;
    pose.time = meas.time;
    pose.rot = Rd * meas.rot;
    pose.trans = Rd * meas.trans + V3D(m_cfg.datum_x, m_cfg.datum_y, m_cfg.datum_z);
    // 协方差: ROS [x,y,z,rot] -> 我们 [rot(3),trans(3)]
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e2;
    auto mi = [](int r) { return r < 3 ? r + 3 : r - 3; };
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            cov(r, c) = msg->pose.covariance[mi(r) * 6 + mi(c)];
    ensureBaseFrames();
    RigidTransform base_imu;
    {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        base_imu = m_base_imu;
    }
    if (!base_imu.valid) return;
    pose = composeRight(pose, base_imu);   // map<-base -> map<-imu
    cov = transformCovariance(cov, base_imu.R);
    m_algo->onAbsolute(pose, cov);
}

void SlamNode::onOutput(const SlamOutput& out) {
  std::lock_guard<std::mutex> lock(m_out_mutex);
  // EMA 平滑只用于定位输出; 建图时位姿必须与 world_cloud 同帧对齐, 平滑会让 TF/odom 滞后于地图.
  const bool loc = (m_algo_raw && m_algo_raw->getPipelineMode() == "localization");
  if (!loc || !m_has_smoothed) {
    m_smoothed_pose = out.pose;
    m_has_smoothed = true;
  } else {
    double a = m_ema_alpha;
    m_smoothed_pose.trans = a * out.pose.trans + (1.0 - a) * m_smoothed_pose.trans;
    Eigen::Quaterniond q_curr(m_smoothed_pose.rot);
    Eigen::Quaterniond q_next(out.pose.rot);
    m_smoothed_pose.rot = q_curr.slerp(a, q_next).toRotationMatrix();
    m_smoothed_pose.time = out.pose.time;
    m_smoothed_pose.vel = a * out.pose.vel + (1.0 - a) * m_smoothed_pose.vel;
  }
  m_latest = out;
  // 用平滑后的位姿替换原始位姿
  m_latest.pose = m_smoothed_pose;
  m_have_output = true;
}

void SlamNode::timerCB() {
    auto* base = dynamic_cast<SlamBase*>(m_algo.get());
    // 一次 tick 内排空所有已同步的包, 防止重处理慢时数据在内部缓冲区积压
    if (base) { while (base->tryPopAndProcess()) {} }
    // 定位模式的 LiDAR 重处理已从 sensor callback 移到 work timer.
    if (auto* pipeline = dynamic_cast<SlamPipeline*>(m_algo.get())) {
        while (pipeline->tryPopLocalization()) {}
    }
    SlamOutput out; bool should_publish = false;
    {
        std::lock_guard<std::mutex> lock(m_out_mutex);
        if (m_have_output && m_latest.has_new_pose) {
            out = m_latest;
            // 消费掉本次新位姿. 之前 has_new_pose 一直为 true, 处理定时器
            // 会以几十 Hz 重复发布同一帧 odom/TF: 相邻消息 stamp 相同,
            // twist.angular 差分为 0, 看起来像 SLAM 不输出角速度.
            m_latest.has_new_pose = false;
            should_publish = true;
        }
    }
    if (should_publish) publish(out);
}

void SlamNode::publish(const SlamOutput& out) {
    // 行业标准: 核心输出 IMU body; 所有对外位姿/TF/body_cloud 在 ROS2 层转 base_link。
    // world_cloud 已在 map 系, 不参与变换。
    RigidTransform base_imu;
    {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        base_imu = m_base_imu;
    }
    PoseStamped pose = out.pose;
    CloudType::Ptr body_cloud = out.body_cloud;
    if (base_imu.valid) {
        pose = composeRightInv(out.pose, base_imu);
        if (out.body_cloud) {
            body_cloud.reset(new CloudType(*out.body_cloud));
            transformCloud(*body_cloud, base_imu);
        }
    }

    builtin_interfaces::msg::Time stamp;
    if (pose.time > 0) stamp = toRosTime(pose.time);
    else stamp = now();
    // 更新最新协方差 (定位模式); 位姿换到 base 后协方差同步旋转.
    PoseStamped pv;
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
    if (m_algo && m_algo->getPoseCov(pv, cov)) {
        std::lock_guard<std::mutex> lk(m_cov_mutex);
        m_pose_cov = base_imu.valid ? transformCovariance(cov, base_imu.R) : cov;
        m_has_cov = true;
    }
    Eigen::Quaterniond q(pose.rot);
    // 定位模式且开启高频输出时, odom/TF 由 highRateCB 高频发布; 此处不重复
    const bool hr = (m_cfg.output_rate_hz > 0.0) &&
                    (m_algo_raw && m_algo_raw->getPipelineMode() == "localization");
    if (!hr) publishOdomTf(pose, stamp);
    auto pub_cloud = [&](const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& p, const CloudType::Ptr& c, const std::string& fid){
        if (!p || p->get_subscription_count() <= 0 || !c) return;
        sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(*c, m); m.header.frame_id = fid; m.header.stamp = stamp; p->publish(m);
    };
    pub_cloud(m_body_cloud_pub, body_cloud, m_cfg.body_frame);
    pub_cloud(m_world_cloud_pub, out.world_cloud, m_cfg.world_frame);
    if (m_path_pub->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseStamped ps; ps.header.frame_id = m_cfg.world_frame; ps.header.stamp = stamp;
        ps.pose.position.x = pose.trans.x(); ps.pose.position.y = pose.trans.y(); ps.pose.position.z = pose.trans.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        std::lock_guard<std::mutex> lock(m_out_mutex);
        m_path.poses.push_back(ps);
        // 长时间运行不能无限增长; 按块裁剪, 避免每帧 O(N) 搬移.
        if (m_path.poses.size() > 25000) {
            m_path.poses.erase(m_path.poses.begin(),
                               m_path.poses.begin() + 5000);
        }
        m_path_pub->publish(m_path);
    }
}

void SlamNode::publishOdomTf(const PoseStamped& pose, const builtin_interfaces::msg::Time& stamp) {
    const Eigen::Quaterniond q(pose.rot);
    const double stamp_sec = static_cast<double>(stamp.sec) +
        static_cast<double>(stamp.nanosec) * 1e-9;

    // 同一个位姿不要重复发 odom/TF. 重复 stamp 会让 twist 差分为 0,
    // 也会让下游 TF 缓存/控制周期看到不推进的旧帧.
    if (m_last_odom_twist_valid &&
        std::abs(stamp_sec - m_last_odom_twist_sec) < 1e-6) {
        return;
    }

    // REP-105: Odometry.twist 在 child_frame(base) 系. 核心给的是 world 系
    // velocity, 必须旋转到 body 系; 角速度用相邻姿态四元数差分.
    const Eigen::Vector3d vel_body = q.conjugate() * pose.vel;
    double wz = 0.0;
    if (m_last_odom_twist_valid) {
        const double dt = stamp_sec - m_last_odom_twist_sec;
        if (dt > 1e-3 && dt < 0.5) {
            Eigen::Quaterniond dq = m_last_odom_twist_q.conjugate() * q;
            if (dq.w() < 0.0) dq.coeffs() *= -1.0;  // shortest rotation
            const Eigen::AngleAxisd aa(dq);
            if (std::abs(aa.angle()) > 1e-6) {
                wz = aa.axis().z() * aa.angle() / dt;
            }
        }
    }
    if (stamp_sec > 0.0) {
        m_last_odom_twist_valid = true;
        m_last_odom_twist_sec = stamp_sec;
        m_last_odom_twist_q = q;
    }

    const bool wheel_mode = (m_cfg.odom_mode == "wheel");
    const bool want_lio_odom = m_odom_pub && m_odom_pub->get_subscription_count() > 0;
    const bool want_odom = !wheel_mode && m_odom_nav_pub &&
        m_odom_nav_pub->get_subscription_count() > 0;
    if (want_lio_odom || want_odom) {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = m_cfg.world_frame; odom.child_frame_id = m_cfg.body_frame; odom.header.stamp = stamp;
        odom.pose.pose.position.x = pose.trans.x(); odom.pose.pose.position.y = pose.trans.y(); odom.pose.pose.position.z = pose.trans.z();
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y(); odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = vel_body.x();
        odom.twist.twist.linear.y = vel_body.y();
        odom.twist.twist.linear.z = vel_body.z();
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = wz;
        // 协方差: 我们的顺序 [rot(3),trans(3)] -> ROS [x,y,z,rot(3)]
        Eigen::Matrix<double, 6, 6> cov;
        bool has_cov = false;
        {
            std::lock_guard<std::mutex> lk(m_cov_mutex);
            cov = m_pose_cov;
            has_cov = m_has_cov;
        }
        auto mi = [](int r) { return r < 3 ? r + 3 : r - 3; };
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                odom.pose.covariance[r * 6 + c] = has_cov ? cov(mi(r), mi(c)) : 0.0;
        // 两个别名分别按各自订阅者发布, 不能被 lio_odom 的订阅数门控.
        if (want_lio_odom) m_odom_pub->publish(odom);
        if (want_odom) m_odom_nav_pub->publish(odom);
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    if (wheel_mode) {
        // map <- base = SLAM pose; odom <- base = wheel odom;
        // map <- odom = map<-base * (odom<-base)^-1.
        PoseStamped wheel;
        bool wheel_valid = false;
        {
            std::lock_guard<std::mutex> lk(m_wheel_odom_mutex);
            wheel = m_wheel_odom;
            wheel_valid = m_wheel_odom_valid &&
                (now().seconds() - m_wheel_odom_sec <= 1.0);
        }
        if (!wheel_valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "wheel odom unavailable; cannot publish %s -> %s",
                m_cfg.map_frame.c_str(), m_cfg.odom_frame.c_str());
            return;
        }
        const Eigen::Matrix3d R_odom_base = wheel.rot;
        const Eigen::Vector3d t_odom_base = wheel.trans;
        const Eigen::Matrix3d R_map_base = pose.rot;
        const Eigen::Vector3d t_map_base = pose.trans;
        const Eigen::Matrix3d R_map_odom = R_map_base * R_odom_base.transpose();
        const Eigen::Vector3d t_map_odom = t_map_base - R_map_odom * t_odom_base;
        const Eigen::Quaterniond q_map_odom(R_map_odom);
        tf.header.frame_id = m_cfg.map_frame;
        tf.child_frame_id = m_cfg.odom_frame;
        tf.transform.translation.x = t_map_odom.x();
        tf.transform.translation.y = t_map_odom.y();
        tf.transform.translation.z = t_map_odom.z();
        tf.transform.rotation.x = q_map_odom.x();
        tf.transform.rotation.y = q_map_odom.y();
        tf.transform.rotation.z = q_map_odom.z();
        tf.transform.rotation.w = q_map_odom.w();
        m_tf->sendTransform(tf);
    } else {
        tf.header.frame_id = m_cfg.world_frame;
        tf.child_frame_id = m_cfg.body_frame;
        tf.transform.translation.x = pose.trans.x();
        tf.transform.translation.y = pose.trans.y();
        tf.transform.translation.z = pose.trans.z();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        m_tf->sendTransform(tf);
    }
}

void SlamNode::highRateCB() {
    if (!m_algo_raw || m_algo_raw->getPipelineMode() != "localization") return;
    const rclcpp::Time nowt = this->now();
    const double t = nowt.seconds();
    PoseStamped p;
    if (!m_algo->getHighRatePose(t, p)) return;   // 定位未初始化/丢失时跳过
    m_out_filter.update(t, p);
    PoseStamped o;
    if (!m_out_filter.poseAt(t, o)) return;
    RigidTransform base_imu;
    {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        base_imu = m_base_imu;
    }
    if (base_imu.valid) o = composeRightInv(o, base_imu);
    publishOdomTf(o, nowt);
}

void SlamNode::mapTimerCB() {
    std::string mode = m_algo_raw->getPipelineMode();
    bool is_localization = (mode == "localization");
    bool always_pub = is_localization;
    const uint64_t map_revision = m_algo_raw->getRevision();

    // 1. cloud_map: 建图按 revision 事件发布, 定位模式保持周期发布.
    if ((always_pub || m_global_map_pub->get_subscription_count() > 0) &&
        (always_pub || map_revision != m_last_cloud_map_revision))
    {
        PointVec pts;
        if (m_algo->getGlobalMap(pts) && !pts.empty()) {
            CloudType cloud; for (auto& p : pts) cloud.push_back(p);
            cloud.width = cloud.size(); cloud.height = 1;
            sensor_msgs::msg::PointCloud2 m; pcl::toROSMsg(cloud, m);
            m.header.frame_id = m_cfg.world_frame; m.header.stamp = now(); m_global_map_pub->publish(m);
            m_last_cloud_map_revision = map_revision;
        }
    }

    // 2. grid_map (nav_msgs/OccupancyGrid)
    // 事件驱动: 只有地图版本变化 (新关键帧/后端优化) 才重建并发布,
    // 静止或转弯时不会按 1s 定时刷新底图.
    if (m_grid_map_published && map_revision == m_last_map_revision) {
      return;
    }
    const bool need_grid = is_localization ||
        m_map_pub->get_subscription_count() > 0 ||
        m_grid_map_pub->get_subscription_count() > 0;
    if (need_grid) {
        rosiwit_slam::GridInfo grid_info;
        bool have_grid = false;
        if (is_localization) {
            grid_info = m_algo_raw->getGridInfo();
            have_grid = grid_info.width > 0 && grid_info.height > 0;
        } else {
            // 走 generateGridMap 的同一路径 (逐帧观测 + 地面处理), 保证实时图与存盘一致.
            have_grid = m_algo_raw->getRealtimeGrid(m_cfg.grid_map_resolution, grid_info);
            if (!have_grid) {
                grid_info = m_algo_raw->getGridInfo();
                have_grid = grid_info.width > 0 && grid_info.height > 0;
            }
        }
        if (have_grid) {
            nav_msgs::msg::OccupancyGrid grid;
            grid.header.frame_id = m_cfg.world_frame;
            grid.header.stamp = now();
            grid.info.resolution = grid_info.resolution;
            grid.info.width = grid_info.width;
            grid.info.height = grid_info.height;
            grid.info.origin.position.x = grid_info.origin_x;
            grid.info.origin.position.y = grid_info.origin_y;
            grid.info.origin.position.z = 0.0;
            grid.info.origin.orientation.w = 1.0;
            // 内部 GridInfo 与 ROS OccupancyGrid 都是 row0 = 最小 y, 直接拷贝.
            grid.data = grid_info.data;
            m_grid_map_pub->publish(grid);
            if (m_map_pub) m_map_pub->publish(grid);
            m_grid_map_published = true;
            m_last_map_revision = map_revision;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "Published grid_map (%d x %d, res=%.3f)",
                grid.info.width, grid.info.height, grid.info.resolution);
        }
    }
}

void SlamNode::initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (m_algo_raw->getPipelineMode() != "localization") return;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // 全零/近零四元数 = "无初始位姿"; normalize 后是 NaN, 不能送进核心.
    if (q.norm() < 0.5) {
        RCLCPP_WARN(get_logger(), "initialpose quaternion too small (|q|<0.5); ignored");
        return;
    }
    q.normalize();
    PoseStamped pose;
    pose.trans.x() = msg->pose.pose.position.x;
    pose.trans.y() = msg->pose.pose.position.y;
    pose.trans.z() = msg->pose.pose.position.z;
    pose.rot = q.toRotationMatrix();
    pose.time = RosUtils::getSec(msg->header);
    // 上层下发 base_link 位姿; 核心吃 IMU body: T_world_imu = T_world_base * T_base_imu.
    ensureBaseFrames();
    RigidTransform base_imu;
    {
        std::lock_guard<std::mutex> lk(m_mount_mu);
        base_imu = m_base_imu;
    }
    if (base_imu.valid) {
        pose = composeRight(pose, base_imu);
    } else {
        RCLCPP_WARN(get_logger(),
                    "initialpose received before TF '%s' <- '%s'; using raw pose",
                    m_cfg.body_frame.c_str(), m_cfg.imu_frame.c_str());
    }
    if (m_algo_raw->setInitPose(pose)) {
        RCLCPP_INFO(get_logger(), "Set init pose: %.2f %.2f %.2f",
                    pose.trans.x(), pose.trans.y(), pose.trans.z());
    }
}

builtin_interfaces::msg::Time SlamNode::toRosTime(double sec) { return RosUtils::getTime(sec); }

void SlamNode::handleSaveMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::SaveMap::Request> req, std::shared_ptr<rosiwit_slam_ros2::srv::SaveMap::Response> res) {
    res->success = m_algo_raw->saveMap(req->path);
    res->message = res->success ? "Saved" : "Failed";
}

void SlamNode::handleLoadMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::LoadMap::Request> req, std::shared_ptr<rosiwit_slam_ros2::srv::LoadMap::Response> res) {
    res->success = m_algo_raw->loadMap(req->path);
    res->message = res->success ? "Loaded" : "Failed";
    if (res->success) resetPublishState();   // 新地图必须允许 /map 重新发布
}

void SlamNode::handleSaveGridMap(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::SaveGridMap::Request> req, std::shared_ptr<rosiwit_slam_ros2::srv::SaveGridMap::Response> res) {
    if (!req->generator.empty() && !m_algo_raw->setGridMapGenerator(req->generator)) {
        res->success = false;
        res->message = "Unknown generator: " + req->generator;
        return;
    }
    res->success = m_algo_raw->saveGridMap(req->pgm_path, req->yaml_path, req->resolution);
    res->message = res->success ? "Saved" : "Failed";
}

void SlamNode::handleSetSlamMode(const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<rosiwit_slam_ros2::srv::SetSlamMode::Request> req, std::shared_ptr<rosiwit_slam_ros2::srv::SetSlamMode::Response> res) {
    if (m_algo_raw->setPipelineMode(req->mode)) {
        resetPublishState();   // 清轨迹/平滑器, 并允许新模式的 /map 重新发布
        res->success = true; res->message = "Mode -> " + req->mode;
    } else {
        res->success = false; res->message = "Unknown: " + req->mode;
    }
}

} // namespace rosiwit_slam
