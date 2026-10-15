// ============================================================================
// SlamEngine 实现 —— 全部从原 rosiwit_slam_ros1_node.cpp 搬移, 除 input_profile
// 与 legacy 输入话题覆盖外不改任何行为。
// ============================================================================
#include "slam_engine.h"

#include <ros/package.h>

#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <iostream>

#include "slam_core/slam_factory.h"

namespace rosiwit_slam_ros1 {

using namespace rosiwit_slam;

LegacyConfPaths defaultLegacyPaths() {
    LegacyConfPaths p;
    // 旧实现 (xz_3d_slam/CMakeLists.txt:24/26):
    //   install 分支  PROJECT_PATH = /opt/ros/melodic/share/xz_3d_slam
    //   其余分支      PROJECT_PATH = ${PROJECT_SOURCE_DIR}
    // 主配置恒为 PROJECT_PATH "/conf/slam_conf.json"。
    // ros::package::getPath 在 devel/overlay 下给源码目录、在 install 下给安装目录,
    // 两种情形都与旧实现对得上。
    const std::string pkg = ros::package::getPath("rosiwit_slam_ros1");
    if (pkg.empty()) {
        ROS_WARN("[slam_engine] ros::package::getPath('rosiwit_slam_ros1') failed; "
                 "legacy conf falls back to built-in defaults");
    }
    p.proj_conf_json = pkg + "/conf/slam_conf.json";
    // 旧实现只有一个主配置来源。T1 的 LegacyConfig 留了 proj(优先)/pkg(兜底) 两级,
    // 这里让二者指向同一份文件: 日常布局下内容与旧实现同一份, 同时给
    // "安装目录里没有但源码目录里有"这类非常规布局留个兜底。
    p.pkg_conf_json = pkg + "/conf/slam_conf.json";
    p.opt_sensor_json = "/opt/xzrobot/configs/sensor_config.json";
    p.opt_slam_json = "/opt/xzrobot/configs/3d_slam/slam_conf.json";
    return p;
}

SlamEngine::SlamEngine(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // ---- input_profile (M12: 默认 rosiwit, 只有 drop-in launch 显式传 legacy) ----
    pnh_.param<std::string>("input_profile", input_profile_, "rosiwit");
    if (input_profile_ != "rosiwit" && input_profile_ != "legacy") {
        ROS_WARN_STREAM("[slam_engine] unknown input_profile '" << input_profile_
                        << "', falling back to 'rosiwit'");
        input_profile_ = "rosiwit";
    }

    std::string algo_name = "fast_lio2_pipeline";
    pnh_.param<std::string>("slam_algorithm", algo_name, algo_name);

    pnh_.param<std::string>("config_file", config_path_, "");
    if (config_path_.empty()) pnh_.param<std::string>("config_path", config_path_, "");

    cfg_.imu_topic   = "/imu";
    cfg_.lidar_topic = "/velodyne_points";
    cfg_.body_frame  = "base_link";
    cfg_.world_frame = "odom";
    cfg_.scan_period_ms = 100.0f;
    cfg_.lidar_filter_num = 3;
    cfg_.lidar_min_range = 0.5;
    cfg_.lidar_max_range = 100.0;
    bool vo_enabled = false, odom_enabled = false, absolute_enabled = false;
    if (!config_path_.empty()) {
        try {
            YAML::Node c = YAML::LoadFile(config_path_);
            if (c) {
                cfg_.imu_topic   = c["imu_topic"].as<std::string>(cfg_.imu_topic);
                cfg_.lidar_topic = c["lidar_topic"].as<std::string>(cfg_.lidar_topic);
                cfg_.body_frame  = c["body_frame"].as<std::string>(cfg_.body_frame);
                cfg_.world_frame = c["world_frame"].as<std::string>(cfg_.world_frame);
                cfg_.scan_period_ms = c["scan_period_ms"].as<float>(cfg_.scan_period_ms);
                cfg_.lidar_filter_num = c["lidar_filter_num"].as<int>(cfg_.lidar_filter_num);
                cfg_.lidar_min_range = c["lidar_min_range"].as<double>(cfg_.lidar_min_range);
                cfg_.lidar_max_range = c["lidar_max_range"].as<double>(cfg_.lidar_max_range);
                if (c["vision"]) vo_enabled = c["vision"]["enabled"].as<bool>(false);
                odom_enabled = c["enable_odom"].as<bool>(false);
                absolute_enabled = c["absolute_enabled"].as<bool>(false);
            }
        } catch (const std::exception& e) {
            ROS_WARN("[rosiwit_slam_ros1] config parse error: %s", e.what());
        }
    }

    // ---- legacy profile: 旧配置决定输入话题名 (M13: 外参/单位仍取核心 YAML) ----
    if (legacyProfile()) {
        std::string model;
        nh_.param<std::string>("/machine/model", model, "");
        legacy_cfg_ = LegacyConfig::load(defaultLegacyPaths(), model);
        // 旧实现 startSub (legacy.cpp:200-207) 选输入话题靠的是 **laser_type**,
        // 不是 imu_sub_name/laser_sub_name:
        //   mid360 / new_mid360 -> "livox/imu" + "livox/lidar"
        //   其它                -> <imu_sub_name> + <laser_sub_name>
        // 这一步很关键: 发行 conf 的顶层 laser_sub_name 是 "vanjee_points722",
        // 但对 mid360 机型旧实现**根本不看它** —— 照 laser_sub_name 订阅会在真机上收不到数据。
        // (D4: livox/lidar 在我们这边是 sensor_msgs/PointCloud2, 不是 livox 的 CustomMsg。)
        const std::string& lt = legacy_cfg_.laser_type;
        if (lt == "mid360" || lt == "new_mid360") {
            cfg_.imu_topic = "livox/imu";
            cfg_.lidar_topic = "livox/lidar";
        } else {
            // 空值时保留核心 YAML 里的名字 (不应发生, 但别把订阅名清成空串)
            if (!legacy_cfg_.imu_sub_name.empty())   cfg_.imu_topic   = legacy_cfg_.imu_sub_name;
            if (!legacy_cfg_.laser_sub_name.empty()) cfg_.lidar_topic = legacy_cfg_.laser_sub_name;
        }
        std::cout << "[slam_engine] legacy laser_type='" << lt << "' -> input topics: imu="
                  << cfg_.imu_topic << " lidar=" << cfg_.lidar_topic << std::endl;
    }

    algo_ = SlamFactory::create(algo_name);
    if (!algo_) { ROS_FATAL("unknown algo: %s", algo_name.c_str()); ros::shutdown(); return; }
    algo_raw_ = algo_.get();
    base_ = dynamic_cast<SlamBase*>(algo_.get());
    if (!algo_->init(config_path_)) { ROS_FATAL("init failed"); ros::shutdown(); return; }
    algo_->setOutputCallback([this](const SlamOutput& o) { onOutput(o); });

    // ---- 订阅 ----
    imu_sub_  = nh_.subscribe(cfg_.imu_topic, 1000, &SlamEngine::imuCB, this);
    lidar_sub_ = nh_.subscribe(cfg_.lidar_topic, 100, &SlamEngine::lidarCB, this);
    init_pose_sub_ = nh_.subscribe("/initialpose", 10, &SlamEngine::initialPoseCB, this);
    if (odom_enabled) odom_sub_ = nh_.subscribe("/odom", 100, &SlamEngine::odomCB, this);
    if (vo_enabled) vo_sub_ = nh_.subscribe("/visual_odometry", 100, &SlamEngine::voCB, this);
    if (absolute_enabled) absolute_sub_ = nh_.subscribe("/absolute_pose", 100, &SlamEngine::absoluteCB, this);

    // ---- 泵 (0.02s = 50Hz): 排空核心输出 + 跑各接口挂的 tick ----
    process_timer_ = nh_.createTimer(ros::Duration(0.02), &SlamEngine::processCB, this);

    ok_ = true;
    ROS_INFO("rosiwit_slam_ros1 ready: algo=%s imu=%s lidar=%s",
             algo_name.c_str(), cfg_.imu_topic.c_str(), cfg_.lidar_topic.c_str());
    // 必须能被读到: 用 std::cout + endl 而不是 ROS_INFO —— roslaunch 把 stdout 重定向到
    // 管道时是块缓冲的, ROS_INFO 要等缓冲满才落盘, 实测在 `roslaunch ... | head` 里看不到。
    // (约定: 所有"上机器人后要核对"的启动日志都用 std::cout << ... << std::endl)
    std::cout << "[slam_engine] profile=" << input_profile_
              << " config=" << (config_path_.empty() ? "(none)" : config_path_)
              << " imu=" << cfg_.imu_topic
              << " lidar=" << cfg_.lidar_topic << std::endl;
}

SlamEngine::~SlamEngine() = default;

CloudType::Ptr SlamEngine::ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                   int filter_num, double min_r, double max_r) {
    CloudType::Ptr cloud(new CloudType());
    pcl::fromROSMsg(*msg, *cloud);
    filter_num = std::max(1, filter_num);
    CloudType::Ptr out(new CloudType());
    out->reserve(cloud->size() / filter_num + 1);
    const double min2 = min_r * min_r, max2 = max_r * max_r;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (i % filter_num != 0) continue;
        const auto& p = cloud->points[i];
        if (!pcl::isFinite(p)) continue;
        const double d2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
                          static_cast<double>(p.z) * p.z;
        if (d2 < min2 || d2 > max2) continue;
        out->push_back(p);
    }
    return out;
}

void SlamEngine::onImuMsg(const sensor_msgs::Imu::ConstPtr& msg) {
    IMUSample s;
    s.acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    s.gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.time = getSec(msg->header.stamp);
    if (input_seen_hook_) input_seen_hook_(false);
    if (algo_) algo_->onImu(s);
}

void SlamEngine::onLidarMsg(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!algo_) return;
    LidarFrame f;
    f.cloud = ros2Pcl(msg, cfg_.lidar_filter_num, cfg_.lidar_min_range, cfg_.lidar_max_range);
    if (!f.cloud || f.cloud->empty()) return;
    f.start_time = getSec(msg->header.stamp);
    f.end_time = f.start_time + cfg_.scan_period_ms / 1000.0;
    if (input_seen_hook_) input_seen_hook_(true);
    if (raw_lidar_hook_) raw_lidar_hook_(*f.cloud, msg->header.stamp);
    algo_->onLidar(f);
}

void SlamEngine::onInitialPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!algo_raw_ || algo_raw_->getPipelineMode() != "localization") return;
    PoseStamped p;
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    p.time = getSec(msg->header.stamp);
    algo_raw_->setInitPose(p);
}

void SlamEngine::onOdomMsg(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!algo_) return;
    PoseStamped p;
    p.time = getSec(msg->header.stamp);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    algo_->onOdom(p);
}

void SlamEngine::imuCB(const sensor_msgs::Imu::ConstPtr& msg) { onImuMsg(msg); }
void SlamEngine::lidarCB(const sensor_msgs::PointCloud2::ConstPtr& msg) { onLidarMsg(msg); }
void SlamEngine::odomCB(const nav_msgs::Odometry::ConstPtr& msg) { onOdomMsg(msg); }
void SlamEngine::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    onInitialPoseMsg(msg);
}

void SlamEngine::voCB(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!algo_) return;
    PoseStamped p;
    p.time = getSec(msg->header.stamp);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    algo_->onVisualOdometry(p.time, p);
}

void SlamEngine::absoluteCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!algo_) return;
    PoseStamped p;
    p.time = getSec(msg->header.stamp);
    p.trans = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    p.rot = q.toRotationMatrix();
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e2;
    auto mi = [](int r) { return r < 3 ? r + 3 : r - 3; };
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            cov(r, c) = msg->pose.covariance[mi(r) * 6 + mi(c)];
    algo_->onAbsolute(p, cov);
}

void SlamEngine::onOutput(const SlamOutput& out) {
    {
        std::lock_guard<std::mutex> lk(out_mutex_);
        latest_ = out;
        have_output_ = true;
    }
    if (output_hook_) output_hook_(out);
}

void SlamEngine::processCB(const ros::TimerEvent&) {
    if (base_) { while (base_->tryPopAndProcess()) {} }
    for (auto& h : tick_hooks_) if (h) h();
}

bool SlamEngine::latestOutput(SlamOutput& out) const {
    std::lock_guard<std::mutex> lk(out_mutex_);
    if (!have_output_) return false;
    out = latest_;
    return true;
}

bool SlamEngine::getGlobalMap(PointVec& out) const {
    if (!algo_raw_) return false;
    return algo_raw_->getGlobalMap(out);
}

bool SlamEngine::getGridInfo(GridInfo& out) const {
    if (!algo_raw_) return false;
    out = algo_raw_->getGridInfo();
    return true;
}

bool SlamEngine::getPoseCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) const {
    if (!algo_raw_) return false;
    return algo_raw_->getPoseCov(out, cov);
}

std::string SlamEngine::pipelineMode() const {
    return algo_raw_ ? algo_raw_->getPipelineMode() : std::string();
}

void SlamEngine::addTickHook(std::function<void()> hook) {
    if (hook) tick_hooks_.push_back(std::move(hook));
}

void SlamEngine::setOutputHook(std::function<void(const SlamOutput&)> hook) {
    output_hook_ = std::move(hook);
}

void SlamEngine::setInputSeenHook(std::function<void(bool)> hook) {
    input_seen_hook_ = std::move(hook);
}

void SlamEngine::setRawLidarHook(
    std::function<void(const CloudType&, const ros::Time&)> hook) {
    raw_lidar_hook_ = std::move(hook);
}

} // namespace rosiwit_slam_ros1
