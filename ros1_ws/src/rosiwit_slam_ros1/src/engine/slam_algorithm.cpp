// ============================================================================
// slam_algorithm.cpp — 算法实例的生命周期
//
// 只负责一件事: 把一个 ROS-free 的核心算法"跑起来"并管住它的线程。
//   * 构造/析构、start()/shutdown() (FrontEnd + 50Hz tick + 50Hz publish);
//   * 输入入队后的处理链入口 (processImu/processLidar 见 input_pipeline.cpp);
//   * 核心输出的快照 (onOutput/latestOutput) 与 initial pose 落地。
// 不在这里做的事: 配置装配 (config_bridge)、外参解析 (mount_frames)、
// 点云转换 (cloud_utils)、与核心的持锁边界 (core_gateway)。
// ============================================================================
#include "slam_algorithm.h"

#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include "slam_core/diagnostics.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <chrono>
#include <future>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "slam_core/slam_factory.h"
#include "slam_core/slam_pipeline.h"

namespace rosiwit_slam_ros1 {

using namespace rosiwit_slam;









SlamAlgorithm::SlamAlgorithm(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                             rosiwit_common::IExecutor* executor)
    : nh_(nh), pnh_(pnh), executor_(executor) {

    std::string algo_name = "fast_lio2_pipeline";
    pnh_.param<std::string>("slam_algorithm", algo_name, algo_name);

    // 配置装配拆到 config_bridge.cpp (S1): 校准 / 输入 / 核心注入 三段各自独立,
    // 构造函数只负责"按顺序装配 + 建核心"。
    initCalibrationConfig();
    bool vo_enabled = false, odom_enabled = false, absolute_enabled = false;
    loadInputConfig(vo_enabled, odom_enabled, absolute_enabled);
    prepareCoreConfig();


    algo_ = SlamFactory::create(algo_name);
    if (!algo_) { ROS_FATAL("unknown algo: %s", algo_name.c_str()); ros::shutdown(); return; }
    algo_raw_ = algo_.get();
    base_ = dynamic_cast<SlamBase*>(algo_.get());
    if (!algo_->init(config_path_)) { ROS_FATAL("init failed"); ros::shutdown(); return; }
    algo_->setOutputCallback([this](const SlamOutput& o) { onOutput(o); });

    // 输入订阅由 SlamRosNode 统一负责 (本类只暴露 onXxxMsg), 避免"一个进程两处订阅"。
    // 轮式里程计由 Ros1SlamInterface 统一订阅并喂核心, 避免同一份 /odom 注入两次。
    (void)odom_enabled;
    vo_enabled_ = vo_enabled;
    absolute_enabled_ = absolute_enabled;

    ok_ = true;
    ROS_INFO("rosiwit_slam_ros1 ready: algo=%s imu=%s lidar=%s",
             algo_name.c_str(), cfg_.imu_topic.c_str(), cfg_.lidar_topic.c_str());
    // 必须能被读到: 用 std::cout + endl 而不是 ROS_INFO —— roslaunch 把 stdout 重定向到
    // 管道时是块缓冲的, ROS_INFO 要等缓冲满才落盘, 实测在 `roslaunch ... | head` 里看不到。
    // (约定: 所有"上机器人后要核对"的启动日志都用 std::cout << ... << std::endl)
    std::cout << "[slam_algo] config=" << (config_path_.empty() ? "(none)" : config_path_)
              << " imu=" << cfg_.imu_topic
              << " lidar=" << cfg_.lidar_topic << std::endl;
}

SlamAlgorithm::~SlamAlgorithm() { shutdown(); }

void SlamAlgorithm::start()
{
    if (!ok_ || frontend_) { return; }
    // FrontEnd: 唯一消费帧的线程 (替代原 50Hz processCB 里的 tryPopAndProcess)
    frontend_.reset(new rosiwit_common::Thread("slam_fe",
        [this](rosiwit_common::CancelToken token) { frontendLoop(token); }));
    // Tick: 新接口的精简输出 (只发 /lio_odom 等), 50Hz
    tick_.reset(new rosiwit_common::PeriodicThread("slam_tick",
        std::chrono::milliseconds(20), [this] { tickLoop(); }));
    // Publish: 老契约位姿/TF/健康话题, 独立线程 —— 与 tick 抢同一线程会把
    // /lio_odom 从 ~96Hz 拖到 67Hz (实测)。
    publish_.reset(new rosiwit_common::PeriodicThread("slam_pub",
        std::chrono::milliseconds(20), [this] { publishLoop(); }));
    ROS_INFO("[slam_algo] started: frontend thread + 50Hz tick%s",
             executor_ ? " (+ injected executor)" : "");
}

void SlamAlgorithm::shutdown()
{
    // 顺序: 停止接收 -> 唤醒并等 FrontEnd -> 停 tick。
    // 注意 executor 由组装根关闭, 这里不能碰 (LOST 时还要投递飞行记录)。
    input_.close();
    if (frontend_) { frontend_->requestStop(); frontend_->join(); frontend_.reset(); }
    if (tick_) { tick_->requestStop(); tick_->join(); tick_.reset(); }
    if (publish_) { publish_->requestStop(); publish_->join(); publish_.reset(); }
    // 后端的线程是 abandon 语义 (可能卡在 Ceres 里), 退出前给它一个有界窗口,
    // 免得进程在静态对象析构时还有线程在跑 (I6)。
    if (algo_raw_) { algo_raw_->waitForBackgroundIdle(2000); }
}

void SlamAlgorithm::frontendLoop(rosiwit_common::CancelToken token)
{
    while (!token.cancelled()) {
        auto item = input_.pop_for(std::chrono::milliseconds(10));
        if (!item.has_value()) {
            if (input_.closed()) { break; }
            continue;
        }
        if (item->kind == InputKind::IMU) {
            processImu(item->imu);
        } else {
            processLidar(item->lidar);
        }
        // 建图前端: 有活就推进 (原来在 50Hz 定时器里做的事, 现在跟着帧走)
        {
            std::lock_guard<std::mutex> lk(core_mu_);
            if (base_) { while (base_->tryPopAndProcess()) {} }
        }
        logInputStats();
    }
}

void SlamAlgorithm::tickLoop()
{
    for (auto & h : tick_hooks_) { if (h) { h(); } }
}

void SlamAlgorithm::publishLoop()
{
    for (auto & h : publish_hooks_) { if (h) { h(); } }
}























void SlamAlgorithm::applyInitPose(const geometry_msgs::PoseWithCovarianceStamped& in,
                               bool zero_if_insane, bool force_z_zero,
                               bool count_as_jump) {
    std::lock_guard<std::mutex> core_lk(core_mu_);
    if (!algo_raw_) return;
    // 只有"坐标系真的换了"才计数; 内部 2D 小纠偏 (0.1~0.3m) 不算跳变
    if (count_as_jump) { ++pose_jump_seq_; }
    double x = in.pose.pose.position.x;
    double y = in.pose.pose.position.y;
    double z = in.pose.pose.position.z;
    if (zero_if_insane && (std::fabs(x) > 1e4 || std::fabs(y) > 1e4)) { x = 0.0; y = 0.0; }
    if (force_z_zero) z = 0.0;

    Eigen::Quaterniond q(in.pose.pose.orientation.w, in.pose.pose.orientation.x,
                         in.pose.pose.orientation.y, in.pose.pose.orientation.z);
    // App 没有下发初始位姿时，四元数通常是全零。它不是 base_link 的 (0,0,0)，
    // 而是"没有设置"的占位值；此时由系统补核心 map/IMU 零点，直接 setInitPose，
    // 不能做 base_link -> IMU 换算（否则会从 yaw=90° 开始，正好偏离地图零点 90°）。
    const bool default_map_pose = (q.norm() < 0.5);
    if (default_map_pose) {
        std::cout << "[slam_algo] no initial pose -> core map/IMU identity 0,0,0"
                  << std::endl;
        x = 0.0;
        y = 0.0;
        z = 0.0;
        q = Eigen::Quaterniond::Identity();
    }
    q.normalize();

    PoseStamped p;
    p.trans = V3D(x, y, z);
    p.rot = q.toRotationMatrix();
    p.time = in.header.stamp.toSec();

    // 有效初始位姿来自 App，语义为 base_link；核心吃 IMU body 位姿。
    // 缺省 map/IMU zero 位姿已经在上面的分支直通，不能再次叠加安装角。
    if (default_map_pose) {
        if (force_z_zero) p.trans.z() = 0.0;
    } else if (computeBaseFrames(/*blocking=*/true) && baseImu().valid) {
        const RigidTransform base_imu = baseImu();
        const double yaw_b = std::atan2(p.rot(1, 0), p.rot(0, 0)) * 180 / M_PI;
        p = composeRight(p, base_imu);
        if (force_z_zero) p.trans.z() = 0.0;
        const double yaw_a = std::atan2(p.rot(1, 0), p.rot(0, 0)) * 180 / M_PI;
        char b[256];
        std::snprintf(b, sizeof(b),
                      "[slam_algo] init pose converted base->imu: yaw_before=%.1fdeg yaw_after=%.1fdeg",
                      yaw_b, yaw_a);
        std::cout << b << std::endl;
    } else {
        if (force_z_zero) p.trans.z() = 0.0;
        ROS_WARN_STREAM("[slam_algo] base_link <- imu unavailable; init pose left in base frame");
    }
    algo_raw_->setInitPose(p);
}





void SlamAlgorithm::onOutput(const SlamOutput& out) {
    if (online_calib_enabled_ && out.has_new_pose && out.pose.time > 0.0) {
        const Eigen::Quaterniond q(out.pose.rot);
        const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
        std::lock_guard<std::mutex> lk(calib_mutex_);
        online_time_calib_.addLioPose(out.pose.time, yaw);
        if (have_imu_yaw_ && std::abs(out.pose.time - last_imu_stamp_) < 0.15) {
            yaw_calib_.addSample(last_imu_yaw_, yaw);
        }
    }
    {
        std::lock_guard<std::mutex> lk(out_mutex_);
        latest_ = out;
        have_output_ = true;
    }
    if (output_hook_) output_hook_(out);
}


bool SlamAlgorithm::latestOutput(SlamOutput& out) const {
    std::lock_guard<std::mutex> lk(out_mutex_);
    if (!have_output_) return false;
    out = latest_;
    return true;
}

void SlamAlgorithm::clearLatestOutput() {
    std::lock_guard<std::mutex> lk(out_mutex_);
    latest_ = SlamOutput();
    have_output_ = false;
}








rosiwit_slam::CalibrationDecision SlamAlgorithm::updateCalibration() {
    std::lock_guard<std::mutex> lk(calib_mutex_);
    const auto t = online_time_calib_.estimate();
    const auto y = yaw_calib_.estimate();
    const auto d = calib_controller_.update(t, y);
    imu_time_offset_.store(d.time_offset);
    return d;
}

rosiwit_slam::TimeCalibrationResult SlamAlgorithm::onlineTimeCalibration() const {
    std::lock_guard<std::mutex> lk(calib_mutex_);
    return online_time_calib_.estimate();
}

rosiwit_slam::YawCalibrationResult SlamAlgorithm::onlineYawCalibration() const {
    std::lock_guard<std::mutex> lk(calib_mutex_);
    return yaw_calib_.estimate();
}




void SlamAlgorithm::clearTickHooks() {
    tick_hooks_.clear();
}

void SlamAlgorithm::clearPublishHooks() {
    publish_hooks_.clear();
}

void SlamAlgorithm::addPublishHook(std::function<void()> hook) {
    if (hook) publish_hooks_.push_back(std::move(hook));
}

void SlamAlgorithm::clearInputHooks() {
    input_seen_hook_ = nullptr;
    raw_lidar_hook_ = nullptr;
}

void SlamAlgorithm::addTickHook(std::function<void()> hook) {
    if (hook) tick_hooks_.push_back(std::move(hook));
}

void SlamAlgorithm::setOutputHook(std::function<void(const SlamOutput&)> hook) {
    output_hook_ = std::move(hook);
}

void SlamAlgorithm::setInputSeenHook(std::function<void(bool)> hook) {
    input_seen_hook_ = std::move(hook);
}

void SlamAlgorithm::setRawLidarHook(
    std::function<void(const CloudType&, const ros::Time&)> hook) {
    raw_lidar_hook_ = std::move(hook);
}

} // namespace rosiwit_slam_ros1