// ============================================================================
// legacy 装配层: 构造/发布与订阅注册/定时器/服务注册/模式机 —— 从 legacy_xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "legacy_xz3dslam_interface.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "legacy_convert.h"
#include "legacy_xz3dslam_internal.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::legacy_internal;

namespace rosiwit_slam_ros1 {
LegacyXz3dSlamInterface::LegacyXz3dSlamInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                                 SlamEngine& engine)
    : nh_(nh), pnh_(pnh), engine_(engine),
      cfg_(engine.legacyConfig()),
      debug_publish_(engine.legacyConfig().debug_publish),
      is_debug_(engine.legacyConfig().is_debug),
      use_tf_(engine.legacyConfig().use_tf) {
    initPub();
    loadWarningCodeTable();   // 告警码表 (M6: 从本包 share 读, 不读 /opt)
    initTimerPub();
    initSub();
    initClients();
    initServer();
    // 与旧实现一致: 切建图时清空上一轮地图 (见 resetCoreAndCaches 注释)。
    // 默认开启; `_reset_on_mode_switch:=false` 可退回"不清图"的行为做 A/B。
    pnh_.param<bool>("reset_on_mode_switch", reset_on_mode_switch_, true);

    // [T9] TF: 旧实现构造里就建了 listener (legacy.cpp:94) 与 broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    // 旧 node main 第 73 行就是 `node->run()`, 而 run() 末尾会置
    // warning_code_start_task_ = true (legacy.cpp:184) —— 所以**开机时抑制窗口是开的**,
    // 之后 100 个告警 tick (rt_grid 周期, 默认 1s -> 100s) 不发告警。
    setWarningSuppression(true);
    // 地面 z 来自 base_link <- IMU; 避免 cloud timer 早于 pose timer 时用默认值.
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        scan_ground_z_ = -engine_.baseImu().t.z();
    }

    // 旧实现: 构造里按 slam_init_type_ 定初始 slam_mode_ (legacy.cpp:57-83)
    const std::string& init = cfg_.slam_init_type;
    SlamMode m = SlamMode::FREE;
    if (init == "mapping")           m = SlamMode::MAPPING;
    else if (init == "localization") m = SlamMode::LOCALIZATION;
    setModeFromLegacy(m);
    std::cout << "[legacy_iface] slam_init_type=" << init << " -> mode=" << modeName(m)
              << " debug_publish=" << (debug_publish_ ? 1 : 0)
              << " is_debug=" << (is_debug_ ? 1 : 0)
              << " use_tf=" << (use_tf_ ? 1 : 0) << std::endl;

    // 实时栅格 (T2): 数据源 = 核心的全局地图; 只在建图模式真的去算
    GridMapConfig gcfg;
    GridPostConfig gpost;
    std::string gen;
    std::string yaml = engine_.configPath();
    if (yaml.empty()) yaml = ros::package::getPath("rosiwit_slam_ros1") + "/config/default.yaml";
    loadGridConfigFromYaml(yaml, gcfg, gpost, gen);
    grid_resolution_ = gcfg.resolution;   // 存 pgm/yaml 时用同一个分辨率
    realtime_grid_.setConfig(gcfg, gpost, gen);
    // **实时 /map 走核心的 generateGridMap 路径** (逐帧观测 + 地面处理), 与存 pgm 完全一致。
    // 之前是自己拿合并后的全局点云 gen->build(), 没有地面处理 -> 实时图有地面噪点而 pgm 干净。
    realtime_grid_.setGridSource([this](rosiwit_slam::GridInfo& out) {
        if (slam_mode_ != SlamMode::MAPPING) return false;   // 非建图模式没必要算
        return engine_.getRealtimeGrid(grid_resolution_, out);
    });
    // 重算频率与发布频率一致 (旧实现是"发布前若变了就重算")
    realtime_grid_.start(static_cast<double>(cfg_.rt_grid_map_publish_hz > 0
                                                 ? cfg_.rt_grid_map_publish_hz
                                                 : 1));
}

void LegacyXz3dSlamInterface::initPub() {
    // legacy.cpp:282-297 逐字: 名字 / 类型 / queue / latch 都不改
    point_map_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
    grip_map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("global_grid_map", 1, true);
    realtime_grid_map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10);
    pointcloud_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("curr_scan_point", 10);
    ros1_pointcloud_puber_ = nh_.advertise<sensor_msgs::PointCloud>("scan_f_map", 10);
    submap_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("curr_submap_point", 10);
    or_pointcloud_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("or_curr_scan_point", 10);
    pose_puber_ = nh_.advertise<geometry_msgs::PoseStamped>("curr_localization_pose", 10);
    path_puber_ = nh_.advertise<nav_msgs::Path>("mapping_path", 10);
    loop_constraint_puber_ = nh_.advertise<visualization_msgs::MarkerArray>("loop_constraint", 10);
    slam_info_puber_ = nh_.advertise<xzrobot_ros_dependency::SlamInfo>("slam/info", 10);
    scan_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("points_2d", 10);
    warning_code_puber_ = nh_.advertise<std_msgs::String>("slam_warning_code", 10);
    point_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("origin_point", 10);
}

void LegacyXz3dSlamInterface::initTimerPub() {
    // legacy.cpp:254-255: 先按 publish_hz_ 算, 紧接着**硬编码覆盖成 0.1** —— publish_hz 无效 (M2)
    const double pub_time_gap = 0.1;
    double rt_hz = static_cast<double>(cfg_.rt_grid_map_publish_hz);
    if (!(rt_hz > 0.0)) rt_hz = 1.0;
    const double rt_gap = 1.0 / rt_hz;

    std::cout << "[legacy_iface] timers: pub=" << pub_time_gap << "s, pose=0.01s, rt_grid="
              << rt_gap << "s (rt_grid_map_publish_hz=" << cfg_.rt_grid_map_publish_hz
              << ", publish_hz=" << cfg_.publish_hz << " 无效-硬编码 0.1)" << std::endl;

    point_cloud_mapping_timer_ = nh_.createTimer(
        ros::Duration(pub_time_gap), &LegacyXz3dSlamInterface::timerPublishLocalCloudAndSubmap, this);
    localization_pose_timer_ = nh_.createTimer(
        ros::Duration(0.01), &LegacyXz3dSlamInterface::timerPublishLocalizationPose, this);
    realtime_grid_map_timer_ = nh_.createTimer(
        ros::Duration(rt_gap), &LegacyXz3dSlamInterface::timerPublishRealtimeGridMap, this);
    // 告警定时器与 rt_grid 同周期 (legacy.cpp:267)
    warning_code_timer_ = nh_.createTimer(
        ros::Duration(rt_gap), &LegacyXz3dSlamInterface::timerPublishWarningCode, this);
}

// ---------------------------------------------------------------- 模式

SlamMode LegacyXz3dSlamInterface::updateModeFromCore() {
    const std::string m = engine_.pipelineMode();
    if (m == "mapping")           slam_mode_ = SlamMode::MAPPING;
    else if (m == "localization") slam_mode_ = SlamMode::LOCALIZATION;
    else if (m == "idle")         slam_mode_ = SlamMode::FREE;
    else                          slam_mode_ = SlamMode::UNKNOWN;
    return slam_mode_;
}

void LegacyXz3dSlamInterface::applyModeToCore(SlamMode m) {
    auto* algo = engine_.algo();
    if (!algo) return;
    const char* core_mode = (m == SlamMode::MAPPING)      ? "mapping"
                          : (m == SlamMode::LOCALIZATION) ? "localization"
                                                          : "idle";
    if (!algo->setPipelineMode(core_mode)) {
        ROS_WARN_STREAM("[legacy_iface] core rejected pipeline mode '" << core_mode << "'");
    }
}

void LegacyXz3dSlamInterface::setModeFromLegacy(SlamMode m) {
    const bool changed = (slam_mode_ != m);
    slam_mode_ = m;
    applyModeToCore(m);
    if (changed) {
        debug_map_published_ = false;   // 旧实现每进模式重置
        relocating_ = false;
        std::cout << "[legacy_iface] mode -> " << modeName(m) << std::endl;
    }
}

void LegacyXz3dSlamInterface::computeLidarToBase() {
    // 核心 body 是 IMU; 地图系原点=初始 IMU. 地面 z = -T_base_link_imu.t.z()(base 在地面时).
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        scan_ground_z_ = -engine_.baseImu().t.z();
    }
}

void LegacyXz3dSlamInterface::resetCoreAndCaches() {
    // **默认开启** (与旧实现语义一致): 旧 xz_3d_slam 每次切建图都 DELETE_PTR(mapping_ptr_)
    // 再 new, 所以上一轮绝不残留。我们单实例, 只能显式重建内部模块达到同样效果。
    //
    // 为什么必须清: 不清的话新一帧点云会**在上一轮的旧图上做配准**, 旧图(已漂过的那一层)
    // 与新图叠在一起, 表现为"进建图先看到残留, 跑一会儿两层越错越开 → 地图全糊"。
    // 实机已复现: 日志里出现 skip core rebuild 后, 地图 occupied/total 长时间完全不变。
    //
    // 历史说明: 早期曾因"重建后地图不再累积"而默认关闭, 但后续实测发现**关闭时同样不累积**,
    // 该观察不可靠; 现按旧实现语义恢复默认开启, 并保留开关便于 A/B:
    //   _reset_on_mode_switch:=false 可退回不清图的行为。
    if (!reset_on_mode_switch_) {
        std::cout << "[legacy_iface] reset_on_mode_switch=false -> 保留上一轮地图 (会有残留)"
                  << std::endl;
        return;
    }
    std::cout << "[legacy_iface] reset_on_mode_switch=true -> rebuilding core (清空上一轮地图)"
              << std::endl;
    engine_.resetCore();   // 注意: 这里必须是 engine_.resetCore(), 不是本函数自己
    // 本层缓存一并复位 (见头文件注释)
    realtime_grid_.clear();
    realtime_grid_map_ = nav_msgs::OccupancyGrid();
    realtime_grid_map_area_ = 0.0;
    curr_pose_ = LegacyPose();
    last_pose_stamp_ = 0.0;
    mapping_path_msg_ = nav_msgs::Path();
    debug_map_published_ = false;
    std::cout << "[legacy_iface] core rebuilt + local caches cleared" << std::endl;
}

void LegacyXz3dSlamInterface::resetLocalizationLatch() {    relocating_ = true;
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
}

// ---------------------------------------------------------------- [T6] 输入

void LegacyXz3dSlamInterface::setInputSeen(bool imu, bool laser) {
    if (imu) have_imu_ = true;
    if (laser) have_laser_ = true;
}

// ---------------------------------------------------------------- [T6] 订阅

void LegacyXz3dSlamInterface::initSub() {
    // 旧 startSub (legacy.cpp:193-215) 一共 7 个订阅:
    //   imu + lidar (由 laser_type 分两个分支选话题)  -> 已在 SlamEngine 里按 laser_type 复刻
    //   initialpose                                  -> 引擎已经订阅 (行为一致), 不重复
    //   scan / odom / lift/state / lift_type         -> 这里补上
    // 所以可观测的订阅集合与旧实现一致, 只是前 3 个的句柄在引擎里。
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
        "scan", 10, &LegacyXz3dSlamInterface::laserScanCB, this);
    chassis_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "odom", 100, &LegacyXz3dSlamInterface::chassisOdomCB, this);
    lift_state_sub_ = nh_.subscribe<std_msgs::String>(
        "lift/state", 10, &LegacyXz3dSlamInterface::liftStateCB, this);
    lift_type_sub_ = nh_.subscribe<std_msgs::String>(
        "lift_type", 10, &LegacyXz3dSlamInterface::liftTypeCB, this);

    // 引擎的输入钩子: 复刻旧的 get_imu_data_/get_laser_data_ 门控 + origin_point 数据源
    engine_.setInputSeenHook([this](bool is_lidar) {
        if (is_lidar) have_laser_ = true; else have_imu_ = true;
        tickFreqWindow(is_lidar);   // [T7] 1s 频率窗口 -> 20003001/20003002
    });
    engine_.setRawLidarHook([this](const CloudType& c, const ros::Time& t) {
        onRawInputCloud(c, t);
    });
    std::cout << "[legacy_iface] subscribed: scan, odom, lift/state, lift_type "
              << "(imu/lidar/initialpose 由 SlamEngine 订阅)" << std::endl;
}
void LegacyXz3dSlamInterface::initClients() {
    tag_detect_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::TagDetect>("get_charging_point");
    set_detect_mode_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::CommonIntRequestService>("set_detect_mode");
    switch_map_pro_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::SwitchMap>("/switch_map_pro");
}
void LegacyXz3dSlamInterface::initServer() {
    // legacy.cpp:235-242 逐字 (注意 update_map_operation 没有前导 '/', 解析到全局同名)
    srv_mode_switch_ = nh_.advertiseService("/slam/mode_switch",
        &LegacyXz3dSlamInterface::handleModeSwitch, this);
    srv_switch_map_ = nh_.advertiseService("/switch_map",
        &LegacyXz3dSlamInterface::handleSwitchMap, this);
    srv_relocation_ = nh_.advertiseService("/relocation",
        &LegacyXz3dSlamInterface::handleRelocation, this);
    srv_stop_map_ = nh_.advertiseService("/stop_map_operation",
        &LegacyXz3dSlamInterface::handleStopMap, this);
    srv_global_loc_ = nh_.advertiseService("/global_localization",
        &LegacyXz3dSlamInterface::handleGlobalLocalization, this);
    srv_dump_recorder_ = nh_.advertiseService("/slam/dump_recorder",
        &LegacyXz3dSlamInterface::handleDumpRecorder, this);
    srv_finish_task_ = nh_.advertiseService("/finish_task",
        &LegacyXz3dSlamInterface::handleFinishTask, this);
    srv_start_task_ = nh_.advertiseService("/start_task",
        &LegacyXz3dSlamInterface::handleStartTask, this);
    srv_expand_map_ = nh_.advertiseService("update_map_operation",
        &LegacyXz3dSlamInterface::handleExpandMap, this);
}

} // namespace rosiwit_slam_ros1
