// ============================================================================
// Ros1SlamInterface 装配层：构造、发布器/订阅器/服务注册、模式机、定时器。
// 行为契约仍以旧 xz_3d_slam 接口为准；算法全部委托 SlamAlgorithm。
// ============================================================================

#include "ros1_slam_interface.h"
#include "xzrobot_ros_dependency/LocalizationState.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "convert.h"
#include "internal.h"
#include "map_adapter.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {
Ros1SlamInterface::Ros1SlamInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh,
                                                 SlamAlgorithm& engine)
    : nh_(nh), pnh_(pnh), engine_(engine),
      cfg_(engine.xzConfig()),
      debug_publish_(engine.xzConfig().debug_publish),
      is_debug_(engine.xzConfig().is_debug),
      use_tf_(engine.xzConfig().use_tf) {
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
    setModeFromConfig(m);
    std::cout << "[ros1_slam] slam_init_type=" << init << " -> mode=" << modeName(m)
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
    grid_resolution_ = gcfg.resolution;   // 存 pgm/yaml 时用 mapping.grid.resolution
    try {
        const YAML::Node yc = YAML::LoadFile(yaml);
        if (yc["mapping"] && yc["mapping"]["grid"] &&
            yc["mapping"]["grid"]["realtime_resolution"]) {
            realtime_grid_resolution_ =
                yc["mapping"]["grid"]["realtime_resolution"].as<double>();
        }
        // C2b: 2D 平面约束开关 (localization.planar_constraint.enabled).
        if (yc["localization"] && yc["localization"]["planar_constraint"]) {
            const YAML::Node pc = yc["localization"]["planar_constraint"];
            if (pc["enabled"]) planar_constraint_enabled_ = pc["enabled"].as<bool>();
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[ros1_slam] config parse failed: " << e.what());
    }
    realtime_grid_.setConfig(gcfg, gpost, gen);
    // **实时 /map 走核心的 generateGridMap 路径** (逐帧观测 + 地面处理), 与存 pgm 完全一致。
    // 之前是自己拿合并后的全局点云 gen->build(), 没有地面处理 -> 实时图有地面噪点而 pgm 干净。
    realtime_grid_.setGridSource([this](rosiwit_slam::GridInfo& out) {
        if (slam_mode_ != SlamMode::MAPPING) return false;   // 非建图模式没必要算
        // 没人订阅 /map 时不做全量栅格重建: ray casting 是 O(点数) 的重活,
        // 0.05/0.3 反复重建都会挤占后端线程. 最终 0.05 图由 save_grid_map 服务生成.
        if (realtime_grid_map_puber_.getNumSubscribers() == 0) return false;
        return engine_.getRealtimeGrid(realtime_grid_resolution_, out);
    });
    // 重算频率与发布频率一致 (旧实现是"发布前若变了就重算")
    realtime_grid_.start(static_cast<double>(cfg_.rt_grid_map_publish_hz > 0
                                                 ? cfg_.rt_grid_map_publish_hz
                                                 : 1));
}

Ros1SlamInterface::~Ros1SlamInterface() {
    // 后台切图任务若还在解析地图，等它结束；避免析构后访问 engine/this。
    // 任务跑在执行器的 "disk_read" lane 上，必须在此处同步等待 (执行器由
    // 组装根持有, 关停顺序晚于本对象析构)。
    std::unique_lock<std::mutex> lk(map_load_mtx_);
    if (map_load_inflight_ > 0) {
        ROS_INFO("[ros1_slam] waiting for %d in-flight map load job(s) to finish",
                 map_load_inflight_);
    }
    map_load_cv_.wait(lk, [this] { return map_load_inflight_ == 0; });
}

void Ros1SlamInterface::initPub() {
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
    // LocalizationServer 契约: 一个话题带齐位姿/协方差/健康等级/匹配度/地图标识.
    localization_state_puber_ =
        nh_.advertise<xzrobot_ros_dependency::LocalizationState>("/localization/state", 10);
    // C2b 2D 约束开关: 由 simulation.yaml 的 localization.planar_constraint 控制.
    // (不改变默认行为; 打开前需要 seed_max_drift 收紧, 否则核心会假定位.)
    scan_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("points_2d", 10);
    warning_code_puber_ = nh_.advertise<std_msgs::String>("slam_warning_code", 10);
    point_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("origin_point", 10);
}

void Ros1SlamInterface::initTimerPub() {
    // legacy.cpp:254-255: 先按 publish_hz_ 算, 紧接着**硬编码覆盖成 0.1** —— publish_hz 无效 (M2)
    const double pub_time_gap = 0.1;
    double rt_hz = static_cast<double>(cfg_.rt_grid_map_publish_hz);
    if (!(rt_hz > 0.0)) rt_hz = 1.0;
    const double rt_gap = 1.0 / rt_hz;

    std::cout << "[ros1_slam] timers: pub=" << pub_time_gap << "s, pose=tick(20ms), rt_grid="
              << rt_gap << "s (rt_grid_map_publish_hz=" << cfg_.rt_grid_map_publish_hz
              << ", publish_hz=" << cfg_.publish_hz << " 无效-硬编码 0.1)" << std::endl;

    point_cloud_mapping_timer_ = nh_.createTimer(
        ros::Duration(pub_time_gap), &Ros1SlamInterface::timerPublishLocalCloudAndSubmap, this);
    // 位姿 / TF / slam/info / localization/state 改由 slam_tick 线程驱动 (§C3):
    // 主线程一旦被存图/切图/服务回调占住, 这些也照常发布。50Hz 足够 (核心输出按雷达帧)。
    engine_.addPublishHook([this] { publishCurrPose(); });
    realtime_grid_map_timer_ = nh_.createTimer(
        ros::Duration(rt_gap), &Ros1SlamInterface::timerPublishRealtimeGridMap, this);
    // 告警定时器与 rt_grid 同周期 (legacy.cpp:267)
    warning_code_timer_ = nh_.createTimer(
        ros::Duration(rt_gap), &Ros1SlamInterface::timerPublishWarningCode, this);
}

// ---------------------------------------------------------------- 模式

SlamMode Ros1SlamInterface::updateModeFromCore() {
    // 切图过渡：核心先被置 idle 清输出，本层仍保持 LOCALIZATION 语义，
    // 避免 timer 把 slam_mode_ 改成 FREE 后界面状态跳变/地图名被清。
    if (map_switch_busy_.load()) return slam_mode_;
    const std::string m = engine_.pipelineMode();
    if (m == "mapping")           slam_mode_ = SlamMode::MAPPING;
    else if (m == "localization") slam_mode_ = SlamMode::LOCALIZATION;
    else if (m == "idle") {
        slam_mode_ = SlamMode::FREE;
        // 切图过渡期间 core 会短暂 idle；此时不能清掉刚设置的地图状态。
        if (!map_switch_busy_.load() && !relocating_.load()) {
            setMapName(std::string());
            curr_map_path_.clear();
            realtime_grid_map_area_ = 0.0;
            static_map_valid_.store(false);
            curr_pose_valid_.store(false);
            clearStaticMap();
            last_static_map_pub_ = ros::Time();
        }
    } else {
        slam_mode_ = SlamMode::UNKNOWN;
    }
    return slam_mode_;
}

void Ros1SlamInterface::applyModeToCore(SlamMode m) {
    const char* core_mode = (m == SlamMode::MAPPING)      ? "mapping"
                          : (m == SlamMode::LOCALIZATION) ? "localization"
                                                          : "idle";
    // 改模式是结构操作: 走持锁入口, 拿不到 (正在装载地图/重建) 就明确失败
    if (!engine_.setCorePipelineMode(core_mode)) {
        ROS_WARN_STREAM("[ros1_slam] core rejected/deferred pipeline mode '" << core_mode
                        << "' (busy with map load?)");
    }
}

void Ros1SlamInterface::setModeFromConfig(SlamMode m) {
    const bool changed = (slam_mode_ != m);
    if (m != SlamMode::LOCALIZATION) static_map_valid_.store(false);
    slam_mode_ = m;
    applyModeToCore(m);
    if (changed) {
        debug_map_published_ = false;   // 旧实现每进模式重置
        relocating_ = false;
        std::cout << "[ros1_slam] mode -> " << modeName(m) << std::endl;
    }
}

void Ros1SlamInterface::computeLidarToBase() {
    // 核心 body 是 IMU; 地图系原点=初始 IMU. 地面 z = -T_base_link_imu.t.z()(base 在地面时).
    if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
        scan_ground_z_ = -engine_.baseImu().t.z();
    }
}

void Ros1SlamInterface::resetCoreAndCaches() {
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
        std::cout << "[ros1_slam] reset_on_mode_switch=false -> 保留上一轮地图 (会有残留)"
                  << std::endl;
        return;
    }
    std::cout << "[ros1_slam] reset_on_mode_switch=true -> rebuilding core (清空上一轮地图)"
              << std::endl;
    engine_.resetCore();   // 注意: 这里必须是 engine_.resetCore(), 不是本函数自己
    // 本层缓存一并复位 (见头文件注释)
    realtime_grid_.clear();
    clearStaticMap();
    realtime_grid_map_area_ = 0.0;
    curr_pose_ = Xz3dPose();
    curr_pose_valid_.store(false);
    last_new_pose_ros_ = ros::Time();
    last_high_rate_pub_ = ros::Time();
    last_high_rate_tf_ = ros::Time();
    last_pose_stamp_ = 0.0;
    last_mapping_path_time_ = -1.0;
    last_mapping_path_pub_ = ros::Time();
    last_location_info_pub_ = ros::Time();
    mapping_path_msg_ = nav_msgs::Path();
    last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
    core_ever_localized_ = false;
    debug_map_published_ = false;
    std::cout << "[ros1_slam] core rebuilt + local caches cleared" << std::endl;
}

void Ros1SlamInterface::resetLocalizationLatch() {
    relocating_ = true;
    last_core_loc_status_ = rosiwit_slam::ILocalization::INIT;
    core_ever_localized_ = false;
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
}

// ---------------------------------------------------------------- [T6] 输入

void Ros1SlamInterface::setInputSeen(bool imu, bool laser) {
    if (imu) have_imu_ = true;
    if (laser) have_laser_ = true;
}

// ---------------------------------------------------------------- [T6] 订阅

void Ros1SlamInterface::initSub() {
    // 旧 startSub (legacy.cpp:193-215) 一共 7 个订阅:
    //   imu + lidar (由 laser_type 分两个分支选话题)  -> 已在 SlamAlgorithm 里按 laser_type 复刻
    //   initialpose                                  -> 引擎已经订阅 (行为一致), 不重复
    //   scan / odom / lift/state / lift_type         -> 这里补上
    // 所以可观测的订阅集合与旧实现一致, 只是前 3 个的句柄在引擎里。
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
        "scan", 10, &Ros1SlamInterface::laserScanCB, this);
    chassis_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "odom", 100, &Ros1SlamInterface::chassisOdomCB, this);
    lift_state_sub_ = nh_.subscribe<std_msgs::String>(
        "lift/state", 10, &Ros1SlamInterface::liftStateCB, this);
    lift_type_sub_ = nh_.subscribe<std_msgs::String>(
        "lift_type", 10, &Ros1SlamInterface::liftTypeCB, this);

    // 引擎的输入钩子: 复刻旧的 get_imu_data_/get_laser_data_ 门控 + origin_point 数据源
    engine_.setInputSeenHook([this](bool is_lidar) {
        if (is_lidar) have_laser_ = true; else have_imu_ = true;
        tickFreqWindow(is_lidar);   // [T7] 1s 频率窗口 -> 20003001/20003002
    });
    engine_.setRawLidarHook([this](const CloudType& c, const ros::Time& t) {
        onRawInputCloud(c, t);
    });
    std::cout << "[ros1_slam] subscribed: scan, odom, lift/state, lift_type "
              << "(imu/lidar/initialpose 由 SlamAlgorithm 订阅)" << std::endl;
}
void Ros1SlamInterface::initClients() {
    tag_detect_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::TagDetect>("get_charging_point");
    set_detect_mode_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::CommonIntRequestService>("set_detect_mode");
    switch_map_pro_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::SwitchMap>("/switch_map_pro");
}
void Ros1SlamInterface::initServer() {
    // legacy.cpp:235-242 逐字 (注意 update_map_operation 没有前导 '/', 解析到全局同名)
    srv_mode_switch_ = nh_.advertiseService("/slam/mode_switch",
        &Ros1SlamInterface::handleModeSwitch, this);
    srv_switch_map_ = nh_.advertiseService("/switch_map",
        &Ros1SlamInterface::handleSwitchMap, this);
    srv_relocation_ = nh_.advertiseService("/relocation",
        &Ros1SlamInterface::handleRelocation, this);
    srv_stop_map_ = nh_.advertiseService("/stop_map_operation",
        &Ros1SlamInterface::handleStopMap, this);
    srv_global_loc_ = nh_.advertiseService("/global_localization",
        &Ros1SlamInterface::handleGlobalLocalization, this);
    srv_dump_recorder_ = nh_.advertiseService("/slam/dump_recorder",
        &Ros1SlamInterface::handleDumpRecorder, this);
    srv_finish_task_ = nh_.advertiseService("/finish_task",
        &Ros1SlamInterface::handleFinishTask, this);
    srv_start_task_ = nh_.advertiseService("/start_task",
        &Ros1SlamInterface::handleStartTask, this);
    srv_expand_map_ = nh_.advertiseService("update_map_operation",
        &Ros1SlamInterface::handleExpandMap, this);

    // ---- LocalizationServer 契约别名 ----
    // 与旧服务同处理器, 只是换到 /localization/ 命名空间并给出统一语义:
    //   relocalize       = 用给定初始位姿重定位 (Relocation)
    //   set_initial_pose = 同上, map_name 可为空 (当前图)
    //   switch_map       = 切图 (SwitchMap, 可带 initial_pose)
    srv_loc_relocalize_ = nh_.advertiseService("/localization/relocalize",
        &Ros1SlamInterface::handleRelocation, this);
    srv_loc_set_initial_pose_ = nh_.advertiseService("/localization/set_initial_pose",
        &Ros1SlamInterface::handleRelocation, this);
    srv_loc_switch_map_ = nh_.advertiseService("/localization/switch_map",
        &Ros1SlamInterface::handleSwitchMap, this);
}

} // namespace rosiwit_slam_ros1
