// ============================================================================
// assembly.cpp — 兼容层的"接线"
//
// 单一职责: 把发布者/订阅者/服务/定时器注册起来 (init*), 并在构造时完成装配。
// 运行期的模式状态机 (mode_state.cpp) 与具体发布实现 (pose/cloud/services) 都不在这里。
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
      debug_publish_(engine.debugPublish()),
      is_debug_(engine.isDebug()),
      use_tf_(engine.useTf()) {
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
    const std::string init = engine_.slamInitType();
    SlamMode m = SlamMode::FREE;
    if (init == "mapping")           m = SlamMode::MAPPING;
    else if (init == "localization") m = SlamMode::LOCALIZATION;
    setModeFromConfig(m);
    std::cout << "[ros1_slam] slam_init_type=" << init << " -> mode=" << modeName(m)
              << " debug_publish=" << (debug_publish_ ? 1 : 0)
              << " is_debug=" << (is_debug_ ? 1 : 0)
              << " use_tf=" << (use_tf_ ? 1 : 0) << std::endl;

    // 实时栅格 (T2): 数据源 = 核心的全局地图; 只在建图模式真的去算。
    // 栅格参数由核心持有, 这一层不再镜像一份 (S3)。本层只从 YAML 读两个
    // **接口自己的**参数: 存图分辨率 + 实时图分辨率 + 平面约束开关。
    std::string yaml = engine_.configPath();
    if (yaml.empty()) yaml = ros::package::getPath("rosiwit_slam_ros1") + "/config/default.yaml";
    try {
        const YAML::Node yc = YAML::LoadFile(yaml);
        const YAML::Node grid = (yc["mapping"] && yc["mapping"]["grid"])
                                    ? yc["mapping"]["grid"] : YAML::Node();
        if (grid && grid["resolution"]) {
            grid_resolution_ = grid["resolution"].as<double>();   // 存 pgm/yaml 用
        }
        if (grid && grid["realtime_resolution"]) {
            realtime_grid_resolution_ = grid["realtime_resolution"].as<double>();
        }
        // C2b: 2D 平面约束开关 (localization.planar_constraint.enabled).
        if (yc["localization"] && yc["localization"]["planar_constraint"]) {
            const YAML::Node pc = yc["localization"]["planar_constraint"];
            if (pc["enabled"]) planar_constraint_enabled_ = pc["enabled"].as<bool>();
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[ros1_slam] config parse failed: " << e.what());
    }
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
    realtime_grid_.start(static_cast<double>(engine_.rtGridPublishHz() > 0
                                                 ? engine_.rtGridPublishHz()
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
    // 在途计数已挪到 DoneFn, 一定会归零; 这里仍给上限并周期告警, 避免
    // 文件系统卡死时进程永远退不掉。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (map_load_inflight_ > 0) {
        if (map_load_cv_.wait_for(lk, std::chrono::seconds(5)) == std::cv_status::no_timeout) {
            continue;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            ROS_FATAL("[ros1_slam] map load still in flight after 120s, giving up wait "
                      "(inflight=%d)", map_load_inflight_);
            break;
        }
        ROS_WARN("[ros1_slam] still waiting for %d in-flight map load job(s)",
                 map_load_inflight_);
    }
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
    double rt_hz = static_cast<double>(engine_.rtGridPublishHz());
    if (!(rt_hz > 0.0)) rt_hz = 1.0;
    const double rt_gap = 1.0 / rt_hz;

    std::cout << "[ros1_slam] timers: pub=" << pub_time_gap << "s, pose=tick(20ms), rt_grid="
              << rt_gap << "s (rt_grid_map_publish_hz=" << engine_.rtGridPublishHz()
              << ", publish_hz=" << engine_.publishHz() << " 无效-硬编码 0.1)" << std::endl;

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







// ---------------------------------------------------------------- [T6] 输入


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
