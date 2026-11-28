// ============================================================================
// assembly.cpp — 兼容层的"接线"
//
// 单一职责: 把发布者/订阅者/服务/定时器注册起来 (init*), 并在构造时完成装配。
// 运行期的模式状态机 (mode_state.cpp) 与具体发布实现 (pose/cloud/services) 都不在这里。
// ============================================================================
#include "ros1_slam_interface.h"
#include "xzrobot_ros_dependency/LocationInfo.h"

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
    : nh_(nh), pnh_(pnh), srv_nh_(nh), engine_(engine),
      debug_publish_(engine.debugPublish()),
      is_debug_(engine.isDebug()),
      use_tf_(engine.useTf()) {
    initPub();
    loadWarningCodeTable();   // 告警码表 (M6: 从本包 share 读, 不读 /opt)
    initTimerPub();
    initSub();
    initClients();
    // 服务走独立回调队列 (main.cpp 里单独起 spinner): 切图/切模式/存图的
    // 有界等待只阻塞服务线程, 不再让话题回调与定时器跟着停摆。
    srv_nh_.setCallbackQueue(&srv_queue_);
    initServer();
    // 管理命令线程 (载图/切模式/存图): 服务只投命令, 真正的执行在这里串行
    control_.start();
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
        // overlay 调试: 只显示 w >= anchor 的"已确认"变化结构 -> 阈值跟配置走,
        // 避免 convert.cpp 的上色刻度和 maintainer 的判据不一致.
        const YAML::Node mt = (yc["localization"] && yc["localization"]["maintainer"])
                                  ? yc["localization"]["maintainer"] : YAML::Node();
        if (mt && mt["persistency_anchor"]) {
            overlay_anchor_ = mt["persistency_anchor"].as<double>();
        }
        // /scan_f_map 高度带: 按 base_footprint 系过滤 (不是地图系常量地面), 可配置。
        // 2026-11-28 现场要求: 0.15 ~ 2.5 m, 坡道/俯仰时跟着车体走。
        if (yc["scan_f_map"]) {
            const YAML::Node sf = yc["scan_f_map"];
            if (sf["z_min"]) scan_f_z_min_ = sf["z_min"].as<double>();
            if (sf["z_max"]) scan_f_z_max_ = sf["z_max"].as<double>();
        }
        std::cout << "[ros1_slam] grid: save_res=" << grid_resolution_
                  << " realtime_res=" << realtime_grid_resolution_ << std::endl;
        std::cout << "[ros1_slam] scan_f_map: footprint z band = "
                  << scan_f_z_min_ << " ~ " << scan_f_z_max_ << " m" << std::endl;
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
    // 关停顺序:
    //   1) 控制线程: 关闸 (不再接命令) 并把正在跑的载图/存图做完
    //   2) 线程池任务: 等它们收尾 (任务体捕获 this)
    //   3) 之后才析构发布者/订阅者
    control_.stop();
    waitForAuxJobs();
}

void Ros1SlamInterface::initPub() {
    // legacy.cpp:282-297 逐字: 名字 / 类型 / queue / latch 都不改
    point_map_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("global_map", 1, true);
    grip_map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("global_grid_map", 1, true);
    overlay_map_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("slam/debug/overlay_map", 1, true);
    session_map_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("slam/debug/session_map", 1, true);
    loc_stats_puber_ = nh_.advertise<std_msgs::String>("/slam/debug/loc_stats", 10);
    // [偏离 legacy] 这里加 latching: 厂商原版是非 latched, 靠 1Hz 重发兜底,
    // 但 App/导航在两次重发之间启动会短暂"地图空白"。latching 对消费端无副作用。
    realtime_grid_map_puber_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    pointcloud_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("curr_scan_point", 10);
    ros1_pointcloud_puber_ = nh_.advertise<sensor_msgs::PointCloud>("scan_f_map", 10);
    submap_puber_ = nh_.advertise<sensor_msgs::PointCloud2>("curr_submap_point", 10);
    pose_puber_ = nh_.advertise<geometry_msgs::PoseStamped>("curr_localization_pose", 10);
    path_puber_ = nh_.advertise<nav_msgs::Path>("mapping_path", 10);
    slam_info_puber_ = nh_.advertise<xzrobot_ros_dependency::SlamInfo>("slam/info", 10);
    // 注: 不发布 /location_info —— 厂商节点不发它, App 只订 /slam/info。
    // [偏离 legacy] latched: 瞬时故障码不会因订阅端晚到而漏掉
    warning_code_puber_ = nh_.advertise<std_msgs::String>("slam_warning_code", 1, true);
}

void Ros1SlamInterface::initTimerPub() {
    // 厂商 legacy.cpp:254-255 先按 publish_hz_ 算, 紧接着硬编码覆盖成 0.1 —— 也就是说真机上
    // 这个参数**从来没生效**。为保持接口时序与真机一致, 这里同样固定 0.1s, 并把配置项删掉
    // (留着会让人以为能调, 实际不生效)。
    const double pub_time_gap = 0.1;
    double rt_hz = static_cast<double>(engine_.rtGridPublishHz());
    if (!(rt_hz > 0.0)) rt_hz = 1.0;
    const double rt_gap = 1.0 / rt_hz;

    std::cout << "[ros1_slam] timers: pub=" << pub_time_gap << "s, pose=tick(20ms), rt_grid="
              << rt_gap << "s (rt_grid_map_publish_hz=" << engine_.rtGridPublishHz()
              << ")" << std::endl;

    point_cloud_mapping_timer_ = nh_.createTimer(
        ros::Duration(pub_time_gap), &Ros1SlamInterface::timerPublishLocalCloudAndSubmap, this);
    // 位姿 / TF / slam/info / 告警码: 20ms 心跳定时器, 跑在 io 线程上。
    // 这些全是小消息 (µs 级序列化), io 线程上的重活已经全部搬进线程池,
    // 所以不会再出现"切图期间界面冻结"。
    heartbeat_timer_ = nh_.createTimer(
        ros::Duration(0.02), &Ros1SlamInterface::timerHeartbeat, this);
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

    // 引擎的输入钩子: 复刻旧的 get_imu_data_/get_laser_data_ 门控
    engine_.setInputSeenHook([this](bool is_lidar) {
        if (is_lidar) have_laser_ = true; else have_imu_ = true;
        tickFreqWindow(is_lidar);   // [T7] 1s 频率窗口 -> 20003001/20003002
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
    srv_mode_switch_ = srv_nh_.advertiseService("/slam/mode_switch",
        &Ros1SlamInterface::handleModeSwitch, this);
    srv_switch_map_ = srv_nh_.advertiseService("/switch_map",
        &Ros1SlamInterface::handleSwitchMap, this);
    srv_relocation_ = srv_nh_.advertiseService("/relocation",
        &Ros1SlamInterface::handleRelocation, this);
    srv_stop_map_ = srv_nh_.advertiseService("/stop_map_operation",
        &Ros1SlamInterface::handleStopMap, this);
    srv_global_loc_ = srv_nh_.advertiseService("/global_localization",
        &Ros1SlamInterface::handleGlobalLocalization, this);
    srv_dump_recorder_ = srv_nh_.advertiseService("/slam/dump_recorder",
        &Ros1SlamInterface::handleDumpRecorder, this);
    srv_finish_task_ = srv_nh_.advertiseService("/finish_task",
        &Ros1SlamInterface::handleFinishTask, this);
    srv_start_task_ = srv_nh_.advertiseService("/start_task",
        &Ros1SlamInterface::handleStartTask, this);
    srv_expand_map_ = srv_nh_.advertiseService("update_map_operation",
        &Ros1SlamInterface::handleExpandMap, this);

    // 注: 不再提供 /localization/* 别名服务 —— 真机 App 只用 /relocation 与
    // /switch_map, 多余别名属于「私加接口」(2026-11-20 审计后移除)。
}

} // namespace rosiwit_slam_ros1
