// ============================================================================
// 旧 xz_3d_slam 接口 —— 单文件实现
//
// 原按关注点拆成 assembly/cloud/pose/warning/services 五个 .cpp，
// 但 ROS2 薄层是单文件可维护结构；这里合并回一个实现文件，按 SECTION 分区，
// 仍由 xz3dslam_interface.h 的单类声明驱动。
// ============================================================================

// ============================================================================
// SECTION: assembly.cpp
// ============================================================================
// ============================================================================
// legacy 装配层: 构造/发布与订阅注册/定时器/服务注册/模式机 —— 从 xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "ros1_slam_interface.h"

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
                                                 SlamEngine& engine)
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

Ros1SlamInterface::~Ros1SlamInterface() {
    // 后台切图任务若还在解析地图，等它结束；避免析构后访问 engine/this。
    std::lock_guard<std::mutex> lk(map_switch_jobs_mtx_);
    for (auto& job : map_switch_jobs_) {
        if (!job.valid()) continue;
        try { job.get(); } catch (...) {}
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

    std::cout << "[ros1_slam] timers: pub=" << pub_time_gap << "s, pose=0.01s, rt_grid="
              << rt_gap << "s (rt_grid_map_publish_hz=" << cfg_.rt_grid_map_publish_hz
              << ", publish_hz=" << cfg_.publish_hz << " 无效-硬编码 0.1)" << std::endl;

    point_cloud_mapping_timer_ = nh_.createTimer(
        ros::Duration(pub_time_gap), &Ros1SlamInterface::timerPublishLocalCloudAndSubmap, this);
    localization_pose_timer_ = nh_.createTimer(
        ros::Duration(0.01), &Ros1SlamInterface::timerPublishLocalizationPose, this);
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
            curr_map_name_ = "";
            curr_map_path_.clear();
            realtime_grid_map_area_ = 0.0;
            static_map_valid_ = false;
            curr_pose_valid_ = false;
            realtime_grid_map_ = nav_msgs::OccupancyGrid();
            last_static_map_pub_ = ros::Time();
        }
    } else {
        slam_mode_ = SlamMode::UNKNOWN;
    }
    return slam_mode_;
}

void Ros1SlamInterface::applyModeToCore(SlamMode m) {
    auto* algo = engine_.algo();
    if (!algo) return;
    const char* core_mode = (m == SlamMode::MAPPING)      ? "mapping"
                          : (m == SlamMode::LOCALIZATION) ? "localization"
                                                          : "idle";
    if (!algo->setPipelineMode(core_mode)) {
        ROS_WARN_STREAM("[ros1_slam] core rejected pipeline mode '" << core_mode << "'");
    }
}

void Ros1SlamInterface::setModeFromConfig(SlamMode m) {
    const bool changed = (slam_mode_ != m);
    if (m != SlamMode::LOCALIZATION) static_map_valid_ = false;
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
    realtime_grid_map_ = nav_msgs::OccupancyGrid();
    realtime_grid_map_area_ = 0.0;
    curr_pose_ = Xz3dPose();
    curr_pose_valid_ = false;
    last_new_pose_ros_ = ros::Time();
    last_high_rate_pub_ = ros::Time();
    last_high_rate_tf_ = ros::Time();
    last_pose_stamp_ = 0.0;
    last_mapping_path_time_ = -1.0;
    last_mapping_path_pub_ = ros::Time();
    last_location_info_pub_ = ros::Time();
    mapping_path_msg_ = nav_msgs::Path();
    debug_map_published_ = false;
    std::cout << "[ros1_slam] core rebuilt + local caches cleared" << std::endl;
}

void Ros1SlamInterface::resetLocalizationLatch() {    relocating_ = true;
    lost_monitor_.reset();
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
    //   imu + lidar (由 laser_type 分两个分支选话题)  -> 已在 SlamEngine 里按 laser_type 复刻
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
              << "(imu/lidar/initialpose 由 SlamEngine 订阅)" << std::endl;
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
}

} // namespace rosiwit_slam_ros1

// ============================================================================
// SECTION: cloud_publisher.cpp
// ============================================================================
// ============================================================================
// legacy 点云与栅格发布侧: 输入回调 + 定时发布 + 实时栅格 —— 从 xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "ros1_slam_interface.h"

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
void Ros1SlamInterface::laserScanCB(const sensor_msgs::LaserScan::ConstPtr&) {
    // 旧实现只用它做雷达频率统计 (legacy.cpp:209 -> laserScanCallBack)
    const ros::Time now = ros::Time::now();
    if (scan_window_start_.isZero()) { scan_window_start_ = now; scan_count_ = 0; return; }
    ++scan_count_;
    const double dt = (now - scan_window_start_).toSec();
    if (dt >= 1.0) {
        scan_hz_ = scan_count_ / dt;
        scan_window_start_ = now;
        scan_count_ = 0;
        std::cout << "[ros1_slam] /scan freq = " << scan_hz_ << " Hz" << std::endl;
    }
}

void Ros1SlamInterface::chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {
    // 旧实现无条件订阅并把它当运动先验喂给定位 (legacy.cpp:210 chassisOdomCallBack)
    engine_.onOdomMsg(msg);
    // 位姿-里程计一致性监测的输入 (同一定时器线程消费, 无锁)
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;
    last_odom_t_ = msg->header.stamp.toSec();
}

void Ros1SlamInterface::liftStateCB(const std_msgs::String::ConstPtr& msg) {
    lift_state_ = msg->data;   // 旧实现 localiza_ptr_->setCurrLiftState(lift_state); 核心无此接口
}

void Ros1SlamInterface::liftTypeCB(const std_msgs::String::ConstPtr& msg) {
    if (lift_type_ != msg->data) {
        lift_type_ = msg->data;
        std::cout << "[ros1_slam] lift_type = " << lift_type_ << std::endl;
    }
}

void Ros1SlamInterface::onRawInputCloud(const CloudType& raw_cloud, const ros::Time& stamp) {
    CloudType::Ptr snapshot(new CloudType(raw_cloud));
    {
        std::lock_guard<std::mutex> lk(raw_input_mtx_);
        raw_input_ = snapshot;
        raw_input_stamp_ = stamp;
    }
    // origin_point: 每帧雷达都发 (legacy.cpp:3169/3299/3427)。
    // frame_id 用 "map" 是照抄旧实现 (xz_slam_ros1_interface.cpp:3115) —— 点其实是未配准的输入点。
    if (!snapshot->empty() && point_puber_.getNumSubscribers() > 0) {
        point_puber_.publish(toPointCloud2(*snapshot, kMapFrame, stamp));
    }
}

// ---------------------------------------------------------------- 定时器

void Ros1SlamInterface::timerPublishLocalCloudAndSubmap(const ros::TimerEvent&) {
    startPub();   // legacy.cpp:834
}
void Ros1SlamInterface::timerPublishRealtimeGridMap(const ros::TimerEvent&) {
    if (slam_mode_ == SlamMode::MAPPING) publishRealtimeGridMap();   // legacy.cpp:836-838
}

// ---------------------------------------------------------------- 0.1s 分发

void Ros1SlamInterface::startPub() {
    updateModeFromCore();   // 核心是模式的唯一权威
    switch (slam_mode_) {
        case SlamMode::LOCALIZATION:
            publishCurrScan();
            if (debug_publish_) {
                if (!debug_map_published_) { publishPointMap(); debug_map_published_ = true; }
                // publishCurrSubMap();  // D2: 注册不发
            }
            break;

        case SlamMode::MAPPING:
            if (have_imu_ && have_laser_) {
                if (debug_publish_) {
                    if (!debug_map_published_) { publishPointMap(); debug_map_published_ = true; }
                    // publishCurrSubMap();  // D2
                }
                publishCurrScan();
                publishMappingPath();
                // publishLoopBetweenKF();  // D2
            }
            break;

        default:
            break;
    }
}

void Ros1SlamInterface::publishCurrScan() {
    SlamOutput out;
    if (!engine_.latestOutput(out) || !out.world_cloud || out.world_cloud->empty()) return;
    const CloudType& world = *out.world_cloud;   // 核心给的就是 map 系下的当前帧
    const ros::Time stamp = out.pose.time > 0 ? ros::Time(out.pose.time) : ros::Time::now();

    if (slam_mode_ == SlamMode::LOCALIZATION) {
        // scan_f_map: 抽稀到约 2000 + z∈(0.1,2) (legacy.cpp:982-1011 旧带 (0,2) 在
        // "地面=0" 的车体系下成立; 我们 z_ref 有 ~7cm 标定偏差, 下界抬 0.1 防地面泄漏)。
        // 先看订阅者：没有订阅时不做 shape/旧 PointCloud 转换。
        const bool need_scan = ros1_pointcloud_puber_.getNumSubscribers() > 0;
        const bool need_debug_cloud =
            debug_publish_ && pointcloud_puber_.getNumSubscribers() > 0;
        if (need_scan || need_debug_cloud) {
            CloudType::Ptr shaped = scanShape(world, kLocZMax, 1, scan_ground_z_, kScanZMin);
            if (!shaped->empty()) {
                if (need_scan)
                    ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
                if (need_debug_cloud)
                    pointcloud_puber_.publish(toPointCloud2(*shaped, kMapFrame, stamp));
            }
        }
        // or_curr_scan_point: 原始输入帧, 抽稀倍率 x2, **不过滤 z** (legacy.cpp:1026-1058)。
        // frame_id "map" 照抄旧实现 (xz_slam_ros1_interface.cpp:1053)。
        if (or_pointcloud_puber_.getNumSubscribers() > 0) {
            CloudType::Ptr raw_snapshot;
            ros::Time raw_stamp;
            {
                std::lock_guard<std::mutex> lk(raw_input_mtx_);
                raw_snapshot = raw_input_;
                raw_stamp = raw_input_stamp_;
            }
            if (raw_snapshot && !raw_snapshot->empty()) {
                CloudType::Ptr orc = scanShape(*raw_snapshot, -1.0, 2, scan_ground_z_);
                if (!orc->empty())
                    or_pointcloud_puber_.publish(toPointCloud2(*orc, kMapFrame, raw_stamp));
            }
        }
    } else if (slam_mode_ == SlamMode::MAPPING) {
        // scan_f_map: 抽稀到约 2000 + z∈(0.1,1) (legacy.cpp:1071-1096, 下界同理抬 0.1)
        if (pointcloud_puber_.getNumSubscribers() > 0) {
            pointcloud_puber_.publish(toPointCloud2(world, kMapFrame, stamp));
        }
        if (ros1_pointcloud_puber_.getNumSubscribers() > 0) {
            CloudType::Ptr shaped = scanShape(world, kMapZMax, 1, scan_ground_z_, kScanZMin);
            if (!shaped->empty())
                ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
        }
        // or_curr_scan_point: 建图**不发** (legacy.cpp:1102 那行是注释掉的)
    }
}

void Ros1SlamInterface::publishMappingPath() {
    if (!path_puber_.getNumSubscribers() || mapping_path_msg_.poses.empty()) return;
    mapping_path_msg_.header.frame_id = kMapFrame;
    // 整条 Path 重发非常贵 (O(轨迹长度))；10Hz 调用只保留 1Hz 实际发送。
    const ros::Time now = ros::Time::now();
    if (!last_mapping_path_pub_.isZero() &&
        (now - last_mapping_path_pub_).toSec() < 1.0) return;
    last_mapping_path_pub_ = now;
    path_puber_.publish(mapping_path_msg_);   // 整条重发 (legacy.cpp:885-887)
}

void Ros1SlamInterface::publishRealtimeGridMap() {
    GridInfo gi;
    if (realtime_grid_.latest(gi)) {
        realtime_grid_map_ = toRosGrid(gi, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(gi.width) * gi.height * gi.resolution * gi.resolution;
        // legacy.cpp:1749 把建图面积塞进 origin.position.z
        realtime_grid_map_.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        std::cout << "[ros1_slam] realtime grid updated: " << gi.width << "x" << gi.height
                  << " res=" << gi.resolution << " area=" << realtime_grid_map_area_ << std::endl;
    }
    // 无更新也照发上一张 (legacy.cpp:909 无条件 publish)
    realtime_grid_map_puber_.publish(realtime_grid_map_);
}

void Ros1SlamInterface::publishPointMap() {
    PointVec pts;
    if (engine_.getGlobalMap(pts) && !pts.empty()) {
        CloudType cloud;
        cloud.reserve(pts.size());
        for (auto& p : pts) cloud.push_back(p);
        cloud.width = cloud.size();
        cloud.height = 1;
        point_map_puber_.publish(toPointCloud2(cloud, kMapFrame, ros::Time::now()));
    }
    // global_grid_map: 由 set_grid_map_ 标志驱动, 发一次就清 (legacy.cpp:926-946)
    if (set_grid_map_) {
        grip_map_puber_.publish(global_grid_map_msg_);
        std::cout << "pub once grid map" << std::endl;
        set_grid_map_ = false;
    }
}

// ---------------------------------------------------------------- 0.01s 位姿 + slam/info

} // namespace rosiwit_slam_ros1

// ============================================================================
// SECTION: pose_publisher.cpp
// ============================================================================
// ============================================================================
// legacy 位姿与状态发布侧: curr_localization_pose + slam/info + TF —— 从 xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "ros1_slam_interface.h"

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
void Ros1SlamInterface::timerPublishLocalizationPose(const ros::TimerEvent&) {
    publishCurrPose();   // legacy.cpp:826-827
}
void Ros1SlamInterface::publishCurrPose() {
    updateModeFromCore();

    // 切图期间核心已 idle、旧输出快照已清空；这里再兜底，任何残留的旧图输出都不发布。
    if (slam_mode_ == SlamMode::LOCALIZATION && map_switch_busy_.load()) return;

    SlamOutput out;
    const bool have_out = engine_.latestOutput(out) && out.has_new_pose;

    // [PubProbe] 发布心跳: 每 100 tick 打一行, 说明"定时器在跑 / 有没有新位姿"。
    // 用于区分"输出链路被锁住"与"核心根本没产出新位姿"(这两种现象的界面表现一样)。
    {
        static int pub_n = 0;
        if (++pub_n % 1000 == 0) {   // 10s 一条心跳
            std::cout << "[PubProbe] tick=" << pub_n << " mode=" << modeName(slam_mode_)
                      << " have_new_pose=" << (have_out ? 1 : 0)
                      << " pose_time=" << out.pose.time << std::endl;
        }
    }

    // **核心估的是"雷达/IMU 刚体"的位姿**, 而这辆车的雷达偏航 90°、俯装 41.67°、抬高 0.9m。
    // 直接当车身位姿发出去 -> 朝向差 90° 且带 42° 俯仰 (实测就是这个现象)。
    // 这里用 TF 里的安装变换换到车身 (base_link) 系:
    //   T_world_base = T_world_lidar * T_lidar_base,  T_lidar_base = inv(T_base_lidar)
    //   R_base = R_lidar * R_lb^T ;  t_base = t_lidar - R_base * t_lb
    if (have_out) {
        computeLidarToBase();
        // 行业标准: 核心输出 IMU body; 对外位姿转 base_link.
        if (engine_.computeBaseFrames(/*blocking=*/false) && engine_.baseImu().valid) {
            out.pose = composeRightInv(out.pose, engine_.baseImu());
        }
    }

    bool publish_info = true;

    if (slam_mode_ == SlamMode::LOCALIZATION) {
        // 旧定位分支有"同帧去重", 重复帧直接 return —— 连 slam/info 都不发 (legacy.cpp:1121-1125)
        publish_info = false;
        if (have_out) {
            const double t = out.pose.time;
            const bool dup = (t - last_pose_stamp_ < 1e-4);
            last_pose_stamp_ = t;
            if (!dup) {
                const double yaw = stdYawFromRot(out.pose.rot);
                if (!std::isfinite(out.pose.trans.x()) || !std::isfinite(out.pose.trans.y()) ||
                    !std::isfinite(yaw)) {
                    std::cout << "XZSlamInterFace::publishCurrPose(), curr_pose has NaN, drop this frame"
                              << std::endl;
                    return;   // legacy.cpp:1127-1134 (连 slam/info 一起跳过)
                }
                curr_pose_.x = out.pose.trans.x();
                curr_pose_.y = out.pose.trans.y();
                curr_pose_.z = out.pose.trans.z();
                curr_pose_.yaw = yaw;
                curr_pose_.q = Eigen::Quaterniond(out.pose.rot);
                curr_pose_.q.normalize();

                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = kMapFrame;
                ps.header.stamp = (t > 0) ? ros::Time(t) : ros::Time::now();
                ps.pose.position.x = curr_pose_.x;
                ps.pose.position.y = curr_pose_.y;
                ps.pose.position.z = curr_pose_.z;
                ps.pose.orientation.x = curr_pose_.q.x();
                ps.pose.orientation.y = curr_pose_.q.y();
                ps.pose.orientation.z = curr_pose_.q.z();
                ps.pose.orientation.w = curr_pose_.q.w();
                pose_puber_.publish(ps);
                // 核心在 LOST 时也会输出最后有效位姿用于界面保持位置，
                // 但不能把 loose output 当成“定位成功”。
                if (out.state != rosiwit_slam::SlamState::LOST) {
                    curr_pose_valid_ = true;
                    last_new_pose_ros_ = ros::Time::now();
                    lost_monitor_.onLocalizedPose();
                }

                // confidently-wrong 防线 (2026-10-17 推行丢失事故): 匹配分 0.98 但
                // 位姿锁错时状态机无感; 激光位移 vs 轮速位移长期背离是唯一信号。
                if (last_odom_t_ > 0.0) {
                    odom_mon_.onPose(t, curr_pose_.x, curr_pose_.y, last_odom_x_, last_odom_y_);
                    const bool wrong = odom_mon_.wrongness();
                    if (wrong && !odom_mon_wrong_prev_) {
                        addWarningCode(20003505);
                        std::cout << "[ros1_slam] pose-odom DEVIATION "
                                  << odom_mon_.deviation() << "m -> warning 20003505"
                                  << std::endl;
                    }
                    odom_mon_wrong_prev_ = wrong;
                }
                publish_info = true;
            }
        }
    } else if (slam_mode_ == SlamMode::MAPPING) {
        // 建图分支没有去重, 每 tick 都发 slam/info (legacy.cpp:1170-1205)
        if (have_out) {
            last_pose_stamp_ = out.pose.time;
            curr_pose_.x = out.pose.trans.x();
            curr_pose_.y = out.pose.trans.y();
            curr_pose_.z = out.pose.trans.z();
            curr_pose_.yaw = stdYawFromRot(out.pose.rot);
            curr_pose_.q = Eigen::Quaterniond(out.pose.rot);
            curr_pose_.q.normalize();

            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = kMapFrame;
            ps.header.stamp = (out.pose.time > 0) ? ros::Time(out.pose.time) : ros::Time::now();
            ps.pose.position.x = curr_pose_.x;
            ps.pose.position.y = curr_pose_.y;
            ps.pose.position.z = curr_pose_.z;
            ps.pose.orientation.x = curr_pose_.q.x();
            ps.pose.orientation.y = curr_pose_.q.y();
            ps.pose.orientation.z = curr_pose_.q.z();
            ps.pose.orientation.w = curr_pose_.q.w();
            pose_puber_.publish(ps);
            // 轨迹去重: publishCurrPose 是 100Hz，核心位姿通常 10Hz；
            // 同一 timestamp 不重复入队，否则 Path 会以 5~10 倍速度膨胀。
            if (out.pose.time > last_mapping_path_time_ + 1e-6) {
                last_mapping_path_time_ = out.pose.time;
                mapping_path_msg_.poses.push_back(ps);
                if (mapping_path_msg_.poses.size() > kMaxMappingPathPoses) {
                    const size_t drop = kMaxMappingPathPoses / 10;
                    mapping_path_msg_.poses.erase(mapping_path_msg_.poses.begin(),
                                                  mapping_path_msg_.poses.begin() + drop);
                }
            }
        }
    }

    if (publish_info) {
        // /slam/info 最高 10Hz：FREE/建图分支原来会 100Hz 发同一个状态，网络和
        // 序列化开销都白付；定位有新位姿时本来约 10Hz，不受影响。
        const ros::Time now = ros::Time::now();
        if (last_location_info_pub_.isZero() ||
            (now - last_location_info_pub_).toSec() >= 0.1) {
            publishLocationInfo();
            last_location_info_pub_ = now;
        }
    }
    // 建图时 TF 跟随 100Hz 位姿定时器；定位时由下面高频块统一限到 20Hz，
    // 避免每次 10ms tick 都做一次较贵的 base_footprint<-odom 查询。
    if (slam_mode_ == SlamMode::MAPPING && use_tf_) sendTransform();

    // 定位 50Hz 高频输出: 核心匹配约 10Hz，底盘/导航若需要更平滑的
    // /curr_localization_pose 与 map->odom，在两次 10Hz 更新之间重发
    // 最近一次位姿（最多 0.3s，超时后交给 LostMonitor 判定）。
    if (slam_mode_ == SlamMode::LOCALIZATION && curr_pose_valid_) {
        const ros::Time now = ros::Time::now();
        const bool fresh = !last_new_pose_ros_.isZero() &&
                           (now - last_new_pose_ros_).toSec() <= 0.30;
        const bool due = last_high_rate_pub_.isZero() ||
                         (now - last_high_rate_pub_).toSec() >= 0.02;
        if (fresh && due) {
            if (!publish_info) {
                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = kMapFrame;
                ps.header.stamp = now;
                ps.pose.position.x = curr_pose_.x;
                ps.pose.position.y = curr_pose_.y;
                ps.pose.position.z = curr_pose_.z;
                ps.pose.orientation.x = curr_pose_.q.x();
                ps.pose.orientation.y = curr_pose_.q.y();
                ps.pose.orientation.z = curr_pose_.q.z();
                ps.pose.orientation.w = curr_pose_.q.w();
                pose_puber_.publish(ps);
                // TF 的 lookupTransform 较贵；高频位姿 50Hz，TF 单独限到 20Hz。
                if (use_tf_ && (last_high_rate_tf_.isZero() ||
                                (now - last_high_rate_tf_).toSec() >= 0.05)) {
                    sendTransform();
                    last_high_rate_tf_ = now;
                }
            } else if (use_tf_ && (last_high_rate_tf_.isZero() ||
                                   (now - last_high_rate_tf_).toSec() >= 0.05)) {
                sendTransform();
                last_high_rate_tf_ = now;
            }
            last_high_rate_pub_ = now;
        }
    }

    // 兼容兜底: 部分实机上 realtime_grid_map_timer_ 在切定位后不再触发
    // (定时器本身无异常，表现是 /map 一直停在切换瞬间)。用 100Hz 的
    // 位姿定时器按 1Hz 重发已加载的 PGM，保证 App/RViz 能拿到灰色未知区。
    if (slam_mode_ == SlamMode::LOCALIZATION && static_map_valid_ &&
        !realtime_grid_map_.data.empty()) {
        const ros::Time now = ros::Time::now();
        if (last_static_map_pub_.isZero() || (now - last_static_map_pub_).toSec() >= 1.0) {
            realtime_grid_map_puber_.publish(realtime_grid_map_);
            last_static_map_pub_ = now;
        }
    }
}

void Ros1SlamInterface::publishZeroPoseInfo() {
    curr_pose_ = Xz3dPose();
    last_location_info_pub_ = ros::Time::now();
    slam_info_puber_.publish(
        buildSlamInfo(/*slam_state=*/1, /*location_status=*/1, curr_map_name_,
                      0.0, 0.0, 0.0,
                      static_cast<float>(realtime_grid_map_area_), 0.0f));
}

void Ros1SlamInterface::publishLocationInfo() {
    Xz3dPose p = curr_pose_;
    // legacy.cpp:1826-1830: 任一 NaN 则三者归零
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw)) {
        p.x = 0.0; p.y = 0.0; p.yaw = 0.0;
    }

    int8_t slam_state = 2;
    int8_t location_status = 0;
    float map_area = 0.0f;
    // 定位匹配度: 旧实现在 LOCALIZATION 且**不在重定位中**时取 localiza.getFitness()
    // (legacy.cpp:1866, 在 status 的 if/else **之后**, 所以 status=2 时也会带上真值 ——
    //  实机实测正是 location_status=2 且 matching_degree=0.831)。其它模式留 0。
    float matching_degree = 0.0f;

    switch (slam_mode_) {
        case SlamMode::LOCALIZATION: {
            slam_state = 1;   // legacy.cpp:1842
            map_area = static_cast<float>(realtime_grid_map_area_);
            // 旧实现: relocating_ = isRelocalization(); 重定位中 -> 1,
            // 否则 getLocalizationStatus() ∈{0,1} -> 0, 其它 -> 2。
            // 核心不暴露 relocalization/status, 用 T3 的 LostMonitor 等价表达 0/1/2 (设计文档 §5.11 B)
            PoseStamped pose;
            Eigen::Matrix<double, 6, 6> cov;
            const bool cov_ok = engine_.getPoseCov(pose, cov);
            const LocStatus st = lost_monitor_.update(cov_ok);
            location_status = static_cast<int8_t>(st);
            relocating_ = (st == LocStatus::RECOVERING);
            if (!relocating_) {
                matching_degree = static_cast<float>(engine_.getMatchingDegree());
            }
            break;
        }
        case SlamMode::MAPPING:   // legacy.cpp:1873-1882
            slam_state = 0;
            location_status = 0;
            map_area = static_cast<float>(realtime_grid_map_area_);
            break;
        case SlamMode::FREE:      // legacy.cpp:1883-1886
            slam_state = 1;
            location_status = 1;
            break;
        default:                  // legacy.cpp:1887-1889
            slam_state = 2;
            break;
    }
    // 最终覆盖 (legacy.cpp:1891-1894): 在 switch **之后**, 会改写 FREE/MAPPING 的结果
    if (relocating_) {
        slam_state = 1;
        location_status = 1;
    }

    // [T11] 丢失边沿 -> 20003504 (pose lost, level=FAULT, 取自 conf/warning_code.json)。
    // 与 update() 同一 tick 消费(同线程), 所以真实的丢失不会被漏报;
    // 被作废的只是"丢失后又立刻恢复、发布方还没看过"的瞬时抖动 (T3 的设计)。
    // 核心不暴露告警码通道, 旧系统的 20003504 是定位模块自己 push 的
    // (nano_gicp_localization.cpp:9403) —— 这里是我们**补上**。
    if (lost_monitor_.consumeLostEdge()) {
        addWarningCode(20003504);
        std::cout << "[ros1_slam] localization LOST -> warning 20003504 (pose lost)" << std::endl;
    }

    slam_info_puber_.publish(
        buildSlamInfo(slam_state, location_status, curr_map_name_, p.x, p.y, p.yaw, map_area,
                      matching_degree));
}

// ---------------------------------------------------------------- [T7] 告警
void Ros1SlamInterface::sendTransform() {
    // legacy.cpp:1899-1936 逐行:
    //   Tfo = lookupTransform("base_footprint", "odom", Time(0))     = T(base_footprint <- odom)
    //   Tmf = (origin=(x, y, -0.07), rot=AngleAxis(-yaw, Z))         = T(map <- base_footprint)
    //        其中 yaw = atan2(R(0,1), R(0,0))  (即 xzYawFromRot)
    //   Tmo = Tmf * Tfo                                             = T(map <- odom)
    //   broadcast(map -> odom), stamp = now
    geometry_msgs::TransformStamped tfo_msg;
    try {
        tfo_msg = tf_buffer_->lookupTransform("base_footprint", "odom", ros::Time(0));
    } catch (const tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    if (!std::isfinite(curr_pose_.x) || !std::isfinite(curr_pose_.y) ||
        !std::isfinite(curr_pose_.yaw)) {
        return;
    }

    // 用 Eigen 直接做刚体复合: T(a<-b) * T(b<-c) = T(a<-c), 与 tf2::Transform 的 A*B 同义。
    // 这样不必为一次乘法引入 tf2_geometry_msgs 依赖。
    // 注意这里必须用 xzYawFromRot(q)(= -标准航向) 再取负, 与旧实现成对出现 ——
    // 直接用 curr_pose_.yaw 会把符号弄反。
    const double yaw =
        xzYawFromRot(curr_pose_.q.normalized().toRotationMatrix());
    const Eigen::Matrix3d R_f =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d t_f(curr_pose_.x, curr_pose_.y, -0.07);

    const Eigen::Quaterniond q_o(tfo_msg.transform.rotation.w, tfo_msg.transform.rotation.x,
                                 tfo_msg.transform.rotation.y, tfo_msg.transform.rotation.z);
    const Eigen::Matrix3d R_o = q_o.toRotationMatrix();
    const Eigen::Vector3d t_o(tfo_msg.transform.translation.x, tfo_msg.transform.translation.y,
                              tfo_msg.transform.translation.z);

    const Eigen::Matrix3d R_mo = R_f * R_o;
    const Eigen::Vector3d t_mo = R_f * t_o + t_f;
    Eigen::Quaterniond q_mo(R_mo);
    q_mo.normalize();

    geometry_msgs::TransformStamped out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";
    out.child_frame_id = "odom";
    out.transform.translation.x = t_mo.x();
    out.transform.translation.y = t_mo.y();
    out.transform.translation.z = t_mo.z();
    out.transform.rotation.x = q_mo.x();
    out.transform.rotation.y = q_mo.y();
    out.transform.rotation.z = q_mo.z();
    out.transform.rotation.w = q_mo.w();
    tf_broadcaster_->sendTransform(out);
}


} // namespace rosiwit_slam_ros1

// ============================================================================
// SECTION: warning.cpp
// ============================================================================
// ============================================================================
// legacy 告警侧: 告警码表 + 频率窗口 + 告警发布定时器 —— 从 xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "ros1_slam_interface.h"

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
void Ros1SlamInterface::loadWarningCodeTable() {
    // M6: 从**本包 share**读, 不读 /opt —— 旧实现用的是编译期 PROJECT_PATH
    // (json_warning_code.cpp:168 的 project_path)。
    const std::string path =
        ros::package::getPath("rosiwit_slam_ros1") + "/conf/warning_code.json";
    std::ifstream fin(path);
    if (!fin.good()) {
        ROS_WARN_STREAM("[ros1_slam] warning_code.json not readable: " << path);
        return;
    }
    try {
        const YAML::Node n = YAML::LoadFile(path);   // JSON 是 YAML 的子集
        if (!n.IsMap()) {
            ROS_WARN_STREAM("[ros1_slam] warning_code.json is not a map: " << path);
            return;
        }
        for (auto it = n.begin(); it != n.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            int code = 0;
            try { code = std::stoi(key); } catch (...) { continue; }
            const YAML::Node v = it->second;
            WarningInfo wi;
            wi.level   = v["level"]   ? v["level"].as<std::string>()   : "FAULT";
            wi.name    = v["name"]    ? v["name"].as<std::string>()    : "";
            wi.content = v["content"] ? v["content"].as<std::string>() : "";
            warning_code_table_[code] = wi;
        }
        std::cout << "[ros1_slam] warning_code table loaded: " << warning_code_table_.size()
                  << " codes from " << path << std::endl;
        // 自定义码 (json 未定义, 需在 app 侧同步登记): 位姿-里程计长期背离
        {
            WarningInfo wi;
            wi.level = "WARNING";
            wi.name = "pose odom inconsistent";
            wi.content = "slam pose deviates from wheel odometry (possible wrong lock)";
            warning_code_table_[20003505] = wi;
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[ros1_slam] warning_code.json parse error (" << e.what()
                        << "): " << path);
    }
}

void Ros1SlamInterface::addWarningCode(int code) {
    std::lock_guard<std::mutex> lk(warning_code_mtx_);
    if (std::find(pending_warning_codes_.begin(), pending_warning_codes_.end(), code) ==
        pending_warning_codes_.end()) {
        pending_warning_codes_.push_back(code);
    }
}

void Ros1SlamInterface::setWarningSuppression(bool on) {
    warning_code_start_task_ = on;
    warning_code_suppress_ticks_ = 0;
}

void Ros1SlamInterface::tickFreqWindow(bool is_lidar) {
    // legacy.cpp:620-641 (imu) 与 740-751 (lidar): 各自一个 1s 窗口, 窗口结束时
    // 若计数 <= 阈值就报对应码。阈值: imu <= 160 -> 20003002, lidar <= 6 -> 20003001。
    // odom 的窗口只打印不报警 (legacy.cpp:2706-2718) -> 我们**不发** 20003003。
    // 注意这套只能在"消息有来但偏慢"时触发: 回调不触发就没有窗口结束。
    const ros::Time now = ros::Time::now();
    FreqWindow& w = is_lidar ? lidar_freq_ : imu_freq_;
    if (w.start.isZero()) w.start = now;
    ++w.count;
    const double dt = (now - w.start).toSec();
    if (dt < 1.0) return;

    const int count = w.count;
    w.count = 0;
    w.start = now;

    if (is_lidar) {
        std::cout << "laser_fre_count_ is : " << count << std::endl;
        if (count <= 6) addWarningCode(20003001);
    } else {
        std::cout << "imu_fre_count_ is : " << count << std::endl;
        if (count <= 160) addWarningCode(20003002);
    }
}

void Ros1SlamInterface::timerPublishWarningCode(const ros::TimerEvent&) {
    // 抑制窗口 (legacy.cpp:459-468): 只在 start_task 之后才生效
    if (warning_code_start_task_) {
        ++warning_code_suppress_ticks_;
        if (warning_code_suppress_ticks_ < 100) return;
        warning_code_start_task_ = false;
        warning_code_suppress_ticks_ = 0;
    }

    std::vector<int> codes;
    {
        std::lock_guard<std::mutex> lk(warning_code_mtx_);
        codes.swap(pending_warning_codes_);
    }
    if (codes.empty()) return;   // 没有码就不发 (legacy.cpp:543)

    std::vector<WarningFault> faults;
    faults.reserve(codes.size());
    for (int c : codes) {
        WarningFault f;
        auto it = warning_code_table_.find(c);
        if (it != warning_code_table_.end()) {
            f.code = std::to_string(c);
            f.level = it->second.level;
            f.name = it->second.name;
            f.content = it->second.content;
        } else {
            // 旧实现未定义码的兜底 (json_warning_code.cpp 的 else 分支):
            // name="not define warning code", level="FAULT", code=begin_code_+998
            f.code = std::to_string(20003001 + 998);
            f.level = "FAULT";
            f.name = "not define warning code";
            f.content = "xz_3d_slam not define this warning code! code is " + std::to_string(c);
        }
        faults.push_back(f);
    }
    const std::string json = buildWarningJson(faults);
    std_msgs::String msg;
    msg.data = json;
    warning_code_puber_.publish(msg);
    std::cout << "timerPublishWarningCode, string_warning_codes is : " << json << std::endl;
}

// ---------------------------------------------------------------- [T7] clients

} // namespace rosiwit_slam_ros1

// ============================================================================
// SECTION: services.cpp
// ============================================================================
// ============================================================================
// legacy 服务侧: 初始位姿 + 全部服务处理器 + 外部服务客户端 —— 从 xz3dslam_interface.cpp 按关注点拆出 (2026-10-18 薄层整合,
// 纯代码搬移, 行为与拆分前逐字一致; 分区方式对照 ros2_ws SlamNode)。
// ============================================================================
#include "ros1_slam_interface.h"

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
bool Ros1SlamInterface::callTagDetect(std::string& charging_pile_json) {
    charging_pile_json.clear();
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时 (100ms x 10)
        xzrobot_ros_dependency::TagDetect srv;
        if (tag_detect_client_.call(srv)) {
            charging_pile_json = srv.response.charging_pile_json;
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[ros1_slam] get_charging_point unavailable");
    return false;   // 旧行为: 失败给空 charging_pile_json (D14)
}

bool Ros1SlamInterface::callSetDetectMode(int mode) {
    for (int i = 0; i < 5; ++i) {    // 0.5s 超时 + 100ms 重试
        xzrobot_ros_dependency::CommonIntRequestService srv;
        srv.request.mode = static_cast<int8_t>(mode);
        if (set_detect_mode_client_.call(srv)) return srv.response.result;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[ros1_slam] set_detect_mode unavailable");
    return false;
}

bool Ros1SlamInterface::callSwitchMapPro(xzrobot_ros_dependency::SwitchMap::Request& req) {
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时
        xzrobot_ros_dependency::SwitchMap srv;
        srv.request = req;
        if (switch_map_pro_client_.call(srv)) return srv.response.success;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[ros1_slam] /switch_map_pro unavailable");
    return false;
}

// ---------------------------------------------------------------- [T8] 8 个 server
void Ros1SlamInterface::applyInitPose(
    const geometry_msgs::PoseWithCovarianceStamped& in, bool zero_if_insane) {
    // **不在这里重复实现**: /switch_map、/relocation 与 /initialpose 共用
    // SlamEngine::applyInitPose() —— insane 归零、z 归零、车身->雷达安装变换换算、setInitPose
    // 全部只有一份。这里只保留本路径的差异: z 强制归零。
    if (!engine_.algo()) return;
    engine_.applyInitPose(in, zero_if_insane, /*force_z_zero=*/true);
}

bool Ros1SlamInterface::handleModeSwitch(
    xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
    xzrobot_ros_dependency::SlamModeSwitchService::Response& res) {
    if (req.mode == 0) {
        // 旧 switchToLocalization (legacy.cpp:2060-2115): 先置 FREE, 再调 /set_detect_mode(0),
        // **要求成功** —— 失败就 result=0 + return false。
        // 旧实现同时 DELETE_PTR 掉了建图/定位实例 -> 我们在这里也重建核心(丢上一轮地图)。
        resetCoreAndCaches();
        setModeFromConfig(SlamMode::FREE);
        const bool ok = callSetDetectMode(0);
        res.result = ok ? 1 : 0;
        return ok;
    }
    if (req.mode == 1) {
        // 旧 switchToMapping (legacy.cpp:1974-2058): 也调 /set_detect_mode(1), 但
        // "if (!success) return false;" 那行在 2026 是**注释掉的** -> 结果不影响返回值;
        // 随后 run() -> warning_code_start_task_ = true (184), 即重新开启抑制窗口。
        // **关键**: 旧实现先 DELETE_PTR(mapping_ptr_) 再 new 一个全新实例, 所以退出建图再
        // 进建图**不会残留上一轮地图**; 我们单核心实例, 必须显式重建。
        resetCoreAndCaches();
        callSetDetectMode(1);
        setModeFromConfig(SlamMode::MAPPING);
        setWarningSuppression(true);
        res.result = 1;
        return true;
    }
    return false;   // 非法 mode: result 保持默认 0
}

bool Ros1SlamInterface::handleSwitchMap(
    xzrobot_ros_dependency::SwitchMap::Request& req,
    xzrobot_ros_dependency::SwitchMap::Response& res) {
    // 切图不限流：允许用户连续切换，最后一次请求胜出。
    // 后台加载旧图时若来了新请求，旧 job 完成后发现 generation 已过期，
    // 不再应用旧图的定位状态；新 job 继续加载新图。

    // 分支 1: 参数不合法
    if (req.map_dir.empty() || req.map_name.empty()) {
        res.success = false;
        return true;
    }
    // 分支 2: 同一张图且**核心确实在定位** -> 只重设初始位姿 + 转发 /switch_map_pro。
    // 不能只看 legacy 自己的 slam_mode_: set_slam_mode idle 后核心已 idle，
    // 但 timer 尚未刷新 slam_mode_ 时会误走这条分支，导致“切了但没定位输出”。
    const bool core_is_localization = (engine_.pipelineMode() == "localization");
    if (req.map_dir == curr_map_path_ && slam_mode_ == SlamMode::LOCALIZATION &&
        core_is_localization) {
        applyInitPose(req.initial_pose, true);
        // /switch_map_pro 由外部感知/地图管理节点提供；实测它可能阻塞数秒并
        // 返回 false，不能让它拖延/否决核心已经完成的同图切换。这里不再同步调用。
        res.success = true;
        return true;
    }
    // 分支 3: 换图
    // 先"停用并清空"上一张图：核心置 idle（不再产出旧图位姿/不再消费定位帧），
    // 清掉最近一次输出快照。随后后台加载新图，加载完成再切回 localization。
    // 这是行业常见的 deactivate -> swap -> activate，而不是继续让旧图输出再靠 10Hz 重发掩盖。
    if (engine_.algo()) {
        engine_.algo()->cancelMapLoad();
        engine_.algo()->setPipelineMode("idle");
    }
    engine_.clearLatestOutput();
    const uint64_t switch_gen = ++map_switch_generation_;
    relocating_ = true;
    map_switch_busy_.store(true);
    const bool detect_ok = callSetDetectMode(0);
    const std::string base = xzMapBase(req.map_dir);   // "/dir/map"

    // 1) 先在 ROS 层把老地图的 PGM 发布出去。App/RViz 依赖 /map，
    //    不能等核心解析完 25s 的 xzmap 才更新；PGM 的 127/128 会保持灰色未知区。
    rosiwit_slam::GridInfo static_grid;
    const bool pgm_ok = interface_internal::loadGridPgm(base, static_grid) &&
        static_grid.width > 0 && static_grid.height > 0;
    if (pgm_ok) {
        realtime_grid_map_ = toRosGrid(static_grid, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(static_grid.width) * static_grid.height *
            static_grid.resolution * static_grid.resolution;
        realtime_grid_map_.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        realtime_grid_map_puber_.publish(realtime_grid_map_);
        std::cout << "[ros1_slam] /switch_map: published PGM map "
                  << static_grid.width << "x" << static_grid.height
                  << " unknown=" << std::count(static_grid.data.begin(), static_grid.data.end(), -1)
                  << std::endl;
    }

    static_map_valid_ = pgm_ok;
    // 切图默认显示地图零点 (0,0,0)，不保留上一张图的位姿。
    // 核心已 idle + 输出快照已清空；这里同步清本层缓存，只发一次零点 slam/info，
    // 之后等新图加载/定位结果，不再周期性重发。
    curr_pose_ = Xz3dPose();
    curr_pose_valid_ = false;
    last_new_pose_ros_ = ros::Time();
    last_high_rate_pub_ = ros::Time();
    last_high_rate_tf_ = ros::Time();
    last_pose_stamp_ = 0.0;

    // 2) 服务立即返回，避免 App 5s 超时；完整定位地图在后台串行加载。
    res.success = detect_ok || pgm_ok;
    slam_mode_ = SlamMode::LOCALIZATION;
    curr_map_path_ = req.map_dir;
    curr_map_name_ = req.map_name;
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    lost_monitor_.setMapLoaded(true);
    setWarningSuppression(true);
    publishZeroPoseInfo();   // 立即发布零位姿，App 不必等核心加载完

    const geometry_msgs::PoseWithCovarianceStamped init_pose = req.initial_pose;
    {
        std::lock_guard<std::mutex> jobs_lk(map_switch_jobs_mtx_);
        // 回收已完成 job，避免连续切图时 future 列表无限增长。
        for (auto it = map_switch_jobs_.begin(); it != map_switch_jobs_.end();) {
            if (it->valid() && it->wait_for(std::chrono::seconds(0)) ==
                                   std::future_status::ready) {
                try { it->get(); } catch (...) {}
                it = map_switch_jobs_.erase(it);
            } else {
                ++it;
            }
        }
        map_switch_jobs_.push_back(std::async(std::launch::async,
            [this, base, init_pose, switch_gen]() {
                std::lock_guard<std::mutex> lk(map_switch_mtx_);
                map_switch_busy_.store(true);
                rosiwit_slam::MapData map_data;
                const bool loaded = map_adapter::loadMapData(base, map_data) &&
                                    engine_.algo() &&
                                    engine_.algo()->loadLocalizationMapData(map_data);
                if (switch_gen != map_switch_generation_.load()) {
                    std::cout << "[ros1_slam] /switch_map: discard stale map load: "
                              << base << std::endl;
                    map_switch_busy_.store(false);
                    return;
                }
                if (!loaded) {
                    ROS_ERROR_STREAM("[ros1_slam] /switch_map background loadMap failed: "
                                     << base);
                    relocating_ = false;
                    map_switch_busy_.store(false);
                    return;
                }
                engine_.algo()->setPipelineMode("localization");
                applyInitPose(init_pose, true);
                relocating_ = false;
                map_switch_busy_.store(false);
                std::cout << "[ros1_slam] /switch_map background localization map ready: "
                          << base << std::endl;
            }));
    }
    return true;
}

bool Ros1SlamInterface::handleRelocation(
    xzrobot_ros_dependency::Relocation::Request& req,
    xzrobot_ros_dependency::Relocation::Response& res) {
    // 旧 relocate (legacy.cpp:2394-2418): relocating_ 置真又立刻置假;
    // setInitPose 的 **z 强制 0**; success 恒 true。
    // 注意这里**没有** |x|,|y| > 1e4 归零 —— 那只在 /switch_map 里。
    relocating_ = true;
    applyInitPose(req.initial_pose, false);
    relocating_ = false;
    // [T11] 进入重定位 -> 清 latch (下次真的定位上之前报 1, 不报 0)
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    res.success = true;
    return true;
}

bool Ros1SlamInterface::handleStopMap(
    xzrobot_ros_dependency::StopMapOperation::Request& req,
    xzrobot_ros_dependency::StopMapOperation::Response& res) {
    // 旧 stopMap (legacy.cpp:2419-2516)
    const std::string folder = req.map_dir;
    struct stat st;
    const bool exists = (!folder.empty() && ::stat(folder.c_str(), &st) == 0 && S_ISDIR(st.st_mode));
    if (!exists) {
        if (::mkdir(folder.c_str(), 0755) != 0) {
            // 旧实现: 建目录失败 -> return false 且 success 留 false
            ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: cannot create dir: " << folder);
            return false;
        }
    }
    // 旧实现要求 mapping_ptr_ 非空
    if (slam_mode_ != SlamMode::MAPPING || !engine_.algo()) return false;

    // 旧: mapping_ptr_->saveMap(folderPath, "map") -> 等价于 saveMap("<dir>/map")
    const std::string base = xzMapBase(req.map_dir);
    rosiwit_slam::MapData map_data;
    if (!engine_.algo()->exportMapData(map_data) ||
        !map_adapter::saveMapData(map_data, base)) {
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: xzmap export failed: " << base);
    }

    // 同时补 map.pgm / map.yaml: 旧实现的地图目录里有这两个文件, 且旧
    // LocalizationSetGridMap() (legacy.cpp:373-455) 会 imread("<base>.pgm") 并读
    // "<base>.yaml" 的 resolution/origin/occupied_thresh。核心的 saveGridMap 写的正是
    // 这个格式 (grid_map_io.cpp:7-39: P5, 首行=最大 y; yaml 含 image/resolution/
    // origin/negate/occupied_thresh/free_thresh/mode)。
    if (!engine_.algo()->saveGridMap(base + ".pgm", base + ".yaml", grid_resolution_)) {
        // 与 saveMap 同理: 失败只报错, 不改 success (旧实现也没有这一步可对照)
        ROS_ERROR_STREAM("[ros1_slam] /stop_map_operation: saveGridMap failed: " << base);
    } else {
        std::cout << "[ros1_slam] saved grid map: " << base << ".pgm / " << base
                  << ".yaml (resolution=" << grid_resolution_ << ")" << std::endl;
        // 旧实现还会从同一张灰度图导出 <base>.jpg 与 <dir>/thumb.png (App 的缩略图)
        writeMapThumbnails(base);
    }

    std::string charging_json;
    if (!callTagDetect(charging_json)) {
        std::cout << "XZSlamInterFace::stopMap(), call get_charging_point failed" << std::endl;
        charging_json.clear();   // 旧实现失败时取默认构造的空串
    }

    res.current_pose.header.frame_id = kMapFrame;
    res.current_pose.pose.pose.position.x = curr_pose_.x;
    res.current_pose.pose.pose.position.y = curr_pose_.y;
    res.current_pose.pose.pose.position.z = curr_pose_.z;   // **真实 z**, 不是 -0.07
    res.current_pose.pose.pose.orientation.x = curr_pose_.q.x();
    res.current_pose.pose.pose.orientation.y = curr_pose_.q.y();
    res.current_pose.pose.pose.orientation.z = curr_pose_.q.z();
    res.current_pose.pose.pose.orientation.w = curr_pose_.q.w();
    res.charging_pile_json = charging_json;
    // map_resolution / map_origin / map_negate / *_thresh / error 全部留默认空值:
    // 旧代码里那几行是注释掉的 (legacy.cpp:2506-2515), 照旧留空。
    res.success = true;
    return true;
}

bool Ros1SlamInterface::handleGlobalLocalization(std_srvs::Empty::Request&,
                                                       std_srvs::Empty::Response&) {
    // 旧实现 legacy.cpp:2634 就一行 `return true` (真正干活的是老定位模块自己)。
    // 我们把它接成**按需触发一次全局重定位**: 核心侧置 LOST 并清冷却, 下一帧进入冷启动分支
    // (localization.pipeline.relocalizer=bbs 时即全局检索; =none 时退化为有界局部重锁)。
    if (!engine_.algo()) return false;
    engine_.requestRelocalization();
    lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
    relocating_ = true;
    std::cout << "[ros1_slam] /global_localization -> relocalization requested" << std::endl;
    return true;
}

bool Ros1SlamInterface::handleDumpRecorder(std_srvs::Trigger::Request&,
                                                 std_srvs::Trigger::Response& res) {
    // 飞行记录仪手动落盘 (诊断; LOST 翻转时已自动落盘, 这里给现场排查主动触发)
    const std::string path = engine_.dumpFlightRecorder("manual");
    res.success = !path.empty();
    res.message = path.empty() ? "no frames recorded" : path;
    return true;
}

bool Ros1SlamInterface::handleFinishTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 finishTask (legacy.cpp:2601-2609): 有定位实例 -> updateMap() + status=true;
    // 否则 return false 且 status 留 false。降级: 定位模式且有核心 -> true。
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool Ros1SlamInterface::handleStartTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 startTask (legacy.cpp:2591-2599): setUpdateMapFlag() + status=true
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool Ros1SlamInterface::handleExpandMap(
    xzrobot_ros_dependency::UpdateMap::Request& req,
    xzrobot_ros_dependency::UpdateMap::Response& res) {
    // **有意降级 (D5)**: 旧实现 (legacy.cpp:2538-2589) 是"重建建图实例 +
    // setIncreaseMapFlag() + loadCurrentMap(map_dir + \"/map.xzmap\") +
    // setInitPose(pos, rot * AngleAxis(angle,Z))", 核心没有"增量扩图后写回"的能力。
    ROS_WARN("[ros1_slam] update_map_operation: 降级实现 (D5) —— 用 loadMap + 切定位代替 "
             "旧实现的增量扩图写回; 行为不等价");
    const std::string base = xzMapBase(req.map_dir);
    bool ok = false;
    if (engine_.algo()) {
        ok = engine_.algo()->loadMap(base);
        if (ok) {
            engine_.algo()->setPipelineMode("localization");
            slam_mode_ = SlamMode::LOCALIZATION;
            applyInitPose(req.initial_pose, false);
            lost_monitor_.reset();
    odom_mon_.reset();
    odom_mon_wrong_prev_ = false;
            lost_monitor_.setMapLoaded(true);
            curr_map_path_ = req.map_dir;
        } else {
            ROS_ERROR_STREAM("[ros1_slam] update_map_operation: loadMap failed: " << base);
        }
    }
    res.status = ok;
    return true;
}

// ---------------------------------------------------------------- [T9] TF map->odom

} // namespace rosiwit_slam_ros1
