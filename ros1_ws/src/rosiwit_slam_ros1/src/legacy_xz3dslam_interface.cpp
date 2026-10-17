// ============================================================================
// 旧 xz_3d_slam 发布侧实现 —— 逐条照抄 legacy.cpp, 行号见头文件。
//
// 生产环境下哪些话题其实是"哑"的 (重要, 别以为是 bug):
//   随包发行的 conf/slam_conf.json 里 `is_debug: false` 且**没有** debug_publish 键,
//   按旧实现 debug_publish_ 回落 is_debug (legacy.cpp:1470-1474) => **false**。
//   而 global_map / global_grid_map / curr_submap_point / loop_constraint 在旧实现里
//   就只在 debug_publish_ 为真时才发 (legacy.cpp:353-359 / 341-345),
//   points_2d 更是一次都没发过。也就是说这 5 个话题在真机上**本来就不出数据**,
//   我们照抄这个行为 —— 这才是"对上"。
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

using namespace rosiwit_slam;

namespace rosiwit_slam_ros1 {

namespace {
constexpr const char* kMapFrame = "map";
// 旧实现的 z 过滤上界: 定位 2.0 (legacy.cpp:989) / 建图 1.0 (legacy.cpp:1078)
constexpr double kLocZMax = 2.0;
constexpr double kMapZMax = 1.0;

const char* modeName(SlamMode m) {
    switch (m) {
        case SlamMode::FREE:         return "FREE";
        case SlamMode::MAPPING:      return "MAPPING";
        case SlamMode::LOCALIZATION: return "LOCALIZATION";
        default:                     return "UNKNOWN";
    }
}

/// 从我们刚写出的 `<base>.pgm` 生成 `<base>.jpg` 与 `<dir>/thumb.png`。
/// 照抄旧实现 (multilayer_grid_map.cpp:284-288): jpg 与 pgm 是**同一张图**,
/// thumb 是它的 0.2 倍缩放 (App 用来显示地图缩略图)。
/// 旧的那批 grid_map_fused_*/grid_map_of_layer_* 属于旧的"多层栅格"模块,
/// 现场已确认**不需要**, 故不生成。
void writeMapThumbnails(const std::string& base) {
    const std::string pgm = base + ".pgm";
    const cv::Mat img = cv::imread(pgm, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        ROS_WARN_STREAM("[legacy_iface] cannot read " << pgm << " -> skip jpg/thumb");
        return;
    }
    if (!cv::imwrite(base + ".jpg", img)) {
        ROS_WARN_STREAM("[legacy_iface] imwrite failed: " << base + ".jpg");
    }
    cv::Mat thumb;
    cv::resize(img, thumb, cv::Size(0, 0), 0.2, 0.2);
    const size_t slash = base.find_last_of('/');
    const std::string dir = (slash == std::string::npos) ? std::string(".") : base.substr(0, slash);
    const std::string tpath = dir + "/thumb.png";
    if (!cv::imwrite(tpath, thumb)) {
        ROS_WARN_STREAM("[legacy_iface] imwrite failed: " << tpath);
    }
    std::cout << "[legacy_iface] wrote " << base << ".jpg and " << tpath << " (thumb "
              << thumb.cols << "x" << thumb.rows << ")" << std::endl;
}
} // namespace

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
    realtime_grid_.setSource([this](PointVec& out) {
        if (slam_mode_ != SlamMode::MAPPING) return false;   // 非建图模式没必要算
        if (!engine_.getGlobalMap(out) || out.empty()) return false;
        return true;
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
    if (have_lidar_mount_) return;
    std::string lidar_frame = "lidar";
    pnh_.param<std::string>("lidar_frame", lidar_frame, lidar_frame);
    std::string base_frame = "base_link";
    pnh_.param<std::string>("base_frame", base_frame, base_frame);
    // 等最多 3 秒 (启动时 robot_state_publisher 的 /tf_static 可能还没到)
    for (int i = 0; i < 30; ++i) {
        try {
            const geometry_msgs::TransformStamped t =
                tf_buffer_->lookupTransform(base_frame, lidar_frame, ros::Time(0), ros::Duration(0.1));
            Eigen::Quaterniond q(t.transform.rotation.w, t.transform.rotation.x,
                                 t.transform.rotation.y, t.transform.rotation.z);
            q.normalize();
            r_lb_ = q.toRotationMatrix();                 // base <- lidar
            t_lb_ = Eigen::Vector3d(t.transform.translation.x, t.transform.translation.y,
                                    t.transform.translation.z);
            have_lidar_mount_ = true;
            char b[400];
            const Eigen::Vector3d rpy = r_lb_.eulerAngles(2, 1, 0);   // yaw,pitch,roll
            std::snprintf(b, sizeof(b),
                          "[legacy_iface] lidar mount from TF ('%s' <- '%s'): rpy(deg)=[%.2f %.2f %.2f] xyz=[%.3f %.3f %.3f]",
                          base_frame.c_str(), lidar_frame.c_str(),
                          rpy.z() * 180 / M_PI, rpy.y() * 180 / M_PI, rpy.x() * 180 / M_PI,
                          t_lb_.x(), t_lb_.y(), t_lb_.z());
            std::cout << b << std::endl;
            return;
        } catch (const tf2::TransformException&) {
            ros::Duration(0.05).sleep();
        }
    }
    ROS_WARN_STREAM("[legacy_iface] TF '" << base_frame << "' <- '" << lidar_frame
                    << "' unavailable; pose will be published in the LIDAR frame (heading may be off)");
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
}

void LegacyXz3dSlamInterface::setGlobalGridMap(const nav_msgs::OccupancyGrid& grid) {
    global_grid_map_msg_ = grid;
    set_grid_map_ = true;
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

void LegacyXz3dSlamInterface::laserScanCB(const sensor_msgs::LaserScan::ConstPtr&) {
    // 旧实现只用它做雷达频率统计 (legacy.cpp:209 -> laserScanCallBack)
    const ros::Time now = ros::Time::now();
    if (scan_window_start_.isZero()) { scan_window_start_ = now; scan_count_ = 0; return; }
    ++scan_count_;
    const double dt = (now - scan_window_start_).toSec();
    if (dt >= 1.0) {
        scan_hz_ = scan_count_ / dt;
        scan_window_start_ = now;
        scan_count_ = 0;
        std::cout << "[legacy_iface] /scan freq = " << scan_hz_ << " Hz" << std::endl;
    }
}

void LegacyXz3dSlamInterface::chassisOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {
    // 旧实现无条件订阅并把它当运动先验喂给定位 (legacy.cpp:210 chassisOdomCallBack)
    engine_.onOdomMsg(msg);
}

void LegacyXz3dSlamInterface::liftStateCB(const std_msgs::String::ConstPtr& msg) {
    lift_state_ = msg->data;   // 旧实现 localiza_ptr_->setCurrLiftState(lift_state); 核心无此接口
}

void LegacyXz3dSlamInterface::liftTypeCB(const std_msgs::String::ConstPtr& msg) {
    if (lift_type_ != msg->data) {
        lift_type_ = msg->data;
        std::cout << "[legacy_iface] lift_type = " << lift_type_ << std::endl;
    }
}

void LegacyXz3dSlamInterface::onRawInputCloud(const CloudType& raw_cloud, const ros::Time& stamp) {
    CloudType::Ptr snapshot(new CloudType(raw_cloud));
    {
        std::lock_guard<std::mutex> lk(raw_input_mtx_);
        raw_input_ = snapshot;
        raw_input_stamp_ = stamp;
    }
    // origin_point: 每帧雷达都发 (legacy.cpp:3169/3299/3427)。
    // frame_id 用 "map" 是照抄旧实现 (xz_slam_ros1_interface.cpp:3115) —— 点其实是未配准的输入点。
    if (!snapshot->empty()) {
        point_puber_.publish(toPointCloud2(*snapshot, kMapFrame, stamp));
    }
}

// ---------------------------------------------------------------- 定时器

void LegacyXz3dSlamInterface::timerPublishLocalCloudAndSubmap(const ros::TimerEvent&) {
    startPub();   // legacy.cpp:834
}

void LegacyXz3dSlamInterface::timerPublishLocalizationPose(const ros::TimerEvent&) {
    publishCurrPose();   // legacy.cpp:826-827
}

void LegacyXz3dSlamInterface::timerPublishRealtimeGridMap(const ros::TimerEvent&) {
    if (slam_mode_ == SlamMode::MAPPING) publishRealtimeGridMap();   // legacy.cpp:836-838
}

// ---------------------------------------------------------------- 0.1s 分发

void LegacyXz3dSlamInterface::startPub() {
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

void LegacyXz3dSlamInterface::publishCurrScan() {
    SlamOutput out;
    if (!engine_.latestOutput(out) || !out.world_cloud || out.world_cloud->empty()) return;
    const CloudType& world = *out.world_cloud;   // 核心给的就是 map 系下的当前帧
    const ros::Time stamp = out.pose.time > 0 ? ros::Time(out.pose.time) : ros::Time::now();

    if (slam_mode_ == SlamMode::LOCALIZATION) {
        // scan_f_map: 抽稀到约 2000 + z∈(0,2) (legacy.cpp:982-1011), **无条件**发
        CloudType::Ptr shaped = legacyScanShape(world, kLocZMax, 1);
        if (shaped->empty()) return;
        ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
        // curr_scan_point: 同一个点云, 但只有 debug_publish_ 才发 (legacy.cpp:1007-1009)
        if (debug_publish_) pointcloud_puber_.publish(toPointCloud2(*shaped, kMapFrame, stamp));
        // or_curr_scan_point: 原始输入帧, 抽稀倍率 x2, **不过滤 z** (legacy.cpp:1026-1058)。
        // frame_id "map" 照抄旧实现 (xz_slam_ros1_interface.cpp:1053)。
        CloudType::Ptr raw_snapshot;
        ros::Time raw_stamp;
        {
            std::lock_guard<std::mutex> lk(raw_input_mtx_);
            raw_snapshot = raw_input_;
            raw_stamp = raw_input_stamp_;
        }
        if (raw_snapshot && !raw_snapshot->empty()) {
            CloudType::Ptr orc = legacyScanShape(*raw_snapshot, -1.0, 2);
            if (!orc->empty())
                or_pointcloud_puber_.publish(toPointCloud2(*orc, kMapFrame, raw_stamp));
        }
    } else if (slam_mode_ == SlamMode::MAPPING) {
        // scan_f_map: 抽稀到约 2000 + z∈(0,1) (legacy.cpp:1071-1096)
        CloudType::Ptr shaped = legacyScanShape(world, kMapZMax, 1);
        // curr_scan_point: **完整的**去畸变点云 (legacy.cpp:1097 把过滤结果覆盖掉了), 无条件发
        pointcloud_puber_.publish(toPointCloud2(world, kMapFrame, stamp));
        if (!shaped->empty()) ros1_pointcloud_puber_.publish(toOldPointCloud(*shaped, kMapFrame, stamp));
        // or_curr_scan_point: 建图**不发** (legacy.cpp:1102 那行是注释掉的)
    }
}

void LegacyXz3dSlamInterface::publishMappingPath() {
    mapping_path_msg_.header.frame_id = kMapFrame;
    path_puber_.publish(mapping_path_msg_);   // 整条重发 (legacy.cpp:885-887)
}

void LegacyXz3dSlamInterface::publishRealtimeGridMap() {
    GridInfo gi;
    if (realtime_grid_.latest(gi)) {
        realtime_grid_map_ = toLegacyGrid(gi, kMapFrame);
        realtime_grid_map_area_ =
            static_cast<double>(gi.width) * gi.height * gi.resolution * gi.resolution;
        // legacy.cpp:1749 把建图面积塞进 origin.position.z
        realtime_grid_map_.info.origin.position.z = realtime_grid_map_area_ * 0.00001;
        std::cout << "[legacy_iface] realtime grid updated: " << gi.width << "x" << gi.height
                  << " res=" << gi.resolution << " area=" << realtime_grid_map_area_ << std::endl;
    }
    // 无更新也照发上一张 (legacy.cpp:909 无条件 publish)
    realtime_grid_map_puber_.publish(realtime_grid_map_);
}

void LegacyXz3dSlamInterface::publishPointMap() {
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

void LegacyXz3dSlamInterface::publishCurrPose() {
    updateModeFromCore();

    SlamOutput out;
    const bool have_out = engine_.latestOutput(out) && out.has_new_pose;

    // **核心估的是"雷达/IMU 刚体"的位姿**, 而这辆车的雷达偏航 90°、俯装 41.67°、抬高 0.9m。
    // 直接当车身位姿发出去 -> 朝向差 90° 且带 42° 俯仰 (实测就是这个现象)。
    // 这里用 TF 里的安装变换换到车身 (base_link) 系:
    //   T_world_base = T_world_lidar * T_lidar_base,  T_lidar_base = inv(T_base_lidar)
    //   R_base = R_lidar * R_lb^T ;  t_base = t_lidar - R_base * t_lb
    if (have_out) {
        computeLidarToBase();
        if (have_lidar_mount_) {
            const Eigen::Matrix3d R_body = out.pose.rot * r_lb_.transpose();
            out.pose.trans = out.pose.trans - R_body * t_lb_;
            out.pose.rot = R_body;
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

                // INIT 陷阱的 latch 就靠这里: 核心在定位模式下**只在 LOCALIZED 时**
                // 才产出 has_new_pose 的输出 (slam_pipeline.cpp:178-182),
                // 所以"收到过新位姿" == "真的定位上过"。
                lost_monitor_.onLocalizedPose();
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
            mapping_path_msg_.poses.push_back(ps);   // 累积, 供 publishMappingPath 整条重发
        }
    }

    if (publish_info) publishLocationInfo();
    // legacy.cpp:1208-1210: 非 FREE 且 use_tf_ 才发 (FREE 下不发)
    if (slam_mode_ != SlamMode::FREE && use_tf_) sendTransform();
}

void LegacyXz3dSlamInterface::publishLocationInfo() {
    LegacyPose p = curr_pose_;
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
        std::cout << "[legacy_iface] localization LOST -> warning 20003504 (pose lost)" << std::endl;
    }

    slam_info_puber_.publish(
        buildSlamInfo(slam_state, location_status, curr_map_name_, p.x, p.y, p.yaw, map_area,
                      matching_degree));
}

// ---------------------------------------------------------------- [T7] 告警

void LegacyXz3dSlamInterface::loadWarningCodeTable() {
    // M6: 从**本包 share**读, 不读 /opt —— 旧实现用的是编译期 PROJECT_PATH
    // (json_warning_code.cpp:168 的 project_path)。
    const std::string path =
        ros::package::getPath("rosiwit_slam_ros1") + "/conf/warning_code.json";
    std::ifstream fin(path);
    if (!fin.good()) {
        ROS_WARN_STREAM("[legacy_iface] warning_code.json not readable: " << path);
        return;
    }
    try {
        const YAML::Node n = YAML::LoadFile(path);   // JSON 是 YAML 的子集
        if (!n.IsMap()) {
            ROS_WARN_STREAM("[legacy_iface] warning_code.json is not a map: " << path);
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
        std::cout << "[legacy_iface] warning_code table loaded: " << warning_code_table_.size()
                  << " codes from " << path << std::endl;
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[legacy_iface] warning_code.json parse error (" << e.what()
                        << "): " << path);
    }
}

void LegacyXz3dSlamInterface::addWarningCode(int code) {
    std::lock_guard<std::mutex> lk(warning_code_mtx_);
    if (std::find(pending_warning_codes_.begin(), pending_warning_codes_.end(), code) ==
        pending_warning_codes_.end()) {
        pending_warning_codes_.push_back(code);
    }
}

void LegacyXz3dSlamInterface::setWarningSuppression(bool on) {
    warning_code_start_task_ = on;
    warning_code_suppress_ticks_ = 0;
}

void LegacyXz3dSlamInterface::tickFreqWindow(bool is_lidar) {
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

void LegacyXz3dSlamInterface::timerPublishWarningCode(const ros::TimerEvent&) {
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

void LegacyXz3dSlamInterface::initClients() {
    tag_detect_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::TagDetect>("get_charging_point");
    set_detect_mode_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::CommonIntRequestService>("set_detect_mode");
    switch_map_pro_client_ =
        nh_.serviceClient<xzrobot_ros_dependency::SwitchMap>("/switch_map_pro");
}

bool LegacyXz3dSlamInterface::callTagDetect(std::string& charging_pile_json) {
    charging_pile_json.clear();
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时 (100ms x 10)
        xzrobot_ros_dependency::TagDetect srv;
        if (tag_detect_client_.call(srv)) {
            charging_pile_json = srv.response.charging_pile_json;
            return true;
        }
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[legacy_iface] get_charging_point unavailable");
    return false;   // 旧行为: 失败给空 charging_pile_json (D14)
}

bool LegacyXz3dSlamInterface::callSetDetectMode(int mode) {
    for (int i = 0; i < 5; ++i) {    // 0.5s 超时 + 100ms 重试
        xzrobot_ros_dependency::CommonIntRequestService srv;
        srv.request.mode = static_cast<int8_t>(mode);
        if (set_detect_mode_client_.call(srv)) return srv.response.result;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[legacy_iface] set_detect_mode unavailable");
    return false;
}

bool LegacyXz3dSlamInterface::callSwitchMapPro(xzrobot_ros_dependency::SwitchMap::Request& req) {
    for (int i = 0; i < 10; ++i) {   // 1.0s 超时
        xzrobot_ros_dependency::SwitchMap srv;
        srv.request = req;
        if (switch_map_pro_client_.call(srv)) return srv.response.success;
        ros::Duration(0.1).sleep();
    }
    ROS_WARN("[legacy_iface] /switch_map_pro unavailable");
    return false;
}

// ---------------------------------------------------------------- [T8] 8 个 server

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
    srv_finish_task_ = nh_.advertiseService("/finish_task",
        &LegacyXz3dSlamInterface::handleFinishTask, this);
    srv_start_task_ = nh_.advertiseService("/start_task",
        &LegacyXz3dSlamInterface::handleStartTask, this);
    srv_expand_map_ = nh_.advertiseService("update_map_operation",
        &LegacyXz3dSlamInterface::handleExpandMap, this);
}

void LegacyXz3dSlamInterface::applyInitPose(
    const geometry_msgs::PoseWithCovarianceStamped& in, bool zero_if_insane) {
    if (!engine_.algo()) return;
    double x = in.pose.pose.position.x;
    double y = in.pose.pose.position.y;
    if (zero_if_insane && (std::fabs(x) > 1e4 || std::fabs(y) > 1e4)) { x = 0.0; y = 0.0; }
    Eigen::Quaterniond q(in.pose.pose.orientation.w, in.pose.pose.orientation.x,
                         in.pose.pose.orientation.y, in.pose.pose.orientation.z);
    q.normalize();
    PoseStamped p;
    p.trans = V3D(x, y, 0.0);   // z 强制 0: /switch_map 与 /relocation 都是
    p.rot = q.toRotationMatrix();
    p.time = in.header.stamp.toSec();
    engine_.algo()->setInitPose(p);
}

bool LegacyXz3dSlamInterface::handleModeSwitch(
    xzrobot_ros_dependency::SlamModeSwitchService::Request& req,
    xzrobot_ros_dependency::SlamModeSwitchService::Response& res) {
    if (req.mode == 0) {
        // 旧 switchToLocalization (legacy.cpp:2060-2115): 先置 FREE, 再调 /set_detect_mode(0),
        // **要求成功** —— 失败就 result=0 + return false。
        // 旧实现同时 DELETE_PTR 掉了建图/定位实例 -> 我们在这里也重建核心(丢上一轮地图)。
        resetCoreAndCaches();
        setModeFromLegacy(SlamMode::FREE);
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
        setModeFromLegacy(SlamMode::MAPPING);
        setWarningSuppression(true);
        res.result = 1;
        return true;
    }
    return false;   // 非法 mode: result 保持默认 0
}

bool LegacyXz3dSlamInterface::handleSwitchMap(
    xzrobot_ros_dependency::SwitchMap::Request& req,
    xzrobot_ros_dependency::SwitchMap::Response& res) {
    // 10s 限流 —— 注意这在 **/switch_map** 里, 不在 /slam/mode_switch (legacy.cpp:2121-2137)
    const ros::Time now = ros::Time::now();
    if (!last_switch_map_time_.isZero() && (now - last_switch_map_time_).toSec() < 10.0) {
        addWarningCode(20003005);   // set mode operation frequent
        res.success = false;
        return false;
    }
    last_switch_map_time_ = now;

    // 分支 1: 参数不合法
    if (req.map_dir.empty() || req.map_name.empty()) {
        res.success = false;
        return true;
    }
    // 分支 2: 同一张图且已在定位 -> 只重设初始位姿 + 转发 /switch_map_pro
    if (req.map_dir == curr_map_path_ && slam_mode_ == SlamMode::LOCALIZATION) {
        applyInitPose(req.initial_pose, true);
        res.success = callSwitchMapPro(req);
        return true;
    }
    // 分支 3: 换图
    relocating_ = true;
    // 旧实现这里 DELETE_PTR 掉了建图实例 (legacy.cpp:2245-2246) -> 我们也重建核心
    resetCoreAndCaches();
    const bool detect_ok = callSetDetectMode(0);
    res.success = detect_ok;   // 旧实现: **该结果单独决定** success

    const std::string base = legacyMapBase(req.map_dir);   // "/dir/map"
    if (engine_.algo() && !engine_.algo()->loadMap(base)) {
        // D13: 旧实现不检查加载结果; 我们检查并报错(同时把 success 拉低)
        res.success = false;
        ROS_ERROR_STREAM("[legacy_iface] /switch_map: loadMap failed: " << base);
    }
    if (engine_.algo()) engine_.algo()->setPipelineMode("localization");
    slam_mode_ = SlamMode::LOCALIZATION;
    curr_map_path_ = req.map_dir;
    curr_map_name_ = req.map_name;
    applyInitPose(req.initial_pose, true);
    lost_monitor_.reset();
    lost_monitor_.setMapLoaded(true);
    setWarningSuppression(true);   // 旧实现这里也调 run() (legacy.cpp:2345)
    return true;
}

bool LegacyXz3dSlamInterface::handleRelocation(
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
    res.success = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleStopMap(
    xzrobot_ros_dependency::StopMapOperation::Request& req,
    xzrobot_ros_dependency::StopMapOperation::Response& res) {
    // 旧 stopMap (legacy.cpp:2419-2516)
    const std::string folder = req.map_dir;
    struct stat st;
    const bool exists = (!folder.empty() && ::stat(folder.c_str(), &st) == 0 && S_ISDIR(st.st_mode));
    if (!exists) {
        if (::mkdir(folder.c_str(), 0755) != 0) {
            // 旧实现: 建目录失败 -> return false 且 success 留 false
            ROS_ERROR_STREAM("[legacy_iface] /stop_map_operation: cannot create dir: " << folder);
            return false;
        }
    }
    // 旧实现要求 mapping_ptr_ 非空
    if (slam_mode_ != SlamMode::MAPPING || !engine_.algo()) return false;

    // 旧: mapping_ptr_->saveMap(folderPath, "map") -> 等价于 saveMap("<dir>/map")
    const std::string base = legacyMapBase(req.map_dir);
    if (!engine_.algo()->saveMap(base)) {
        // 旧实现的 saveMap 返回 void, **不检查**结果 -> 我们只报错, 不改 success
        ROS_ERROR_STREAM("[legacy_iface] /stop_map_operation: saveMap failed: " << base);
    }

    // 同时补 map.pgm / map.yaml: 旧实现的地图目录里有这两个文件, 且旧
    // LocalizationSetGridMap() (legacy.cpp:373-455) 会 imread("<base>.pgm") 并读
    // "<base>.yaml" 的 resolution/origin/occupied_thresh。核心的 saveGridMap 写的正是
    // 这个格式 (grid_map_io.cpp:7-39: P5, 首行=最大 y; yaml 含 image/resolution/
    // origin/negate/occupied_thresh/free_thresh/mode)。
    if (!engine_.algo()->saveGridMap(base + ".pgm", base + ".yaml", grid_resolution_)) {
        // 与 saveMap 同理: 失败只报错, 不改 success (旧实现也没有这一步可对照)
        ROS_ERROR_STREAM("[legacy_iface] /stop_map_operation: saveGridMap failed: " << base);
    } else {
        std::cout << "[legacy_iface] saved grid map: " << base << ".pgm / " << base
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

bool LegacyXz3dSlamInterface::handleGlobalLocalization(std_srvs::Empty::Request&,
                                                       std_srvs::Empty::Response&) {
    return true;   // legacy.cpp:2634 全文就这一行
}

bool LegacyXz3dSlamInterface::handleFinishTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 finishTask (legacy.cpp:2601-2609): 有定位实例 -> updateMap() + status=true;
    // 否则 return false 且 status 留 false。降级: 定位模式且有核心 -> true。
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleStartTask(
    xzrobot_ros_dependency::CommonBoolService::Request&,
    xzrobot_ros_dependency::CommonBoolService::Response& res) {
    // 旧 startTask (legacy.cpp:2591-2599): setUpdateMapFlag() + status=true
    if (slam_mode_ != SlamMode::LOCALIZATION || !engine_.algo()) return false;
    res.status = true;
    return true;
}

bool LegacyXz3dSlamInterface::handleExpandMap(
    xzrobot_ros_dependency::UpdateMap::Request& req,
    xzrobot_ros_dependency::UpdateMap::Response& res) {
    // **有意降级 (D5)**: 旧实现 (legacy.cpp:2538-2589) 是"重建建图实例 +
    // setIncreaseMapFlag() + loadCurrentMap(map_dir + \"/map.xzmap\") +
    // setInitPose(pos, rot * AngleAxis(angle,Z))", 核心没有"增量扩图后写回"的能力。
    ROS_WARN("[legacy_iface] update_map_operation: 降级实现 (D5) —— 用 loadMap + 切定位代替 "
             "旧实现的增量扩图写回; 行为不等价");
    const std::string base = legacyMapBase(req.map_dir);
    bool ok = false;
    if (engine_.algo()) {
        ok = engine_.algo()->loadMap(base);
        if (ok) {
            engine_.algo()->setPipelineMode("localization");
            slam_mode_ = SlamMode::LOCALIZATION;
            applyInitPose(req.initial_pose, false);
            lost_monitor_.reset();
            lost_monitor_.setMapLoaded(true);
            curr_map_path_ = req.map_dir;
        } else {
            ROS_ERROR_STREAM("[legacy_iface] update_map_operation: loadMap failed: " << base);
        }
    }
    res.status = ok;
    return true;
}

// ---------------------------------------------------------------- [T9] TF map->odom

void LegacyXz3dSlamInterface::sendTransform() {
    // legacy.cpp:1899-1936 逐行:
    //   Tfo = lookupTransform("base_footprint", "odom", Time(0))     = T(base_footprint <- odom)
    //   Tmf = (origin=(x, y, -0.07), rot=AngleAxis(-yaw, Z))         = T(map <- base_footprint)
    //        其中 yaw = atan2(R(0,1), R(0,0))  (即 legacyYawFromRot)
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
    // 注意这里必须用 legacyYawFromRot(q)(= -标准航向) 再取负, 与旧实现成对出现 ——
    // 直接用 curr_pose_.yaw 会把符号弄反。
    const double legacy_yaw =
        legacyYawFromRot(curr_pose_.q.normalized().toRotationMatrix());
    const Eigen::Matrix3d R_f =
        Eigen::AngleAxisd(-legacy_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
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