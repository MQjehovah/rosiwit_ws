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
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

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
    // **优先读老包的那份**: 旧实现的主配置 = 编译期 PROJECT_PATH/conf/slam_conf.json,
    // 装机后就是 /opt/ros/melodic/share/xz_3d_slam/conf/slam_conf.json —— 那才是这台机器
    // 真正生效的配置 (运维可能改过, 且与我们随包的那份并不一致: 实测老包那份没有
    // laser_params 子树、x_new/z_new 也不同)。读不到才退回我们自己 conf/ 里的副本。
    const std::string old_pkg = "/opt/ros/melodic/share/xz_3d_slam";
    const std::string old_conf = old_pkg + "/conf/slam_conf.json";
    std::ifstream probe(old_conf);
    p.proj_conf_json = probe.good() ? old_conf : (pkg + "/conf/slam_conf.json");
    std::cout << "[slam_engine] legacy main conf source: " << p.proj_conf_json << std::endl;
    // 旧实现只有一个主配置来源。T1 的 LegacyConfig 留了 proj(优先)/pkg(兜底) 两级,
    // 这里让二者指向同一份文件。见计划 §5.6。
    p.pkg_conf_json = p.proj_conf_json;
    p.opt_sensor_json = "/opt/xzrobot/configs/sensor_config.json";
    p.opt_slam_json = "/opt/xzrobot/configs/3d_slam/slam_conf.json";
    return p;
}

namespace {

/// legacy profile: 给核心一份 override YAML —— 只加 `map_directory: "/"` 一个键 (M8/D9)。
/// 做法是把基底文件**按文本**复制过来再在末尾追加一行, 而不是重新 Dump 整个 Node ——
/// 这样原文件里的注释与排版都保留, 也不会被 YAML 重排改变语义。
std::string writeLegacyOverrideYaml(const std::string& base_yaml,
                                    const Eigen::Matrix3d& r_il,
                                    const Eigen::Vector3d& t_il,
                                    bool write_extrinsic) {
    const char* home = std::getenv("HOME");
    const std::string dir = std::string(home ? home : ".") + "/.ros/rosiwit_slam_ros1";
    ::mkdir(dir.c_str(), 0755);                 // 失败也无所谓(可能已存在)
    ::mkdir((std::string(home ? home : ".") + "/.ros").c_str(), 0755);
    const std::string out_path = dir + "/compat_legacy.yaml";

    std::ifstream in(base_yaml);
    std::ofstream out(out_path, std::ios::trunc);
    if (!out.good()) {
        ROS_ERROR_STREAM("[slam_engine] cannot write override yaml: " << out_path);
        return base_yaml;   // 写不出来就退回基底, 至少不把 core 指向不存在的文件
    }
    if (in.good()) {
        // 写外参时**跳过基底里原有的 r_il/t_il 行**: 同一个键在 YAML 里出现两次属重复键,
        // yaml-cpp 取哪一份是未定义行为 (实测就出现了"注入成功但生效值还是老值")。
        // 先丢掉旧行, 再在末尾写推导值, 保证唯一。
        std::string line;
        while (std::getline(in, line)) {
            if (write_extrinsic) {
                const size_t b = line.find_first_not_of(" \t");
                if (b != std::string::npos) {
                    const std::string t = line.substr(b);
                    if (t.rfind("r_il:", 0) == 0 || t.rfind("t_il:", 0) == 0) continue;
                }
            }
            out << line << "\n";
        }
    }
    out << "\n# --- injected by rosiwit_slam_ros1 (legacy profile, M8) ---\n"
           "# 旧实现的地图读写都是 <dir>/map.pcd|.xzmap|.pgm|.yaml 的绝对路径, 而核心的\n"
           "# PcdMapManager 是 map_dir_ + \"/\" + name; 置成 \"/\" 让拼接退化为绝对路径。\n"
           "map_directory: \"/\"\n";
    // 雷达-IMU 外参: 由启动时按 TF 推导 (见 computeLegacyExtrinsic), 写在这里覆盖基底配置。
    // write_extrinsic=false 表示 TF 推导失败 -> 不写, 让基底配置里的值生效。
    if (write_extrinsic) {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "\n# --- r_il/t_il 由启动时推导 (TF/同名帧), 不是手工写死 ---\n"
                      "r_il: [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n"
                      "t_il: [%.8f, %.8f, %.8f]\n",
                      r_il(0, 0), r_il(0, 1), r_il(0, 2),
                      r_il(1, 0), r_il(1, 1), r_il(1, 2),
                      r_il(2, 0), r_il(2, 1), r_il(2, 2),
                      t_il.x(), t_il.y(), t_il.z());
        out << buf;
    }
    out.close();
    return out_path;
}

} // namespace  <-- 临时收起匿名 namespace: 成员函数必须定义在 rosiwit_slam_ros1 里

bool SlamEngine::computeLegacyExtrinsic(Eigen::Matrix3d& R, Eigen::Vector3d& t,
                                       std::string& source) {
    // 帧名可配; 默认都是 lidar —— 本机点云与 IMU 由同一个万集 SDK 节点发布, frame_id 相同
    std::string lidar_frame = "lidar", imu_frame = "lidar";
    pnh_.param<std::string>("lidar_frame", lidar_frame, lidar_frame);
    pnh_.param<std::string>("imu_frame", imu_frame, imu_frame);

    if (lidar_frame == imu_frame) {
        // **同一个 frame 就是同一个坐标系** -> 恒等变换。
        // 雷达内置 IMU 正是这种情形 (frame_id 都是 lidar), 此时任何旋转都是错的。
        R = Eigen::Matrix3d::Identity();
        t = Eigen::Vector3d::Zero();
        source = std::string("identity (lidar_frame == imu_frame == '") + lidar_frame + "')";
        return true;
    }

    // 独立 IMU: 从 TF 取 T_(imu <- lidar)。启动时 /tf_static 可能还没到, 等最多 3 秒。
    if (!tf_buffer_) tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    if (!tf_listener_) tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    for (int i = 0; i < 30; ++i) {
        try {
            const geometry_msgs::TransformStamped tf =
                tf_buffer_->lookupTransform(imu_frame, lidar_frame, ros::Time(0), ros::Duration(0.1));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
            q.normalize();
            R = q.toRotationMatrix();
            t = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                                tf.transform.translation.z);
            source = "TF lookupTransform('" + imu_frame + "' <- '" + lidar_frame + "')";
            return true;
        } catch (const tf2::TransformException&) {
            ros::Duration(0.05).sleep();
        }
    }
    ROS_WARN_STREAM("[slam_engine] TF '" << imu_frame << "' <- '" << lidar_frame
                    << "' unavailable within 3s; keeping r_il/t_il from the core yaml");
    return false;
}

namespace {  // 重新打开匿名 namespace, 供下面的 printCompatDiagnostics 使用

/// D7 / §5.10 B / 待办项 C 要的可诊断性: 把核心**实际生效**的外参与 acc_scale 打出来,
/// 因为"完全没位姿"最常见的两个原因就是加速度单位(acc_scale)与雷达-IMU 外参(r_il/t_il)。
/// 用 std::cout + endl (ROS_INFO 在 roslaunch 下是块缓冲的, 看不到)。
void printCompatDiagnostics(const std::string& base_yaml, const std::string& injected_yaml,
                            const LegacyConfig& lc) {
    double acc_scale = 1.0;                  // 核心默认 (fast_lio2/map_builder/commons.h:47)
    std::string r_il = "(缺省)", t_il = "(缺省)";
    try {
        // **读注入后的文件**: 那才是核心实际拿到的配置 (基底里的 r_il/t_il 可能已被覆盖)。
        const YAML::Node c = YAML::LoadFile(injected_yaml);
        if (c && c["acc_scale"]) acc_scale = c["acc_scale"].as<double>(acc_scale);
        if (c && c["r_il"]) r_il = YAML::Dump(c["r_il"]);
        if (c && c["t_il"]) t_il = YAML::Dump(c["t_il"]);
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[slam_engine] compat diagnostics: cannot read base yaml (" << e.what() << ")");
    }
    std::cout << "[compat] core yaml = " << injected_yaml << std::endl;
    std::cout << "[compat] effective acc_scale=" << acc_scale << " r_il=" << r_il
              << " t_il=" << t_il << std::endl;
    std::cout << "[compat] ignored legacy keys: roll_new=" << lc.roll_new
              << " pitch_new=" << lc.pitch_new << " yaw_new=" << lc.yaw_new
              << " imu_in_car_frame_yaw_livox=" << lc.imu_in_car_frame_yaw_livox
              << " (M13: 外参/单位按核心 YAML, 不采纳旧实现的 9.81 与安装角)" << std::endl;
    if (std::fabs(acc_scale - 1.0) < 1e-12) {
        std::cout << "[compat] WARN acc_scale=1.0 —— 若雷达内置 IMU 的加速度是 g 单位, "
                     "这里应为 9.81 (待办项 C: 首次上机必须核对本行)" << std::endl;
    }
}

} // namespace

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
                cfg_.odom_topic = c["odom_topic"].as<std::string>(cfg_.odom_topic);
                cfg_.absolute_topic = c["absolute_topic"].as<std::string>(cfg_.absolute_topic);
                if (c["vision"]) {
                    vo_enabled = c["vision"]["enabled"].as<bool>(false);
                    cfg_.vo_topic = c["vision"]["vo_topic"].as<std::string>(cfg_.vo_topic);
                }
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

        // ---- M8: 给核心注入 map_directory: "/" 的 override YAML ----
        std::string base = config_path_;
        if (base.empty()) {
            base = ros::package::getPath("rosiwit_slam_ros1") + "/config/default.yaml";
            ROS_WARN_STREAM("[slam_engine] legacy profile: ~config_file 未设置, 核心 YAML 回落到 "
                            << base);
        }
        // 雷达-IMU 外参: 优先按 TF / 同名帧自动推导; 推不出来就保留基底配置里的值。
        Eigen::Matrix3d r_il = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_il = Eigen::Vector3d::Zero();
        std::string extr_src;
        const bool have_extr = computeLegacyExtrinsic(r_il, t_il, extr_src);
        if (!have_extr) extr_src = "config (core yaml)";
        config_path_ = writeLegacyOverrideYaml(base, r_il, t_il, have_extr);
        std::cout << "[compat] extrinsic source = " << extr_src << std::endl;
        if (have_extr) {
            char eb[320];
            std::snprintf(eb, sizeof(eb),
                          "[compat] r_il(t_il) = [%.6f %.6f %.6f | %.6f %.6f %.6f | %.6f %.6f %.6f] t=[%.6f %.6f %.6f]",
                          r_il(0,0), r_il(0,1), r_il(0,2), r_il(1,0), r_il(1,1), r_il(1,2),
                          r_il(2,0), r_il(2,1), r_il(2,2), t_il.x(), t_il.y(), t_il.z());
            std::cout << eb << std::endl;
        }        printCompatDiagnostics(base, config_path_, legacy_cfg_);
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
    // 轮式里程计: legacy profile 下由 LegacyXz3dSlamInterface 独占订阅 (它无条件喂核心),
    // 引擎这里就不再重复订阅, 免得同一份 /odom 被注入两次。
    if (odom_enabled && !legacyProfile())
        odom_sub_ = nh_.subscribe(cfg_.odom_topic, 100, &SlamEngine::odomCB, this);
    if (vo_enabled) vo_sub_ = nh_.subscribe(cfg_.vo_topic, 100, &SlamEngine::voCB, this);
    if (absolute_enabled) absolute_sub_ = nh_.subscribe(cfg_.absolute_topic, 100, &SlamEngine::absoluteCB, this);

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

double SlamEngine::getMatchingDegree() const {
    return algo_raw_ ? algo_raw_->getMatchingDegree() : 0.0;
}

bool SlamEngine::resetCore() {
    if (!algo_raw_) return false;
    // 先复位 latch/快照类状态, 免得重建后还带着上一轮的位姿
    {
        std::lock_guard<std::mutex> lk(out_mutex_);
        latest_ = SlamOutput();
        have_output_ = false;
    }
    // 旧实现切模式时会删掉建图/定位实例; 我们重建核心模块达到同样效果。
    const bool ok = algo_raw_->resetForNewMapping(config_path_);
    std::cout << "[slam_engine] resetCore -> " << (ok ? "ok" : "unsupported/failed") << std::endl;
    return ok;
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
