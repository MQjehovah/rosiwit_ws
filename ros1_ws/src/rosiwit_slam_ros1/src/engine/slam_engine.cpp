// ============================================================================
// SlamEngine 实现 —— 全部从原 rosiwit_slam_ros1_node.cpp 搬移。
// 输入话题由机器人现有 sensor_config 决定，其余行为保持不变。
// ============================================================================
#include "slam_engine.h"

#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include "slam_core/diagnostics.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
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

Xz3dConfPaths defaultXz3dPaths() {
    Xz3dConfPaths p;
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
    // 旧实现只有一个主配置来源。T1 的 Xz3dConfig 留了 proj(优先)/pkg(兜底) 两级,
    // 这里让二者指向同一份文件。见计划 §5.6。
    p.pkg_conf_json = p.proj_conf_json;
    p.opt_sensor_json = "/opt/xzrobot/configs/sensor_config.json";
    p.opt_slam_json = "/opt/xzrobot/configs/3d_slam/slam_conf.json";
    return p;
}

namespace {

/// 给核心写一份 override YAML: 文本复制基底, 末尾追加实际生效的 r_il/t_il / bbs_mount_r。
/// legacy profile 额外写 `map_directory: "/"` (M8/D9); 新接口不写, 保持原有相对路径语义。
std::string writeXz3dOverrideYaml(const std::string& base_yaml,
                                    const Eigen::Matrix3d& r_il,
                                    const Eigen::Vector3d& t_il,
                                    bool write_extrinsic,
                                    const Eigen::Matrix3d* r_bl,
                                    bool write_map_dir = true) {
    const char* home = std::getenv("HOME");
    const std::string dir = std::string(home ? home : ".") + "/.ros/rosiwit_slam_ros1";
    ::mkdir(dir.c_str(), 0755);                 // 失败也无所谓(可能已存在)
    ::mkdir((std::string(home ? home : ".") + "/.ros").c_str(), 0755);
    const std::string out_path =
        dir + (write_map_dir ? "/compat_legacy.yaml" : "/core_extrinsic.yaml");

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
            const size_t b = line.find_first_not_of(" \t");
            if (b != std::string::npos) {
                const std::string t = line.substr(b);
                if (t.rfind("esti_il:", 0) == 0 ||
                    t.rfind("estimate_imu_lidar_extrinsic:", 0) == 0 ||
                    t.rfind("bbs_mount_r:", 0) == 0) continue;
                if (write_extrinsic &&
                    (t.rfind("r_il:", 0) == 0 ||
                     t.rfind("imu_lidar_rotation:", 0) == 0 ||
                     t.rfind("t_il:", 0) == 0 ||
                     t.rfind("imu_lidar_translation:", 0) == 0)) continue;
            }
            out << line << "\n";
        }
    }
    out << "esti_il: false   # ROS layer feeds IMU body; never estimate extrinsic online\n";
    if (write_map_dir) {
        out << "\n# --- injected by rosiwit_slam_ros1 (legacy profile, M8) ---\n"
               "# 旧实现的地图读写都是 <dir>/map.pcd|.xzmap|.pgm|.yaml 的绝对路径, 而核心的\n"
               "# PcdMapManager 是 map_dir_ + \"/\" + name; 置成 \"/\" 让拼接退化为绝对路径。\n"
               "map_directory: \"/\"\n";
    }
    // 雷达-IMU 外参: 由启动时按 TF 推导 (见 computeXz3dExtrinsic), 写在这里覆盖基底配置。
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
    // 安装旋转 (base<-lidar) 以**矩阵**注入: 规避欧拉角顺序歧义, 供 BBS 车体系投影使用
    if (r_bl) {
        char rb[512];
        std::snprintf(rb, sizeof(rb),
                      "\n# base<-lidar 旋转(direct from TF), BBS 车体系投影用\n"
                      "bbs_mount_r: [%.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f]\n",
                      (*r_bl)(0, 0), (*r_bl)(0, 1), (*r_bl)(0, 2),
                      (*r_bl)(1, 0), (*r_bl)(1, 1), (*r_bl)(1, 2),
                      (*r_bl)(2, 0), (*r_bl)(2, 1), (*r_bl)(2, 2));
        out << rb;
    }
    out.close();
    return out_path;
}

} // namespace  <-- 临时收起匿名 namespace: 成员函数必须定义在 rosiwit_slam_ros1 里

bool SlamEngine::computeXz3dExtrinsic(Eigen::Matrix3d& R, Eigen::Vector3d& t,
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

bool SlamEngine::computeBaseFrames(bool blocking) {
    // 行业标准: 算法估计 IMU body; ROS1 接口层负责 base_link <-> IMU 的输出/输入换算.
    // base_lidar 只给 BBS 的车体系投影用 (输入仍是原始 lidar 点云).
    if (base_imu_.valid) return true;   // 输出/初始位姿只需要 base_link <- imu
    // 失败后**限频重试**(每 2s 最多一次):
    //   - 每帧重试: lookupTransform 最多阻塞 3s, 会把 100Hz 定时器拖死(实测 /slam/info 停发);
    //   - 只试一次: 启动瞬间 robot_state_publisher 还没发 /tf_static 就永远失败。
    const ros::Time now_try = ros::Time::now();
    if (!mount_last_try_.isZero() && (now_try - mount_last_try_).toSec() < 2.0) return false;
    mount_last_try_ = now_try;
    if (!tf_buffer_) tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    if (!tf_listener_) tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pnh_.param<std::string>("lidar_frame", mount_lidar_frame_, mount_lidar_frame_);
    pnh_.param<std::string>("imu_frame", mount_imu_frame_, mount_imu_frame_);
    pnh_.param<std::string>("base_frame", mount_base_frame_, mount_base_frame_);

    auto lookup = [&](const std::string& target, const std::string& source,
                      RigidTransform& out, const char* tag) -> bool {
        try {
            const geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
                target, source, ros::Time(0), ros::Duration(0.1));
            Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                                 tf.transform.rotation.y, tf.transform.rotation.z);
            q.normalize();
            out.set(q.toRotationMatrix(),
                    Eigen::Vector3d(tf.transform.translation.x,
                                    tf.transform.translation.y,
                                    tf.transform.translation.z));
            const Eigen::Vector3d rpy = out.R.eulerAngles(2, 1, 0);
            char b[320];
            std::snprintf(b, sizeof(b),
                          "[slam_engine] %s mount from TF ('%s' <- '%s'): rpy(deg)=[%.2f %.2f %.2f] xyz=[%.3f %.3f %.3f]",
                          tag, target.c_str(), source.c_str(),
                          rpy.z() * 180 / M_PI, rpy.y() * 180 / M_PI, rpy.x() * 180 / M_PI,
                          out.t.x(), out.t.y(), out.t.z());
            std::cout << b << std::endl;
            return true;
        } catch (const tf2::TransformException&) {
            return false;
        }
    };

    // blocking=true (启动/初始位姿): 最多 ~3s; false: 只试一次, 2s 节流后重试.
    const int attempts = blocking ? 30 : 1;
    for (int i = 0; i < attempts; ++i) {
        bool ok_imu = base_imu_.valid;
        bool ok_lidar = base_lidar_.valid;
        if (!ok_imu) {
            if (mount_imu_frame_ == mount_base_frame_) {
                base_imu_.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                ok_imu = true;
            } else {
                ok_imu = lookup(mount_base_frame_, mount_imu_frame_, base_imu_, "imu");
            }
        }
        if (!ok_lidar) {
            if (mount_lidar_frame_ == mount_base_frame_) {
                base_lidar_.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
                ok_lidar = true;
            } else if (mount_lidar_frame_ == mount_imu_frame_ && base_imu_.valid) {
                base_lidar_ = base_imu_;
                ok_lidar = true;
            } else {
                ok_lidar = lookup(mount_base_frame_, mount_lidar_frame_, base_lidar_, "lidar");
            }
        }
        if (ok_imu && ok_lidar) return true;
        if (i + 1 < attempts) ros::Duration(0.05).sleep();
    }
    if (!base_imu_.valid) {
        ROS_WARN_STREAM("[slam_engine] TF base_link <- imu unavailable; retry in 2s");
    }
    return base_imu_.valid;
}

bool SlamEngine::imuFromChild(const std::string& child_frame, RigidTransform& out) {
    // 核心 body 是 IMU: T_world_imu = T_world_child * T_child_imu.
    if (child_frame.empty() || child_frame == mount_imu_frame_) {
        out.set(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
        return true;
    }
    if (child_frame == mount_base_frame_ && base_imu_.valid) {
        out = base_imu_;   // T_base_link_imu
        return true;
    }
    auto it = child_to_imu_.find(child_frame);
    if (it != child_to_imu_.end()) {
        out = it->second;
        return true;
    }
    if (!tf_buffer_) tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    if (!tf_listener_) tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    try {
        const geometry_msgs::TransformStamped tf = tf_buffer_->lookupTransform(
            child_frame, mount_imu_frame_, ros::Time(0), ros::Duration(0.1));
        Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z);
        q.normalize();
        out.set(q.toRotationMatrix(),
                Eigen::Vector3d(tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z));
        child_to_imu_[child_frame] = out;
        return true;
    } catch (const tf2::TransformException&) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[slam_engine] TF imu <- " << child_frame
                                 << " unavailable; drop this measurement");
        return false;
    }
}

namespace {  // 重新打开匿名 namespace, 供下面的 printCompatDiagnostics 使用

/// D7 / §5.10 B / 待办项 C 要的可诊断性: 把核心**实际生效**的外参与 acc_scale 打出来,
/// 因为"完全没位姿"最常见的两个原因就是加速度单位(acc_scale)与雷达-IMU 外参(r_il/t_il)。
/// 用 std::cout + endl (ROS_INFO 在 roslaunch 下是块缓冲的, 看不到)。
void printCompatDiagnostics(const std::string& base_yaml, const std::string& injected_yaml,
                            const Xz3dConfig& lc) {
    double acc_scale = 1.0;                  // 核心默认 (fast_lio2/map_builder/commons.h:47)
    std::string r_il = "(缺省)", t_il = "(缺省)";
    try {
        // **读注入后的文件**: 那才是核心实际拿到的配置 (基底里的 r_il/t_il 可能已被覆盖)。
        const YAML::Node c = YAML::LoadFile(injected_yaml);
        const YAML::Node inp = (c && c["global"] && c["global"]["input"])
                                   ? c["global"]["input"] : YAML::Node();
        if (inp && inp["acc_scale"]) acc_scale = inp["acc_scale"].as<double>(acc_scale);
        else if (c && c["acc_scale"]) acc_scale = c["acc_scale"].as<double>(acc_scale);
        const YAML::Node fe = (c && c["mapping"] && c["mapping"]["frontend"])
                                  ? c["mapping"]["frontend"] : YAML::Node();
        const YAML::Node rnode = (c && c["r_il"]) ? c["r_il"]
                                 : (fe && fe["imu_lidar_rotation"]) ? fe["imu_lidar_rotation"]
                                 : (fe ? fe["r_il"] : YAML::Node());
        const YAML::Node tnode = (c && c["t_il"]) ? c["t_il"]
                                 : (fe && fe["imu_lidar_translation"]) ? fe["imu_lidar_translation"]
                                 : (fe ? fe["t_il"] : YAML::Node());
        if (rnode) r_il = YAML::Dump(rnode);
        if (tnode) t_il = YAML::Dump(tnode);
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
        std::cout << "[compat] INFO acc_scale=1.0（本车万集 IMU 实测为 m/s^2；"
                     "若换用 g 单位传感器需改为 9.81）" << std::endl;
    }
}

} // namespace

SlamEngine::SlamEngine(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    pnh_.param<bool>("online_calibration", online_calib_enabled_, true);
    bool calib_apply = false;
    pnh_.param<bool>("online_calibration_apply", calib_apply, false);
    rosiwit_slam::CalibrationApplyConfig calib_cfg;
    calib_cfg.enabled = calib_apply;
    pnh_.param<double>("calib_min_confidence", calib_cfg.min_confidence, 0.60);
    pnh_.param<int>("calib_stable_windows", calib_cfg.stable_windows, 3);
    pnh_.param<int>("calib_rollback_windows", calib_cfg.rollback_windows, 3);
    pnh_.param<double>("calib_max_time_offset", calib_cfg.max_time_offset, 0.05);
    pnh_.param<double>("calib_max_yaw_offset", calib_cfg.max_yaw_offset, 0.15);
    pnh_.param<double>("calib_max_step", calib_cfg.max_step, 0.01);
    calib_controller_.setConfig(calib_cfg);

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
                const YAML::Node glob = c["global"];
                const YAML::Node inp = (glob && glob["input"]) ? glob["input"] : YAML::Node();
                const YAML::Node frames = (glob && glob["frames"]) ? glob["frames"] : YAML::Node();
                auto gi = [&](const char* key) -> YAML::Node {
                    if (inp && inp[key]) return inp[key];
                    return c[key];
                };
                auto gf = [&](const char* key) -> YAML::Node {
                    if (frames && frames[key]) return frames[key];
                    return c[key];
                };
                cfg_.imu_topic   = gi("imu_topic").as<std::string>(cfg_.imu_topic);
                cfg_.lidar_topic = gi("lidar_topic").as<std::string>(cfg_.lidar_topic);
                cfg_.body_frame  = gf("body_frame").as<std::string>(cfg_.body_frame);
                cfg_.world_frame = gf("world_frame").as<std::string>(cfg_.world_frame);
                cfg_.scan_period_ms = gi("scan_period_ms").as<float>(cfg_.scan_period_ms);
                cfg_.lidar_filter_num = gi("lidar_filter_num").as<int>(cfg_.lidar_filter_num);
                cfg_.lidar_min_range = gi("lidar_min_range").as<double>(cfg_.lidar_min_range);
                cfg_.lidar_max_range = gi("lidar_max_range").as<double>(cfg_.lidar_max_range);
                cfg_.odom_topic = gi("odom_topic").as<std::string>(cfg_.odom_topic);
                if (c["localization"] && c["localization"]["absolute"]) {
                    const YAML::Node a = c["localization"]["absolute"];
                    cfg_.absolute_topic = a["topic"].as<std::string>(cfg_.absolute_topic);
                    absolute_enabled = a["enabled"].as<bool>(false);
                }
                if (c["vision"]) {
                    vo_enabled = c["vision"]["enabled"].as<bool>(false);
                    cfg_.vo_topic = c["vision"]["vo_topic"].as<std::string>(cfg_.vo_topic);
                }
                odom_enabled = (inp && inp["use_odom"])
                                   ? inp["use_odom"].as<bool>(false)
                                   : c["enable_odom"].as<bool>(false);
            }
        } catch (const std::exception& e) {
            ROS_WARN("[rosiwit_slam_ros1] config parse error: %s", e.what());
        }
    }

    // ---- ROS1 输入: 沿用机器人现有 sensor_config 决定输入话题; 外参/单位仍取核心 YAML ----
    {
        std::string model;
        nh_.param<std::string>("/machine/model", model, "");
        xz_cfg_ = Xz3dConfig::load(defaultXz3dPaths(), model);
        // 旧实现 startSub (legacy.cpp:200-207) 选输入话题靠的是 **laser_type**,
        // 不是 imu_sub_name/laser_sub_name:
        //   mid360 / new_mid360 -> "livox/imu" + "livox/lidar"
        //   其它                -> <imu_sub_name> + <laser_sub_name>
        // 这一步很关键: 发行 conf 的顶层 laser_sub_name 是 "vanjee_points722",
        // 但对 mid360 机型旧实现**根本不看它** —— 照 laser_sub_name 订阅会在真机上收不到数据。
        // (D4: livox/lidar 在我们这边是 sensor_msgs/PointCloud2, 不是 livox 的 CustomMsg。)
        const std::string& lt = xz_cfg_.laser_type;
        if (lt == "mid360" || lt == "new_mid360") {
            cfg_.imu_topic = "livox/imu";
            cfg_.lidar_topic = "livox/lidar";
        } else {
            // 空值时保留核心 YAML 里的名字 (不应发生, 但别把订阅名清成空串)
            if (!xz_cfg_.imu_sub_name.empty())   cfg_.imu_topic   = xz_cfg_.imu_sub_name;
            if (!xz_cfg_.laser_sub_name.empty()) cfg_.lidar_topic = xz_cfg_.laser_sub_name;
        }
        std::cout << "[slam_engine] sensor laser_type='" << lt << "' -> input topics: imu="
                  << cfg_.imu_topic << " lidar=" << cfg_.lidar_topic << std::endl;

        // ---- M8: 给核心注入 map_directory: "/" 的 override YAML ----
        std::string base = config_path_;
        if (base.empty()) {
            base = ros::package::getPath("rosiwit_slam_ros1") + "/config/default.yaml";
            ROS_WARN_STREAM("[slam_engine] ROS1 输入: ~config_file 未设置, 核心 YAML 回落到 "
                            << base);
        }
        // 雷达-IMU 外参: 优先按 TF / 同名帧自动推导; 推不出来就保留基底配置里的值。
        Eigen::Matrix3d r_il = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_il = Eigen::Vector3d::Zero();
        std::string extr_src;
        const bool have_extr = computeXz3dExtrinsic(r_il, t_il, extr_src);
        if (!have_extr) extr_src = "config (core yaml)";
        // base_link <- imu/lidar 先取一次(TF): 输出换算与 BBS 投影都要用.
        computeBaseFrames(/*blocking=*/true);
        // 实验开关: 在 TF 得到的安装旋转上叠加一个固定旋转 (排查 URDF/地图/驱动问题时用)。
        // ⚠ 2026-10-17 实测定案: 默认必须全 0, 平时**不许**带任何 extra:
        //   - IMU 静止重力 = [-0.02, 0.66, 0.75](传感器系), 与 URDF 原始 rpy(41.67/1.82/89.89) 的预测完全一致;
        //   - 原始点云用 R_orig 投到车体系后地面为 -0.93m 紧贴薄层(=安装高度), 叠 Rx90 则无地面层;
        //   - 旧图 AAA 世界系斜 59.8°(关键帧 z 漂到 69m), 当年"Rx90 使 BBS 0.598→0.9958"是在那张废图上
        //     得出的错误结论 —— Rx90 补偿的是坏地图的倾斜, 不是驱动/安装。DDD/FFF 图已水平, 无需任何 extra。
        //   - 另: rosrun 的 _param 会留在参数服务器上不随节点退出而清除(实测踩坑), 排查后必须
        //     `rosparam delete /xz_3d_slam_node/mount_extra_{roll,pitch,yaw}_deg`。
        double ex_roll = 0.0, ex_pitch = 0.0, ex_yaw = 0.0;
        pnh_.param<double>("mount_extra_roll_deg", ex_roll, ex_roll);
        pnh_.param<double>("mount_extra_pitch_deg", ex_pitch, ex_pitch);
        pnh_.param<double>("mount_extra_yaw_deg", ex_yaw, ex_yaw);
        Eigen::Matrix3d r_bl_used = base_lidar_.R;
        if (base_lidar_.valid && (std::fabs(ex_roll) + std::fabs(ex_pitch) + std::fabs(ex_yaw) > 1e-9)) {
            const Eigen::Matrix3d R_ex =
                (Eigen::AngleAxisd(ex_yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(ex_pitch * M_PI / 180.0, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(ex_roll * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
            r_bl_used = R_ex * base_lidar_.R;
            std::cout << "[compat] mount extra rpy(deg)=[" << ex_roll << " " << ex_pitch
                      << " " << ex_yaw << "] applied (experimental)" << std::endl;
        }
        config_path_ = writeXz3dOverrideYaml(base, r_il, t_il, have_extr,
                                              base_lidar_.valid ? &r_bl_used : nullptr,
                                              /*write_map_dir=*/true);
        std::cout << "[compat] extrinsic source = " << extr_src << std::endl;
        if (have_extr) {
            char eb[320];
            std::snprintf(eb, sizeof(eb),
                          "[compat] r_il(t_il) = [%.6f %.6f %.6f | %.6f %.6f %.6f | %.6f %.6f %.6f] t=[%.6f %.6f %.6f]",
                          r_il(0,0), r_il(0,1), r_il(0,2), r_il(1,0), r_il(1,1), r_il(1,2),
                          r_il(2,0), r_il(2,1), r_il(2,2), t_il.x(), t_il.y(), t_il.z());
            std::cout << eb << std::endl;
        }        printCompatDiagnostics(base, config_path_, xz_cfg_);
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
    // 轮式里程计由 Ros1SlamInterface 统一订阅并喂核心，避免同一份 /odom 被注入两次。
    (void)odom_enabled;
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
    std::cout << "[slam_engine] config=" << (config_path_.empty() ? "(none)" : config_path_)
              << " imu=" << cfg_.imu_topic
              << " lidar=" << cfg_.lidar_topic << std::endl;
}

SlamEngine::~SlamEngine() = default;

CloudType::Ptr SlamEngine::ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                   int filter_num, double min_r, double max_r) {
    CloudType::Ptr cloud(new CloudType());
    filter_num = std::max(1, filter_num);

    // 万集点云没有 normal_x/y/z 字段；直接 fromROSMsg 到 PointXYZINormal
    // 会每帧打印 "Failed to find match for field 'normal_*'”，并污染日志。
    // 这里按它实际有的 x/y/z/intensity 读取，再拷贝到核心点类型，
    // normal 字段保持默认零值。
    bool has_intensity = false;
    for (const auto& field : msg->fields) {
        if (field.name == "intensity") { has_intensity = true; break; }
    }
    if (has_intensity) {
        pcl::PointCloud<pcl::PointXYZI> tmp;
        pcl::fromROSMsg(*msg, tmp);
        cloud->resize(tmp.size());
        for (size_t i = 0; i < tmp.size(); ++i) {
            PointType& dst = cloud->points[i];
            dst.x = tmp.points[i].x;
            dst.y = tmp.points[i].y;
            dst.z = tmp.points[i].z;
            dst.intensity = tmp.points[i].intensity;
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
    } else {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        pcl::fromROSMsg(*msg, tmp);
        cloud->resize(tmp.size());
        for (size_t i = 0; i < tmp.size(); ++i) {
            PointType& dst = cloud->points[i];
            dst.x = tmp.points[i].x;
            dst.y = tmp.points[i].y;
            dst.z = tmp.points[i].z;
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
    }

    // ⚠ **逐点时间必须手动补到 curvature** —— 核心的运动畸变补偿(IMUProcessor::undistort)用的是
    //    point.curvature/1000 = 该点相对帧起始的秒数, 并要求点按时间升序。
    //    pcl::fromROSMsg 不认识万集驱动的 `timestamp` 字段, 会把它丢掉 -> curvature 全 0
    //    -> 去畸变完全不生效。实机表现正是"平移还行, 一旋转就叠图"(一帧 100ms 内转过的角度
    //    没有被补偿)。万集字段实测: x,y,z,intensity,ring, timestamp(FLOAT64, 相对 header.stamp
    //    的秒数, 0 -> 0.0999, 升序; ts_first_point=true 表示 header.stamp 取首点时间)。
    bool has_pt_time = false;
    for (const auto& f : msg->fields) {
        if (f.name == "timestamp" && f.datatype == sensor_msgs::PointField::FLOAT64) {
            has_pt_time = true;
            break;
        }
    }
    if (has_pt_time) {
        sensor_msgs::PointCloud2ConstIterator<double> t_it(*msg, "timestamp");
        size_t vi = 0;   // 第几个**有效**点 (跳采后仍保持时间升序)
        for (size_t i = 0; i < cloud->size(); ++i, ++t_it) {
            if (i % filter_num != 0) continue;
            cloud->points[i].curvature = static_cast<float>((*t_it) * 1000.0);   // ms
            ++vi;
        }
        (void)vi;
    }

    CloudType::Ptr out(new CloudType());
    out->reserve(cloud->size() / filter_num + 1);
    const double min2 = min_r * min_r, max2 = max_r * max_r;
    double curv_min = 1e9, curv_max = -1e9;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (i % filter_num != 0) continue;
        const auto& p = cloud->points[i];
        if (!pcl::isFinite(p)) continue;
        const double d2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
                          static_cast<double>(p.z) * p.z;
        if (d2 < min2 || d2 > max2) continue;
        if (p.curvature < curv_min) curv_min = p.curvature;
        if (p.curvature > curv_max) curv_max = p.curvature;
        out->push_back(p);
    }
    // 一次性核对日志: 确认逐点时间真的进来了 (0 表示没填上 -> 去畸变失效)
    {
        static int once = 0;
        if (once++ == 0 && !out->empty()) {
            std::cout << "[slam_engine] point-time check: src=" << msg->width << "x" << msg->height
                      << " kept=" << out->size()
                      << " has_timestamp_field=" << (has_pt_time ? 1 : 0)
                      << " curvature_ms=[" << curv_min << " .. " << curv_max << "]" << std::endl;
        }
    }
    return out;
}

void SlamEngine::onImuMsg(const sensor_msgs::Imu::ConstPtr& msg) {
    if (online_calib_enabled_) {
        std::lock_guard<std::mutex> lk(calib_mutex_);
        online_time_calib_.addImu(msg->header.stamp.toSec(), msg->angular_velocity.z);
        const auto & q = msg->orientation;
        if (std::abs(q.w) + std::abs(q.x) + std::abs(q.y) + std::abs(q.z) > 0.5) {
            last_imu_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            last_imu_stamp_ = msg->header.stamp.toSec();
            have_imu_yaw_ = true;
        }
    }
    IMUSample s;
    s.acc = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    s.gyro = V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.time = getSec(msg->header.stamp) + imu_time_offset_.load();
    if (input_seen_hook_) input_seen_hook_(false);
    if (algo_) algo_->onImu(s);
}

void SlamEngine::onLidarMsg(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!algo_) return;

    // FREE/idle 不跑算法，也不做昂贵的 PointCloud2 -> PCL 转换。
    // 仍调用 input_seen_hook_，保持旧接口的雷达频率统计/告警。
    if (algo_raw_ && algo_raw_->getPipelineMode() == "idle") {
        if (input_seen_hook_) input_seen_hook_(true);
        return;
    }

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
    // 与 /switch_map、/relocation 共用同一份实现 (换算只写一次)
    applyInitPose(*msg, /*zero_if_insane=*/false, /*force_z_zero=*/false);
}

void SlamEngine::requestRelocalization() {
    if (algo_raw_) algo_raw_->requestRelocalization();
}

std::string SlamEngine::dumpFlightRecorder(const std::string& reason) {
    return diag::FlightRecorder::instance().dump(reason);
}

void SlamEngine::applyInitPose(const geometry_msgs::PoseWithCovarianceStamped& in,
                               bool zero_if_insane, bool force_z_zero) {
    if (!algo_raw_) return;
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
        std::cout << "[slam_engine] no initial pose -> core map/IMU identity 0,0,0"
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
    } else if (computeBaseFrames(/*blocking=*/true) && base_imu_.valid) {
        const double yaw_b = std::atan2(p.rot(1, 0), p.rot(0, 0)) * 180 / M_PI;
        p = composeRight(p, base_imu_);
        if (force_z_zero) p.trans.z() = 0.0;
        const double yaw_a = std::atan2(p.rot(1, 0), p.rot(0, 0)) * 180 / M_PI;
        char b[256];
        std::snprintf(b, sizeof(b),
                      "[slam_engine] init pose converted base->imu: yaw_before=%.1fdeg yaw_after=%.1fdeg",
                      yaw_b, yaw_a);
        std::cout << b << std::endl;
    } else {
        if (force_z_zero) p.trans.z() = 0.0;
        ROS_WARN_STREAM("[slam_engine] base_link <- imu unavailable; init pose left in base frame");
    }
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
    RigidTransform imu_child;
    if (!imuFromChild(msg->child_frame_id, imu_child)) return;
    algo_->onOdom(composeRight(p, imu_child));
}

void SlamEngine::imuCB(const sensor_msgs::Imu::ConstPtr& msg) { onImuMsg(msg); }
void SlamEngine::lidarCB(const sensor_msgs::PointCloud2::ConstPtr& msg) { onLidarMsg(msg); }
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
    RigidTransform imu_child;
    if (!imuFromChild(msg->child_frame_id, imu_child)) return;
    algo_->onVisualOdometry(p.time, composeRight(p, imu_child));
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
    if (!computeBaseFrames(/*blocking=*/false) || !base_imu_.valid) return;
    p = composeRight(p, base_imu_);   // map<-base -> map<-imu
    cov = transformCovariance(cov, base_imu_.R);
    algo_->onAbsolute(p, cov);
}

void SlamEngine::onOutput(const SlamOutput& out) {
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

void SlamEngine::processCB(const ros::TimerEvent&) {
    if (base_) {
        while (base_->tryPopAndProcess()) {}
        // 定位模式的 LiDAR 由 SlamPipeline 内部 worker 处理，避免重定位阻塞 spinner。
    }
    for (auto& h : tick_hooks_) if (h) h();
}

bool SlamEngine::latestOutput(SlamOutput& out) const {
    std::lock_guard<std::mutex> lk(out_mutex_);
    if (!have_output_) return false;
    out = latest_;
    return true;
}

void SlamEngine::clearLatestOutput() {
    std::lock_guard<std::mutex> lk(out_mutex_);
    latest_ = SlamOutput();
    have_output_ = false;
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

bool SlamEngine::getRealtimeGrid(double resolution, GridInfo& out) {
    if (!algo_raw_) return false;
    return algo_raw_->getRealtimeGrid(resolution, out);
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

rosiwit_slam::CalibrationDecision SlamEngine::updateCalibration() {
    std::lock_guard<std::mutex> lk(calib_mutex_);
    const auto t = online_time_calib_.estimate();
    const auto y = yaw_calib_.estimate();
    const auto d = calib_controller_.update(t, y);
    imu_time_offset_.store(d.time_offset);
    return d;
}

rosiwit_slam::TimeCalibrationResult SlamEngine::onlineTimeCalibration() const {
    std::lock_guard<std::mutex> lk(calib_mutex_);
    return online_time_calib_.estimate();
}

rosiwit_slam::YawCalibrationResult SlamEngine::onlineYawCalibration() const {
    std::lock_guard<std::mutex> lk(calib_mutex_);
    return yaw_calib_.estimate();
}

std::string SlamEngine::pipelineMode() const {
    return algo_raw_ ? algo_raw_->getPipelineMode() : std::string();
}

rosiwit_slam::ILocalization::Status SlamEngine::localizationStatus() const {
    return algo_raw_ ? algo_raw_->localizationStatus() : rosiwit_slam::ILocalization::LOST;
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
