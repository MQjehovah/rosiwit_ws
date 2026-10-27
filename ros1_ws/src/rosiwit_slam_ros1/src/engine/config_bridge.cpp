// ============================================================================
// config_bridge.cpp — 把 legacy 配置变成核心能吃的配置
//
// 单一职责: ROS 参数 / legacy JSON / 核心 YAML → 一份注入给核心的 override YAML。
//   * initCalibrationConfig(): 在线标定参数
//   * loadInputConfig(): 输入话题、自过滤、可选输入开关
//   * prepareCoreConfig(): legacy sensor_config → 输入话题 → 外参 → 写 override YAML → 兼容诊断
// 外参本身怎么从 TF 算出来, 在 mount_frames.cpp; 这里只调用。
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
#include <cmath>
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

namespace {
Xz3dConfPaths defaultXz3dPathsImpl(ros::NodeHandle* pnh) {
    Xz3dConfPaths p;
    // 旧实现 (xz_3d_slam/CMakeLists.txt:24/26):
    //   install 分支  PROJECT_PATH = /opt/ros/melodic/share/xz_3d_slam
    //   其余分支      PROJECT_PATH = ${PROJECT_SOURCE_DIR}
    // 主配置恒为 PROJECT_PATH "/conf/slam_conf.json"。
    // ros::package::getPath 在 devel/overlay 下给源码目录、在 install 下给安装目录,
    // 两种情形都与旧实现对得上。
    const std::string pkg = ros::package::getPath("rosiwit_slam_ros1");
    if (pkg.empty()) {
        ROS_WARN("[slam_algo] ros::package::getPath('rosiwit_slam_ros1') failed; "
                 "legacy conf falls back to built-in defaults");
    }
    // Simulation / alternate sensor rigs can point at their own legacy JSON.
    std::string legacy_override;
    if (pnh) pnh->param<std::string>("legacy_conf_file", legacy_override, "");
    if (!legacy_override.empty()) {
        p.proj_conf_json = legacy_override;
    } else {
        // **优先读老包的那份**: 旧实现的主配置 = 编译期 PROJECT_PATH/conf/slam_conf.json,
        // 装机后就是 /opt/ros/melodic/share/xz_3d_slam/conf/slam_conf.json —— 那才是这台机器
        // 真正生效的配置 (运维可能改过, 且与我们随包的那份并不一致: 实测老包那份没有
        // laser_params 子树、x_new/z_new 也不同)。读不到才退回我们自己 conf/ 里的副本。
        const std::string old_pkg = "/opt/ros/melodic/share/xz_3d_slam";
        const std::string old_conf = old_pkg + "/conf/slam_conf.json";
        std::ifstream probe(old_conf);
        p.proj_conf_json = probe.good() ? old_conf : (pkg + "/conf/slam_conf.json");
    }
    std::cout << "[slam_algo] legacy main conf source: " << p.proj_conf_json << std::endl;
    // 旧实现只有一个主配置来源。T1 的 Xz3dConfig 留了 proj(优先)/pkg(兜底) 两级,
    // 这里让二者指向同一份文件。见计划 §5.6。
    p.pkg_conf_json = p.proj_conf_json;
    p.opt_sensor_json = "/opt/xzrobot/configs/sensor_config.json";
    p.opt_slam_json = "/opt/xzrobot/configs/3d_slam/slam_conf.json";
    return p;
}
}  // namespace

Xz3dConfPaths defaultXz3dPaths() { return defaultXz3dPathsImpl(nullptr); }

Xz3dConfPaths defaultXz3dPaths(ros::NodeHandle& pnh) { return defaultXz3dPathsImpl(&pnh); }

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
        ROS_ERROR_STREAM("[slam_algo] cannot write override yaml: " << out_path);
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
        ROS_WARN_STREAM("[slam_algo] compat diagnostics: cannot read base yaml (" << e.what() << ")");
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

// ---------- 配置装配的三个入口 (构造函数只负责按顺序调用) ----------
void SlamAlgorithm::initCalibrationConfig() {
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
}

void SlamAlgorithm::loadInputConfig(bool& vo_enabled, bool& odom_enabled,
                                        bool& absolute_enabled) {
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
    // 三个开关由参数带回调用方 (构造函数), 这里不再重复声明
    vo_enabled = odom_enabled = absolute_enabled = false;
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
                // 机身自过滤: mapping.frontend.self_filter.*
                const YAML::Node frontend =
                    (c["mapping"] && c["mapping"]["frontend"])
                        ? c["mapping"]["frontend"] : YAML::Node();
                const YAML::Node sf =
                    (frontend && frontend["self_filter"])
                        ? frontend["self_filter"] : YAML::Node();
                if (sf) {
                    cfg_.self_filter_enabled = sf["enabled"].as<bool>(cfg_.self_filter_enabled);
                    cfg_.self_x_min = sf["x_min"].as<double>(cfg_.self_x_min);
                    cfg_.self_x_max = sf["x_max"].as<double>(cfg_.self_x_max);
                    cfg_.self_y_min = sf["y_min"].as<double>(cfg_.self_y_min);
                    cfg_.self_y_max = sf["y_max"].as<double>(cfg_.self_y_max);
                    cfg_.self_z_min = sf["z_min"].as<double>(cfg_.self_z_min);
                    cfg_.self_z_max = sf["z_max"].as<double>(cfg_.self_z_max);
                }
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
}

void SlamAlgorithm::prepareCoreConfig() {
    // ---- ROS1 输入: 沿用机器人现有 sensor_config 决定输入话题; 外参/单位仍取核心 YAML ----
    {
        std::string model;
        nh_.param<std::string>("/machine/model", model, "");
        xz_cfg_ = Xz3dConfig::load(defaultXz3dPaths(pnh_), model);
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
        std::cout << "[slam_algo] sensor laser_type='" << lt << "' -> input topics: imu="
                  << cfg_.imu_topic << " lidar=" << cfg_.lidar_topic << std::endl;

        // ---- M8: 给核心注入 map_directory: "/" 的 override YAML ----
        std::string base = config_path_;
        // 某些现场 launch 会把 config_file 留在上一轮生成的 override 上; 不能把它当基底,
        // 否则新参数(如 self_filter / realtime_post_pipeline)永远写不进核心 YAML.
        if (base.empty() || base.find("compat_legacy.yaml") != std::string::npos ||
            base.find("core_extrinsic.yaml") != std::string::npos) {
            base = ros::package::getPath("rosiwit_slam_ros1") + "/config/default.yaml";
            if (config_path_.empty()) {
                ROS_WARN_STREAM("[slam_algo] ROS1 输入: ~config_file 未设置, 核心 YAML 回落到 "
                                << base);
            }
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
        const RigidTransform base_lidar = baseLidar();
        Eigen::Matrix3d r_bl_used = base_lidar.R;
        if (base_lidar.valid && (std::fabs(ex_roll) + std::fabs(ex_pitch) + std::fabs(ex_yaw) > 1e-9)) {
            const Eigen::Matrix3d R_ex =
                (Eigen::AngleAxisd(ex_yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(ex_pitch * M_PI / 180.0, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(ex_roll * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                    .toRotationMatrix();
            r_bl_used = R_ex * base_lidar.R;
            std::cout << "[compat] mount extra rpy(deg)=[" << ex_roll << " " << ex_pitch
                      << " " << ex_yaw << "] applied (experimental)" << std::endl;
        }
        config_path_ = writeXz3dOverrideYaml(base, r_il, t_il, have_extr,
                                              base_lidar.valid ? &r_bl_used : nullptr,
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
}

} // namespace rosiwit_slam_ros1
