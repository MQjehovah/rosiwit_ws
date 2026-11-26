// src/algorithms/localization/localization_pipeline.cpp
#include "algorithms/localization/localization_pipeline.h"

#include <malloc.h>
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/localization/maintainer/insert_gate.h"
#include "algorithms/localization/registration_metrics.h"
#include "algorithms/map_store/map_store_factory.h"
#include <unordered_map>


#include <cmath>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <sys/stat.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

#include "slam_core/diagnostics.h"
#include <sstream>

namespace rosiwit_slam {
using diag::FlightRecorder;

// ---- 诊断探针开关 (环境变量; 默认静默: 生产不刷屏、不写盘) ----
//   ROSIWIT_LOC_PROBE=1  : 逐帧 [LocProbe]/[RejectProbe]/[DeskewDiag]/[FitSweep]
//   ROSIWIT_DUMP_FRAME=1 : 首个好帧/坏帧转储 PCD 到 /tmp/slamdump{,_good} (离线对账)
// 回放/实车排查时给节点置位即可, 不用重编 (逐帧对账: /tmp/frame_lag.py).
static const bool kLocProbe = std::getenv("ROSIWIT_LOC_PROBE") != nullptr;
static const bool kDumpFrame = std::getenv("ROSIWIT_DUMP_FRAME") != nullptr;

namespace {
// 底图-only 匹配度用的 KD 树/目标云: 放在 .cpp 里(单实例), 避免头文件引 pcl
CloudType::Ptr g_base_only_target;         // 底图-only 裁剪缓存
pcl::KdTreeFLANN<PointType> g_base_only_kdtree;
bool g_base_only_valid = false;
int g_base_only_age = 0;

// map.pcd -> map.overlay.pcd (工作图文件路径): 工作图 = 底图(0.3) + overlay,
// 每个点 intensity = 持久度 w。map.pcd 只做原图备份, 永不修改 —— 回滚随时可行。
// 只有这一个工作文件 (全量, 1 小时落盘一次), 没有增量日志。
std::string workingMapPath(const std::string& map_path) {
    const std::string suffix = ".pcd";
    const std::string work_suffix = ".overlay.pcd";
    if (map_path.size() > work_suffix.size() &&
        map_path.compare(map_path.size() - work_suffix.size(), work_suffix.size(),
                         work_suffix) == 0) {
        return map_path;   // 已经是工作图, 不再叠一层 .overlay
    }
    if (map_path.size() > suffix.size() &&
        map_path.compare(map_path.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return map_path.substr(0, map_path.size() - suffix.size()) + work_suffix;
    }
    return map_path + work_suffix;
}

// 简单体素降采样 (底图-only 覆盖率用): 与 matcher 的口径一致即可, 不必完全相同
CloudType::Ptr voxelDownsample(const CloudType::Ptr& in, float voxel) {
    CloudType::Ptr out(new CloudType());
    std::unordered_map<uint64_t, int> seen;
    const float inv = 1.0f / std::max(0.05f, voxel);
    out->reserve(in->size() / 4 + 8);
    for (const auto& p : in->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int32_t ix = static_cast<int32_t>(std::floor(p.x * inv));
        const int32_t iy = static_cast<int32_t>(std::floor(p.y * inv));
        const int32_t iz = static_cast<int32_t>(std::floor(p.z * inv));
        const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(ix)) * 73856093ull) ^
                             (static_cast<uint64_t>(static_cast<uint32_t>(iy)) * 19349663ull) ^
                             (static_cast<uint64_t>(static_cast<uint32_t>(iz)) * 83492791ull);
        if (seen.find(key) != seen.end()) continue;
        seen.emplace(key, 0);
        out->push_back(p);
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    return out;
}
} // namespace

namespace {
#ifndef YAML_CPP_DISABLED
template <typename T>
T getv(const YAML::Node& n, const char* key, const T& def) {
    if (n && n[key]) { try { return n[key].as<T>(); } catch (...) {} }
    return def;
}
#endif

} // namespace

bool LocalizationPipeline::init(const std::string& config_path) {
#ifndef YAML_CPP_DISABLED
    YAML::Node root;
    try {
        root = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[LocPipeline] config load failed: " << e.what() << std::endl;
        return false;
    }
    YAML::Node loc = root["localization"];

    // 雷达 -> IMU 外参: ROS1 层在 override YAML 顶层注入 r_il(9)/t_il(3)。
    // 定位匹配前把扫描转到 IMU 系 (与建图前端一致); 没有则保持原样。
    if (root["r_il"] && root["r_il"].IsSequence() && root["r_il"].size() == 9) {
        Eigen::Matrix3d R;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R(r, c) = root["r_il"][r * 3 + c].as<double>();
            }
        }
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        if (root["t_il"] && root["t_il"].IsSequence() && root["t_il"].size() == 3) {
            t << root["t_il"][0].as<double>(), root["t_il"][1].as<double>(),
                 root["t_il"][2].as<double>();
        }
        m_lidar_to_imu.setIdentity();
        m_lidar_to_imu.block<3, 3>(0, 0) = R.cast<float>();
        m_lidar_to_imu.block<3, 1>(0, 3) = t.cast<float>();
        m_lidar_to_imu_valid = true;
        std::cout << "[LocPipeline] lidar->imu extrinsic t=[" << t.transpose()
                  << "], scan will be transformed to IMU frame" << std::endl;
    }

    // 自转中心 (转动瞬心) 在 base 系的 xy: 差速底盘的轮速模型假设 base 原点就是瞬心;
    // 若 base 原点不在轮轴中点上则会差一个固定偏移. 标定方法见 yaml 注释 (必须用自转数据).
    if (loc["rotation_center"] && loc["rotation_center"].IsSequence() &&
        loc["rotation_center"].size() >= 2) {
        m_rot_center.x() = loc["rotation_center"][0].as<double>(0.0);
        m_rot_center.y() = loc["rotation_center"][1].as<double>(0.0);
    }
    YAML::Node pipe = loc["pipeline"];
    m_relocalizer_name = getv<std::string>(pipe, "relocalizer", m_relocalizer_name);
    m_prior_name       = getv<std::string>(pipe, "motion_prior", m_prior_name);
    m_local_map_name   = getv<std::string>(pipe, "local_map", m_local_map_name);
    m_matcher_name     = getv<std::string>(pipe, "matcher", m_matcher_name);
    m_monitor_name     = getv<std::string>(pipe, "monitor", m_monitor_name);
    m_maintainer_name  = getv<std::string>(pipe, "map_maintainer", m_maintainer_name);
    // 工作图落盘周期 (分钟, <=0 = 关闭周期落盘); 现场临时 10 分钟验证
    m_working_map_save_min = getv<double>(pipe, "working_map_save_min", m_working_map_save_min);
    // 输出位姿限速 (安全防护): 超物理上限的位姿跳变按上限滑移, 不直通导航
    m_out_v_max  = getv<double>(pipe, "output_v_max", m_out_v_max);
    m_out_wz_max = getv<double>(pipe, "output_wz_max", m_out_wz_max);
    m_fusion_name      = getv<std::string>(pipe, "pose_fusion", m_fusion_name);
    if (!pipe || !pipe["pose_fusion"]) {
        // 兼容旧配置: pose_fusion 直接写在 localization 下 (设备 compat_legacy.yaml 就是这种)
        m_fusion_name = getv<std::string>(loc, "pose_fusion", m_fusion_name);
    }
    std::cout << "[LocPipeline] pose_fusion=" << m_fusion_name
              << (m_fusion_name == "direct" ? " (LIO relative observations ignored)" : "")
              << std::endl;
    {
        // 融合层参数 (localization.fusion.*), 只在 pose_fusion=eskf 时生效
        auto fc = loc["fusion"];
        if (fc) {
            m_fusion_cfg.gain_trans              = getv<double>(fc, "gain_trans", m_fusion_cfg.gain_trans);
            m_fusion_cfg.gravity                 = getv<double>(fc, "gravity", m_fusion_cfg.gravity);
            m_fusion_cfg.gravity_tol             = getv<double>(fc, "gravity_tol", m_fusion_cfg.gravity_tol);
            m_fusion_cfg.gain_rot                = getv<double>(fc, "gain_rot", m_fusion_cfg.gain_rot);
            m_fusion_cfg.max_correction_trans    = getv<double>(fc, "max_correction_trans", m_fusion_cfg.max_correction_trans);
            m_fusion_cfg.max_correction_rot_deg  = getv<double>(fc, "max_correction_rot_deg", m_fusion_cfg.max_correction_rot_deg);
            m_fusion_cfg.trans_cov_floor         = getv<double>(fc, "trans_cov_floor", m_fusion_cfg.trans_cov_floor);
            m_fusion_cfg.rot_cov_floor           = getv<double>(fc, "rot_cov_floor", m_fusion_cfg.rot_cov_floor);
            m_fusion_cfg.process_trans           = getv<double>(fc, "process_trans", m_fusion_cfg.process_trans);
            m_fusion_cfg.process_rot             = getv<double>(fc, "process_rot", m_fusion_cfg.process_rot);
            m_fusion_cfg.chi2_gate               = getv<double>(fc, "chi2_gate", m_fusion_cfg.chi2_gate);
            m_fusion_cfg.max_odom_speed          = getv<double>(fc, "max_odom_speed", m_fusion_cfg.max_odom_speed);
            m_fusion_cfg.reject_in_place_trans   = getv<bool>(fc, "reject_in_place_trans", m_fusion_cfg.reject_in_place_trans);
            m_fusion_cfg.in_place_yaw_step_deg   = getv<double>(fc, "in_place_yaw_step_deg", m_fusion_cfg.in_place_yaw_step_deg);
            m_fusion_cfg.slip_ratio_lo           = getv<double>(fc, "slip_ratio_lo", m_fusion_cfg.slip_ratio_lo);
            m_fusion_cfg.slip_ratio_hi           = getv<double>(fc, "slip_ratio_hi", m_fusion_cfg.slip_ratio_hi);
            m_fusion_cfg.slip_yaw_thresh_deg     = getv<double>(fc, "slip_yaw_thresh_deg", m_fusion_cfg.slip_yaw_thresh_deg);
            m_fusion_cfg.odom_weight_min         = getv<double>(fc, "odom_weight_min", m_fusion_cfg.odom_weight_min);
            m_fusion_cfg.rel_max_gap             = getv<double>(fc, "rel_max_gap", m_fusion_cfg.rel_max_gap);
            m_fusion_cfg.rel_max_step            = getv<double>(fc, "rel_max_step", m_fusion_cfg.rel_max_step);
            m_fusion_cfg.match_vel_r             = getv<double>(fc, "match_vel_r", m_fusion_cfg.match_vel_r);
        }
    }

    auto mc = loc["matcher"];
    if (mc) {
        m_matcher_cfg.voxel_resolution        = getv<double>(mc, "voxel_resolution", m_matcher_cfg.voxel_resolution);
        m_matcher_cfg.max_iterations          = getv<int>(mc, "max_iterations", m_matcher_cfg.max_iterations);
        m_matcher_cfg.min_match_points        = getv<int>(mc, "min_match_points", m_matcher_cfg.min_match_points);
        m_matcher_cfg.max_correspondence_dist = getv<double>(mc, "max_correspondence_dist", m_matcher_cfg.max_correspondence_dist);
        m_matcher_cfg.transformation_epsilon  = getv<double>(mc, "transformation_epsilon", m_matcher_cfg.transformation_epsilon);
        m_matcher_cfg.inlier_dist             = getv<double>(mc, "inlier_dist", m_matcher_cfg.inlier_dist);
        m_matcher_cfg.use_corr_decay          = getv<bool>(mc, "use_corr_decay", m_matcher_cfg.use_corr_decay);
        m_matcher_cfg.corr_dist_init          = getv<double>(mc, "corr_dist_init", m_matcher_cfg.corr_dist_init);
        m_matcher_cfg.corr_dist_min           = getv<double>(mc, "corr_dist_min", m_matcher_cfg.corr_dist_min);
        m_matcher_cfg.use_huber               = getv<bool>(mc, "use_huber", m_matcher_cfg.use_huber);
        m_matcher_cfg.huber_delta             = getv<double>(mc, "huber_delta", m_matcher_cfg.huber_delta);
        {
            const std::string rk = getv<std::string>(mc, "robust_kernel", "");
            if      (rk == "none")   m_matcher_cfg.robust_kernel = RobustKernel::NONE;
            else if (rk == "huber")  m_matcher_cfg.robust_kernel = RobustKernel::HUBER;
            else if (rk == "cauchy") m_matcher_cfg.robust_kernel = RobustKernel::CAUCHY;
            else if (rk == "tukey")  m_matcher_cfg.robust_kernel = RobustKernel::TUKEY;
            else if (rk == "gnc")    m_matcher_cfg.robust_kernel = RobustKernel::GNC;
            else if (!m_matcher_cfg.use_huber) m_matcher_cfg.robust_kernel = RobustKernel::NONE;
        }
        m_matcher_cfg.gnc_mu0 = getv<double>(mc, "gnc_mu0", m_matcher_cfg.gnc_mu0);
        m_matcher_cfg.gnc_decay = getv<double>(mc, "gnc_decay", m_matcher_cfg.gnc_decay);
        m_matcher_cfg.use_edge_feature        = getv<bool>(mc, "use_edge_feature", m_matcher_cfg.use_edge_feature);
        m_matcher_cfg.edge_line_ratio         = getv<double>(mc, "edge_line_ratio", m_matcher_cfg.edge_line_ratio);
        m_matcher_cfg.edge_valid_thr          = getv<double>(mc, "edge_valid_thr", m_matcher_cfg.edge_valid_thr);
        m_matcher_cfg.edge_min_extent         = getv<double>(mc, "edge_min_extent", m_matcher_cfg.edge_min_extent);
        m_matcher_cfg.edge_vertical_cos       = getv<double>(mc, "edge_vertical_cos", m_matcher_cfg.edge_vertical_cos);
        m_matcher_cfg.degeneracy_gate         = getv<bool>(mc, "degeneracy_gate", m_matcher_cfg.degeneracy_gate);
        m_matcher_cfg.degeneracy_eig_ratio    = getv<double>(mc, "degeneracy_eig_ratio", m_matcher_cfg.degeneracy_eig_ratio);
        m_matcher_cfg.min_correspondences     = getv<int>(mc, "min_correspondences", m_matcher_cfg.min_correspondences);
        m_matcher_cfg.dynamic_filter          = getv<bool>(mc, "dynamic_filter", m_matcher_cfg.dynamic_filter);
        m_matcher_cfg.dynamic_dist            = getv<double>(mc, "dynamic_dist", m_matcher_cfg.dynamic_dist);
        m_matcher_cfg.match_z_filter          = getv<bool>(mc, "match_z_filter", m_matcher_cfg.match_z_filter);
        m_matcher_cfg.match_z_min             = getv<double>(mc, "match_z_min", m_matcher_cfg.match_z_min);
        m_matcher_cfg.match_z_max             = getv<double>(mc, "match_z_max", m_matcher_cfg.match_z_max);
        m_matcher_cfg.min_coverage_points     = getv<int>(mc, "min_coverage_points", m_matcher_cfg.min_coverage_points);
        m_matcher_cfg.min_inliers_full_score  = getv<int>(mc, "min_inliers_full_score", m_matcher_cfg.min_inliers_full_score);
        m_matcher_cfg.ndt2d_z_min             = getv<double>(mc, "ndt2d_z_min", m_matcher_cfg.ndt2d_z_min);
        m_matcher_cfg.ndt2d_z_max             = getv<double>(mc, "ndt2d_z_max", m_matcher_cfg.ndt2d_z_max);
        m_matcher_cfg.ndt2d_cell              = getv<double>(mc, "ndt2d_cell", m_matcher_cfg.ndt2d_cell);
    }
    auto lmc = loc["local_map"];
    if (lmc) {
        m_local_map_cfg.voxel_resolution = getv<double>(lmc, "voxel_resolution", m_local_map_cfg.voxel_resolution);
        m_local_map_cfg.radius           = getv<double>(lmc, "radius", m_local_map_cfg.radius);
        m_local_map_cfg.cell_size        = getv<double>(lmc, "cell_size", m_local_map_cfg.cell_size);
        m_local_map_cfg.max_keyframes    = getv<int>(lmc, "max_keyframes", m_local_map_cfg.max_keyframes);
        m_local_map_cfg.cache_move       = getv<double>(lmc, "cache_move", m_local_map_cfg.cache_move);
        m_local_map_cfg.anchor_hold        = getv<bool>(lmc, "anchor_hold", m_local_map_cfg.anchor_hold);
        m_local_map_cfg.anchor_max_lag     = getv<double>(lmc, "anchor_max_lag", m_local_map_cfg.anchor_max_lag);
        m_local_map_cfg.anchor_min_fitness = getv<double>(lmc, "anchor_min_fitness", m_local_map_cfg.anchor_min_fitness);
        m_local_map_cfg.anchor_min_eig_ratio =
            getv<double>(lmc, "anchor_min_eig_ratio", m_local_map_cfg.anchor_min_eig_ratio);
    }
    auto dkc = loc["deskew"];
    if (dkc) {
        m_deskew_enabled       = getv<bool>(dkc, "enabled", m_deskew_enabled);
        m_deskew_max_gap       = getv<double>(dkc, "max_gap", m_deskew_max_gap);
        m_deskew_min_angle_deg = getv<double>(dkc, "min_angle_deg", m_deskew_min_angle_deg);
    }
    auto mpc = loc["motion_prior"];
    if (mpc) {
        m_motion_cfg.max_dt  = getv<double>(mpc, "max_dt", m_motion_cfg.max_dt);
        m_motion_cfg.max_vel = getv<double>(mpc, "max_vel", m_motion_cfg.max_vel);
        m_motion_cfg.use_odom = getv<bool>(mpc, "use_odom", m_motion_cfg.use_odom);
        m_motion_cfg.max_odom_speed =
            getv<double>(mpc, "max_odom_speed", m_motion_cfg.max_odom_speed);
        m_motion_cfg.ekf_q_v     = getv<double>(mpc, "ekf_q_v", m_motion_cfg.ekf_q_v);
        m_motion_cfg.ekf_r_odom  = getv<double>(mpc, "ekf_r_odom", m_motion_cfg.ekf_r_odom);
        m_motion_cfg.ekf_r_match = getv<double>(mpc, "ekf_r_match", m_motion_cfg.ekf_r_match);
        m_motion_cfg.ekf_chi2    = getv<double>(mpc, "ekf_chi2", m_motion_cfg.ekf_chi2);
        m_motion_cfg.slip_yaw_thresh = getv<double>(mpc, "slip_yaw_thresh", m_motion_cfg.slip_yaw_thresh);
        m_motion_cfg.imu_calib_samples = getv<int>(mpc, "imu_calib_samples", m_motion_cfg.imu_calib_samples);
        m_motion_cfg.static_gyro_std = getv<double>(mpc, "static_gyro_std", m_motion_cfg.static_gyro_std);
        m_motion_cfg.static_accel_std = getv<double>(mpc, "static_accel_std", m_motion_cfg.static_accel_std);
        m_motion_cfg.gyro_bias_gain = getv<double>(mpc, "gyro_bias_gain", m_motion_cfg.gyro_bias_gain);
        m_motion_cfg.gyro_bias_max = getv<double>(mpc, "gyro_bias_max", m_motion_cfg.gyro_bias_max);
        m_motion_cfg.acc_bias_gain = getv<double>(mpc, "acc_bias_gain", m_motion_cfg.acc_bias_gain);
        m_motion_cfg.acc_bias_max = getv<double>(mpc, "acc_bias_max", m_motion_cfg.acc_bias_max);
        m_motion_cfg.use_gravity = getv<bool>(mpc, "use_gravity", m_motion_cfg.use_gravity);
        m_motion_cfg.gravity = getv<double>(mpc, "gravity", m_motion_cfg.gravity);
        m_motion_cfg.gravity_gain = getv<double>(mpc, "gravity_gain", m_motion_cfg.gravity_gain);
    }
    // ---- IMU 安装姿态 → up_body (统一入口, 见 slam_types.h: imuYawRate) ----
    // 安装外参可能来自 TF/URDF/启动参数; 由上层解析后写成下面任一参数即可.
    // 都没有时 fusion/motion_prior 用加速度自动估计 up_body, 不依赖写死的安装角.
    {
        const char* mount_keys[] = {"mount_r_base_lidar", "bbs_mount_r"};
        for (const char* key : mount_keys) {
            const YAML::Node n = root[key];
            if (n && n.IsSequence() && n.size() == 9) {
                Eigen::Matrix3d R;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) R(r, c) = n[r * 3 + c].as<double>();
                const V3D up = upBodyFromMount(R);
                m_fusion_cfg.up_body = up;
                m_fusion_cfg.up_body_valid = true;
                m_motion_cfg.up_body = up;
                m_motion_cfg.up_body_valid = true;
                m_matcher_cfg.crop_r = R;
                m_matcher_cfg.crop_r_valid = true;
                m_base_r = R;
                m_base_r_valid = true;
                std::cout << "[LocPipeline] imu up_body from yaml '" << key << "' = ["
                          << up.x() << ", " << up.y() << ", " << up.z() << "]" << std::endl;
                break;
            }
        }
    }
    // ---- 地面约束开关: localization.planar.mode = none | mount_lock ----
    // mount_lock: 位姿只解 x/y/yaw, roll/pitch 保持安装外参, z 锁安装高度.
    // (scan_plane/lock_zrp 仍由 ROS1 层 PlanarConstraint 处理; 本开关只控制核心侧)
    {
        const YAML::Node pl = loc["planar"];
        if (pl && pl["mode"]) {
            const std::string pm = pl["mode"].as<std::string>("none");
            // lock_zrp: 先扣安装角 -> 锁 body 的 z/roll/pitch -> 再贴回安装角.
            // (旧实现直接令 roll=pitch=0, 等于把雷达安装角抹掉, 对俯装/侧装 IMU 是错的)
            m_fix_planar = (pm == "lock_zrp" || pm == "mount_lock");
        }
        if (pl && pl["z"]) m_planar_z = pl["z"].as<double>(0.0);
    }
    // ---- 定位 odom: 单一开关 localization.odom_mode (none | fuse) ----
    // fuse 同时打开两处 odom 输入, 不再需要单独配 motion_prior.use_odom:
    //   1) motion_prior: 平移/朝向预测 (imu / imu_ekf)
    //   2) pose_fusion=eskf: odom 速度观测 (打滑门控后连续降权)
    // 没有 odom_mode 的旧配置仍按 motion_prior.use_odom 兼容.
    {
        const bool legacy_use_odom = m_motion_cfg.use_odom;
        const YAML::Node om_node = loc["odom_mode"];
        if (om_node) {
            const std::string om = om_node.as<std::string>("none");
            if (om == "fuse") {
                m_odom_fuse = true;
            } else if (om == "none" || om.empty()) {
                m_odom_fuse = false;
            } else {
                std::cerr << "[LocPipeline] unknown localization.odom_mode '" << om
                          << "' (none|fuse); using none" << std::endl;
                m_odom_fuse = false;
            }
        } else {
            m_odom_fuse = legacy_use_odom;
            if (legacy_use_odom) {
                std::cout << "[LocPipeline] DEPRECATED: localization.motion_prior.use_odom=true; "
                             "use localization.odom_mode: fuse" << std::endl;
            }
        }
        // 先验内部也走同一个开关, 避免"pipeline 关了、先验还在用"的双开关状态.
        m_motion_cfg.use_odom = m_odom_fuse;
        std::cout << "[LocPipeline] odom_mode=" << (m_odom_fuse ? "fuse" : "none")
                  << " (motion_prior.use_odom=" << m_motion_cfg.use_odom
                  << ", pose_fusion=" << m_fusion_name << ")";
        if (m_odom_fuse && m_fusion_name == "direct") {
            std::cout << " -> odom feeds motion_prior only "
                         "(pose_fusion=direct ignores it)";
        }
        std::cout << std::endl;
    }
    const YAML::Node common_sc =
        (root["global"] && root["global"]["scan_context"])
            ? root["global"]["scan_context"] : root["scan_context"];
    if (common_sc) {
        m_global_cfg.sc_num_rings = getv<int>(common_sc, "num_rings", m_global_cfg.sc_num_rings);
        m_global_cfg.sc_num_sectors = getv<int>(common_sc, "num_sectors", m_global_cfg.sc_num_sectors);
        m_global_cfg.sc_max_radius = getv<double>(common_sc, "max_radius", m_global_cfg.sc_max_radius);
        m_global_cfg.sc_dist_thresh = getv<double>(common_sc, "dist_thresh", m_global_cfg.sc_dist_thresh);
        m_global_cfg.sc_candidate_ratio = getv<double>(common_sc, "candidate_ratio", m_global_cfg.sc_candidate_ratio);
    }
    auto gc = loc["global"];
    if (gc) {
        m_global_cfg.min_score          = getv<double>(gc, "min_score", m_global_cfg.min_score);
        m_global_cfg.top_k              = getv<int>(gc, "top_k", m_global_cfg.top_k);
        m_global_cfg.reloc_submap_radius =
            getv<double>(gc, "reloc_submap_radius", m_global_cfg.reloc_submap_radius);
        m_global_cfg.reloc_accept_score =
            getv<double>(gc, "reloc_accept_score", m_global_cfg.reloc_accept_score);
        m_global_cfg.seed_max_drift =
            getv<double>(gc, "seed_max_drift", m_global_cfg.seed_max_drift);
        m_global_cfg.seed_max_yaw_deg =
            getv<double>(gc, "seed_max_yaw_deg", m_global_cfg.seed_max_yaw_deg);
        m_global_cfg.seed_neighbor_check =
            getv<bool>(gc, "seed_neighbor_check", m_global_cfg.seed_neighbor_check);
        m_global_cfg.seed_check_trans_m =
            getv<double>(gc, "seed_check_trans_m", m_global_cfg.seed_check_trans_m);
        m_global_cfg.seed_check_yaw_deg =
            getv<double>(gc, "seed_check_yaw_deg", m_global_cfg.seed_check_yaw_deg);
        m_global_cfg.seed_check_margin =
            getv<double>(gc, "seed_check_margin", m_global_cfg.seed_check_margin);
        m_ambiguity_margin = getv<double>(gc, "ambiguity_margin", m_ambiguity_margin);
        m_reseed_radius    = getv<double>(gc, "reseed_radius", m_reseed_radius);
        m_global_cfg.retry_interval     = getv<int>(gc, "retry_interval", m_global_cfg.retry_interval);
        m_global_cfg.voxel_resolution   = getv<double>(gc, "voxel_resolution", m_global_cfg.voxel_resolution);
        m_global_cfg.icp_max_corr_dist  = getv<double>(gc, "icp_max_corr_dist", m_global_cfg.icp_max_corr_dist);
        m_global_cfg.icp_fitness_thresh = getv<double>(gc, "icp_fitness_thresh", m_global_cfg.icp_fitness_thresh);
        m_global_cfg.icp_max_iter       = getv<int>(gc, "icp_max_iter", m_global_cfg.icp_max_iter);
        m_global_cfg.sc_num_rings       = getv<int>(gc, "sc_num_rings", m_global_cfg.sc_num_rings);
        m_global_cfg.sc_num_sectors     = getv<int>(gc, "sc_num_sectors", m_global_cfg.sc_num_sectors);
        m_global_cfg.sc_max_radius      = getv<double>(gc, "sc_max_radius", m_global_cfg.sc_max_radius);
        m_global_cfg.sc_min_radius      = gc["min_radius"]
                                              ? getv<double>(gc, "min_radius", m_global_cfg.sc_min_radius)
                                              : getv<double>(gc, "sc_min_radius", m_global_cfg.sc_min_radius);
        m_global_cfg.sc_dist_thresh     = getv<double>(gc, "sc_dist_thresh", m_global_cfg.sc_dist_thresh);
        m_global_cfg.sc_candidate_ratio = getv<double>(gc, "sc_candidate_ratio", m_global_cfg.sc_candidate_ratio);
        m_global_cfg.sc_z_cap           = getv<double>(gc, "sc_z_cap", m_global_cfg.sc_z_cap);
        m_global_cfg.sc_min_overlap_cells = getv<int>(gc, "sc_min_overlap_cells", m_global_cfg.sc_min_overlap_cells);
        m_global_cfg.sc_db_radius       = getv<double>(gc, "sc_db_radius", m_global_cfg.sc_db_radius);
        m_global_cfg.sc_db_spacing      = getv<double>(gc, "sc_db_spacing", m_global_cfg.sc_db_spacing);
        m_global_cfg.sc_db_max_places   = getv<int>(gc, "sc_db_max_places", m_global_cfg.sc_db_max_places);
        m_global_cfg.sc_db_from_map     = getv<bool>(gc, "sc_db_from_map", m_global_cfg.sc_db_from_map);
        m_global_cfg.bbs_z_min          = getv<double>(gc, "bbs_z_min", m_global_cfg.bbs_z_min);
                // 安装矩阵由 ROS 层注入在 override YAML 的**顶层**(与 r_il/map_directory 同级);
    // 这里同时接受 localization.global 与顶层两种位置(顶层优先取不到时回退)。
    YAML::Node mr_node = gc["bbs_mount_r"];
    if (!mr_node) {
        try {
            YAML::Node root = YAML::LoadFile(config_path);
            if (root["bbs_mount_r"]) mr_node = root["bbs_mount_r"];
        } catch (const std::exception&) {}
    }
    if (mr_node && mr_node.size() == 9) {
            for (size_t bi = 0; bi < 9; ++bi)
                m_global_cfg.bbs_mount_r[bi] = mr_node[bi].as<double>();
        } else {
            m_global_cfg.bbs_mount_r = {{1,0,0,0,1,0,0,0,1}};
        }
m_global_cfg.bbs_mount_roll_deg = getv<double>(gc, "bbs_mount_roll_deg", m_global_cfg.bbs_mount_roll_deg);
        m_global_cfg.bbs_mount_pitch_deg = getv<double>(gc, "bbs_mount_pitch_deg", m_global_cfg.bbs_mount_pitch_deg);
        m_global_cfg.bbs_mount_yaw_deg = getv<double>(gc, "bbs_mount_yaw_deg", m_global_cfg.bbs_mount_yaw_deg);
        m_global_cfg.bbs_z_max          = getv<double>(gc, "bbs_z_max", m_global_cfg.bbs_z_max);
        m_global_cfg.bbs_theta_bins     = getv<int>(gc, "bbs_theta_bins", m_global_cfg.bbs_theta_bins);
        m_global_cfg.bbs_max_levels     = getv<int>(gc, "bbs_max_levels", m_global_cfg.bbs_max_levels);
        m_global_cfg.bbs_min_points_per_cell = getv<double>(gc, "bbs_min_points_per_cell", m_global_cfg.bbs_min_points_per_cell);
        m_global_cfg.fpfh_normal_radius  = getv<double>(gc, "fpfh_normal_radius", m_global_cfg.fpfh_normal_radius);
        m_global_cfg.fpfh_feature_radius = getv<double>(gc, "fpfh_feature_radius", m_global_cfg.fpfh_feature_radius);
        m_global_cfg.fpfh_voxel_resolution =
            getv<double>(gc, "fpfh_voxel_resolution", m_global_cfg.fpfh_voxel_resolution);
        m_global_cfg.fpfh_threads        = getv<int>(gc, "fpfh_threads", m_global_cfg.fpfh_threads);
        m_global_cfg.fpfh_max_iter       = getv<int>(gc, "fpfh_max_iter", m_global_cfg.fpfh_max_iter);
        m_global_cfg.fpfh_inlier_fraction = getv<double>(gc, "fpfh_inlier_fraction", m_global_cfg.fpfh_inlier_fraction);
        m_global_cfg.fpfh_similarity     = getv<double>(gc, "fpfh_similarity", m_global_cfg.fpfh_similarity);
    }
    auto moc = loc["monitor"];
    if (moc) {
        m_monitor_cfg.enabled            = getv<bool>(moc, "enabled", m_monitor_cfg.enabled);
        m_monitor_cfg.credible_fitness   = getv<double>(moc, "credible_fitness", m_monitor_cfg.credible_fitness);
        m_monitor_cfg.lost_fitness       = getv<double>(moc, "lost_fitness", m_monitor_cfg.lost_fitness);
        m_min_evidence_points = getv<int>(moc, "min_evidence_points", m_min_evidence_points);
        m_low_evidence_limit  = getv<int>(moc, "low_evidence_limit", m_low_evidence_limit);
        m_monitor_cfg.localized_patience = getv<int>(moc, "localized_patience", m_monitor_cfg.localized_patience);
        m_monitor_cfg.scrutiny_frames    = getv<int>(moc, "scrutiny_frames", m_monitor_cfg.scrutiny_frames);
        m_monitor_cfg.lost_patience      = getv<int>(moc, "lost_patience", m_monitor_cfg.lost_patience);
        m_consensus_trans_m   = getv<double>(moc, "consensus_trans_m", m_consensus_trans_m);
        m_consensus_yaw_deg   = getv<double>(moc, "consensus_yaw_deg", m_consensus_yaw_deg);
        m_monitor_cfg.nees_threshold     = getv<double>(moc, "nees_threshold", m_monitor_cfg.nees_threshold);
        m_monitor_cfg.window_length      = getv<int>(moc, "window_length", m_monitor_cfg.window_length);
        m_monitor_cfg.window_std_k       = getv<double>(moc, "window_std_k", m_monitor_cfg.window_std_k);
        m_monitor_cfg.max_trans_step     = getv<double>(moc, "max_trans_step", m_monitor_cfg.max_trans_step);
        m_monitor_cfg.max_yaw_step_deg   = getv<double>(moc, "max_yaw_step_deg", m_monitor_cfg.max_yaw_step_deg);
    }
    auto ac = loc["accept"];
    if (ac) {
        m_accept_cfg.mode              = getv<int>(ac, "mode", m_accept_cfg.mode);
        m_accept_cfg.min_coverage      = getv<double>(ac, "min_coverage", m_accept_cfg.min_coverage);
        m_accept_cfg.residual_margin   = getv<double>(ac, "residual_margin", m_accept_cfg.residual_margin);
        m_accept_cfg.consistent_frames = getv<int>(ac, "consistent_frames", m_accept_cfg.consistent_frames);
        m_accept_cfg.mahalanobis_gate = getv<bool>(ac, "mahalanobis_gate", m_accept_cfg.mahalanobis_gate);
        m_accept_cfg.mahalanobis_chi2 = getv<double>(ac, "mahalanobis_chi2", m_accept_cfg.mahalanobis_chi2);
        m_accept_cfg.mahalanobis_trans_floor = getv<double>(ac, "mahalanobis_trans_floor", m_accept_cfg.mahalanobis_trans_floor);
        m_accept_cfg.mahalanobis_rot_floor = getv<double>(ac, "mahalanobis_rot_floor", m_accept_cfg.mahalanobis_rot_floor);
        m_accept_cfg.mahalanobis_ref_points = getv<int>(ac, "mahalanobis_ref_points", m_accept_cfg.mahalanobis_ref_points);
        m_accept_cfg.mahalanobis_coast_widen = getv<double>(ac, "mahalanobis_coast_widen", m_accept_cfg.mahalanobis_coast_widen);
        m_accept_cfg.max_trans_innov         = getv<double>(ac, "max_trans_innov", m_accept_cfg.max_trans_innov);
        m_accept_cfg.degeneracy_partial_update = getv<bool>(ac, "degeneracy_partial_update", m_accept_cfg.degeneracy_partial_update);
        m_accept_cfg.degeneracy_ratio_soft = getv<double>(ac, "degeneracy_ratio_soft", m_accept_cfg.degeneracy_ratio_soft);
        m_accept_cfg.escape_enabled        = getv<bool>(ac, "escape_enabled", m_accept_cfg.escape_enabled);
        m_accept_cfg.escape_frames         = getv<int>(ac, "escape_frames", m_accept_cfg.escape_frames);
        m_accept_cfg.escape_min_coast      = getv<int>(ac, "escape_min_coast", m_accept_cfg.escape_min_coast);
        m_accept_cfg.escape_consist_trans  = getv<double>(ac, "escape_consist_trans", m_accept_cfg.escape_consist_trans);
        m_accept_cfg.escape_consist_rot_deg = getv<double>(ac, "escape_consist_rot_deg", m_accept_cfg.escape_consist_rot_deg);
        m_accept_cfg.escape_max_spread     = getv<double>(ac, "escape_max_spread", m_accept_cfg.escape_max_spread);
        m_accept_cfg.escape_max_jump       = getv<double>(ac, "escape_max_jump", m_accept_cfg.escape_max_jump);
        m_accept_cfg.escape_cooldown_s     = getv<double>(ac, "escape_cooldown_s", m_accept_cfg.escape_cooldown_s);
    }
    auto mtc = loc["maintainer"];
    if (mtc) {
        m_maintainer_cfg.enabled          = getv<bool>(mtc, "enabled", m_maintainer_cfg.enabled);
        m_maintainer_cfg.fitness_trigger  = getv<double>(mtc, "fitness_trigger", m_maintainer_cfg.fitness_trigger);
        m_maintainer_cfg.min_keyframe_dist = getv<double>(mtc, "min_keyframe_dist", m_maintainer_cfg.min_keyframe_dist);
        m_maintainer_cfg.min_confirm      = getv<int>(mtc, "min_confirm", m_maintainer_cfg.min_confirm);
        m_maintainer_cfg.radius           = getv<double>(mtc, "radius", m_maintainer_cfg.radius);
        m_maintainer_cfg.voxel            = getv<double>(mtc, "voxel", m_maintainer_cfg.voxel);
        m_maintainer_cfg.cell             = getv<double>(mtc, "cell", m_maintainer_cfg.cell);
        m_maintainer_cfg.persist_hits     = getv<int>(mtc, "persist_hits", m_maintainer_cfg.persist_hits);
        m_maintainer_cfg.refresh_frames   = getv<int>(mtc, "refresh_frames", m_maintainer_cfg.refresh_frames);
        m_maintainer_cfg.decay_frames     = getv<int>(mtc, "decay_frames", m_maintainer_cfg.decay_frames);
        m_maintainer_cfg.override_base    = getv<bool>(mtc, "override_base", m_maintainer_cfg.override_base);
        m_maintainer_cfg.delta_min        = getv<double>(mtc, "delta_min", m_maintainer_cfg.delta_min);
        m_maintainer_cfg.novel_min_cluster = getv<int>(mtc, "novel_min_cluster", m_maintainer_cfg.novel_min_cluster);
        m_maintainer_cfg.carve_stride     = getv<int>(mtc, "carve_stride", m_maintainer_cfg.carve_stride);
        m_maintainer_cfg.target_w_min     = getv<int>(mtc, "target_w_min", m_maintainer_cfg.target_w_min);
        m_maintainer_cfg.base_voxel_w     = getv<int>(mtc, "base_voxel_w", m_maintainer_cfg.base_voxel_w);
        m_maintainer_cfg.pose_graph       = getv<bool>(mtc, "pose_graph", m_maintainer_cfg.pose_graph);
        m_maintainer_cfg.pg_min_keyframes = getv<int>(mtc, "pg_min_keyframes", m_maintainer_cfg.pg_min_keyframes);
        m_maintainer_cfg.max_voxels       = getv<int>(mtc, "max_voxels", m_maintainer_cfg.max_voxels);
        m_maintainer_cfg.max_insert_divergence =
            getv<double>(mtc, "max_insert_divergence", m_maintainer_cfg.max_insert_divergence);
        // A 阶段: 体素可信度 w (影子: 只统计, 不参与决策)
        m_maintainer_cfg.persistency_anchor = getv<int>(mtc, "persistency_anchor", m_maintainer_cfg.persistency_anchor);
        m_maintainer_cfg.shadow   = getv<bool>(mtc, "shadow", m_maintainer_cfg.shadow);
        m_maintainer_cfg.match_w_min = getv<int>(mtc, "match_w_min", m_maintainer_cfg.match_w_min);
        m_maintainer_cfg.max_rmse = getv<double>(mtc, "max_rmse", m_maintainer_cfg.max_rmse);
        const std::string wu = getv<std::string>(mtc, "persistency_unit", "session");
        if (wu == "frame")        m_maintainer_cfg.persistency_unit = 0;
        else if (wu == "view")    m_maintainer_cfg.persistency_unit = 1;
        else if (wu == "session") m_maintainer_cfg.persistency_unit = 2;
        else if (wu == "day")     m_maintainer_cfg.persistency_unit = 3;
        m_maintainer_cfg.persistency_hit_gain      = getv<int>(mtc, "persistency_hit_gain", m_maintainer_cfg.persistency_hit_gain);
        m_maintainer_cfg.persistency_miss_confirm  = getv<int>(mtc, "persistency_miss_confirm", m_maintainer_cfg.persistency_miss_confirm);
        m_maintainer_cfg.persistency_miss_action   = getv<int>(mtc, "persistency_miss_action", m_maintainer_cfg.persistency_miss_action);
        m_maintainer_cfg.persistency_session_hours = getv<int>(mtc, "persistency_session_hours", m_maintainer_cfg.persistency_session_hours);
    }
    // ---- B 阶段: 突变场景滚动子图 (shadow) ----
    auto subc = loc["submap"];
    if (subc) {
        m_session_map_enabled = getv<bool>(subc, "enabled", m_session_map_enabled);
        m_anchor_min_inliers = getv<int>(subc, "anchor_min_inliers", m_anchor_min_inliers);
        m_anchor_min_ratio = getv<double>(subc, "anchor_min_ratio", m_anchor_min_ratio);
        m_anchor_min_eig_ratio = getv<double>(subc, "anchor_min_eig_ratio", m_anchor_min_eig_ratio);
        m_anchor_patience = getv<int>(subc, "anchor_patience", m_anchor_patience);
        m_anchor_w_support_min = getv<double>(subc, "anchor_w_support_min", m_anchor_w_support_min);
        m_anchor_close_patience =
            getv<int>(subc, "anchor_close_patience", m_anchor_close_patience);
        m_session_map_radius = getv<double>(subc, "radius_m", m_session_map_radius);
        m_session_map_match_radius =
            getv<double>(subc, "match_radius_m", m_session_map_match_radius);
        m_session_map_voxel = getv<double>(subc, "voxel", m_session_map_voxel);
    }
    // ★2026-11-24 修正: radius_m / voxel / match_radius_m 在
    // yaml 里写在 `localization.session_map`, 但这里原来只读 `submap` 段 ->
    // 一直是代码默认值 (radius 25 / voxel 0.15), 改 yaml 根本不生效.
    if (auto smc = loc["session_map"]) {
        m_session_map_enabled = getv<bool>(smc, "enabled", m_session_map_enabled);
        m_session_map_radius = getv<double>(smc, "radius_m", m_session_map_radius);
        m_session_map_voxel = getv<double>(smc, "voxel", m_session_map_voxel);
        m_session_map_match_radius =
            getv<double>(smc, "match_radius_m", m_session_map_match_radius);
        m_sm_write_fitness = getv<double>(smc, "write_fitness_bridging", m_sm_write_fitness);
        m_sm_write_corr_m = getv<double>(smc, "write_max_corr_m", m_sm_write_corr_m);
        m_sm_write_corr_deg = getv<double>(smc, "write_max_corr_deg", m_sm_write_corr_deg);
    }
    m_session_map.configure(SessionMap::Config{
        m_session_map_radius, m_session_map_voxel, m_session_map_max_points});
    auto plc = loc["planar"];
    if (plc) {
        m_planar_cfg.mode          = getv<std::string>(plc, "mode", m_planar_cfg.mode);
        m_planar_cfg.z             = getv<double>(plc, "z", m_planar_cfg.z);
        m_planar_cfg.sensor_height = getv<double>(plc, "sensor_height", m_planar_cfg.sensor_height);
        m_planar_cfg.cell          = getv<double>(plc, "cell", m_planar_cfg.cell);
        m_planar_cfg.ground_band   = getv<double>(plc, "ground_band", m_planar_cfg.ground_band);
        m_planar_cfg.max_tilt_deg  = getv<double>(plc, "max_tilt_deg", m_planar_cfg.max_tilt_deg);
        m_planar_cfg.max_correction_deg = getv<double>(plc, "max_correction_deg", m_planar_cfg.max_correction_deg);
    }
    auto hpc = loc["hypothesis"];
    if (hpc) {
        m_max_hypotheses = getv<int>(hpc, "max", m_max_hypotheses);
        m_hyp_margin     = getv<double>(hpc, "margin", m_hyp_margin);
        m_hyp_stale      = getv<int>(hpc, "stale", m_hyp_stale);
        m_hyp_reseed_frames = getv<int>(hpc, "reseed_frames", m_hyp_reseed_frames);
    }
    m_hyp.setMax(m_max_hypotheses);
    auto abc = loc["absolute"];
    if (abc) {
        m_abs_cfg.enabled   = getv<bool>(abc, "enabled", m_abs_cfg.enabled);
        m_abs_cfg.window    = getv<double>(abc, "window", m_abs_cfg.window);
        m_abs_cfg.alpha     = getv<double>(abc, "alpha", m_abs_cfg.alpha);
        m_abs_cfg.max_trans = getv<double>(abc, "max_trans", m_abs_cfg.max_trans);
        m_abs_cfg.max_yaw_deg = getv<double>(abc, "max_yaw_deg", m_abs_cfg.max_yaw_deg);
    }
    m_store_name = getv<std::string>(loc, "map_store", m_store_name);
    m_store_leaf = getv<double>(loc, "map_store_leaf", m_store_leaf);
    auto smc = loc["smoother"];
    if (smc) {
        m_smoother_cfg.enabled      = getv<bool>(smc, "enabled", m_smoother_cfg.enabled);
        m_smoother_cfg.lag_sec      = getv<double>(smc, "lag_sec", m_smoother_cfg.lag_sec);
        m_smoother_cfg.motion_trans_sigma = getv<double>(smc, "motion_trans_sigma", m_smoother_cfg.motion_trans_sigma);
        m_smoother_cfg.motion_rot_sigma   = getv<double>(smc, "motion_rot_sigma", m_smoother_cfg.motion_rot_sigma);
    }
#else
    (void)config_path;
#endif

    // 创建并配置各阶段
    m_relocalizer = LocalizationFactory::createGlobalLocalizer(m_relocalizer_name);
    m_prior       = LocalizationFactory::createMotionPrior(m_prior_name);
    m_local_map   = LocalizationFactory::createLocalMapProvider(m_local_map_name);
    m_matcher     = LocalizationFactory::createMatcher(m_matcher_name);
    m_monitor     = LocalizationFactory::createMonitor(m_monitor_name);
    m_fusion      = LocalizationFactory::createPoseFusion(m_fusion_name);

    // 飞行记录仪配置 (diagnostics: 分区; 2026-10-18 日志重构)
#ifndef YAML_CPP_DISABLED
    {
        const auto dg = root["diagnostics"];
        if (dg) {
            FlightRecorder::instance().configure(
                dg["flight_max_frames"].as<size_t>(3600),
                dg["flight_auto_dump_dir"].as<std::string>("/tmp/slam_flight"));
        } else {
            FlightRecorder::instance().configure(3600, "/tmp/slam_flight");
        }
    }
#endif
    diag::log('I', "LocPipeline", describe());

    if (!m_matcher) { std::cerr << "[LocPipeline] unknown matcher: " << m_matcher_name << std::endl; return false; }
    if (!m_prior)   { std::cerr << "[LocPipeline] unknown motion_prior: " << m_prior_name << std::endl; return false; }
    if (!m_local_map) { std::cerr << "[LocPipeline] unknown local_map: " << m_local_map_name << std::endl; return false; }
    if (!m_monitor) { std::cerr << "[LocPipeline] unknown monitor: " << m_monitor_name << std::endl; return false; }
    if (!m_relocalizer) { std::cerr << "[LocPipeline] unknown relocalizer: " << m_relocalizer_name << std::endl; return false; }
    if (!m_fusion) { std::cerr << "[LocPipeline] unknown pose_fusion: " << m_fusion_name << std::endl; return false; }

    m_matcher->configure(m_matcher_cfg);
    m_global_matcher = LocalizationFactory::createMatcher(m_matcher_name);
    if (m_global_matcher) m_global_matcher->configure(m_matcher_cfg);
    m_prior->configure(m_motion_cfg);
    m_local_map->configure(m_local_map_cfg);
    m_monitor->configure(m_monitor_cfg);
    m_relocalizer->configure(m_global_cfg);
    m_fusion->configure(m_fusion_cfg);

    // 配置不变量校验 (配置治理, 2026-10-18): 阈值联动关系被打破时立刻报警,
    // 而不是等到现场出现"僵尸带/弱接受循环"再反推 (2026-10-17 事故链的教训)。
    {
        auto warn = [](const std::string& msg) {
            diag::log('W', "LocPipeline", "config invariant: " + msg);
        };
        if (!(m_global_cfg.reloc_accept_score >= m_monitor_cfg.credible_fitness))
            warn("reloc_accept_score(" + std::to_string(m_global_cfg.reloc_accept_score) +
                 ") < credible_fitness(" + std::to_string(m_monitor_cfg.credible_fitness) +
                 ") -> 弱重定位会被接受后立即进入僵尸带");
        if (!(m_monitor_cfg.credible_fitness > m_accept_cfg.min_coverage))
            warn("credible_fitness(" + std::to_string(m_monitor_cfg.credible_fitness) +
                 ") <= accept.min_coverage(" + std::to_string(m_accept_cfg.min_coverage) +
                 ") -> 匹配被接受但健康计数永不增长");
        if (!(m_monitor_cfg.lost_fitness < m_monitor_cfg.credible_fitness))
            warn("lost_fitness >= credible_fitness -> 状态机区间为空");
        if (m_matcher_cfg.max_correspondence_dist < m_matcher_cfg.dynamic_dist * 2.0)
            warn("max_correspondence_dist < 2*dynamic_dist -> 动态滤波剪切大于匹配捕获范围");
    }
    m_planar.configure(m_planar_cfg);
    m_smoother.configure(m_smoother_cfg);
    if (m_store_name != "none") {
        m_store = MapStoreFactory::create(m_store_name, m_store_leaf);
        if (!m_store) std::cerr << "[LocPipeline] unknown map_store: " << m_store_name << std::endl;
    }

    if (m_maintainer_name != "none") {
        m_maintainer = LocalizationFactory::createMapMaintainer(m_maintainer_name);
        if (!m_maintainer) {
            std::cerr << "[LocPipeline] unknown map_maintainer: " << m_maintainer_name << std::endl;
            return false;
        }
        m_maintainer->configure(m_maintainer_cfg);
    }

    {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_stop = false;
        m_global_request = false;
        m_global_scan_valid = false;
    }
    if (!m_executor && !m_global_thread.joinable())
        m_global_thread = std::thread([this] { globalRelocWorker(); });
    std::cout << "[LocPipeline] " << describe() << std::endl;
    return true;
}

LocalizationPipeline::~LocalizationPipeline() {
    // 后台构建线程: 先停再收 (worker 可能正持有 45s 的构建)
    {
        std::lock_guard<std::mutex> lk(m_reloc_mu);
        m_reloc_stop = true;
    }
    m_reloc_cv.notify_all();
    if (m_reloc_thread.joinable()) m_reloc_thread.join();
    {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_stop = true;
        m_global_request = false;
        m_global_scan_valid = false;
    }
    m_global_cv.notify_all();
    if (m_global_thread.joinable()) m_global_thread.join();
    // 工作图 (map.overlay.pcd) 由 1 小时定时器全量落盘, 析构时不再单独写 overlay
    // (只剩一个工作文件, 没有增量日志)。
}

std::string LocalizationPipeline::describe() const {
    return "relocalizer=" + m_relocalizer_name +
           " motion_prior=" + m_prior_name +
           " local_map=" + m_local_map_name +
           " matcher=" + m_matcher_name +
           " monitor=" + m_monitor_name +
           " map_maintainer=" + m_maintainer_name +
           " pose_fusion=" + m_fusion_name +
           " odom_mode=" + (m_odom_fuse ? std::string("fuse") : std::string("none")) +
           " accept_mode=" + std::to_string(m_accept_cfg.mode);
}

void LocalizationPipeline::updateSnapshotLocked() {
    std::lock_guard<std::mutex> lk(m_snapshot_mu);
    m_snapshot_pose = m_pose;
    m_snapshot_match = m_last_match;
    m_snapshot_cov = m_last_match.cov;
    m_snapshot_confidence = m_last_match.fitness;
    m_snapshot_valid = m_initialized.load();
}

void LocalizationPipeline::setMap(const std::string& map_name) {
    setMapImpl(map_name);
}

void LocalizationPipeline::planarizePose(PoseStamped& p) const {
    // 地面机器人: body(雷达)->map 只允许 yaw 变化, roll/pitch 固定为安装姿态, z 固定为安装高度.
    // 竖直墙 + 无地面点地图 => z/roll/pitch 弱观测; 漂 5deg 就足以让 10m 外墙面点错开 ~0.9m.
    if (!m_fix_planar || !m_base_r_valid) return;
    const M3D R_base = p.rot * m_base_r.transpose();      // base_footprint -> map
    const double yaw = std::atan2(R_base(1, 0), R_base(0, 0));
    p.rot = Eigen::AngleAxisd(yaw, V3D::UnitZ()).toRotationMatrix() * m_base_r;
    if (m_fixed_z > 0.0) p.trans.z() = m_fixed_z;
}

void LocalizationPipeline::setMountExtrinsic(const Eigen::Matrix3d& r_base_lidar,
                                             const Eigen::Matrix3d& r_il,
                                             const Eigen::Vector3d& t_il,
                                             const Eigen::Vector3d& t_base_lidar) {
    // 安装外参只有一个来源 (URDF/TF/启动参数, 由 ROS 层解析). 这里统一落到:
    //   1) 扫描 lidar->imu 变换
    //   2) 偏航轴 up_body (slam_types.h: upBodyFromMount) —— fusion 与 motion_prior 共用
    //   3) 杆臂 lever_arm (slam_types.h: leverArmSensorFrame) —— 原地自转轨道项
    // 任何一处都不允许假设"雷达正装"。
    const V3D up = upBodyFromMount(r_base_lidar);
    // 杆臂从**自转中心**量起 (2026-11-23), 默认即 base 原点. 若底盘实际绕轮轴中点自转而
    // base 原点不在其上, 从原点量起会让轨道项每帧差 |偏移| × 帧内转角 (二阶量).
    // 偏移是标定值 (localization.rotation_center), 必须用机器人自转的数据标: 手动推的
    // 轨迹瞬心随手法漂移 (手动 bag 拟合出"后方 0.13m", 未采信, 见 yaml 注释).
    const V3D lever = leverArmSensorFrame(
        r_base_lidar, t_base_lidar - V3D(m_rot_center.x(), m_rot_center.y(), 0.0));
    {
        std::lock_guard<std::mutex> lk(m_prior_mu);
        m_motion_cfg.up_body = up;
        m_motion_cfg.up_body_valid = true;
        m_motion_cfg.lever_arm = lever;
        m_motion_cfg.lever_valid = true;
        m_motion_cfg.r_base_lidar = r_base_lidar;
        if (m_prior) m_prior->configure(m_motion_cfg);
    }
    {
        std::lock_guard<std::mutex> lk(m_fusion_mu);
        m_fusion_cfg.up_body = up;
        m_fusion_cfg.up_body_valid = true;
        m_fusion_cfg.lever_arm = lever;
        m_fusion_cfg.lever_valid = true;
        m_fusion_cfg.r_base_lidar = r_base_lidar;
        if (m_fusion) m_fusion->configure(m_fusion_cfg);
    }
    m_lidar_to_imu.setIdentity();
    m_lidar_to_imu.block<3, 3>(0, 0) = r_il.cast<float>();
    m_lidar_to_imu.block<3, 1>(0, 3) = t_il.cast<float>();
    m_lidar_to_imu_valid = true;
    // 高度带裁剪的坐标系同样来自安装外参 (lidar->base_footprint, 旋转+平移), 与位姿 z 解耦.
    // 平移项 = 雷达安装高度, 裁剪面才落在真实的离地 0.15m 而不是 0.99m.
    m_matcher_cfg.crop_r = r_base_lidar;
    m_matcher_cfg.crop_t = t_base_lidar;
    m_matcher_cfg.crop_r_valid = true;
    if (m_matcher) m_matcher->configure(m_matcher_cfg);
    if (m_global_matcher) m_global_matcher->configure(m_matcher_cfg);
    // 全局重定位描述子也要用同一个"水平系": 本机内部系是俯装的, ScanContext 只对
    // 绕 z 的旋转不变, 不转平的话同地点描述子也对不上(实测 best_d 0.77 > 阈值 0.4).
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            m_global_cfg.sc_level_r[r * 3 + c] = r_base_lidar(r, c);
    if (m_relocalizer) m_relocalizer->configure(m_global_cfg);
    m_base_r = r_base_lidar;
    m_base_r_valid = true;
    diag::log('I', "LocPipeline", "mount injected: up_body=(" + std::to_string(up.x()) + ", " +
                                      std::to_string(up.y()) + ", " + std::to_string(up.z()) +
                                      ") lever=(" + std::to_string(lever.x()) + ", " +
                                      std::to_string(lever.y()) + ", " + std::to_string(lever.z()) +
                                      ") rot_center=(" + std::to_string(m_rot_center.x()) + ", " +
                                      std::to_string(m_rot_center.y()) + ")");
}

void LocalizationPipeline::setMapImpl(const std::string& map_name) {
    // 换图会替换 local_map/matcher/store/maintainer; 与 onLidar 互斥.
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    m_cancel_set_map.store(false);
    // 分段计时: /switch_map 曾实测 48s, 必须知道钱花在哪 (每阶段一行, 只在切图时打)
    const auto t_total0 = std::chrono::steady_clock::now();
    auto stage_ms = [](const std::chrono::steady_clock::time_point& t) {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
    };
    std::string timing;
    auto mark = [&timing, &t_total0, &stage_ms](const char* name) {
        timing += std::string(" ") + name + "=" + std::to_string(stage_ms(t_total0));
    };
    m_status = INIT;
    m_seed_pending = false;
    m_healthy_streak = 0;
    m_scrutiny_frames = 0;
    m_unhealthy_streak = 0;
    m_inconsistent_streak = 0;
    m_initialized = false;
    {
        std::lock_guard<std::mutex> lk(m_cmd_mu);
        m_pending_init_valid = false;
        m_pending_reloc = false;
    }
    {
        std::lock_guard<std::mutex> lk(m_snapshot_mu);
        m_snapshot_valid = false;
    }
    m_has_cached_target = false;
    resetSessionMap();
    m_map_path = map_name;
    ++m_global_generation;
    // ---- 工作图 = map.overlay.pcd (底图 0.3 + overlay, 点 + intensity=w) ----
    //   首次载入: 把 map.pcd 降采样到 0.3 + 赋 w 初值 -> 生成工作图;
    //   之后每次只加载工作图 (0.3 体素 + w), 不再碰稠密的 map.pcd。
    //   map.pcd 只做原图备份, 永不修改 —— 回滚随时可行。
    //   工作图比原图旧 (同名地图被重新导入) 时自动作废重生成。
    const std::string working_map = workingMapPath(map_name);
    // 清理旧版本残留 (现在只有一张全量工作图):
    //   <stem>.overlay.map     —— 旧工作图名
    //   <map>.overlay.pcd      —— 旧的 overlay 增量日志 (map.pcd.overlay.pcd)
    {
        std::string legacy_map = working_map;
        if (legacy_map.size() > 4) {
            legacy_map = legacy_map.substr(0, legacy_map.size() - 4) + ".map";
            std::remove(legacy_map.c_str());
        }
        std::remove((map_name + ".overlay.pcd").c_str());
    }
    {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_request = false;
        m_global_scan_valid = false;
    }
    m_map.reset(new CloudType());
    bool loaded_working = false;
    {
        // 工作图比原图旧 (同名地图被重新导入) -> 工作图作废, 从原图重新生成。
        struct stat sb_map{}, sb_work{};
        const bool has_map = (stat(map_name.c_str(), &sb_map) == 0);
        const bool has_work = (stat(working_map.c_str(), &sb_work) == 0);
        const bool work_fresh = has_work && (!has_map || sb_work.st_mtime >= sb_map.st_mtime);
        if (work_fresh && pcl::io::loadPCDFile(working_map, *m_map) >= 0) {
            loaded_working = true;
            std::cout << "[LocPipeline] loaded WORKING map: " << working_map << " ("
                      << m_map->size() << " pts)" << std::endl;
        } else if (has_work) {
            std::cout << "[LocPipeline] working map older than original map -> regenerate"
                      << std::endl;
        }
    }
    if (!loaded_working) {
        if (pcl::io::loadPCDFile(map_name, *m_map) == -1) {
            std::cerr << "[LocPipeline] failed to load PCD map: " << map_name << std::endl;
            m_map.reset();
            return;
        }
    }
    mark("parse");
    if (m_cancel_set_map.load()) { std::cout << "[LocPipeline] setMap canceled after parse" << std::endl; return; }
    if (m_local_map) m_local_map->setMap(map_name, m_map);
    m_planar.setMap(m_map);
    mark("locmap_planar");
    if (m_store) {
        m_store->buildFromCloud(m_map);
        if (m_matcher) m_matcher->setStore(m_store.get());
        m_planar.setStore(m_store.get());
    }
    mark("store");
    if (m_maintainer) {
        m_maintainer->setBaseEmbeddedWeights(loaded_working);
        m_maintainer->setBaseMap(map_name, m_map);
        if (!loaded_working) {
            // 首次载入: 生成工作图 = 底图降采样到 0.3 + w 初值 (intensity=w)。
            // 之后所有会话只加载这个 0.3 文件, 不再碰稠密的 map.pcd。
            const double vox = m_maintainer_cfg.voxel > 0 ? m_maintainer_cfg.voxel : 0.3;
            CloudType::Ptr wm = voxelDownsample(m_map, static_cast<float>(vox));
            m_maintainer->annotateBaseWeights(*wm);
            pcl::io::savePCDFileBinary(working_map, *wm);
            std::cout << "[LocPipeline] working map generated: " << working_map << " ("
                      << wm->size() << " pts @ " << vox << "m, intensity=w)" << std::endl;
            m_map = wm;   // 本次会话也用 0.3, 与下次加载的完全一致
        }
    }
    mark("maintainer");

    mark("kfs");
    if (m_cancel_set_map.load()) { std::cout << "[LocPipeline] setMap canceled after kfs" << std::endl; return; }
    // 全局重定位库**后台**预建: FPFH 对全图算特征+建树实测 ~45s(aarch64)。
    // 演进史: ①最初 setMap 里同步建(切图 48.5s) ②改懒建(切图 3.2s, 但构建被推迟到
    // 首次冷启动的 onLidar 回调里 —— 2026-10-17 推行丢失事故: 判丢后回调内构建
    // 44.6s, 单线程 spinner 冻结, 雷达停处理+界面冻住) ③现在: setMap 后立即后台
    // 线程构建, 切图依旧 3s, 回调永不阻塞, 构建完成前冷启动按节流自动重试。
    {
        std::lock_guard<std::mutex> lk(m_reloc_mu);
        m_reloc_request = true;
        m_reloc_ready = false;
        m_reloc_map_snap = m_map;
        m_reloc_path_snap = m_map_path;
        // 关键帧还没转换完 (mark("kfs") 之前): 先清空旧图的关键帧, 避免拿旧位姿建库.
        // 末尾还会再 kick 一次, 那时带上真正的关键帧.
        m_reloc_kfs_snap.clear();
    }
    if (m_executor) {
        kickRelocBuild();
    } else if (!m_reloc_thread.joinable()) {
        m_reloc_thread = std::thread([this] { relocBuildWorker(); });
    }
    mark("relocalizer_bg");
    std::cout << "[LocPipeline] setMap profile(ms, 累计):" << timing
              << " total=" << std::to_string(stage_ms(t_total0)) << std::endl;
}


void LocalizationPipeline::setMapData(const MapData& data) {
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    m_out_lim_bypass = true;   // 换图: 输出限速旁路 (新图坐标系)
    m_cancel_set_map.store(false);
    m_status = INIT;
    m_seed_pending = false;
    m_healthy_streak = 0;
    m_scrutiny_frames = 0;
    m_unhealthy_streak = 0;
    m_inconsistent_streak = 0;
    m_initialized = false;
    {
        std::lock_guard<std::mutex> lk(m_cmd_mu);
        m_pending_init_valid = false;
        m_pending_reloc = false;
    }
    {
        std::lock_guard<std::mutex> lk(m_snapshot_mu);
        m_snapshot_valid = false;
    }
    m_has_cached_target = false;
    resetSessionMap();
    m_map_path = data.meta.map_id;
    ++m_global_generation;
    {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_request = false;
        m_global_scan_valid = false;
    }
    m_map.reset(new CloudType());

    if (data.points) m_map = data.points->exportCloud();
    if (!m_map) m_map.reset(new CloudType());

    std::vector<LocalKeyframe> lkfs;
    lkfs.reserve(data.keyframes.size());
    size_t kf_i = 0;
    for (const auto& kf : data.keyframes) {
        if ((kf_i++ & 0x1f) == 0 && m_cancel_set_map.load()) {
            std::cout << "[LocPipeline] setMapData canceled during keyframes" << std::endl;
            return;
        }
        if (!kf.cloud || kf.cloud->empty()) continue;
        LocalKeyframe lk;
        lk.pose = kf.pose;
        lk.cloud = kf.cloud;
        lkfs.push_back(std::move(lk));

        if (!data.points) {
            CloudType::Ptr world(new CloudType());
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3,3>(0,0) = kf.pose.rot.cast<float>();
            T.block<3,1>(0,3) = kf.pose.trans.cast<float>();
            pcl::transformPointCloud(*kf.cloud, *world, T);
            *m_map += *world;
        }
    }
    if (m_map->empty()) {
        std::cerr << "[LocPipeline] setMapData: empty map" << std::endl;
        m_map.reset();
        return;
    }
    // ★统一工作图消费 (w-map P1, 2026-11-25): xzmap/MapData 路径此前只写不读
    // map.overlay.pcd —— 落盘每小时在写, 加载却从不读回 -> 每次重启 w 全部回到
    // base_voxel_w (HMI 全绿), 已提交的 overlay 几何随内存清零 ("变动丢失"根因)。
    // fresh 工作图存在时用它替换底图云: intensity 就是 w,
    // 账本按文件播种 (见下方 setBaseEmbeddedWeights), 重启后持久度/雕刻状态延续。
    bool using_working_map = false;
    {
        const std::string working_map = workingMapPath(m_map_path);
        struct stat sb_map{}, sb_work{};
        const bool has_map = (stat(m_map_path.c_str(), &sb_map) == 0);
        const bool has_work = (stat(working_map.c_str(), &sb_work) == 0);
        const bool work_fresh = has_work && (!has_map || sb_work.st_mtime >= sb_map.st_mtime);
        CloudType::Ptr wm(new CloudType());
        if (work_fresh && pcl::io::loadPCDFile(working_map, *wm) >= 0 && !wm->empty()) {
            m_map = wm;
            using_working_map = true;
            std::cout << "[LocPipeline] setMapData: using WORKING map " << working_map
                      << " (" << m_map->size() << " pts, intensity=w)" << std::endl;
        }
    }
    std::cout << "[LocPipeline] setMapData: " << m_map->size() << " pts, "
              << lkfs.size() << " keyframes" << std::endl;

    if (m_cancel_set_map.load()) return;
    if (m_local_map) m_local_map->setMap(m_map_path, m_map);
    m_planar.setMap(m_map);
    if (m_store) {
        m_store->buildFromCloud(m_map);
        if (m_matcher) m_matcher->setStore(m_store.get());
        m_planar.setStore(m_store.get());
    }
    if (m_maintainer) {
        m_maintainer->setBaseEmbeddedWeights(using_working_map);
        m_maintainer->setBaseMap(m_map_path, m_map);
        // ★w-map: 新结构升格回调 —— 提交的点当场进工作图云 + 分块缓存 (当帧参与匹配)
        m_maintainer->setCommitHook([this](const Eigen::Vector3d& pt, float w) {
            if (m_local_map) m_local_map->addWorldPoint(
                static_cast<float>(pt.x()), static_cast<float>(pt.y()),
                static_cast<float>(pt.z()), w);
        });
    }
    if (using_working_map) {
        // ★w-map (2026-11-25): 工作图就是图, 匹配目标直接从它分块生成
        // (provider 的无关键帧兜底: 全局图按 10m 分块成虚拟关键帧)。
        // 不再注入 xzmap 旧关键帧 —— 否则目标里永远没有已提交的变化结构,
        // 变化区域 (该角落 ~15%) 的点永远配不上, fitness 停在 0.85 拉不回 0.93。
        std::cout << "[LocPipeline] w-map: local_map from WORKING map ("
                  << m_map->size() << " pts), xzmap keyframes skipped" << std::endl;
    } else if (m_local_map && !lkfs.empty()) {
        m_local_map->setKeyframes(lkfs);
        std::cout << "[LocPipeline] setMapData keyframes=" << lkfs.size() << std::endl;
    }
    if (m_cancel_set_map.load()) return;

    {
        std::lock_guard<std::mutex> lk(m_reloc_mu);
        m_reloc_request = true;
        m_reloc_ready = false;
        m_reloc_map_snap = m_map;
        m_reloc_path_snap = m_map_path;
        m_reloc_kfs_snap = lkfs;   // 只拷 shared_ptr, 点云不复制
    }
    if (!m_reloc_thread.joinable())
        m_reloc_thread = std::thread([this] { relocBuildWorker(); });
    m_reloc_cv.notify_all();
}

bool LocalizationPipeline::getMapCloud(PointVec& out) {
    std::lock_guard<std::mutex> lk(m_state_mu);
    if (!m_map || m_map->empty()) return false;
    out.clear();
    out.reserve(m_map->size());
    for (const auto& p : *m_map) out.push_back(p);
    return true;
}

bool LocalizationPipeline::getOverlayCloud(PointVec& out) {
    if (!m_maintainer) return false;
    const CloudType::Ptr c = m_maintainer->snapshotOverlay();
    if (!c || c->empty()) return false;
    out.assign(c->begin(), c->end());
    return true;
}

bool LocalizationPipeline::getWorkingMapCloud(PointVec& out) {
    if (!m_map || m_map->empty()) return false;
    CloudType c = *m_map;                       // 拷贝后注 w, 不动加载原图
    if (m_maintainer) m_maintainer->annotateBaseWeights(c);   // 刷新成**当前** w
    out.assign(c.begin(), c.end());
    return true;
}

uint64_t LocalizationPipeline::workingMapRevision() const {
    return m_maintainer ? m_maintainer->revision() : 0;
}

bool LocalizationPipeline::getSessionMapCloud(PointVec& out) {
    std::lock_guard<std::mutex> lk(m_session_map_mu);
    if (!m_session_map.active()) return false;
    const CloudType::Ptr c = m_session_map.pointsWorld();
    if (!c || c->empty()) return false;
    out.assign(c->begin(), c->end());
    return true;
}

void LocalizationPipeline::relocBuildWorker() {
    // 兼容路径 (未注入执行器): 常驻等待请求。注入执行器时走 kickRelocBuild()。
    while (true) {
        if (!runRelocBuildOnce()) {
            std::unique_lock<std::mutex> lk(m_reloc_mu);
            m_reloc_cv.wait(lk, [this] { return m_reloc_stop || m_reloc_request; });
            if (m_reloc_stop) break;
        }
    }
}

bool LocalizationPipeline::runRelocBuildOnce() {
    std::unique_lock<std::mutex> lk(m_reloc_mu);
    if (m_reloc_stop || !m_reloc_request) return false;
    const CloudType::Ptr map = m_reloc_map_snap;
    const std::string path = m_reloc_path_snap;
    const std::vector<LocalKeyframe> kfs = m_reloc_kfs_snap;   // shared_ptr 拷贝
    m_reloc_request = false;
    lk.unlock();
    m_reloc_building.store(true);
    const auto t0 = std::chrono::steady_clock::now();
    bool ok = false;
    if (m_relocalizer && m_relocalizer_name != "none" && map && !map->empty())
        ok = m_relocalizer->setMapWithKeyframes(path, map, kfs);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    lk.lock();
    m_reloc_building.store(false);
    m_reloc_ready = ok && m_reloc_map_snap == map;
    {
        std::ostringstream os;
        os << "global reloc map built in background, " << ms << "ms ok=" << ok;
        diag::log('I', "LocPipeline", os.str());
        FlightRecorder::instance().event("BG_BUILD", os.str());
    }
    return true;
}

void LocalizationPipeline::kickRelocBuild() {
    if (!m_executor) {
        // 没注入执行器: 叫醒兼容线程
        std::lock_guard<std::mutex> lk(m_reloc_mu);
        m_reloc_cv.notify_all();
        return;
    }
    bool expected = false;
    if (!m_reloc_job.compare_exchange_strong(expected, true)) return;   // 已有在途
    rosiwit_common::JobOptions opt;
    opt.name = "reloc_build";
    const auto id = m_executor->submit(
        "pool", opt,
        [this](rosiwit_common::CancelToken) { runRelocBuildOnce(); },
        [this](rosiwit_common::JobId, bool) {
            m_reloc_job.store(false);
            // 构建期间又切图: 快照已更新, 再投一次
            bool again = false;
            {
                std::lock_guard<std::mutex> lk(m_reloc_mu);
                again = m_reloc_request && !m_reloc_stop;
            }
            if (again) kickRelocBuild();
        });
    if (id == rosiwit_common::kInvalidJobId) m_reloc_job.store(false);
}

bool LocalizationPipeline::ensureRelocalizerMap() {
    if (!m_relocalizer || m_relocalizer_name == "none" || !m_map) return false;
    if (m_reloc_building.load()) return false;   // 构建中: 本帧跳过(回调绝不阻塞)
    std::lock_guard<std::mutex> lk(m_reloc_mu);
    if (m_reloc_ready && m_reloc_map_snap == m_map) return true;
    // 未建或地图已换: 置请求让 worker 建, 立刻返回 false
    m_reloc_request = true;
    m_reloc_map_snap = m_map;
    m_reloc_path_snap = m_map_path;
    kickRelocBuild();
    return false;
}

void LocalizationPipeline::setInitPose(const PoseStamped& pose) {
    // 服务线程只入命令队列；onLidar worker 消费，避免全局重定位持锁时
    // /relocation 等待 m_state_mu 而卡住 ROS spinner。
    // body(车底) 目标高度 = planar.z (地面=0); 雷达高度 = 该值 + 安装高度(由 init pose 带入)
    if (m_fix_planar && pose.trans.z() > 0.0) m_fixed_z = m_planar_z + pose.trans.z();
    {
        std::lock_guard<std::mutex> lk(m_cmd_mu);
        m_pending_init = pose;
        m_pending_init_valid = true;
        m_pending_reloc = false;
    }
    {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_request = false;
    }
    m_status = INIT;
}

void LocalizationPipeline::onImu(const IMUSample& s) {
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        if (m_prior) m_prior->onImu(s);
    }
    {
        std::lock_guard<std::mutex> fk(m_fusion_mu);
        if (m_fusion) m_fusion->onImu(s);
    }
}

void LocalizationPipeline::requestRelocalization() {
    // 上层 /global_localization 触发: 强制回到未定位状态, 下一次 onLidar 走冷启动分支
    // (relocalizer=bbs 时即全局检索; =none 时退化为 tryBoundedRecovery 的有界局部重锁)
    {
        std::lock_guard<std::mutex> lk(m_cmd_mu);
        m_pending_reloc = true;
        m_pending_init_valid = false;
    }
    {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_request = true;
        m_global_cooldown = 0;
    }
    // 重定位属于 INIT：等待健康帧累计到 localized_patience 才进入 LOCALIZED。
    m_status = INIT;
    std::cout << "[LocPipeline] relocalization requested by service" << std::endl;
}

void LocalizationPipeline::onOdom(const PoseStamped& odom) {
    // 单一开关: localization.odom_mode: fuse. none 时既不给运动先验, 也不给融合层.
    if (!m_odom_fuse) return;
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        if (m_prior) m_prior->onOdom(odom);
    }
    {
        std::lock_guard<std::mutex> fk(m_fusion_mu);
        if (m_fusion) m_fusion->onOdom(odom);
    }
}

void LocalizationPipeline::onRelative(const PoseStamped& pose,
                                      const Eigen::Matrix<double, 6, 6>& cov) {
    std::lock_guard<std::mutex> fk(m_fusion_mu);
    if (m_fusion) m_fusion->onRelative(pose, cov);
}

void LocalizationPipeline::resetRelativeAnchor() {
    std::lock_guard<std::mutex> fk(m_fusion_mu);
    if (m_fusion) m_fusion->resetRelative();
}

CloudType::Ptr LocalizationPipeline::toBodyFrame(const CloudType::Ptr& cloud) const {
    if (!m_lidar_to_imu_valid || !cloud || cloud->empty()) return cloud;
    CloudType::Ptr body(new CloudType);
    pcl::transformPointCloud(*cloud, *body, m_lidar_to_imu);
    body->header = cloud->header;
    return body;
}

CloudType::Ptr LocalizationPipeline::prepareScan(const LidarFrame& f,
                                                const PoseStamped& end_pose) {
    CloudType::Ptr scan = deskewScan(f, end_pose);
    // 雷达系 -> IMU 系 (定位路径不经过建图前端, 需要自己做)
    scan = toBodyFrame(scan);
    // 高度带裁剪: 与 matcher 用同一 z 带和同一套安装外参 (地面/贴地点不参与,
    // 也不写进 overlay/子图 —— 否则地面会被当成"新结构")。
    if (m_matcher_cfg.match_z_filter &&
        m_matcher_cfg.match_z_max > m_matcher_cfg.match_z_min &&
        m_matcher_cfg.crop_r_valid && scan && !scan->empty()) {
        CloudType::Ptr kept(new CloudType());
        kept->reserve(scan->size());
        const Eigen::Matrix3d& R = m_matcher_cfg.crop_r;
        const double z_off = m_matcher_cfg.crop_t.z();
        for (const auto& p : scan->points) {
            const double zf = R(2, 0) * p.x + R(2, 1) * p.y + R(2, 2) * p.z + z_off;
            if (zf > m_matcher_cfg.match_z_min && zf < m_matcher_cfg.match_z_max)
                kept->push_back(p);
        }
        if (!kept->empty()) {
            kept->header = scan->header;
            scan = kept;
        }
    }
    return scan;
}

CloudType::Ptr LocalizationPipeline::deskewScan(const LidarFrame& f, const PoseStamped& end_pose) {
    if (!m_deskew_enabled || !f.cloud || f.cloud->empty()) return f.cloud;
    if (!m_have_last_accepted && m_pose.time < 0.0) return f.cloud;

    // 起点: 必须是严格早于 end_pose 的位姿。m_pose 在本帧匹配后已是 f.end_time
    // (写图路径 end=res.pose 时 time 相同), 旧逻辑取 m_pose 会让 end<=start
    // 直接跳过 —— 写子图的扫描从未去畸变, 旋转时整片糊 (2026-11-26 根因)。
    // 候选按"最新且严格早于 end"选: 上一帧输出位姿 > 最后接受位姿 > m_pose。
    const PoseStamped* start = nullptr;
    if (m_deskew_prev_valid && m_deskew_prev_pose.time < end_pose.time - 1e-6)
        start = &m_deskew_prev_pose;
    if (m_last_accepted.time >= 0.0 && m_last_accepted.time < end_pose.time - 1e-6 &&
        (!start || m_last_accepted.time > start->time))
        start = &m_last_accepted;
    if (m_pose.time >= 0.0 && m_pose.time < end_pose.time - 1e-6 &&
        (!start || m_pose.time > start->time))
        start = &m_pose;
    if (!start) return f.cloud;   // 无可用起点 (首帧/注入后): 不补偿, 保底
    const PoseStamped& start_pose = *start;
    const double span = f.end_time - f.start_time;
    const double gap = f.start_time - start_pose.time;
    static int dsn = 0;
    const bool dg = kLocProbe && (++dsn % 20 == 0);
    if (span <= 1e-3 || start_pose.time < 0.0 ||
        gap < -0.05 || gap > m_deskew_max_gap ||
        end_pose.time <= start_pose.time) {
        if (dg) std::cout << "[DeskewDiag] skip=validity span=" << span << " gap=" << gap
                          << " max_gap=" << m_deskew_max_gap << std::endl;
        return f.cloud;
    }

    // 检查逐点时间是否真的可用; 全 0 说明驱动/ROS 层没填时间, 不能盲目补偿.
    double max_curv = 0.0;
    for (const auto& p : f.cloud->points) {
        if (std::isfinite(p.curvature) && p.curvature > max_curv)
            max_curv = p.curvature;
    }
    if (max_curv <= 1e-3) {
        if (dg) std::cout << "[DeskewDiag] skip=no_per_point_time" << std::endl;
        static bool warned = false;
        if (!warned) {
            warned = true;
            std::cout << "[LocPipeline] no per-point time (curvature==0): "
                         "deskew disabled automatically" << std::endl;
        }
        return f.cloud;
    }

    // **姿态约定必须统一**: guess(帧末) 已被 lock_zrp 平面化, 而 start(上一帧输出/融合)
    // 可能还是 rpyToRot 的混合姿态。混用会让 qrel 混进 roll/pitch, deskew 把点云拧斜 ——
    // 静止时看不出来, 一转弯覆盖率就从 0.95 掉到 0.4 (2026-11-23 根因)。
    PoseStamped s_pose = start_pose;
    PoseStamped e_pose = end_pose;
    planarizePose(s_pose);
    planarizePose(e_pose);
    Eigen::Quaterniond q0(s_pose.rot);
    Eigen::Quaterniond q1(e_pose.rot);
    q0.normalize();
    q1.normalize();
    // **帧内相对旋转优先用陀螺自由积分**: 位姿链上的锚定/拒绝会让姿态滞后
    // (实测转弯中先验落后真值 20°), 拿位姿差做帧内插值 => 去畸变欠补偿
    // (真实 5° 只补 1°) => 扫描被抹开 => 覆盖率天花板 0.65 < 接受线 0.7
    // => 拒帧 => 更滞后 (2026-11-23 死亡螺旋根因).
    // 陀螺 0.1s 帧内相对误差 ~0.2°, 与绝对滞后无关.
    Eigen::Quaterniond dq_gyro;
    const bool gyro_ok = [this, &f, &dq_gyro] {
        std::lock_guard<std::mutex> lk(m_prior_mu);
        return m_prior && m_prior->gyroDelta(f.start_time, f.end_time, dq_gyro);
    }();
    const double total = e_pose.time - s_pose.time;
    double a0 = (total > 1e-6) ? (f.start_time - s_pose.time) / total : 0.0;
    a0 = std::max(0.0, std::min(1.0, a0));
    Eigen::Quaterniond qf;
    if (gyro_ok) {
        // start = end·Δ⁻¹: 帧内几何严格等于陀螺增量, 绝对误差只表现为整体旋转
        qf = (q1 * dq_gyro.conjugate()).normalized();
    } else {
        qf = q0.slerp(a0, q1);
    }
    const V3D tf = (1.0 - a0) * s_pose.trans + a0 * e_pose.trans;
    Eigen::Quaterniond qrel = qf.conjugate() * q1;
    qrel.normalize();
    const double angle = 2.0 * std::acos(std::min(1.0, std::fabs(qrel.w())));
    if (angle * 57.2957795 < m_deskew_min_angle_deg) {
        if (dg) std::cout << "[DeskewDiag] skip=small_angle angle=" << angle * 57.2957795 << std::endl;
        return f.cloud;
    }
    // 单帧内实际旋转过大 -> 先验跳变, 宁可不去畸变, 也不要主动把点云转坏.
    if (angle > 0.8) {
        if (dg) std::cout << "[DeskewDiag] skip=big_angle angle=" << angle * 57.2957795 << std::endl;
        return f.cloud;
    }

    CloudType::Ptr out(new CloudType());
    out->header = f.cloud->header;
    out->reserve(f.cloud->size());
    const double span_ms = span * 1000.0;
    for (const auto& p : f.cloud->points) {
        if (!pcl::isFinite(p)) continue;
        double alpha = (std::isfinite(p.curvature) ? p.curvature : 0.0) / span_ms;
        alpha = std::max(0.0, std::min(1.0, alpha));
        const Eigen::Quaterniond qt = qf.slerp(alpha, q1);
        const M3D rt = qt.toRotationMatrix();
        const V3D tt = (1.0 - alpha) * tf + alpha * e_pose.trans;
        const V3D pw = rt * V3D(p.x, p.y, p.z) + tt;
        const V3D pe = q1.conjugate() * (pw - e_pose.trans);

        PointType q = p;
        q.x = static_cast<float>(pe.x());
        q.y = static_cast<float>(pe.y());
        q.z = static_cast<float>(pe.z());
        const V3D n(p.normal_x, p.normal_y, p.normal_z);
        if (n.norm() > 1e-6) {
            const V3D ne = q1.conjugate() * (rt * n);
            q.normal_x = static_cast<float>(ne.x());
            q.normal_y = static_cast<float>(ne.y());
            q.normal_z = static_cast<float>(ne.z());
        }
        out->push_back(q);
    }
    if (dg) std::cout << "[DeskewDiag] run span=" << span << " gap=" << gap << " a0=" << a0
                      << " gyro=" << (gyro_ok ? 1 : 0)
                      << " angle=" << angle * 57.2957795 << " maxcurv=" << max_curv
                      << " in=" << f.cloud->size() << " out=" << out->size() << std::endl;
    if (out->empty()) return f.cloud;
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = f.cloud->is_dense;
    return out;
}

// "子图 vs 先验图 的一致性观测 (只测不改)": 把子图刚性地对齐到先验图,
// 输出残余刚体变换 (Δt, Δyaw)。Δt≈0 => 两者一致; 明显非零 => 子图已经偏了。
// 先验图 = 底图 + overlay(工作站架构里的 buildAnchoredTarget, 与匹配锚定目标同源)。
void LocalizationPipeline::checkSubmapPriorConsistency() {
    if (!m_session_map_enabled) return;
    CloudType::Ptr sub;
    {
        std::lock_guard<std::mutex> lk(m_session_map_mu);
        if (!m_session_map.active()) return;
        sub = m_session_map.pointsWorld(m_pose.trans, m_session_map_match_radius);
    }
    if (!sub || sub->size() < 3000) return;
    CloudType::Ptr prior = buildAnchoredTarget(m_pose);
    if (!prior || prior->size() < 3000) return;
    CloudType::Ptr A = downsampleCloud(sub, 0.4);
    CloudType::Ptr B = downsampleCloud(prior, 0.4);
    if (!A || !B || A->size() < 1500 || B->size() < 1500) return;

    // 第一步: 恒等位姿下的平均最近邻距离 (0.3m 封顶)。
    // 两云一致时 ~0.05~0.15 (体素错位量级); 明显偏移时 >0.2。先量这个, 大多数帧
    // 不需要 GICP —— 2026-11-25 实测教训: 对应距离放到 1.0m 的 GICP 会沿走廊状
    // 结构"滑"出 0.5~2.0m 的假残余 (读数在 0.05 和 1.1m 之间每 3s 来回跳, 物理上
    // 不可能, 纯属局部最优), 恒等评分才是稳定可信的基准。
    pcl::KdTreeFLANN<PointType> kdt;
    kdt.setInputCloud(B);
    double sum = 0.0;
    int cnt = 0;
    std::vector<int> nn_idx(1);
    std::vector<float> nn_d2(1);
    for (const auto& p : A->points) {
        if (kdt.nearestKSearch(p, 1, nn_idx, nn_d2) > 0) {
            const double d = std::sqrt(static_cast<double>(nn_d2[0]));
            if (d <= 0.3) { sum += d; ++cnt; }
        }
    }
    if (cnt < static_cast<int>(A->size() / 4)) {
        // 3/4 的点在 0.3m 内找不到对应 -> 结构性不同 (场景真变了), 不是偏移
        std::cout << "[submap-prior] n=" << A->size() << "/" << B->size()
                  << " fit_id=n/a -> UNDECIDABLE(true-change?)" << std::endl;
        return;
    }
    const double fit_id = sum / cnt;
    if (fit_id < 0.12) {
        std::cout << "[submap-prior] n=" << A->size() << "/" << B->size()
                  << " fit_id=" << fit_id << " -> CONSISTENT" << std::endl;
        return;
    }
    // 第二步: 恒等分数差 -> 用**小对应距离**(0.35m) 的 GICP 找局部残余。
    // 对应距离小, GICP 最多滑 0.35m, 量出的 Delta 才是真实局部偏移。
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setInputSource(A);
    gicp.setInputTarget(B);
    gicp.setMaximumIterations(30);
    gicp.setMaxCorrespondenceDistance(0.35);
    gicp.setTransformationEpsilon(1e-6);
    CloudType aligned;
    gicp.align(aligned);
    if (!gicp.hasConverged()) {
        std::cout << "[submap-prior] n=" << A->size() << "/" << B->size()
                  << " fit_id=" << fit_id << " -> UNDECIDABLE(no-converge)" << std::endl;
        return;
    }
    const Eigen::Matrix4f T = gicp.getFinalTransformation();
    const double dx = T(0, 3), dy = T(1, 3), dz = T(2, 3);
    const double dxy = std::sqrt(dx * dx + dy * dy);
    const double dyaw = std::atan2(T(1, 0), T(0, 0)) * 57.2957795;
    const double fit_al = gicp.getFitnessScore(0.3);
    // 分类 (2026-11-25 v2): 残差含 z —— 子图与先验的差异可能主要在垂直轴
    // (z 基准/安装高差), 只看 xy 会把「对齐良好」误标成 struct-diff。
    //   CONSISTENT : 对齐后残差已经很小 (fit_al<0.12 且 xy/z/yaw 都小)
    //   OFFSET     : 对齐显著改善 (fit_al < 0.7*fit_id) 且 xy 或 z 或 yaw 超阈
    //   UNDECIDABLE: 对齐也无改善 (两朵云结构确实不同, 真变化)
    const bool improved = (fit_al < 0.7 * fit_id);
    const bool small = (dxy < 0.10 && std::fabs(dz) < 0.10 && std::fabs(dyaw) < 1.5);
    const bool offset = improved && !small;
    const char* cls = small ? "CONSISTENT"
                    : (offset ? "OFFSET" : "UNDECIDABLE(struct-diff)");
    std::cout << "[submap-prior] n=" << A->size() << "/" << B->size()
              << " fit_id=" << fit_id << " fit_al=" << fit_al
              << " dxy=" << dxy << "m dz=" << dz << "m/" << dyaw << "deg -> "
              << cls << std::endl;
}

CloudType::Ptr LocalizationPipeline::buildTarget(const PoseStamped& pose) {
    // ---- 子图桥接模式: 目标 = 工作图 crop + 子图【联合】 (ROLL 式联合匹配) ----
    // ★2026-11-26: 旧实现桥接时目标切成纯子图 —— 子图只含"走过的地方"(写入还
    // 停在 fitness 门后, 滞后于机器人), 行驶进入薄弱区时纯子图撑不住, fitness
    // 0.86 -> 0.33 -> LOST (实机 00:02 完整复现)。ROLL 的做法是全局图与临时图
    // 【联合】匹配: 变化区域由子图覆盖, 其余由工作图锚定, 不存在"失去约束"的
    // 空窗。桥接/锚定的差别只剩 overlay 权重, 模式切换的复杂度大幅下降。
    if (m_on_submap) {
        CloudType::Ptr out(new CloudType());
        if (m_local_map) {
            CloudType::Ptr base_crop = m_local_map->getLocalMap(pose);
            if (base_crop && !base_crop->empty()) *out += *base_crop;
        }
        {
            std::lock_guard<std::mutex> lk(m_session_map_mu);   // 与 reset/update 互斥
            CloudType::Ptr sub = m_session_map.pointsWorld();
            if (sub && !sub->empty()) *out += *sub;
        }
        if (!out->empty()) out = downsampleCloud(out, 0.3);   // 并集去重
        return out;
    }
    // ---- overlay 锚定模式: 目标 = 底图 crop + overlay (压制同区域旧点) ----
    return buildAnchoredTarget(pose);
}

CloudType::Ptr LocalizationPipeline::buildAnchoredTarget(const PoseStamped& pose) {
    CloudType::Ptr target = m_local_map->getLocalMap(pose);
    // [LocProbe] 诊断: 局部地图到底取到多少点 (NO Effective Points 排查)
    {
        static int probe_n = 0;
        if (++probe_n % 50 == 0) {   // 5s 一条: 100Hz 输出下 1Hz 仍然太吵
            std::cout << "[LocProbe] local_map_points=" << (target ? target->size() : 0)
                      << " pose=(" << pose.trans.x() << ", " << pose.trans.y() << ")"
                      << " local_map_name=" << m_local_map_name << std::endl;
        }
    }
    if (m_maintainer) {
        // 统一工作图: 剔除 w 已降到 0 的底图点 (墙挪了/拆了, 射线证据判死)。
        target = m_maintainer->filterDeadBase(target);
    }
    // ★w-map (2026-11-26): 匹配目标 = 工作图本体 (含已提交并升格的结构),
    // 不再有独立的 overlay 增量层 —— 新结构提交时直接进工作图云 (见
    // maintainer addPoint 的 commit sink), 配准锚定按 w 加权 (anchorOk 的
    // static_support 判据 + [quality] sup= 字段)。
    return target;
}

bool LocalizationPipeline::tryBoundedRecovery(const CloudType::Ptr& scan) {
    // 出口重锁: 在推算位姿附近做有界局部多起点配准(优于全局检索; 叠加层帮助在新货环境重锁)
    if (!m_local_map || !m_matcher) return false;
    CloudType::Ptr target = m_local_map->getLocalMap(m_pose);
    if (m_maintainer) target = m_maintainer->filterDeadBase(target);
    if (m_maintainer) {
        CloudType::Ptr ov = m_maintainer->getOverlay(m_pose);
        if (ov && !ov->empty()) *target += *ov;
    }
    if (!target || target->empty()) return false;
    m_matcher->setTarget(target);

    // 27 次多起点 GICP 单帧可耗 3~13s; 之前没有预算/提前退出, 一旦位姿偏了
    // 每帧都全量跑满, 单线程 spinner 被独占 -> /relocation 等服务排队"很慢"。
    // 现在两条停止条件: 覆盖率已达标(0.97)即停 / 墙钟预算(3s)耗尽即停。
    const auto t0 = std::chrono::steady_clock::now();
    const double offs[3] = {-1.5, 0.0, 1.5};
    const double yaws[3] = {-10.0, 0.0, 10.0};
    // 达标线用"健康线"(credible 0.75)而非最低接受线(min_coverage 0.35):
    // 2026-10-17 推行复测死循环 —— 判丢后 bounded recovery 以 0.73"成功"(>0.35)
    // -> 回到僵尸态 -> 10s 后再判丢 -> 再截胡, FPFH 冷启动永远轮不到, 位姿/点云
    // 十几秒跳一次。恢复不出健康分数 = 失败, 放行给全局重定位。
    double best_fit = std::max(m_accept_cfg.min_coverage,
                               m_monitor_cfg.enabled ? m_monitor_cfg.credible_fitness
                                                     : m_accept_cfg.min_coverage);
    PoseStamped best;
    bool found = false;
    bool stop = false;
    for (double dx : offs) {
        if (stop) break;
        for (double dy : offs) {
            if (stop) break;
            for (double yd : yaws) {
                PoseStamped g = m_pose;
                g.trans += V3D(dx, dy, 0.0);
                g.rot = m_pose.rot * Eigen::AngleAxisd(yd * M_PI / 180.0, V3D::UnitZ()).toRotationMatrix();
                planarizePose(g);
                MatchResult r;
                if (m_matcher->match(scan, g, r) && r.converged && r.fitness > best_fit) {
                    planarizePose(r.pose);
                    best_fit = r.fitness;
                    best = r.pose;
                    found = true;
                }
                const double ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - t0).count();
                if (best_fit >= 0.97 || ms > 3000.0) { stop = true; break; }
            }
        }
    }
    if (!found) {
        // 失败可见: 之前静默返回, 现场看不出"有界恢复试过但没到健康线"
        diag::log('I', "LocPipeline", "bounded recovery failed (bar=" +
                  std::to_string(best_fit) + ") -> global relocalization");
        return false;
    }
    m_out_lim_bypass = true;
    m_pose = best;
    m_pose.time = best.time;
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        m_prior->reset(m_pose);
    }
    {
        std::lock_guard<std::mutex> fk(m_fusion_mu);
        if (m_fusion) m_fusion->reset(m_pose);
    }
    m_monitor->reset();
    m_healthy_streak = 0;
    m_unhealthy_streak = 0;
    m_has_cached_target = false;
    resetSessionMap();
    m_coast_count = 0;
    diag::log('I', "LocPipeline", "bounded recovery, fitness=" + std::to_string(best_fit));
    FlightRecorder::instance().event("RECOVERY", "bounded recovery fitness=" +
                                      std::to_string(best_fit));
    return true;
}

bool LocalizationPipeline::acceptMatch(const MatchResult& res, const CloudType::Ptr& scan,
                                       const PoseStamped& guess, const char** reject_reason) {
    auto reject = [&](const char* reason) {
        if (reject_reason) *reject_reason = reason;
        return false;
    };
    if (!res.converged) return reject("not_converged");
    // Mahalanobis 门控: 修正量相对匹配协方差异常大 -> 判离群, 不采信
    // (协方差加下限: 匹配协方差是估计自身不确定度, 极小时会误拒正常修正)
    if (m_maha_bypass_frames > 0) {
        // 注入位姿 (重定位 / 2D 约束) 后放行几帧. 理由: reset 运动先验会丢掉
        // "3D 匹配 vs IMU 先验" 的固有 roll/pitch 偏置 (~12°), 创新门限会因此
        // 每帧都拒; 而拒绝的帧不会进入 onMatch, 偏置永远学不回来 -> 死锁在 INIT.
        --m_maha_bypass_frames;
        if (reject_reason) *reject_reason = "bypass_after_reseed";
    } else if (m_accept_cfg.mahalanobis_gate) {
        Eigen::Matrix<double, 6, 1> e;
        poseInnovation(res.pose, guess, e);
        Eigen::Matrix<double, 6, 6> C = res.cov;
        // 对应点少时 GICP 的协方差被低估 (残差平方口径), 常数 floor 会把
        // 0.04m 的正常修正判成离群 (实测旋转时连续 coast 28 帧). 这里按
        // 对应点数放大 floor: n=ref 时系数 1, n 越小越宽松 (上限 6 倍方差).
        double floor_scale = 1.0;
        if (m_accept_cfg.mahalanobis_ref_points > 0 && res.correspondences > 0) {
            floor_scale = std::min(6.0, std::max(1.0,
                static_cast<double>(m_accept_cfg.mahalanobis_ref_points) /
                static_cast<double>(res.correspondences)));
        }
        const double rf = m_accept_cfg.mahalanobis_rot_floor *
                          m_accept_cfg.mahalanobis_rot_floor * floor_scale;
        const double tf = m_accept_cfg.mahalanobis_trans_floor *
                          m_accept_cfg.mahalanobis_trans_floor * floor_scale;
        for (int k = 0; k < 3; ++k) C(k, k) += rf;
        for (int k = 3; k < 6; ++k) C(k, k) += tf;
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(C);
        // 连续拒绝 (coast) 后门限按 ×(1 + k·coast) 放宽 (2026-11-23): 预测一旦偏出门限,
        // 即使匹配一直正确也永远被拒 -> 死锁 (实车: 1.9m 创新量 d²=17.5 对门限 16.81,
        // 卡死到人工重锁). 跟踪正常时 coast=0, 门限不变.
        const double widen = 1.0 + m_accept_cfg.mahalanobis_coast_widen * m_coast_count;
        const double d2 = e.dot(ldlt.solve(e));
        if (std::isfinite(d2) && d2 > m_accept_cfg.mahalanobis_chi2 * widen) {
            // [MahalanobisProbe] 定位"点数少时好修正被误杀": 打印创新量与协方差
            static int probe_n = 0;
            if (++probe_n % 10 == 0) {
                const double myaw = std::atan2(res.pose.rot(1, 0), res.pose.rot(0, 0));
                const double gyaw = std::atan2(guess.rot(1, 0), guess.rot(0, 0));
                std::cout << "[MahalanobisProbe] d2=" << d2
                          << " match_yaw=" << myaw * 57.2957795
                          << " guess_yaw=" << gyaw * 57.2957795
                          << " match_xy=[" << res.pose.trans.x() << "," << res.pose.trans.y() << "]"
                          << " guess_xy=[" << guess.trans.x() << "," << guess.trans.y() << "]"
                          << " rot_innov_deg=[" << e(0) * 57.2957795 << ","
                          << e(1) * 57.2957795 << "," << e(2) * 57.2957795 << "]"
                          << " trans_innov_m=[" << e(3) << "," << e(4) << "," << e(5) << "]"
                          << std::endl;
            }
            return reject("mahalanobis");
        }
        // 位置创新量绝对上限 (2026-11-23): 协方差门限在点云充分时允许 ~1.9m, 挡不住两个
        // 相似结构之间 0.5~0.9m 的"中间解" (实车推车: 输出位姿被拽歪、来回跳).
        // 同样随 coast 放宽: 真实的大偏差连续被拒几帧后仍能被接受, 不会死锁.
        if (m_accept_cfg.max_trans_innov > 0.0 &&
            e.tail<3>().norm() > m_accept_cfg.max_trans_innov * std::sqrt(widen)) {
            return reject("innov_trans");
        }
    }
    if (m_accept_cfg.mode == 0) {
        if (res.fitness >= m_accept_cfg.min_coverage) {
            if (reject_reason) *reject_reason = "accepted";
            return true;
        }
        return reject("coverage");
    }

    // mode 1: 覆盖率达标直接接受; 否则用残差 + 多帧一致性仲裁
    if (res.fitness >= m_accept_cfg.min_coverage) {
        m_corr_prev_valid = false;
        m_consistent_cnt = 0;
        if (reject_reason) *reject_reason = "accepted";
        return true;
    }
    const double res_icp = res.residual;
    const double res_prior = m_matcher->evaluateResidual(scan, guess);
    if (res_icp < 0.0 || res_prior <= 1e-6) return reject("residual");

    const double improve = (res_prior - res_icp) / res_prior;
    const V3D corr = res.pose.trans - guess.trans;
    bool consistent = false;
    if (m_corr_prev_valid && corr.norm() > 1e-6 &&
        (corr.dot(m_corr_prev) > 0.0 || corr.norm() < 0.02))
        consistent = true;
    m_consistent_cnt = consistent ? m_consistent_cnt + 1 : 0;
    m_corr_prev = corr;
    m_corr_prev_valid = true;

    if (improve > 3.0 * m_accept_cfg.residual_margin) {
        if (reject_reason) *reject_reason = "accepted";
        return true;
    }
    if (improve > m_accept_cfg.residual_margin &&
        m_consistent_cnt >= m_accept_cfg.consistent_frames) {
        if (reject_reason) *reject_reason = "accepted";
        return true;
    }
    // [RejectProbe] 拒绝原因 (帧时间): 定位"匹配成功但不输出"时是哪道门限.
    // ROSIWIT_LOC_PROBE=1 逐帧, 否则 50 帧一条.
    static int probe_n = 0;
    if (kLocProbe || ++probe_n % 50 == 0) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(3) << "[RejectProbe] t=" << res.pose.time
           << " fitness=" << std::setprecision(4) << res.fitness
           << " min_coverage=" << m_accept_cfg.min_coverage
           << " residual_improvement=" << improve
           << " residual_margin=" << m_accept_cfg.residual_margin
           << " consistent_frames=" << m_consistent_cnt << "/"
           << m_accept_cfg.consistent_frames;
        std::cout << os.str() << std::endl;
    }
    return reject("consistent_frames");
}

void LocalizationPipeline::globalRelocWorker() {
    // 兼容路径 (未注入执行器): 常驻等待请求。
    while (true) {
        {
            std::unique_lock<std::mutex> lk(m_global_mu);
            m_global_cv.wait(lk, [this] {
                return m_global_stop || (m_global_request && m_global_scan_valid);
            });
            if (m_global_stop) break;
        }
        runGlobalRelocOnce();
    }
}

bool LocalizationPipeline::runGlobalRelocOnce() {
        CloudType::Ptr scan;
        uint64_t generation = 0;
        {
            std::lock_guard<std::mutex> lk(m_global_mu);
            if (m_global_stop || !m_global_request || !m_global_scan_valid) return false;
            scan = m_global_scan;
            generation = m_global_scan_generation;
            m_global_scan_valid = false;
        }
        if (!scan || !m_relocalizer || m_relocalizer_name == "none") {
            std::lock_guard<std::mutex> lk(m_global_mu);
            m_global_request = false;
            return true;
        }
        if (!ensureRelocalizerMap()) {
            // FPFH 地图还在后台构建；保留请求，等下一帧/构建完成后重试。
            std::lock_guard<std::mutex> lk(m_global_mu);
            m_global_cooldown = std::max(1, m_global_cfg.retry_interval);
            return true;
        }
        std::vector<GlobalCandidate> cands;
        const int nk = m_relocalizer->localizeTopK(
            scan, std::max(1, m_global_cfg.top_k), cands);
        if (nk <= 0) {
            std::lock_guard<std::mutex> lk(m_global_mu);
            m_global_cooldown = std::max(1, m_global_cfg.retry_interval);
            // 以前这里静默返回: 现场只看得到"一直 LOST", 分不清是描述子召回失败
            // 还是候选被覆盖率/精配淘汰. 打一条便于定位 (每帧最多一条, 已节流).
            diag::log('W', "LocPipeline", "global reloc: no candidate from " +
                      m_relocalizer_name + " -> retry");
            return true;
        }
        std::sort(cands.begin(), cands.end(),
                  [](const GlobalCandidate& a, const GlobalCandidate& b) {
                      return a.score > b.score;
                  });
        // 全局粗候选不能直接当 seed：用独立 matcher 在全图目标上做精配，
        // 并检查最优/次优差异，避免在重复结构里把 top1 的错误候选锁进来。
        CloudType::Ptr global_map;
        {
            std::lock_guard<std::mutex> lk(m_state_mu);
            global_map = m_map;
        }
        PoseStamped best_pose;
        double best_fit = -1.0;
        double second_fit = -1.0;
        if (m_global_matcher && global_map && !global_map->empty()) {
            // 二级精配: 按候选裁局部子图 (整图 GICP x N 候选 = 重定位的主要耗时)
            pcl::KdTreeFLANN<PointType> gt_kdtree;
            const bool crop_target = m_global_cfg.reloc_submap_radius > 0.0;
            if (crop_target) gt_kdtree.setInputCloud(global_map);
            int refined = 0;
            for (const auto& cand : cands) {
                CloudType::Ptr target = global_map;
                if (crop_target) {
                    CloudType::Ptr sub = cropRadius(
                        gt_kdtree, global_map, cand.pose.trans, m_global_cfg.reloc_submap_radius);
                    if (sub && sub->size() >= 100) target = sub;
                }
                m_global_matcher->setTarget(target);
                MatchResult r;
                if (!m_global_matcher->match(scan, cand.pose, r) || !r.converged)
                    continue;
                ++refined;
                if (r.fitness > best_fit) {
                    second_fit = best_fit;
                    best_fit = r.fitness;
                    best_pose = r.pose;
                } else if (r.fitness > second_fit) {
                    second_fit = r.fitness;
                }
                // 提前收敛: 已经拿到"够好且和次优分得开"的两个候选时不再继续精配.
                // (候选是按粗分降序的, 后面的只会更差; 这能把最坏耗时从 N 次精配压到 2~3 次)
                if (refined >= 2 && best_fit >= m_global_cfg.reloc_accept_score &&
                    (second_fit < m_global_cfg.reloc_accept_score ||
                     best_fit - second_fit >= m_ambiguity_margin)) {
                    break;
                }
            }
        }
        if (best_fit < m_global_cfg.reloc_accept_score) {
            std::lock_guard<std::mutex> lk(m_global_mu);
            m_global_cooldown = std::max(1, m_global_cfg.retry_interval);
            std::ostringstream os;
            os << "global refine weak: best=" << best_fit
               << " second=" << second_fit << " -> retry";
            diag::log('W', "LocPipeline", os.str());
            return true;
        }
        const double gap = (second_fit >= 0.0) ? (best_fit - second_fit) : 1.0;
        if (second_fit >= m_global_cfg.reloc_accept_score &&
            gap < m_ambiguity_margin) {
            std::lock_guard<std::mutex> lk(m_global_mu);
            m_global_cooldown = std::max(1, m_global_cfg.retry_interval);
            std::ostringstream os;
            os << "global refine ambiguous: best=" << best_fit
               << " second=" << second_fit << " gap=" << gap << " -> retry";
            diag::log('W', "LocPipeline", os.str());
            return true;
        }
        {
            std::lock_guard<std::mutex> lk(m_global_mu);
            if (generation != m_global_generation.load() || !m_global_request) {
                // 地图已切换 / 本地 tracking 已自行恢复：旧候选直接丢弃。
                return true;
            }
        }
        {
            std::lock_guard<std::mutex> lk(m_cmd_mu);
            m_pending_init = best_pose;
            m_pending_init_valid = true;
            m_pending_reloc = false;
        }
        {
            std::lock_guard<std::mutex> lk(m_global_mu);
            m_global_request = false;
            m_global_cooldown = 0;
        }
        std::ostringstream os;
        os << "global candidate injected: best_fit=" << best_fit
           << " second=" << second_fit << " gap=" << gap;
        diag::log('I', "LocPipeline", os.str());
        FlightRecorder::instance().event("GLOBAL_SEED", os.str());
    return true;
}

void LocalizationPipeline::kickGlobalReloc() {
    if (!m_executor) {
        std::lock_guard<std::mutex> lk(m_global_mu);
        m_global_cv.notify_all();
        return;
    }
    bool expected = false;
    if (!m_global_job.compare_exchange_strong(expected, true)) return;
    rosiwit_common::JobOptions opt;
    opt.name = "global_reloc";
    const auto id = m_executor->submit(
        "pool", opt,
        [this](rosiwit_common::CancelToken) { runGlobalRelocOnce(); },
        [this](rosiwit_common::JobId, bool) {
            m_global_job.store(false);
            bool again = false;
            {
                std::lock_guard<std::mutex> lk(m_global_mu);
                again = m_global_request && m_global_scan_valid && !m_global_stop;
            }
            if (again) kickGlobalReloc();
        });
    if (id == rosiwit_common::kInvalidJobId) m_global_job.store(false);
}

void LocalizationPipeline::onLidar(const LidarFrame& f) {
    // 唯一的"大状态"写入者: 整个匹配/重定位期间持 m_state_mu.
    // 高频输出 predictAt 只走 m_prior_mu, 因此不会被这里的长耗时阻塞.
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    // 快照守卫：任何 return（包括长时间全局重定位）都会把最后状态发布给 ROS 层，
    // 但 getStatus/getPose 不再等待 m_state_mu。
    struct SnapshotGuard {
        LocalizationPipeline* self;
        ~SnapshotGuard() { self->updateSnapshotLocked(); }
    } snapshot_guard{this};
    {
        bool pending_init = false;
        bool pending_reloc = false;
        PoseStamped pending_pose;
        {
            std::lock_guard<std::mutex> lk(m_cmd_mu);
            pending_init = m_pending_init_valid;
            pending_reloc = m_pending_reloc;
            if (pending_init) pending_pose = m_pending_init;
            m_pending_init_valid = false;
            m_pending_reloc = false;
        }
        if (pending_reloc) {
            // 正常 tracking 不停：当前若已有位姿，继续用本地 GICP/运动先验；
            // 全局线程只负责在后台找候选。没有位姿时才等待全局 seed。
            m_seed_pending = false;
            m_healthy_streak = 0;
            m_unhealthy_streak = 0;
            m_status = INIT;
        }
        if (pending_init) {
            m_out_lim_bypass = true;
            m_pose = pending_pose;
            // 关键: 外部注入新位姿 (重定位 / 2D 约束) 后, 旧的"最近被接受位姿"
            // 锚点已经无效. 不清掉它会导致位姿被硬拉回去:
            //   1) 逸出保护在局部目标为空时 m_pose = m_last_accepted;
            //   2) coast 重锚 (m_coast_count >= N) 把 guess 拉回 m_last_accepted.
            // 实机复现: 拉偏 1m 后注入正确位姿, 下一帧又被推回旧锚点, 状态停在 INIT.
            m_last_accepted = pending_pose;
            m_have_last_accepted = false;
            m_coast_count = 0;
            m_escape_cnt = 0;               // 逃逸候选作废
            m_escape_prev_valid = false;
            m_have_submap_anchor = false;   // 子图窗口锚一起失效
            m_has_cached_target = false;   // 旧缓存目标建在旧位姿附近
            m_deskew_prev_valid = false;   // 旧解的位姿不能当新解的去畸变起点
            // ★重定位/位姿注入: 子图关键帧按旧解注册, 新解下不清理就是"叠图"——
            //   同一面墙在子图里两份 (相差 Δ), 联合目标两层互相打架, 且
            //   popLeavingKfs 会把两层都 mergeChunk 进 w 账本, ghost 墙永久固化。
            //   注入解也可能是错的 (混叠场景实测), 刚性搬移 (applyWorldCorrection)
            //   会把整个补丁钉到错误解上 —— 直接废弃重建最稳:
            //   注入通常发生在 LOST/INIT, 旧子图本来就不可信; 新解被接受后
            //   几帧内重积累 (open 门: LOCALIZED + accepted + fitness>=write_bar)。
            {
                std::lock_guard<std::mutex> lk(m_session_map_mu);
                if (m_session_map.active()) m_session_map.deactivate();
            }
            // 注入位姿后的门限放行 (见 acceptMatch): 让被 reset 掉的先验偏置重新学回来.
            m_maha_bypass_frames = 5;
            {
                std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                if (m_prior) m_prior->reset(pending_pose);
            }
            {
                std::lock_guard<std::mutex> fk(m_fusion_mu);
                if (m_fusion) m_fusion->reset(pending_pose);
            }
            // 不 reset 创新监视器: 它统计的是"传感器 vs 地图"的固有创新特性
            // (含 3D 匹配与 IMU 在 roll/pitch 上约 12° 的固有偏差), 与位姿无关.
            // 清空后窗口内样本不足 -> d2 超过卡方门限 -> acceptMatch 每帧都拒
            // -> 无法累积 -> 状态永远停在 INIT (实测拉偏 1m 后 accepted=0 死锁).
            // 只有在换图 (setMapData) 时才需要重置.
            m_healthy_streak = 0;
            m_unhealthy_streak = 0;
            m_inconsistent_streak = 0;
            m_scrutiny_frames = 0;
            m_corr_prev_valid = false;
            m_consistent_cnt = 0;
            m_initialized = true;
            m_status = INIT;
            m_seed_pending = true;
            m_smoother.reset(m_pose);
            m_hyp.reset(m_pose);
        }
    }
    if (!m_map || !f.cloud || f.cloud->empty()) { m_status = LOST; return; }

    // 全局重定位不再在本地 tracking 路径里执行：由 globalRelocWorker 后台搜索，
    // 命中后通过 m_pending_init 注入；本帧继续做局部 GICP/状态机。
    {
        bool push_global = false;
        {
            std::lock_guard<std::mutex> lk(m_global_mu);
            if (m_global_cooldown > 0) {
                --m_global_cooldown;
            } else if (!m_initialized.load() || m_status.load() == LOST || m_global_request) {
                // 查询点云必须与地图关键帧同系(车体系): 局部跟踪用的是 prepareScan 之后的
                // 点云, 这里不能再用原始雷达系点云 (俯装雷达差 41.67°, 描述子对不上).
                m_global_scan = toBodyFrame(f.cloud);
                m_global_scan_generation = m_global_generation;
                m_global_scan_valid = true;
                m_global_request = true;
                push_global = true;
            }
        }
        if (push_global) m_global_cv.notify_one();
    }
    if (!m_initialized.load()) return;

    PoseStamped guess;
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        guess = m_prior->predict(f.end_time);
        planarizePose(guess);   // 先验也只保留 yaw, 不回传 z/roll/pitch 漂移
    }
    // coast 重锚: matcher 连续失败后先验(尤其 IMU-only)每帧漂 0.1~0.3m,
    // 漂出对应距离(2m)后 correspondences=0, 从此永久失配。连续失败超过
    // m_coast_reanchor_frames 帧时, 匹配初值改用最后被接受的位姿, 给 ICP
    // 一个未被漂移污染的起点; 匹配成功后 onMatch 会把先验拉回来。
    if (m_have_last_accepted && m_coast_count >= m_coast_reanchor_frames) {
        // 只重锚 **base 的位置**: IMU 投影积分的姿态可靠, 拉回旧姿态会让转弯永久冻住.
        // 雷达位置 = base 位置 + R·ρ (杆臂): 原地转弯时雷达绕 base 划弧, 直接把雷达
        // 平移拉回锚点会造出"位置在转前、姿态在转后"的矛盾位姿, 永远匹配不上
        // (2026-11-23 回放复现: 转到 120° 后 fitness 卡 0.47, 输出冻结).
        if (m_motion_cfg.lever_valid) {
            const V3D base_la =
                m_last_accepted.trans - m_last_accepted.rot * m_motion_cfg.lever_arm;
            guess.trans = base_la + guess.rot * m_motion_cfg.lever_arm;
        } else {
            guess.trans = m_last_accepted.trans;
        }
        guess.time = f.end_time;
    }
    // 定位路径原来直接用原始点云做 GICP；建图前端有 IMU 去畸变, 定位路径没有,
    // 转圈时一帧 100ms 内的旋转会让扫描在时间上被拉斜, 表现为 fitness 从 0.9
    // 掉到 0.4~0.7、inlier 比例骤降, 但 accepted 仍为 1. 这里按逐点时间戳和
    // 帧首/帧末位姿先补偿到帧末坐标系.
    CloudType::Ptr scan = prepareScan(f, guess);

    // relocalizer=none 时 globalRelocWorker 对重定位请求不做任何事(直接丢弃),
    // LOST 后没有任何恢复路径。这里在 tracking 线程里接上已有的
    // tryBoundedRecovery(±1.5m/±10° 多起点重锁, 与后续 tracking 同线程故
    // matcher 使用安全), 失败按 retry_interval 冷却后重试。
    if (m_relocalizer_name == "none" && m_initialized.load() &&
        m_status.load() == LOST) {
        bool try_now = false;
        {
            std::lock_guard<std::mutex> lk(m_global_mu);
            try_now = (m_global_cooldown == 0);
        }
        if (try_now && scan && !scan->empty()) {
            const bool recovered = tryBoundedRecovery(scan);
            {
                std::lock_guard<std::mutex> lk(m_global_mu);
                m_global_request = false;
                m_global_cooldown = std::max(1, m_global_cfg.retry_interval);
            }
            if (!recovered) {
                // 有界恢复每 retry_interval 秒建一次 27 个多起点假设, glibc 会把释放的
                // 临时点云留在 arena 里 (实机 LOST 时 RSS 涨到 2GB). 失败后主动 trim.
                malloc_trim(0);
            }
            if (recovered) {
                m_status = INIT;
                m_healthy_streak = 0;
                m_unhealthy_streak = 0;
                m_initialized = true;
                m_seed_pending = false;
                m_hyp.reset(m_pose);
            }
        }
    }

    MatchResult res;
    bool accepted = false;
    const char* reject_reason = "not_run";
    PoseStamped out_pose = guess;

    if (m_hyp.ambiguous()) {
        // L6 多假设: 逐假设局部配准并评分, 取最优, 淘汰落后
        const int nh = m_hyp.size();
        for (int i = 0; i < nh; ++i) {
            const PoseStamped g = m_hyp.all()[i].pose;
            if (!m_matcher->usesStore()) m_matcher->setTarget(buildTarget(g));
            MatchResult r;
            if (m_matcher->match(scan, g, r)) m_hyp.update(i, r);
        }
        const int bi = m_hyp.best();
        m_hyp.prune(m_hyp_margin, m_hyp_stale);
        if (bi >= 0 && bi < static_cast<int>(m_hyp.size())) {
            const PoseHypothesis& h = m_hyp.all()[bi];
            res.converged = true;
            res.fitness = h.fitness;
            res.pose = h.pose;
            out_pose = h.pose;
            accepted = h.fitness >= m_accept_cfg.min_coverage;
            reject_reason = accepted ? "accepted" : "coverage";
        }
        m_has_cached_target = false;
    resetSessionMap();
    } else {
        // 局部地图缓存: 位姿小幅移动(或全图)且**叠加层未变**时复用, 避免每帧重建
        // 降采样+KD-tree。之前配了维护层(m_maintainer)就直接放弃缓存 —— 但 overlay
        // 只在真正写入新点时变(revision 自增), 静态环境基本不增长; 而每帧重建 10k 目标
        // 的降采样+建树是把位姿输出拖到 10Hz 的主因之一(雷达 15.8Hz, 2026-10-17 实测)。
        bool rebuild_target = true;
        // 窗口中心 = 子图锚(只在已验证帧上跟随), 不是当前估计。
        // 这样弱帧(贴墙/稀疏/退化)的漂移不会立刻把 24 关键帧窗口带偏。
        // anchor_hold=false 时退回旧行为(直接用预测位姿建图), 便于 A/B 与现场回滚.
        const PoseStamped& target_center =
            (m_local_map_cfg.anchor_hold && m_have_submap_anchor) ? m_submap_anchor : guess;
        const uint64_t maint_rev = m_maintainer ? m_maintainer->revision() : 0;
        if (m_on_submap) {
            // 桥接: 目标 = 子图, 而子图每帧都在更新(SessionMap 没有 revision)。
            // 不强制刷新会一直用旧目标 -> 刚建的内容配不上。每 5 帧重建一次。
            static int sub_rebuild_n = 0;
            rebuild_target = (!m_has_cached_target) || (++sub_rebuild_n % 5 == 0);
        } else if (m_has_cached_target && maint_rev == m_cached_target_rev &&
            (m_local_map_name == "global" ||
             (target_center.trans - m_cached_target_pose.trans).norm() < m_local_map_cfg.cache_move)) {
            rebuild_target = false;
        }
        if (!m_matcher->usesStore() && rebuild_target) {
            CloudType::Ptr target = buildTarget(target_center);
            // **逸出保护 (fail fast)**: 局部地图为空 => 猜测位姿已经在图外(或该处无关键帧)。
            // 以前会拿空目标去匹配: 空 KD-tree 报错、ok=0、然后运动先验继续积分 ->
            // 位姿越漂越远(实机漂到 -96m), 从此永远匹配不上, 界面上角标彻底冻结。
            // 这里回退到最近一次被接受的位姿并复位运动先验, 让下一次仍有匹配机会。
            if (!target || target->empty()) {
                if (m_have_last_accepted) {
                    // 逸出回退同样"只拉 base 位置, 保留当前积分姿态": 转弯中把姿态
                    // 重置回锚点 = yaw 冻结, 之后永远追不上 (与 coast 重锚同源).
                    if (m_motion_cfg.lever_valid) {
                        const V3D base_la =
                            m_last_accepted.trans - m_last_accepted.rot * m_motion_cfg.lever_arm;
                        m_pose.trans = base_la + m_pose.rot * m_motion_cfg.lever_arm;
                    } else {
                        m_out_lim_bypass = true;
                        m_pose = m_last_accepted;
                    }
                    {
                        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                        if (m_prior) m_prior->reset(m_pose);
                    }
                    {
                        std::lock_guard<std::mutex> fk(m_fusion_mu);
                        if (m_fusion) m_fusion->reset(m_pose);
                    }
                    out_pose = m_pose;
                } else {
                    out_pose = guess;
                }
                out_pose.time = f.end_time;
                return;      // 本次判定为未定位, 让上层报"丢失"而不是静默
            }
            m_cached_target = target;
            m_cached_target_pose = target_center;
            m_cached_target_rev = m_maintainer ? m_maintainer->revision() : 0;
            m_has_cached_target = true;
            m_matcher->setTarget(target);
        }
        const bool ok = m_matcher->match(scan, guess, res);
        // 软着陆: 目标组成刚变(锚定<->桥接)时, 匹配解可能相对预测跳 3~5°.
        // 直接把跳变写进位姿, 会让"跳变前/跳变后"写进地图的点岔开几度(实车叠图).
        // 这里把单帧修正限到 1°/5cm, 用 10 帧爬到新解; 位姿连续 -> 地图不再分两层.
        if (m_soft_land_frames > 0 && res.converged) {
            V3D dt_sl;
            double da_sl = 0.0;
            poseDelta(res.pose, guess, dt_sl, da_sl);
            const double max_t = 0.05;                       // m/frame
            const double max_a = 1.0 * M_PI / 180.0;          // rad/frame
            const double nt = dt_sl.norm();
            const bool over = (nt > max_t) || (std::fabs(da_sl) > max_a);
            if (over) {
                const double kt = (nt > 1e-9) ? std::min(1.0, max_t / nt) : 1.0;
                const double ka = (std::fabs(da_sl) > 1e-9) ? std::min(1.0, max_a / std::fabs(da_sl)) : 1.0;
                const double k = std::min(kt, ka);
                res.pose.trans = guess.trans + k * dt_sl;
                res.pose.rot =
                    guess.rot * Eigen::AngleAxisd(k * da_sl, V3D::UnitZ()).toRotationMatrix();
                planarizePose(res.pose);
            }
            --m_soft_land_frames;
        }
        // [DumpFrame] (ROSIWIT_DUMP_FRAME=1 时) 转储好帧(first fitness>=0.9 ->
        // /tmp/slamdump_good) 与坏帧 (first fitness<0.6 -> /tmp/slamdump), 离线对比
        // z 分布/覆盖率/位姿 (2026-11-23 地面裁剪与 guess 滞后对账). 默认不写盘.
        auto dump_frame = [&](const char* tag, const char* subdir) {
            const std::string dir = std::string("/tmp/") + subdir;
            (void)system(("mkdir -p " + dir).c_str());
            try {
                pcl::io::savePCDFileBinary(dir + "/scan.pcd", *scan);
                if (m_cached_target) pcl::io::savePCDFileBinary(dir + "/target.pcd", *m_cached_target);
                std::ofstream df(dir + "/poses.txt");
                df << "fitness " << res.fitness << "\n";
                df << "guess_time " << std::fixed << std::setprecision(6) << guess.time << "\n";
                df << "guess_trans " << guess.trans.transpose() << "\n";
                df << "guess_rot " << Eigen::Quaterniond(guess.rot).coeffs().transpose() << "\n";
                df << "res_trans " << res.pose.trans.transpose() << "\n";
                df << "res_rot " << Eigen::Quaterniond(res.pose.rot).coeffs().transpose() << "\n";
                df << "frame_start " << std::fixed << std::setprecision(6) << f.start_time << "\n";
                df << "frame_end " << f.end_time << "\n";
            } catch (const std::exception&) {}
            std::cout << "[DumpFrame] " << tag << " fitness=" << res.fitness
                      << " scan=" << scan->size() << std::endl;
        };
        if (kDumpFrame) {
            static bool dumped_good = false;
            static bool dumped_bad = false;
            if (scan && !scan->empty()) {
                if (!dumped_good && res.fitness >= 0.9) {
                    dumped_good = true;
                    dump_frame("good", "slamdump_good");
                }
                if (!dumped_bad && res.fitness > 0.0 && res.fitness < 0.6) {
                    dumped_bad = true;
                    dump_frame("bad", "slamdump");
                }
            }
        }
        // 匹配结果同样锁成 x/y/yaw + 固定安装姿态 (z/roll/pitch 弱观测, 不允许漂)
        if (ok && res.converged) planarizePose(res.pose);
        // [LocProbe] 逐帧诊断: 帧时间 + 可见点/参与点/内点 + fitness + guess/res 完整位姿;
        // 离线对 odom 真值算每帧滞后 (/tmp/frame_lag.py). ROSIWIT_LOC_PROBE=1 逐帧,
        // 否则 50 帧一条.
        static int probe_n = 0;
        if (kLocProbe || ++probe_n % 50 == 0) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(3)
               << "[LocProbe] t=" << f.end_time
               << " vis=" << res.visible << " used=" << res.correspondences
               << " inl=" << res.inliers
               << " ok=" << (ok ? 1 : 0) << " conv=" << (res.converged ? 1 : 0)
               << " fit=" << std::setprecision(4) << res.fitness
               << " fitall=" << res.fitness_all
               << " gx=" << guess.trans.x() << " gy=" << guess.trans.y()
               << " gz=" << guess.trans.z()
               << " gyaw=" << std::atan2(guess.rot(1, 0), guess.rot(0, 0)) * 57.2957795
               << " rx=" << res.pose.trans.x() << " ry=" << res.pose.trans.y()
               << " rz=" << res.pose.trans.z()
               << " ryaw=" << std::atan2(res.pose.rot(1, 0), res.pose.rot(0, 0)) * 57.2957795;
            std::cout << os.str() << std::endl;
        }
        reject_reason = ok ? "accepted" : "matcher_failed";
        accepted = ok && acceptMatch(res, scan, guess, &reject_reason);
        out_pose = accepted ? res.pose : guess;

        // ---- 冻结逃逸 (2026-11-25) ----
        // 只在"连续 N 帧匹配结果彼此一致、且匹配质量本来就够 min_coverage"时放行一次.
        // 典型场景: 位姿偏出 max_trans_innov 后被永久拒收, 匹配却每帧稳定指向真值.
        // 错误匹配(相似结构)通常帧间乱跳, 过不了这里的"彼此一致"检查.
        if (m_accept_cfg.escape_enabled) {
            const bool candidate = !accepted && ok && res.converged && !m_seed_pending &&
                res.fitness >= m_accept_cfg.min_coverage &&
                (res.pose.trans - guess.trans).norm() <= m_accept_cfg.escape_max_jump;
            bool consistent = false;
            if (candidate) {
                if (!m_escape_prev_valid) {
                    m_escape_first = res.pose;
                    m_escape_cnt = 1;
                    consistent = true;
                } else {
                    const double dt = (res.pose.trans - m_escape_prev.trans).norm();
                    const M3D dR = res.pose.rot * m_escape_prev.rot.transpose();
                    const double dang = std::fabs(Eigen::AngleAxisd(dR).angle());   // rad
                    const double spread = (res.pose.trans - m_escape_first.trans).norm();
                    consistent = dt <= m_accept_cfg.escape_consist_trans &&
                                 dang <= m_accept_cfg.escape_consist_rot_deg / 57.2957795 &&
                                 spread <= m_accept_cfg.escape_max_spread;
                    m_escape_cnt = consistent ? m_escape_cnt + 1 : 1;
                    if (!consistent) m_escape_first = res.pose;
                }
                m_escape_prev = res.pose;
                m_escape_prev_valid = true;
            } else {
                m_escape_cnt = 0;
                m_escape_prev_valid = false;
            }
            if (m_escape_cnt >= m_accept_cfg.escape_frames &&
                m_coast_count >= m_accept_cfg.escape_min_coast &&
                f.end_time - m_escape_last_fire_t >= m_accept_cfg.escape_cooldown_s) {
                // 放行: 位姿直接放到匹配结果上, 并把先验/融合一起重置到该位姿
                // (与外部注入位姿同一条路径), 之后靠 bypass 让偏置重新学回来.
                const double jump = (res.pose.trans - guess.trans).norm();
                accepted = true;
                out_pose = res.pose;
                reject_reason = "escape";
                m_escape_last_fire_t = f.end_time;
                m_escape_cnt = 0;
                m_escape_prev_valid = false;
                m_coast_count = 0;
                m_maha_bypass_frames = std::max(m_maha_bypass_frames, 5);
                m_last_accepted = res.pose;
                m_have_last_accepted = true;
                m_consistent_cnt = 0;
                m_corr_prev_valid = false;
                {
                    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                    if (m_prior) m_prior->reset(res.pose);
                }
                {
                    std::lock_guard<std::mutex> fk(m_fusion_mu);
                    if (m_fusion) m_fusion->reset(res.pose);
                }
                std::cout << "[Escape] jump=" << jump
                          << " fit=" << res.fitness
                          << " inl=" << res.inliers
                          << " t=" << f.end_time << std::endl;
            }
        }

        // 退化方向部分更新 (B): 在 JtJ 特征基下按可观测度缩放修正量.
        // 旋转时对应点集中在单面长墙, 沿墙平移方向几乎不可观测, ICP 仍会给出
        // 一个高分的"沿墙滑动"解 (实测 0.5rad/s 转 20s 滑走 3m, fitness 0.9).
        // 弱观测方向增益趋 0 => 保留先验; 良态方向增益≈1 => 正常采纳.
        if (accepted && m_accept_cfg.degeneracy_partial_update && res.has_hessian) {
            const Eigen::Matrix<double, 6, 1> lam = res.hess_eigvals;
            const double lmax = lam.maxCoeff();
            if (lmax > 1e-12) {
                Eigen::Matrix<double, 6, 1> e;
                const M3D dR = res.pose.rot * guess.rot.transpose();
                Eigen::AngleAxisd aa(dR);
                V3D axis = aa.axis();
                if (!axis.allFinite()) axis = V3D::Zero();
                e.head<3>() = axis * aa.angle();
                e.tail<3>() = res.pose.trans - guess.trans;
                const Eigen::Matrix<double, 6, 1> coeff = res.hess_eigvecs.transpose() * e;
                Eigen::Matrix<double, 6, 1> gain;
                for (int k = 0; k < 6; ++k) {
                    const double r = std::max(0.0, lam(k)) / lmax;
                    gain(k) = r / (r + m_accept_cfg.degeneracy_ratio_soft);
                }
                const Eigen::Matrix<double, 6, 1> e2 =
                    res.hess_eigvecs * (gain.array() * coeff.array()).matrix();
                Eigen::Matrix3d Rn = Eigen::Matrix3d::Identity();
                const double ang = e2.head<3>().norm();
                if (ang > 1e-12) {
                    Rn = Eigen::AngleAxisd(ang, e2.head<3>() / ang).toRotationMatrix();
                }
                PoseStamped proj = res.pose;
                proj.rot = Rn * guess.rot;
                proj.trans = guess.trans + e2.tail<3>();
                res.pose = proj;
                out_pose = proj;
                static int dprobe = 0;
                if (++dprobe % 50 == 0) {
                    std::cout << "[DegeneracyProbe] gain=[" << gain(0) << "," << gain(1) << ","
                              << gain(2) << "," << gain(3) << "," << gain(4) << "," << gain(5)
                              << "] eig_ratio_min=" << (lam.minCoeff() / lmax)
                              << " corr_before=" << e.tail<3>().norm()
                              << " corr_after=" << e2.tail<3>().norm() << std::endl;
                }
            }
        }
    }

    // 初始位姿(seed)首帧校验：seed 错误时局部 GICP 可能在相似结构上得到
    // 0.8 左右的“看起来不错”分数并被接受；这里用独立、可配置的门限阻断。
    // 失败仍保持 INIT，交给统一的 lost_patience 连续帧计数判丢。
    bool seed_failed = false;
    if (m_seed_pending && res.correspondences > 0 &&
        res.correspondences < m_matcher_cfg.min_coverage_points) {
        // 退化帧 (参与配准的点太少) 不裁决种子 (2026-11-23): 这种帧的分数与位姿都没有
        // 统计意义 —— 实车重锁时雷达正被近处物体包围, 238 点的"满分"匹配把正确的种子
        // 判成 "漂离 1.7m" 而拒掉, 节点停在 INIT. 保持 pending, 等一帧点数足够的扫描.
    } else if (m_seed_pending) {
        m_seed_pending = false;
        m_soft_land_frames = 20;   // 种子刚生效: 前 20 帧限幅, 别让首帧跳变写进地图
        if (!res.converged || res.fitness < m_global_cfg.reloc_accept_score) {
            seed_failed = true;
            m_status = INIT;
            std::ostringstream os;
            os << "initial seed rejected: fitness=" << res.fitness
               << " < " << m_global_cfg.reloc_accept_score << " -> INIT";
            diag::log('W', "LocPipeline", os.str());
            FlightRecorder::instance().event("SEED_REJECT", os.str());
            m_last_match = res;
            m_last_match.pose = guess;
        } else {
            // 2026-10-24: 距离门限 —— 种子帧一帧漂 3m 也能拿 0.87 分被接受,
            // 然后锁死在图外(实测约 30% 启动概率)。距种子超 1m/15° 按
            // seed 失败处理, 不让 ICP 在种子帧"漂移过去"。
            V3D dseed; double dyaw_seed = 0.0;
            poseDelta(res.pose, guess, dseed, dyaw_seed);
            const double seed_trans = dseed.norm();
            const double seed_yaw_deg = std::fabs(dyaw_seed) * 57.2957795;
            if (seed_trans > m_global_cfg.seed_max_drift ||
                seed_yaw_deg > m_global_cfg.seed_max_yaw_deg) {
                seed_failed = true;
                accepted = false;
                out_pose = guess;
                m_status = INIT;
                std::ostringstream os;
                os << "initial seed rejected: drifted " << seed_trans
                   << "m/" << seed_yaw_deg << "deg from seed (fitness="
                   << res.fitness << ") -> INIT";
                diag::log('W', "LocPipeline", os.str());
                FlightRecorder::instance().event("SEED_REJECT", os.str());
                m_last_match = res;
                m_last_match.pose = guess;
            } else if (m_global_cfg.seed_neighbor_check && m_matcher && scan && !scan->empty() &&
                       res.fitness < 0.95) {
            // 邻域最优性校验: 在种子解附近做 ±trans/±yaw 的候选匹配。
            // 只要有一个候选明显更优(> margin), 说明种子落在局部最优的"错误一侧",
            // 拒绝并保持 INIT —— 宁可让现场重新给位姿, 也不锁死在一张偏了的地图上。
            const double dtc = m_global_cfg.seed_check_trans_m;
            const double dac = m_global_cfg.seed_check_yaw_deg * M_PI / 180.0;
            double best_fit = res.fitness;
            PoseStamped best_pose = res.pose;
            const double offs[4][2] = {{dtc, 0}, {-dtc, 0}, {0, dtc}, {0, -dtc}};
            for (const auto& off : offs) {
                PoseStamped g = res.pose;
                g.trans += V3D(off[0], off[1], 0.0);
                planarizePose(g);
                MatchResult r2;
                if (m_matcher->match(scan, g, r2) && r2.converged && r2.fitness > best_fit) {
                    best_fit = r2.fitness;
                    best_pose = r2.pose;
                }
            }
            for (double dy : {dac, -dac}) {
                PoseStamped g = res.pose;
                g.rot = res.pose.rot * Eigen::AngleAxisd(dy, V3D::UnitZ()).toRotationMatrix();
                planarizePose(g);
                MatchResult r2;
                if (m_matcher->match(scan, g, r2) && r2.converged && r2.fitness > best_fit) {
                    best_fit = r2.fitness;
                    best_pose = r2.pose;
                }
            }
            if (best_fit > res.fitness + m_global_cfg.seed_check_margin) {
                seed_failed = true;
                accepted = false;
                out_pose = guess;
                m_status = INIT;
                std::ostringstream os;
                os << "initial seed rejected: neighbor better fitness=" << best_fit
                   << " vs seed " << res.fitness << " (offset "
                   << (best_pose.trans - res.pose.trans).norm() << "m) -> INIT";
                diag::log('W', "LocPipeline", os.str());
                FlightRecorder::instance().event("SEED_REJECT", os.str());
                m_last_match = res;
                m_last_match.pose = guess;
            }
        }
    }
    }

    out_pose.time = f.end_time;
    // L19 绝对约束融合 (门控互补): 新鲜且创新在门限内才采纳
    if (m_abs_cfg.enabled && m_has_abs &&
        std::fabs(f.end_time - m_abs_time) < m_abs_cfg.window) {
        const V3D dt = m_abs_pose.trans - out_pose.trans;
        Eigen::Quaterniond qe(Eigen::Quaterniond(out_pose.rot).conjugate() *
                              Eigen::Quaterniond(m_abs_pose.rot));
        const double dyaw_deg = std::fabs(2.0 * std::atan2(qe.vec().norm(), std::fabs(qe.w()))) * 57.2957795;
        if (dt.norm() <= m_abs_cfg.max_trans && dyaw_deg <= m_abs_cfg.max_yaw_deg) {
            const double a = m_abs_cfg.alpha;
            out_pose.trans = (1.0 - a) * out_pose.trans + a * m_abs_pose.trans;
            Eigen::Quaterniond q_cur(out_pose.rot);
            out_pose.rot = q_cur.slerp(a, Eigen::Quaterniond(m_abs_pose.rot).normalized())
                               .normalized().toRotationMatrix();
        }
        m_has_abs = false;   // 一次性消费
    }
    // L4 固定滞后平滑: 融合运动先验(between) + 扫描匹配(unary, 带协方差)
    if (m_smoother.enabled()) {
        m_smoother.addFrame(f.end_time, guess, accepted, out_pose, res.cov);
        PoseStamped sp;
        if (m_smoother.getPose(sp)) out_pose = sp;
    }
    // ---- 子图模式: 周期性对 overlay 做探测匹配 (重锚尝试, ELite zip-up) ----
    // overlay 恢复的判据: 用当前位姿对 overlay(底图+w) 目标做一次完整匹配,
    // fitness 达 reloc_accept_score 即重锚; 子图漂移用 Δ 修正 (zip-up)。
    if (m_on_submap && --m_probe_countdown <= 0) {
        m_probe_countdown = m_probe_period;
        CloudType::Ptr probe_target = buildAnchoredTarget(guess);
        if (probe_target && probe_target->size() >= 100) {
            m_matcher->setTarget(probe_target);
            MatchResult pr;
            if (m_matcher->match(scan, out_pose, pr) && pr.converged &&
                pr.fitness >= m_global_cfg.reloc_accept_score) {
                // ★段级出口修正 (SessionMap v2, ROLL Fig.3 简化闭式解): 用可靠
                // 匹配位姿作末端约束, 把整段关键帧的漂移按序号线性摊平 ——
                // 替代 v1 的单点 zip-up 刚性搬移 (漂移大时会撕裂子图内部)。
                {
                    std::lock_guard<std::mutex> lk(m_session_map_mu);
                    m_session_map.applyExitCorrection(pr.pose);
                }
                out_pose = pr.pose;
                res = pr;
                accepted = true;
                m_on_submap = false;
                m_maha_bypass_frames = 5;
                diag::log('I', "LocPipeline",
                          "re-anchored to overlay (segment exit-correction applied)");
            }
            m_has_cached_target = false;   // 下一帧按 m_on_submap 重建常规目标
        }
    }
    // 地面/平面约束 (锁 z/roll/pitch; 坡道跟随地形)
    {
        // [pose_cmp] 追踪 33deg 倾角在哪一步出现/消失 (每 20 帧一条, 纯打印)
        static int cmp_n = 0;
        if (++cmp_n % 20 == 0) {
            auto rpy = [](const M3D& R) {
                Eigen::Vector3d v;
                v.x() = std::atan2(R(2, 1), R(2, 2)) * 57.2957795;
                v.y() = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0)))) * 57.2957795;
                v.z() = std::atan2(R(1, 0), R(0, 0)) * 57.2957795;
                return v;
            };
            const Eigen::Vector3d rm = rpy(res.pose.rot);
            const Eigen::Vector3d rg = rpy(guess.rot);
            const Eigen::Vector3d ro = rpy(out_pose.rot);
            std::cout.setf(std::ios::fixed);
            std::cout.precision(2);
            std::cout << "[pose_cmp] accepted=" << accepted
                      << " match=(" << rm.x() << "," << rm.y() << "," << rm.z() << ")"
                      << " guess=(" << rg.x() << "," << rg.y() << "," << rg.z() << ")"
                      << " out=(" << ro.x() << "," << ro.y() << "," << ro.z() << ")"
                      << " z=" << out_pose.trans.z() << std::endl;
            std::cout.unsetf(std::ios::fixed);
        }
    }
    m_planar.apply(out_pose, scan);
    {
        static int cmp_n = 0;
        if (++cmp_n % 20 == 0) {
            const double r = std::atan2(out_pose.rot(2, 1), out_pose.rot(2, 2)) * 57.2957795;
            const double pi_ = std::asin(std::max(-1.0, std::min(1.0, -out_pose.rot(2, 0)))) * 57.2957795;
            std::cout.setf(std::ios::fixed);
            std::cout.precision(2);
            std::cout << "[pose_cmp] after_planar rpy=(" << r << "," << pi_
                      << ") z=" << out_pose.trans.z() << std::endl;
            std::cout.unsetf(std::ios::fixed);
        }
    }
    if (accepted) {
        {
            std::lock_guard<std::mutex> prior_lk(m_prior_mu);
            m_prior->onMatch(out_pose);
        }
        // 位姿融合层: 把匹配结果作为"带协方差的观测"喂给估计器 (direct 模式下等价于直通)
        {
            std::lock_guard<std::mutex> fk(m_fusion_mu);
            if (m_fusion) m_fusion->onMatch(out_pose, res.cov, res);
            // eskf: 下游看到的位姿改用融合输出 (单帧修正有界)
            if (m_fusion && m_fusion->name() != "direct") {
                out_pose = m_fusion->predict(out_pose.time);
                // [FusProbe] (ROSIWIT_LOC_PROBE=1) 融合层内部状态: 平面锁之前的姿态
                // (roll/pitch 是否被匹配修正带歪)、位置 vs 匹配、修正量与门控诊断
                if (kLocProbe) {
                    const FusionDiag fd = m_fusion->diag();
                    const M3D& Rf = out_pose.rot;
                    std::ostringstream os;
                    os << std::fixed << std::setprecision(3) << "[FusProbe] t=" << out_pose.time
                       << " fx=" << out_pose.trans.x() << " fy=" << out_pose.trans.y()
                       << " fz=" << out_pose.trans.z()
                       << " mx=" << res.pose.trans.x() << " my=" << res.pose.trans.y()
                       << " roll=" << std::setprecision(2)
                       << std::atan2(Rf(2, 1), Rf(2, 2)) * 57.2957795
                       << " pitch=" << std::asin(std::max(-1.0, std::min(1.0, -Rf(2, 0)))) * 57.2957795
                       << " yaw=" << std::atan2(Rf(1, 0), Rf(0, 0)) * 57.2957795
                       << " applied_t=" << fd.applied_trans << " gain_t=" << fd.last_gain_trans
                       << " chi2=" << fd.last_chi2 << " odom_w=" << fd.odom_weight
                       << " acc=" << fd.accepted << " rej_chi2=" << fd.rejected_chi2
                       << " rej_inplace=" << fd.rejected_in_place;
                    std::cout << os.str() << std::endl;
                }
            }
        }
        m_last_accepted = out_pose;      // 逸出保护用的回退锚点
        m_have_last_accepted = true;
    }
    // ---- 输出位姿限速 (安全防护, 2026-11-26) ----
    // 匹配在两个对齐之间振荡时, 单帧位姿跳变可达 ±15m —— 直通导航会让导航
    // 看到"瞬移"而急停/窜车 (实机: 机器人被吓跑)。这里把超物理上限的位姿变化
    // 按上限速度滑移输出; 合法跳变 (种子/有界恢复/换图) 走 m_out_lim_bypass 旁路。
    if (m_out_lim_bypass) {
        m_out_lim_bypass = false;
        m_out_lim_valid = false;
    }
    if (m_out_lim_valid) {
        const double dtl = out_pose.time - m_out_lim_prev.time;
        if (dtl > 1e-3 && dtl < 0.5) {
            V3D dp = out_pose.trans - m_out_lim_prev.trans;
            const double v = dp.norm() / dtl;
            if (v > m_out_v_max) {
                const double kk = (m_out_v_max * dtl) / dp.norm();
                out_pose.trans = m_out_lim_prev.trans + dp * kk;
                static double s_lv = 0.0;
                if (out_pose.time - s_lv > 2.0) {
                    s_lv = out_pose.time;
                    std::cout << "[OutLim] 位姿跳变限速: " << v << " m/s -> "
                              << m_out_v_max << " m/s" << std::endl;
                }
            }
            const M3D Rrel = m_out_lim_prev.rot.transpose() * out_pose.rot;
            const double dyaw = std::atan2(Rrel(1, 0), Rrel(0, 0));
            const double wz = dyaw / dtl;
            if (std::fabs(wz) > m_out_wz_max) {
                const double dyc = std::copysign(m_out_wz_max * dtl, dyaw);
                Eigen::Quaterniond qo(out_pose.rot);
                qo = qo * Eigen::Quaterniond(Eigen::AngleAxisd(dyc - dyaw, V3D::UnitZ()));
                qo.normalize();
                out_pose.rot = qo.toRotationMatrix();
            }
        }
    }
    m_out_lim_prev = out_pose;
    m_out_lim_valid = true;
    m_pose = out_pose;
    m_last_match = res;
    m_last_match.pose = out_pose;
    // ---- 子图窗口锚 (2026-11-25) ----
    // 判据都不看点数(对接工位只有 ~200 点但位姿是对的):
    //   1) 本帧被接受且收敛;
    //   2) 可观测性够 == 不是"沿墙/沿走廊"退化(③-1 给 gicp 补的 eig; 无 Hessian 时跳过);
    //   3) fitness 达到 anchor_min_fitness (默认 0 = 不看 fitness: 本环境正常帧只有 0.6~0.88);
    //   4) 与预测的创新量不超限(不是被硬拉过去的)  <-- 真正起作用的判据.
    // 其余帧保持旧锚点; 滞后超过 anchor_max_lag(默认 1m) 必须跟随, 否则窗口会跑空.
    // **默认关闭**: 同包回归里开它反而更容易丢(窗口滞后会把匹配往后拽), 详见 default.yaml.
    const double innov_t = (res.pose.trans - guess.trans).norm();
    const double anchor_lag = m_have_submap_anchor
        ? (guess.trans - m_submap_anchor.trans).norm() : -1.0;
    if (m_local_map_cfg.anchor_hold) {
        const bool eig_ok = !res.has_hessian ||
            res.smallest_eig_ratio >= m_local_map_cfg.anchor_min_eig_ratio;
        const bool verified = accepted && res.converged && eig_ok &&
            res.fitness >= m_local_map_cfg.anchor_min_fitness &&
            (m_accept_cfg.max_trans_innov <= 0.0 || innov_t <= m_accept_cfg.max_trans_innov);
        // 强制跟随只把窗口中心拉回"预测位姿"(不动位姿本身), 保证锚点不会永久跑空.
        const bool forced = anchor_lag > m_local_map_cfg.anchor_max_lag;
        if (!m_have_submap_anchor || verified) {
            m_submap_anchor = res.pose;
            m_have_submap_anchor = true;
        } else if (forced) {
            m_submap_anchor = guess;
        }
    }
    // [ObsProbe] 方向可观测性 (2026-11-25): gicp 首次真正算出 λmin/λmax 与退化方向数.
    // 与 anchor_hold 无关, 只要 ROSIWIT_LOC_PROBE=1 就每 20 帧打一条, 用于现场标定
    // degeneracy_ratio_soft / anchor eig 门限 (以前 gicp 不填, 日志里恒为 1.000/0).
    {
        static int obs_probe_n = 0;
        if (kLocProbe && (++obs_probe_n % 20 == 0)) {
            std::cout << "[ObsProbe] t=" << f.end_time
                      << " eig=" << res.smallest_eig_ratio
                   << " sup=" << (res.w_support >= 0.0 ? res.w_support : -1.0)
                      << " dirs=" << res.degenerate_dirs
                      << " hess=" << (res.has_hessian ? 1 : 0)
                      << " fit=" << res.fitness
                      << " innov=" << innov_t
                      << " anchor_lag=" << anchor_lag
                      << " anchor_hold=" << (m_local_map_cfg.anchor_hold ? 1 : 0)
                      << std::endl;
        }
    }
    // [DropProbe] 低 fitness 或被拒绝帧逐帧打印（正常帧仍走 10 帧一条的 LocProbe）。
    //   correction_m = |匹配位姿 - 先验猜测|：持续增大说明预测跟不上；
    //   correspondences 骤降说明动态滤波把真实位置证据删掉；
    //   target_points 变化说明目标子图换源/关键帧覆盖缺口。
    if (res.fitness < 0.90 || !accepted) {
        const double lag = (out_pose.trans - guess.trans).norm();
        std::cout << "[DropProbe] t=" << std::fixed << std::setprecision(3) << f.end_time
                  << " fitness=" << std::setprecision(4) << res.fitness
                  << " correction_m=" << lag
                  << " correction_deg="
                  << std::acos(std::max(-1.0, std::min(1.0,
                         (out_pose.rot.transpose() * guess.rot).trace() * 0.5 - 0.5))) * 57.2957795
                  << " correspondences=" << res.correspondences
                  << " inliers=" << res.inliers
                  << " target_points=" << (m_has_cached_target && m_cached_target ? m_cached_target->size() : 0)
                  << " accepted=" << (accepted ? 1 : 0)
                  << " coast_frames=" << m_coast_count
                  << " rejection=" << reject_reason << std::endl;
    }
    // 飞行记录仪: 每帧一条(与 DropProbe 同字段), LOST 翻转自动落盘
    {
        diag::FrameRecord fr;
        fr.t = f.end_time;
        fr.status = (m_status == LOST) ? 2 : (m_status == LOCALIZED ? 1 : 0);
        fr.fitness = res.fitness;
        fr.lag = (out_pose.trans - guess.trans).norm();
        fr.corr = res.correspondences;
        fr.tgt = static_cast<int>(m_has_cached_target && m_cached_target
                                      ? m_cached_target->size() : 0);
        fr.accepted = accepted ? 1 : 0;
        fr.coast = m_coast_count;
        fr.hyp = m_hyp.size();
        FlightRecorder::instance().record(fr);
    }
    // 单假设模式下同步假设容器
    if (!m_hyp.ambiguous()) m_hyp.reset(m_pose);

    // 地图维护: 只在"位姿可信 + 本帧被接受 + 两路来源不矛盾"时写入/雕刻叠加层。
    // 插入比定位更严格: 错误位姿写进先验地图后会变成"地图的一部分", 更难恢复。
    // - LOCALIZED 会跨过 lost_patience 保持, 所以再要求本帧 fitness 达到可信阈值
    // - LIO 残差只在新鲜时参与 (LIO 停掉/只重锚时不拿旧值一直拦住插入)
    // 写图一致性观测: 点云按 guess(帧末预测)去畸变后已经落在"帧末 body 系", 写入时用的是
    // out_pose/m_pose. 两者之差 = 本帧的预测误差; 但真正会被烙进图里的是"最终位姿 vs 匹配
    // 位姿(res.pose)"的差: 匹配位姿才是让这片点云对上先验地图的那个位姿.
    const M3D R_corr = out_pose.rot * guess.rot.transpose();
    const double corr_deg =
        std::acos(std::max(-1.0, std::min(1.0, (R_corr.trace() - 1.0) * 0.5))) * 57.2957795;
    const double corr_m = (out_pose.trans - guess.trans).norm();
    const M3D R_mm = out_pose.rot * res.pose.rot.transpose();
    const double match_deg =
        std::acos(std::max(-1.0, std::min(1.0, (R_mm.trace() - 1.0) * 0.5))) * 57.2957795;
    const double match_m = (out_pose.trans - res.pose.trans).norm();
    static int insert_probe_n = 0;
    static PoseStamped prev_insert;
    static bool have_prev_insert = false;
    double yaw_rate = 0.0;
    if (have_prev_insert && out_pose.time > prev_insert.time + 1e-6) {
        const double dy = std::atan2(
            (out_pose.rot * prev_insert.rot.transpose())(1, 0),
            (out_pose.rot * prev_insert.rot.transpose())(0, 0));
        yaw_rate = dy / (out_pose.time - prev_insert.time);
    }
    prev_insert = out_pose;
    have_prev_insert = true;
    // 大修正(>0.3m / >5°)后, 之前写进会话子图的点在世界系里就"错位"了: 不清掉的话,
    // 目标和调试云里都会出现新旧两层墙(实车"叠图"). 子图是临时补丁, 重建代价很低。
    // 大修正后子图的处理 (2026-11-25 修订: 只在桥接态搬移):
    //   桥接态: 位姿由子图自证, 修正后必须把子图一起搬 (T_GS' = Delta o T_GS),
    //           否则位姿和补丁脱钩, 下帧目标就错 (搬移在这里是必须的)。
    //   锚定态: 位姿由**底图**校正, 世界没动, 子图点本来就写在世界系里 ——
    //           把局部修正抹到整个 35m 补丁上, 是把噪声固化进补丁 (实测: 每次搬移后
    //           [submap-prior] 残差台阶式变大, 累计 ~0.75m 后 fitness 掉到 0.57 判丢)。
    //           修正只影响"之后写入的点", 老点不动。
    if (m_on_submap && m_session_map.active() &&
        (corr_m > 0.15 || corr_deg > 1.5)) {
        // 不管当前是锚定还是桥接, 一律"刚性搬移": 清空子图在任何状态下都可能把
        // 唯一的跟踪源丢掉; 而点不动、只改 anchor 变换(T_GS' = Δ∘T_GS)是等价且无损的。
        // Δ = res ∘ guess⁻¹ (作用在世界系)。
        const M3D dR = res.pose.rot * guess.rot.transpose();
        const V3D dT = res.pose.trans - dR * guess.trans;
        {
            std::lock_guard<std::mutex> lk(m_session_map_mu);
            m_session_map.applyWorldCorrection(dR, dT);
        }
        static int reanchor_log = 0;
        if (++reanchor_log % 20 == 1) {
            std::cout << "[session_map] re-anchor by correction corr_m=" << corr_m
                      << " corr_deg=" << corr_deg << " pts=" << m_session_map.size()
                      << std::endl;
        }
    }
    if (kLocProbe && (++insert_probe_n % 10 == 0) && (corr_deg > 1.0 || match_deg > 0.5)) {
        std::cout << "[InsertProbe] corr_deg=" << corr_deg << " corr_m=" << corr_m
                  << " match_deg=" << match_deg << " match_m=" << match_m
                  << " yawrate=" << yaw_rate
                  << " fitness=" << res.fitness << " accepted=" << (accepted ? 1 : 0)
                  << std::endl;
    }

    // ★写图专用点云: 用匹配解 res.pose 重新去畸变。
    // deskew 是按预测位姿 guess 做的, 与匹配解差 corr_deg(常态 0.1~3°) 时, 整片点云
    // 会被转差那几度 —— 匹配用的是"转过的点云 vs 地图", 还能对上; 但写进世界系时
    // 同一面墙就分成了两层(建图模式不会, 因为它用同一套位姿做去畸变和写图)。
    CloudType::Ptr scan_write;
    if ((m_maintainer || m_session_map_enabled) && res.converged) {
        scan_write = prepareScan(f, res.pose);
    }
    const CloudType::Ptr& scan_map =
        (scan_write && !scan_write->empty()) ? scan_write : scan;

    if (m_maintainer) {
        const FusionDiag fd = fusionDiag();
        double divergence = 0.0;
        if (fd.last_rel_time >= 0.0 && f.end_time - fd.last_rel_time <= 0.5) {
            divergence = fd.last_rel_residual;
        }
        const bool tracking = (m_status == LOCALIZED) &&
            (!m_monitor_cfg.enabled || res.fitness >= m_monitor_cfg.credible_fitness);
        // 插入比定位更严格: 位姿必须通过"锚质量"判据 (内点 + JtJ 最小特征值比),
        // 只看 fitness 会把 0.80~0.84 的假锁当成可信, 把错误几何写进 overlay.
        // 写图用**匹配位姿 res.pose**, 不用融合后的 out_pose: 匹配位姿才是"让这片
        // 去畸变点云对上先验地图"的那个位姿, 与地图几何天然一致; 融合位姿带滤波/限幅
        // 滞后 (转弯时实测差 28°), 直接写进去就是墙歪/叠图, 而且会被当成地图的一部分.
        //
        // T4 (2026-11-25, 工作站): 写图必须锚在**底图**上, 不能拿"含 overlay/子图的目标"判胜
        // (自引用): 桥接期目标就是子图, 位姿由子图自身算出, 再写回 overlay = 自证;
        // cov_base 有效时要求达到可信阈值 (底图真的配上了); cov_base 还没算出来 (-1,
        // 底图 KD 未就绪) 时退回 anchorOk, 否则起步阶段完全学不到东西。
        // 注: cov_base 在本帧稍后才更新, 这里用的是上一帧的值 (1 帧滞后, 可接受)。
        // ★门槛用 res.fitness 本身 (2026-11-25 修订): 锚定态的目标 = 底图 + overlay(无子图),
        // 所以 res.fitness 就是"对先验图的匹配度", 口径与 matching_degree 一致。
        // 原实现拿 cov_base 当门槛 —— 那是每 5 帧一次、融合位姿、0.4 体素降采样的粗测,
        // 实测把 0.93 的真实匹配读成 0.67, 门槛被焊死 (novel_votes 恒 0, overlay 写不进)。
        // overlay 自证的风险由既有约束兜住: 进目标的 overlay 点需 w>=target_w_min/match_w_min,
        // 写入需 rmse<=max_rmse + corr<=10cm/1° + anchorOk。
        const bool base_anchored = !m_on_submap &&
            res.fitness >= m_monitor_cfg.credible_fitness;
        // 机器人侧补充 (2026-11-24): 位姿稳定性门 —— 刚发生 >10cm/>1° 修正的帧、
        // coast 帧(预测顶着的)位姿不可信, rmse 超标说明残余大; 写进去会和原有结构
        // 岔成两层(实车叠图)。与 T4 正交, 叠加生效。
        const bool rmse_ok = (res.rmse < 0.0 || m_maintainer_cfg.max_rmse <= 0.0 ||
                              res.rmse <= m_maintainer_cfg.max_rmse);
        const bool pose_stable = (corr_m <= 0.10 && corr_deg <= 1.0 && rmse_ok);
        // [InsProbe] 写图门逐条件观测 (每 100 帧): 哪个条件在挡, 一眼可见
        static int ins_n = 0;
        const bool ins_show = (++ins_n % 100 == 0);
        const bool ins_anchor_ok = ins_show ? anchorOk(res, accepted) : false;
        if (ins_show) {
            std::cout << "[InsProbe] fit=" << res.fitness
                      << " anchored=" << base_anchored
                      << " shadow=" << m_maintainer_cfg.shadow
                      << " coast=" << m_coast_count
                      << " corr=" << corr_m << "m/" << corr_deg << "deg"
                      << " rmse=" << res.rmse
                      << " stable=" << pose_stable
                      << " tracking=" << tracking
                      << " acc=" << accepted
                      << " div=" << divergence
                      << " anchorOk=" << ins_anchor_ok << std::endl;
        }
        if (base_anchored && !m_maintainer_cfg.shadow &&
            m_coast_count == 0 && pose_stable &&
            allowMapInsert(tracking, accepted, divergence, m_maintainer_cfg) &&
            anchorOk(res, accepted)) {
            m_maintainer->onLocalized(res.pose, scan_map, m_last_match, true);
        }
    }

    // 过桥计数: 连续未采纳帧数
    m_coast_count = accepted ? 0 : (m_coast_count + 1);

    // 跟踪期歧义: 连续未采纳 N 帧后, 用重定位多候选重播假设 (aliasing 纠错)
    m_low_fit_frames = accepted ? 0 : (m_low_fit_frames + 1);
    if (m_hyp_reseed_frames > 0 && !m_hyp.ambiguous() &&
        m_low_fit_frames >= m_hyp_reseed_frames &&
        ensureRelocalizerMap()) {
        std::vector<GlobalCandidate> cands;
        const int nk = m_relocalizer->localizeTopK(scan, std::max(1, m_global_cfg.top_k), cands);
        if (nk > 1) {
            std::vector<PoseStamped> hp;
            std::vector<double> hs;
            for (const auto& c : cands) {
                // 可达性门控: 只有当前位姿附近 reseed_radius 米内的候选才有资格进
                // 假设池。2026-10-17 教训: 无距离约束的全局候选 = 0.3s 内"瞬移"31m。
                if (m_reseed_radius > 0 &&
                    (c.pose.trans - m_pose.trans).norm() > m_reseed_radius)
                    continue;
                hp.push_back(c.pose); hs.push_back(c.score);
            }
            if (hp.size() > 1) {
                m_hyp.seed(hp, hs);
                m_low_fit_frames = 0;
            }
        }
    }

    // lifelong: 周期性用叠加层(环境变化)刷新全局重定位器, 避免重定位失效
    if (m_maintainer && m_relocalizer && m_relocalizer_name != "none" &&
        m_maintainer_cfg.refresh_frames > 0) {
        if (++m_refresh_counter >= m_maintainer_cfg.refresh_frames) {
            m_refresh_counter = 0;
            CloudType::Ptr ov = m_maintainer->snapshotOverlay();
            if (ov && !ov->empty() && ov->size() != m_last_overlay_size) {
                m_last_overlay_size = ov->size();
                m_relocalizer->updateEnvironment(ov);
                // 存储底座增量插入新增 overlay 点 (避免整体重建)
                if (m_store) {
                    CloudType::Ptr d;
                    m_maintainer->popOverlayDelta(d);
                    if (d && !d->empty()) m_store->insert(d);
                }
            }
        }
    }

    MonitorInput mi;
    mi.match = res;
    mi.pose = m_pose;
    mi.time = f.end_time;
    // NEES: 归一化估计误差平方 (一致性检验)
    if (res.converged) {
        Eigen::Matrix<double, 6, 1> e;
        poseInnovation(res.pose, guess, e);
        Eigen::Matrix<double, 6, 6> C = res.cov;
        const double rf = m_accept_cfg.mahalanobis_rot_floor * m_accept_cfg.mahalanobis_rot_floor;
        const double tf = m_accept_cfg.mahalanobis_trans_floor * m_accept_cfg.mahalanobis_trans_floor;
        for (int k = 0; k < 3; ++k) C(k, k) += rf;
        for (int k = 3; k < 6; ++k) C(k, k) += tf;
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(C);
        const double d2 = e.dot(ldlt.solve(e));
        if (std::isfinite(d2)) mi.nees = d2;
    }
    (void)m_monitor->update(mi);   // 更新滑窗，只用于诊断/其它策略
    // 统一状态机：INIT/LOCALIZED 共用同一个连续帧计数器。
    //   INIT  -> LOCALIZED：连续 localized_patience 帧高置信健康
    //   LOCALIZED -> LOST：连续 lost_patience 帧 fitness < lost_fitness
    // credible_fitness 是高置信/恢复线；lost_fitness 才是丢失线。
    // 位于 lost_fitness 与 credible_fitness 之间只是 degraded/coast，不能判丢。
    // ---- 种子观察窗: 刚进入 LOCALIZED 后前 N 帧仍按 reloc_accept_score 复核 ----
    // 弱种子 (0.80~0.84 的局部自洽) 能过 first-frame 校验, 但撑不过几百帧;
    // 这里在锁定初期就要求同样的高置信, 失败直接退回 INIT 等下一次真种子.
    if (m_status == LOCALIZED && m_scrutiny_frames > 0) {
        const bool scrutiny_ok =
            res.converged && accepted &&
            res.fitness >= m_global_cfg.reloc_accept_score;
        if (!scrutiny_ok) {
            m_status = INIT;
            m_scrutiny_frames = 0;
            m_healthy_streak = 0;
            std::ostringstream os;
            os << "seed scrutiny failed: fitness=" << res.fitness
               << " < " << m_global_cfg.reloc_accept_score
               << " (converged=" << (res.converged ? 1 : 0)
               << " accepted=" << (accepted ? 1 : 0) << ") -> INIT";
            diag::log('W', "LocPipeline", os.str());
            FlightRecorder::instance().event("SEED_SCRUTINY_FAIL", os.str());
        } else {
            --m_scrutiny_frames;
        }
    }

    // ---- 匹配↔融合长期分歧: 定位错了但界面仍显示成功的兜底 ----
    // 现场典型: 位姿被拉到错误位置 (甚至 z 掉到地下几米), GICP 每帧都给出 ~10m 外
    // 的匹配, 但 ESKF 卡方门控一直拒收, 发布位姿靠 IMU 推算不动; 上层只看到
    // fitness 0.8+, 于是永远 LOCALIZED. 用"匹配位姿 vs 融合输出"的持续分歧判丢.
    double innov_trans = 0.0;
    double innov_yaw_deg = 0.0;
    if (res.converged) {
        V3D dt;
        double da = 0.0;
        poseDelta(res.pose, m_pose, dt, da);
        innov_trans = dt.norm();
        innov_yaw_deg = std::fabs(da) * 57.2957795;
    }
    const bool inconsistent =
        res.converged &&
        (innov_trans > m_consensus_trans_m ||
         innov_yaw_deg > m_consensus_yaw_deg);
    m_inconsistent_streak = inconsistent ? (m_inconsistent_streak + 1) : 0;
    const bool consensus_unhealthy =
        m_inconsistent_streak >= std::max(1, m_monitor_cfg.lost_patience);

    const bool hard_unhealthy =
        seed_failed || !res.converged ||
        res.fitness < m_monitor_cfg.lost_fitness;

    if (m_status == LOCALIZED) {
        // 跟踪阶段只要不低于 lost_fitness 就继续运行；不要求达到 credible_fitness。
        // 但"匹配与融合输出持续分歧"同样算不健康, 否则错位姿会一直报成功.
        // ★证据不足(靠站/贴墙, 扫描只剩一两百点): 本帧既不加也不减, 只累计中性帧数;
        //   超过 low_evidence_limit 仍按不健康处理(不能无限 coast)。
        const bool low_evidence = (res.visible > 0 && res.visible < m_min_evidence_points);
        if (low_evidence) ++m_low_evidence_frames;
        else m_low_evidence_frames = 0;
        const bool neutral = low_evidence && m_low_evidence_frames < m_low_evidence_limit;
        if (neutral) {
            // 不改变 m_unhealthy_streak
        } else if (hard_unhealthy || consensus_unhealthy) {
            ++m_unhealthy_streak;
        } else {
            m_unhealthy_streak = 0;
        }
        if (m_unhealthy_streak >= std::max(1, m_monitor_cfg.lost_patience)) {
            const bool was_lost = (m_status == LOST);
            m_status = LOST;
            m_scrutiny_frames = 0;
            {
                std::lock_guard<std::mutex> lk(m_global_mu);
                m_global_request = true;
                m_global_cooldown = 0;
            }
            // 只在真正进入 LOST 的那一帧记录/落盘; 之前这里每帧调用，
            // 一次丢定位会在 /tmp/slam_flight 生成上千个 100KB+ 文件并拖慢系统.
            if (!was_lost) {
                resetSessionMap();   // 判丢后清掉临时子图, 避免下次桥接直接锁死
                std::ostringstream os;
                os << "unhealthy frames=" << m_unhealthy_streak
                   << " >= " << m_monitor_cfg.lost_patience
                   << " (consensus=" << (consensus_unhealthy ? 1 : 0)
                   << " innov_t=" << innov_trans << "m innov_yaw="
                   << innov_yaw_deg << "deg fitness=" << res.fitness << ") -> LOST";
                diag::log('W', "LocPipeline", os.str());
                FlightRecorder::instance().event("LOST", os.str());
                FlightRecorder::instance().onStatusChange(true);
            }
        }
    } else {
        // INIT / 重定位后：必须连续高置信帧才恢复 LOCALIZED。
        // 恢复/重定位接受线保持独立参数 global.reloc_accept_score，不要用 credible_fitness 下盖。
        const bool init_healthy =
            !seed_failed && res.converged && accepted &&
            res.fitness >= m_global_cfg.reloc_accept_score;
        if (init_healthy) {
            ++m_healthy_streak;
            m_unhealthy_streak = 0;
            if (m_status != LOCALIZED &&
                m_healthy_streak >= std::max(1, m_monitor_cfg.localized_patience)) {
                m_status = LOCALIZED;
                m_ever_localized = true;   // 只有真正定位成功过, 才允许开会话地图
                m_scrutiny_frames = std::max(0, m_monitor_cfg.scrutiny_frames);
                {
                    std::lock_guard<std::mutex> lk(m_global_mu);
                    m_global_request = false;
                }
                diag::log('I', "LocPipeline", "localized after " +
                          std::to_string(m_healthy_streak) + " healthy frames");
                FlightRecorder::instance().event("LOCALIZED",
                                                 "healthy_frames=" + std::to_string(m_healthy_streak));
            }
        } else {
            m_healthy_streak = 0;
            // 关键: 本帧不硬坏 (fitness >= lost_fitness) 时必须归零, 否则在 INIT 里
            // 间歇性坏帧会一直累计, 几十秒后"凭空"判 LOST (实机 13:48 就是这样丢的).
            if (hard_unhealthy) {
                ++m_unhealthy_streak;
            } else {
                m_unhealthy_streak = 0;
            }
            if (m_unhealthy_streak >= std::max(1, m_monitor_cfg.lost_patience)) {
                const bool was_lost = (m_status == LOST);
                m_status = LOST;
                m_scrutiny_frames = 0;
                {
                    std::lock_guard<std::mutex> lk(m_global_mu);
                    m_global_request = true;
                    m_global_cooldown = 0;
                }
                if (!was_lost) {
                    std::ostringstream os;
                    os << "unhealthy frames=" << m_unhealthy_streak
                       << " >= " << m_monitor_cfg.lost_patience << " -> LOST";
                    diag::log('W', "LocPipeline", os.str());
                    FlightRecorder::instance().event("LOST", os.str());
                    FlightRecorder::instance().onStatusChange(true);
                }
            }
        }
    }

    // 状态机诊断 (1Hz): 定位状态卡在 INIT 时, 看这几个量就能定位是 seed 被拒、
    // 匹配未接受、还是 fitness 不够.
    {
        static double s_last_log = -1.0;
        if (f.end_time - s_last_log >= 1.0) {
            s_last_log = f.end_time;
            std::cout << "[StatusProbe] st=" << static_cast<int>(m_status.load())
                      << " seed_failed=" << (seed_failed ? 1 : 0)
                      << " accepted=" << (accepted ? 1 : 0)
                      << " converged=" << (res.converged ? 1 : 0)
                      << " fitness=" << res.fitness
                      << " innov_t=" << innov_trans
                      << " innov_yaw=" << innov_yaw_deg
                      << " inconsistent=" << m_inconsistent_streak
                      << " scrutiny=" << m_scrutiny_frames
                      << " healthy=" << m_healthy_streak
                      << " unhealthy=" << m_unhealthy_streak
                      << " coast=" << m_coast_count
                      << " have_last=" << (m_have_last_accepted ? 1 : 0)
                      << " rejection=" << reject_reason << std::endl;
            // A 阶段(影子): 体素可信度统计, 只观测不决策. 每天按 w>=persistency_anchor 导出点云,
            // 看是否只剩立柱/墙体, 并对比正常段 / 假锁段 / 堆货区 的 persistency_cov 分布.
            // ---- 先验图(底图+overlay)匹配度 (锚定判据) ----
            // 会话子图参与匹配后, 总 fitness 会自我确认(本会话自己写的点配自己)。
            // "锚是否恢复"必须看对**先验图**的覆盖率 —— 但先验图 = 底图 + overlay:
            // overlay 是跨会话确认过的结构, 属于地图; 只有会话子图是本会话自证的部分。
            //   cov_prior 高 -> 先验图锚得住 -> 关闭子图
            //   cov_prior 低 -> 先验图锚不住 -> 子图自更新
            // 2026-11-25: 不再挂 session_map.enabled —— overlay 的 T4 写图门槛
            // (base_anchored) 也用 cov_base, 关子图开 overlay 时它必须照常计算,
            // 否则永远是 -1, 写图门槛退化成 anchorOk(自引用风险)。
            if (m_ever_localized && scan && !scan->empty() && m_local_map) {
                // 成本: 全点降采样 + 全图 KD 查询 ~几 ms。诊断用途, 每 5 帧算一次即可
                // (锚定切换用的是 m_on_submap 的弱锚 streak, 不依赖这个值的实时性)。
                static int cov_n = 0;
                const bool do_cov = (++cov_n % 5 == 0);
                if (do_cov && (!g_base_only_valid || ++g_base_only_age >= 30)) {
                    m_base_only_age = 0;
                    CloudType::Ptr base_crop = m_local_map->getLocalMap(out_pose);
                    if (base_crop && base_crop->size() >= 100) {
                        g_base_only_kdtree.setInputCloud(base_crop);
                        g_base_only_valid = true;
                    } else {
                        g_base_only_valid = false;
                    }
                }
                if (do_cov && g_base_only_valid) {
                    CloudType::Ptr ds = voxelDownsample(scan, 0.4f);
                    if (ds && ds->size() >= 50) {
                        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                        T.block<3, 3>(0, 0) = out_pose.rot;
                        T.block<3, 1>(0, 3) = out_pose.trans;
                        const CoverageStats cs =
                            evaluateCoverage(g_base_only_kdtree, *ds, T, m_matcher_cfg.inlier_dist);
                        m_cov_base = (cs.total > 0) ? cs.coverage : -1.0;
                    }
                }
                if (m_cov_base >= m_monitor_cfg.credible_fitness) ++m_cov_base_ok_streak;
                else m_cov_base_ok_streak = 0;

                if (m_cov_base >= 0.0) {
                    // cov_base 跨阈值 = 底图锚定质量发生阶跃: 使目标缓存与
                    // 子图窗口锚失效一次 (边沿检测, 不再单独维护第二个状态变量)。
                    static bool prev_on_base = true;
                    const bool on_base = (m_cov_base >= m_monitor_cfg.credible_fitness);
                    if (on_base != prev_on_base) {
                        prev_on_base = on_base;
                        m_have_submap_anchor = false;   // 子图窗口锚一起失效
                        m_has_cached_target = false;
                    }
                }
            } else {
                m_cov_base = -1.0;
                m_cov_base_ok_streak = 0;
            }
            // ★机器人侧新增: 子图↔先验一致性观测 (只测不改)。必须在 m_session_map_mu
            // 作用域之外调用(函数内部自己加锁, 非递归 mutex 二次加锁 = 死锁; 实机踩过)。
            if (m_session_map_enabled && (++m_consistency_tick % 30 == 0)) {
                checkSubmapPriorConsistency();
            }
            }

            // ---- 锚质量 / 会话子图 / 诊断输出 ----
            // ★与 overlay maintainer 解耦 (2026-11-25): 会话子图是独立机制, 由
            // session_map.enabled 单独控制, 不依赖是否配置了 map_maintainer ——
            // 现场曾因 map_maintainer:none 时 maintainer 为空, 整段子图逻辑不执行,
            // 表现为"开了子图却没有 [session_map] open / 话题无数据"。
            int anchor_age = 0;
            size_t persistency_pts = 0;
            // persistency_cov 是 overlay 的诊断量: 没开 overlay 时输出 n/a
            const double ss = m_maintainer
                ? m_maintainer->persistencyCoverage(m_pose, &anchor_age, &persistency_pts)
                : -1.0;

                // ---- B 阶段(影子): 突变场景"会话地图"(session map) ----
                // 锚质量 = 行业标准三件套: 本帧被接受 + inliers + JtJ 最小特征值比
                // (Zhang, ICRA 2016). 弱锚持续 N 帧才开, 恢复 M 帧才关。
                // persistency_cov 只作诊断, 不再参与判定。
                const bool anchor_ok = anchorOk(res, accepted);
                if (anchor_ok) {
                    ++m_anchor_ok_streak;
                    m_anchor_weak_streak = 0;
                } else {
                    ++m_anchor_weak_streak;
                    m_anchor_ok_streak = 0;
                    // overlay 顶不住 -> 切换目标到子图。
                    // **只切换, 不重建**: 子图从定位成功起就一直在建, 内容可信;
                    // 此刻 reset 会让子图变薄, 临时建图必然出错。
                    // ★必须确认子图机制已启用: session_map.enabled=false 时没有子图,
                    //   切桥接 = 目标变成空云, 匹配直接失效 (overlay-only 模式的灾难)。
                    if (m_session_map_enabled && !m_on_submap && m_ever_localized &&
                        m_anchor_weak_streak >= m_anchor_patience &&
                        m_session_map.size() > 3000 &&   // 空子图不接管 (目标会被空云顶替)
                        m_status.load() != LOST) {
                        m_on_submap = true;
                        m_probe_countdown = m_probe_period;
                        m_has_cached_target = false;   // 下一帧目标切到子图
                        std::cout << "[anchor] switch to submap (bridging, pts="
                                  << m_session_map.size() << ")" << std::endl;
                    }
                }

                int  session_map_active = 0;
                size_t session_map_size = 0;
                int  session_map_age = 0;
                if (m_session_map_enabled) {
                    std::lock_guard<std::mutex> lk(m_session_map_mu);
                    const double now_s = (res.pose.time > 0.0) ? res.pose.time : f.end_time;
                    // 会话地图: 真正定位过才开; 开了之后**常驻自更新**, 不关闭。
                    // "锚定在哪"由 fitness_base 决定(见上), 只影响 buildTarget 的目标组成:
                    //   底图匹配度高 -> 位姿锚在底图上;  底图匹配度低 -> 子图桥接并自更新。
                    if (!m_ever_localized) {
                        if (m_session_map.active()) m_session_map.deactivate();
                    } else {
                        // 只要定位成功且覆盖率高, 就立刻开子图并**一直在建**:
                        // 底图某天完全变化锚不住时, 已建好的子图立即顶上 (边建图边定位),
                        // 而不是临时去建 (临时建的子图自身就是错的, 顶不住)。
                        // ★开图/写图门槛用 lost_fitness(=accept.min_coverage, 0.6):
                        //   overlay 关掉后目标只剩底图, fitness 天然 0.78~0.85
                        //   (0.93 那种是带子图自证的分数), 拿 credible(0.85) 当门槛
                        //   子图永远开不了(2026-11-25 实机: 无 [session_map] open)。
                        //   补丁不进匹配目标(只有桥接才进), 写差了可被 zip-up/一致性观测兜住。
                        // 桥接态放宽到 m_sm_write_fitness(0.35): 变化区里对先验的
                        // fitness 天然只有 0.4~0.5, 若沿用 overlay 门槛, 就会死锁
                        // —— 需要子图才能提分, 又因为分不够不让写子图(2026-11-24 实机:
                        // submap_size 一直是 0, 桥接态等于没图可配, 30 帧后必丢)。
                        const double write_bar = !m_on_submap
                                                     ? m_monitor_cfg.lost_fitness
                                                     : m_sm_write_fitness;
                        if (!m_session_map.active() && m_status.load() == LOCALIZED &&
                            accepted && res.fitness >= write_bar) {
                            m_session_map.reset(m_pose, now_s);
                            std::cout << "[session_map] open ("
                                      << (!m_on_submap ? "base anchored" : "bridging")
                                      << ", building ahead) fitness=" << res.fitness << std::endl;
                        }
                        // 自更新: 匹配被接受且置信度达标就写 (锚定态也写, 供"底图低"时桥接)
                        // 与 overlay 同理: 用匹配位姿 res.pose 落点, 保证会话地图与先验
                        // 地图几何一致 (融合位姿在转弯时有滤波滞后, 会把整片点云转一个角).
                        // 子图只要求"位姿没跳"即可, 不套 overlay 的 rmse 门控:
                        // 变化区里残差本来就大, 套上就永远写不进去。
                        const bool pose_stable_sm =
                            (corr_m <= m_sm_write_corr_m && corr_deg <= m_sm_write_corr_deg);
                        // 2026-11-26: 回退异常学习模式 —— 桥接时无条件注册会让
                        // 错位姿的扫描污染子图, 联合目标随即自证错位姿 (高 fitness
                        // 锁死在错误位置)。恢复 accepted+fitness 门; 重新引入需
                        // 改用 LIO 位姿注册 + 可靠性判据 (ROLL 原设计)。
                        if (accepted && res.converged && pose_stable_sm &&
                            res.fitness >= write_bar && scan_map && !scan_map->empty()) {
                            m_session_map.update(*scan_map, res.pose);
                        }
                        // ★走出子图覆盖的旧关键帧: 段级一次性「融合+雕刻」进 w 账本。
                        // 子图覆盖内纯几何跟踪 (零账本交互); 只有整段退出时才与
                        // w 账本结算一次 —— 用户口径: 走出子图 = 融合 + 雕刻一次。
                        if (++m_seg_merge_tick % 25 == 0) {
                            auto leaving = m_session_map.popLeavingKfs(out_pose.trans);
                            for (auto& kf : leaving)
                                if (m_maintainer) m_maintainer->mergeChunk(kf.pose, kf.world);
                        }
                    }
                    session_map_active = m_session_map.active() ? 1 : 0;
                    session_map_size = m_session_map.size();
                    session_map_age = static_cast<int>(m_session_map.ageS(now_s));
                }

                // ---- 工作图定期落盘 (后台线程): working_map_save_min 周期, 没变化不写盘 ----
                // 只有一张全量工作图: 底图(0.3) + overlay → map.overlay.pcd,
                // 每点 intensity = w (w==0 的点不写)。没有增量日志。
                // map.pcd 只做原图备份, 永不修改。
                // 变化判据 = maintainer revision() (几何/权重有增删才 +1)。
                if (m_maintainer && m_ever_localized && m_map &&
                    !m_map->empty() && m_initialized && accepted) {
                    static std::atomic<bool> wm_saving(false);
                    static std::chrono::steady_clock::time_point wm_last =
                        std::chrono::steady_clock::now();
                    static uint64_t wm_last_rev = 0;
                    const uint64_t rev = m_maintainer ? m_maintainer->revision() : 0;
                    const auto now = std::chrono::steady_clock::now();
                    const bool due = (m_working_map_save_min > 0.0) &&
                        (now - wm_last) >= std::chrono::duration_cast<std::chrono::seconds>(
                                              std::chrono::duration<double, std::ratio<60>>(
                                                  m_working_map_save_min));
                    const bool changed = (rev != wm_last_rev);
                    if (!wm_saving.load() && due && changed) {
                        wm_last = now;            // 先占位, 防止重复触发
                        wm_last_rev = rev;
                        wm_saving.store(true);
                        // 存图时刻回收已删除体素的点 (#6): 1h 一次, 全量扫描成本可忽略,
                        // 之后 snapshotOverlay / 工作图落盘都是干净的。
                        m_maintainer->reclaim();
                        // 全量工作图只有一张: 底图先降采样到 voxel(0.3) —— 与 overlay 同网格、
                        // 与匹配口径一致; 再写 intensity=w (w==0 的点不写 = 跨会话删除生效)。
                        const double vox = m_maintainer_cfg.voxel > 0 ? m_maintainer_cfg.voxel : 0.3;
                        CloudType::Ptr base_snap =
                            voxelDownsample(m_map, static_cast<float>(vox));
                        if (m_maintainer) m_maintainer->annotateBaseWeights(*base_snap);
                        CloudType::Ptr ov = m_maintainer ? m_maintainer->snapshotOverlay()
                                                         : nullptr;
                        const std::string working_map = workingMapPath(m_map_path);
                        std::thread([base_snap, ov, working_map, vox, wm_saving]() mutable {
                            // ★detached 线程里绝不能让异常逃逸: 一旦抛出 = std::terminate
                            // = 整个进程 abort (2026-11-25 实机: sudo 恢复的工作图属主 root,
                            // open 失败 -> pcl::IOException -> 节点自杀, curr_scan_point/
                            // slam/info 全部消失)。任何失败只记日志, 本帧放弃落盘。
                            try {
                                if (ov && !ov->empty()) {
                                    suppressBaseByOverlay(base_snap, ov, vox);
                                    *base_snap += *ov;
                                }
                                pcl::io::savePCDFileBinary(working_map, *base_snap);
                                std::cout << "[working_map] saved: " << working_map << " ("
                                          << base_snap->size() << " pts)" << std::endl;
                            } catch (const std::exception& e) {
                                std::cerr << "[working_map] save FAILED: " << e.what()
                                          << " (检查文件属主/磁盘空间)" << std::endl;
                            } catch (...) {
                                std::cerr << "[working_map] save FAILED (unknown)" << std::endl;
                            }
                            wm_saving.store(false);
                        }).detach();
                    }
                std::ostringstream qs;
                qs.setf(std::ios::fixed);
                qs.precision(3);
                qs << "[quality] persistency_cov=";
                if (ss < 0.0) qs << "n/a"; else qs << ss;
                qs << " anchor_age=" << anchor_age
                   << " persistency_pts=" << persistency_pts
                   << " cov_base=" << (m_cov_base >= 0.0 ? m_cov_base : -1.0)
                   << " novel_votes=" << (m_maintainer ? m_maintainer->lastNovelVotes() : 0)
                   << " novel_commits=" << (m_maintainer ? m_maintainer->lastNovelCommitted() : 0)
                   << " state="
                   << (m_status.load() == LOST ? "LOST"
                       : (m_status.load() == LOCALIZED ? "LOCALIZED" : "INIT"))
                  
                   << " persistency_anchor=" << m_maintainer_cfg.persistency_anchor
                   << " fitness=" << res.fitness
                   << " anchor=" << (anchor_ok ? "OK" : "WEAK")
                   << " inl=" << res.inliers
                   << " vis=" << res.visible
                    << " fitness_all=" << (res.fitness_all >= 0.0 ? res.fitness_all : -1.0)
                   << " eig=" << res.smallest_eig_ratio
                   << " session_map=" << session_map_active
                   << " session_map_size=" << session_map_size
                   << " session_map_age=" << session_map_age;
                std::cout << qs.str() << std::endl;
        }
    }
    // 去畸变起点: 帧末才更新 (写图路径的 prepareScan 还在上面用本帧的起点)。
    m_deskew_prev_pose = out_pose;
    m_deskew_prev_valid = true;
}

ILocalization::Status LocalizationPipeline::getStatus() {
    return m_status.load();
}

bool LocalizationPipeline::priorUncertainty(double& xy_sigma, double& yaw_sigma) const {
    std::lock_guard<std::mutex> lk(m_prior_mu);
    if (!m_prior) return false;
    const double ts = m_prior->priorTransSigma();
    const double ys = m_prior->priorYawSigma();
    if (!(ts > 0.0) || !(ys > 0.0)) return false;   // 实现未提供
    xy_sigma = ts;
    yaw_sigma = ys;
    return true;
}

bool LocalizationPipeline::getPose(PoseStamped& out) {
    std::lock_guard<std::mutex> lk(m_snapshot_mu);
    // 从未初始化/从未匹配则返回 false. 否则 ROS 层会把 setMapData 后残留的
    // 上一张图位姿(或默认 identity)当成"当前位姿"发出去 —— 切图瞬间表现为
    // 初始位姿相对地图零点偏 90°(IMU body 默认 identity 再被 -90° 输出).
    // 定位成功过之后 LOST 仍返回最后有效位姿，供界面保持位置.
    if (!m_snapshot_valid) return false;
    out = m_snapshot_pose;
    return true;
}

bool LocalizationPipeline::getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) {    std::lock_guard<std::mutex> lk(m_snapshot_mu);
    // LOST/未初始化必须返回 false：ROS 层的 LostMonitor 用它判断“当前是否可信”。
    // getPose() 在定位成功后 LOST 仍可返回最后有效位姿，供界面保持最后位置显示。
    if (!m_snapshot_valid || m_status.load() == LOST) return false;
    out = m_snapshot_pose;
    cov = m_snapshot_cov;
    return true;
}

bool LocalizationPipeline::predictAt(double t, PoseStamped& out) {
    if (!m_initialized || !m_prior) return false;
    // eskf: 位姿由融合层输出 (IMU/odom 推进 + 匹配观测限幅修正);
    // direct: 沿用运动先验预测 (与改造前一致)。
    if (m_fusion && m_fusion->name() != "direct") {
        std::lock_guard<std::mutex> fk(m_fusion_mu);
        out = m_fusion->predict(t);
        return true;
    }
    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
    out = m_prior->predict(t);   // 运动先验(IMU)从最后校正位姿传播
    return true;
}

bool LocalizationPipeline::anchorOk(const MatchResult& res, bool accepted) const {
    AnchorQuality q;
    q.converged = res.converged;
    q.accepted = accepted;
    q.inliers = res.inliers;
    q.visible = res.visible;
    q.correspondences = res.correspondences;
    q.has_hessian = res.has_hessian;
    q.smallest_eig_ratio = res.smallest_eig_ratio;
    q.static_support = res.w_support;   // P4: 加权锚定 (已确认结构支撑比例)
    AnchorCriteria c;
    c.min_inliers = m_anchor_min_inliers;
    c.min_ratio = m_anchor_min_ratio;
    c.min_eig_ratio = m_anchor_min_eig_ratio;
    c.min_static_support = m_anchor_w_support_min;
    return anchorQualityOk(q, c);
}

FusionDiag LocalizationPipeline::fusionDiag() const {
    std::lock_guard<std::mutex> fk(m_fusion_mu);
    return m_fusion ? m_fusion->diag() : FusionDiag{};
}

std::string LocalizationPipeline::poseFusionDiagnostics() const {
    const FusionDiag d = fusionDiag();
    std::ostringstream os;
    os << "{\"mode\":\"" << m_fusion_name << "\""
       << ",\"accepted\":" << d.accepted
       << ",\"relative_accepted\":" << d.relative_accepted
       << ",\"relative_reanchored\":" << d.relative_reanchored
       << ",\"relative_rejected\":" << d.relative_rejected
       << ",\"last_rel_residual\":" << d.last_rel_residual
       << ",\"last_rel_chi2\":" << d.last_rel_chi2
       << ",\"odom_weight\":" << d.odom_weight
       << ",\"rejected_chi2\":" << d.rejected_chi2
       << ",\"rejected_in_place\":" << d.rejected_in_place
       << ",\"last_chi2\":" << d.last_chi2
       << ",\"gain_trans\":" << d.last_gain_trans
       << ",\"gain_rot\":" << d.last_gain_rot
       << ",\"applied_trans\":" << d.applied_trans
       << ",\"applied_rot_deg\":" << d.applied_rot_deg << "}";
    return os.str();
}

void LocalizationPipeline::onAbsolute(const PoseStamped& pose,
                                      const Eigen::Matrix<double, 6, 6>& cov) {
    std::lock_guard<std::mutex> lk(m_state_mu);
    if (!m_abs_cfg.enabled) return;
    (void)cov;   // 门控互补融合暂只用位姿; 协方差供后续因子图
    m_abs_pose = pose;
    m_abs_time = pose.time;
    m_has_abs = true;
}

} // namespace rosiwit_slam
