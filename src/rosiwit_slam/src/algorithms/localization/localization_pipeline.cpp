// src/algorithms/localization/localization_pipeline.cpp
#include "algorithms/localization/localization_pipeline.h"
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_utils.h"
#include "algorithms/map_store/map_store_factory.h"
#include <cmath>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

#include "slam_core/diagnostics.h"
#include <sstream>

namespace rosiwit_slam {
using diag::FlightRecorder;

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

    YAML::Node pipe = loc["pipeline"];
    m_relocalizer_name = getv<std::string>(pipe, "relocalizer", m_relocalizer_name);
    m_prior_name       = getv<std::string>(pipe, "motion_prior", m_prior_name);
    m_local_map_name   = getv<std::string>(pipe, "local_map", m_local_map_name);
    m_matcher_name     = getv<std::string>(pipe, "matcher", m_matcher_name);
    m_monitor_name     = getv<std::string>(pipe, "monitor", m_monitor_name);
    m_maintainer_name  = getv<std::string>(pipe, "map_maintainer", m_maintainer_name);

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
    auto gc = loc["global"];
    if (gc) {
        m_global_cfg.min_score          = getv<double>(gc, "min_score", m_global_cfg.min_score);
        m_global_cfg.top_k              = getv<int>(gc, "top_k", m_global_cfg.top_k);
        m_global_cfg.reloc_accept_score =
            getv<double>(gc, "reloc_accept_score", m_global_cfg.reloc_accept_score);
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
        m_global_cfg.sc_min_radius      = getv<double>(gc, "sc_min_radius", m_global_cfg.sc_min_radius);
        m_global_cfg.sc_dist_thresh     = getv<double>(gc, "sc_dist_thresh", m_global_cfg.sc_dist_thresh);
        m_global_cfg.sc_candidate_ratio = getv<double>(gc, "sc_candidate_ratio", m_global_cfg.sc_candidate_ratio);
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
        m_global_cfg.fpfh_max_iter       = getv<int>(gc, "fpfh_max_iter", m_global_cfg.fpfh_max_iter);
        m_global_cfg.fpfh_inlier_fraction = getv<double>(gc, "fpfh_inlier_fraction", m_global_cfg.fpfh_inlier_fraction);
        m_global_cfg.fpfh_similarity     = getv<double>(gc, "fpfh_similarity", m_global_cfg.fpfh_similarity);
    }
    auto moc = loc["monitor"];
    if (moc) {
        m_monitor_cfg.enabled            = getv<bool>(moc, "enabled", m_monitor_cfg.enabled);
        m_monitor_cfg.credible_fitness   = getv<double>(moc, "credible_fitness", m_monitor_cfg.credible_fitness);
        m_monitor_cfg.mediocre_patience  = getv<int>(moc, "mediocre_patience", m_monitor_cfg.mediocre_patience);
        m_monitor_cfg.lost_fitness       = getv<double>(moc, "lost_fitness", m_monitor_cfg.lost_fitness);
        m_monitor_cfg.lost_patience      = getv<int>(moc, "lost_patience", m_monitor_cfg.lost_patience);
        m_monitor_cfg.lost_patience_low  = getv<int>(moc, "lost_patience_low", m_monitor_cfg.lost_patience_low);
        m_monitor_cfg.coast_patience     = getv<int>(moc, "coast_patience", m_monitor_cfg.coast_patience);
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
        m_maintainer_cfg.autosave_frames  = getv<int>(mtc, "autosave_frames", m_maintainer_cfg.autosave_frames);
        m_maintainer_cfg.refresh_frames   = getv<int>(mtc, "refresh_frames", m_maintainer_cfg.refresh_frames);
        m_maintainer_cfg.decay_frames     = getv<int>(mtc, "decay_frames", m_maintainer_cfg.decay_frames);
        m_maintainer_cfg.override_base    = getv<bool>(mtc, "override_base", m_maintainer_cfg.override_base);
        m_maintainer_cfg.delta_min        = getv<double>(mtc, "delta_min", m_maintainer_cfg.delta_min);
        m_maintainer_cfg.carve_stride     = getv<int>(mtc, "carve_stride", m_maintainer_cfg.carve_stride);
        m_maintainer_cfg.pose_graph       = getv<bool>(mtc, "pose_graph", m_maintainer_cfg.pose_graph);
        m_maintainer_cfg.pg_min_keyframes = getv<int>(mtc, "pg_min_keyframes", m_maintainer_cfg.pg_min_keyframes);
    }
    auto plc = loc["planar"];
    if (plc) {
        m_planar_cfg.mode          = getv<std::string>(plc, "mode", m_planar_cfg.mode);
        m_planar_cfg.z             = getv<double>(plc, "z", m_planar_cfg.z);
        m_planar_cfg.sensor_height = getv<double>(plc, "sensor_height", m_planar_cfg.sensor_height);
        m_planar_cfg.cell          = getv<double>(plc, "cell", m_planar_cfg.cell);
        m_planar_cfg.ground_band   = getv<double>(plc, "ground_band", m_planar_cfg.ground_band);
        m_planar_cfg.max_tilt_deg  = getv<double>(plc, "max_tilt_deg", m_planar_cfg.max_tilt_deg);
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

    m_matcher->configure(m_matcher_cfg);
    m_prior->configure(m_motion_cfg);
    m_local_map->configure(m_local_map_cfg);
    m_monitor->configure(m_monitor_cfg);
    m_relocalizer->configure(m_global_cfg);

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
    // 非破坏性: 叠加层单独落盘, 原图不改
    if (m_maintainer && !m_map_path.empty()) m_maintainer->save(m_map_path + ".overlay.pcd");
}

std::string LocalizationPipeline::describe() const {
    return "relocalizer=" + m_relocalizer_name +
           " motion_prior=" + m_prior_name +
           " local_map=" + m_local_map_name +
           " matcher=" + m_matcher_name +
           " monitor=" + m_monitor_name +
           " map_maintainer=" + m_maintainer_name +
           " accept_mode=" + std::to_string(m_accept_cfg.mode);
}

void LocalizationPipeline::setMap(const std::string& map_name) {
    setMapImpl(map_name);
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
    m_initialized = false;
    m_has_cached_target = false;
    m_map_path = map_name;
    m_map.reset(new CloudType());
    if (pcl::io::loadPCDFile(map_name, *m_map) == -1) {
        std::cerr << "[LocPipeline] failed to load PCD map: " << map_name << std::endl;
        m_map.reset();
        return;
    }
    std::cout << "[LocPipeline] loaded PCD map: " << map_name << " (" << m_map->size()
              << " pts)" << std::endl;
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
    if (m_maintainer) m_maintainer->setBaseMap(map_name, m_map);
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
    }
    if (!m_reloc_thread.joinable())
        m_reloc_thread = std::thread([this] { relocBuildWorker(); });
    m_reloc_cv.notify_all();
    mark("relocalizer_bg");
    std::cout << "[LocPipeline] setMap profile(ms, 累计):" << timing
              << " total=" << std::to_string(stage_ms(t_total0)) << std::endl;
}


void LocalizationPipeline::setMapData(const MapData& data) {
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    m_cancel_set_map.store(false);
    m_status = INIT;
    m_initialized = false;
    m_has_cached_target = false;
    m_map_path = data.meta.map_id;
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
    if (m_maintainer) m_maintainer->setBaseMap(m_map_path, m_map);
    if (m_local_map && !lkfs.empty()) {
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
    }
    if (!m_reloc_thread.joinable())
        m_reloc_thread = std::thread([this] { relocBuildWorker(); });
    m_reloc_cv.notify_all();
}

void LocalizationPipeline::relocBuildWorker() {
    std::unique_lock<std::mutex> lk(m_reloc_mu);
    while (!m_reloc_stop) {
        m_reloc_cv.wait(lk, [this] { return m_reloc_stop || m_reloc_request; });
        if (m_reloc_stop) break;
        const CloudType::Ptr map = m_reloc_map_snap;
        const std::string path = m_reloc_path_snap;
        m_reloc_request = false;
        lk.unlock();
        m_reloc_building.store(true);
        const auto t0 = std::chrono::steady_clock::now();
        bool ok = false;
        if (m_relocalizer && m_relocalizer_name != "none" && map && !map->empty())
            ok = m_relocalizer->setMap(path, map);
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
        lk.lock();
        m_reloc_building.store(false);
        // 构建期间又切图(request 已再次置位)则快照已更新, 循环继续重建
        m_reloc_ready = ok && m_reloc_map_snap == map;
        {
            std::ostringstream os;
            os << "global reloc map built in background, " << ms << "ms ok=" << ok;
            diag::log('I', "LocPipeline", os.str());
            FlightRecorder::instance().event("BG_BUILD", os.str());
        }
    }
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
    m_reloc_cv.notify_all();
    return false;
}

void LocalizationPipeline::setInitPose(const PoseStamped& pose) {
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    m_pose = pose;
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        if (m_prior) m_prior->reset(pose);
    }
    if (m_monitor) m_monitor->reset();
    m_mediocre_streak = 0;
    m_corr_prev_valid = false;
    m_consistent_cnt = 0;
    m_initialized = true;
    m_status = INIT;
    m_smoother.reset(m_pose);
    m_hyp.reset(m_pose);
}

void LocalizationPipeline::onImu(const IMUSample& s) {
    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
    if (m_prior) m_prior->onImu(s);
}

void LocalizationPipeline::requestRelocalization() {
    // 上层 /global_localization 触发: 强制回到未定位状态, 下一次 onLidar 走冷启动分支
    // (relocalizer=bbs 时即全局检索; =none 时退化为 tryBoundedRecovery 的有界局部重锁)
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    m_status = LOST;
    m_initialized = false;
    m_reloc_cooldown = 0;
    std::cout << "[LocPipeline] relocalization requested by service" << std::endl;
}

void LocalizationPipeline::onOdom(const PoseStamped& odom) {
    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
    if (m_prior) m_prior->onOdom(odom);
}

CloudType::Ptr LocalizationPipeline::buildTarget(const PoseStamped& pose) {
    CloudType::Ptr target = m_local_map->getLocalMap(pose);
    // [LocProbe] 诊断: 局部地图到底取到多少点 (NO Effective Points 排查)
    {
        static int probe_n = 0;
        if (++probe_n % 50 == 0) {   // 5s 一条: 100Hz 输出下 1Hz 仍然太吵
            std::cout << "[LocProbe] local_map=" << (target ? target->size() : 0)
                      << " pose=(" << pose.trans.x() << ", " << pose.trans.y() << ")"
                      << " local_map_name=" << m_local_map_name << std::endl;
        }
    }
    if (m_maintainer) {
        CloudType::Ptr ov = m_maintainer->getOverlay(pose);
        if (ov && !ov->empty()) {
            if (m_maintainer_cfg.override_base)
                suppressBaseByOverlay(target, ov, m_maintainer_cfg.voxel);
            *target += *ov;
        }
    }
    return target;
}

bool LocalizationPipeline::tryBoundedRecovery(const CloudType::Ptr& scan) {
    // 出口重锁: 在推算位姿附近做有界局部多起点配准(优于全局检索; 叠加层帮助在新货环境重锁)
    if (!m_local_map || !m_matcher) return false;
    CloudType::Ptr target = m_local_map->getLocalMap(m_pose);
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
                MatchResult r;
                if (m_matcher->match(scan, g, r) && r.converged && r.fitness > best_fit) {
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
    m_pose = best;
    m_pose.time = best.time;
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        m_prior->reset(m_pose);
    }
    m_monitor->reset();
    m_mediocre_streak = 0;
    m_has_cached_target = false;
    m_coast_count = 0;
    diag::log('I', "LocPipeline", "bounded recovery, fitness=" + std::to_string(best_fit));
    FlightRecorder::instance().event("RECOVERY", "bounded recovery fitness=" +
                                      std::to_string(best_fit));
    return true;
}

bool LocalizationPipeline::acceptMatch(const MatchResult& res, const CloudType::Ptr& scan,
                                       const PoseStamped& guess) {
    if (!res.converged) return false;
    // Mahalanobis 门控: 修正量相对匹配协方差异常大 -> 判离群, 不采信
    // (协方差加下限: 匹配协方差是估计自身不确定度, 极小时会误拒正常修正)
    if (m_accept_cfg.mahalanobis_gate) {
        Eigen::Matrix<double, 6, 1> e;
        poseInnovation(res.pose, guess, e);
        Eigen::Matrix<double, 6, 6> C = res.cov;
        const double rf = 0.0175 * 0.0175, tf = 0.05 * 0.05;
        for (int k = 0; k < 3; ++k) C(k, k) += rf;
        for (int k = 3; k < 6; ++k) C(k, k) += tf;
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(C);
        const double d2 = e.dot(ldlt.solve(e));
        if (std::isfinite(d2) && d2 > m_accept_cfg.mahalanobis_chi2) return false;
    }
    if (m_accept_cfg.mode == 0)
        return res.fitness >= m_accept_cfg.min_coverage;

    // mode 1: 覆盖率达标直接接受; 否则用残差 + 多帧一致性仲裁
    if (res.fitness >= m_accept_cfg.min_coverage) {
        m_corr_prev_valid = false;
        m_consistent_cnt = 0;
        return true;
    }
    const double res_icp = res.residual;
    const double res_prior = m_matcher->evaluateResidual(scan, guess);
    if (res_icp < 0.0 || res_prior <= 1e-6) return false;

    const double improve = (res_prior - res_icp) / res_prior;
    const V3D corr = res.pose.trans - guess.trans;
    bool consistent = false;
    if (m_corr_prev_valid && corr.norm() > 1e-6 &&
        (corr.dot(m_corr_prev) > 0.0 || corr.norm() < 0.02))
        consistent = true;
    m_consistent_cnt = consistent ? m_consistent_cnt + 1 : 0;
    m_corr_prev = corr;
    m_corr_prev_valid = true;

    if (improve > 3.0 * m_accept_cfg.residual_margin) return true;
    if (improve > m_accept_cfg.residual_margin &&
        m_consistent_cnt >= m_accept_cfg.consistent_frames) return true;
    // [LocProbe] 拒绝原因 (每 10 帧一行): 定位"匹配成功但不输出"时靠它定位是哪道门限
    {
        static int probe_n = 0;
        if (++probe_n % 50 == 0) {   // 5s 一条: 100Hz 输出下 1Hz 仍然太吵
            std::cout << "[LocProbe] REJECT fitness=" << res.fitness
                      << " (min_coverage=" << m_accept_cfg.min_coverage << ")"
                      << " improve=" << improve
                      << " margin=" << m_accept_cfg.residual_margin
                      << " consistent=" << m_consistent_cnt << "/"
                      << m_accept_cfg.consistent_frames << std::endl;
        }
    }
    return false;
}

void LocalizationPipeline::onLidar(const LidarFrame& f) {
    // 唯一的"大状态"写入者: 整个匹配/重定位期间持 m_state_mu.
    // 高频输出 predictAt 只走 m_prior_mu, 因此不会被这里的长耗时阻塞.
    std::lock_guard<std::mutex> state_lk(m_state_mu);
    if (!m_map || !f.cloud || f.cloud->empty()) { m_status = LOST; return; }

    // 冷启动 / 丢失恢复: 尝试全局重定位 (多候选, 取最优通过者), 节流避免在未知环境刷爆
    if (!m_initialized || m_status == LOST) {
        if (m_reloc_cooldown > 0) {
            --m_reloc_cooldown;
        } else if (m_initialized && tryBoundedRecovery(f.cloud)) {
            // 出口重锁成功
            m_status = LOCALIZED;
            m_reloc_cooldown = 0;
            m_last_match = MatchResult();
            m_last_match.converged = true;
            m_last_match.pose = m_pose;
        } else {
            std::vector<GlobalCandidate> cands;
            const int nk = ensureRelocalizerMap()
                ? m_relocalizer->localizeTopK(f.cloud, std::max(1, m_global_cfg.top_k), cands)
                : 0;
            if (nk > 0) {
                // 分数降序: 正确候选几乎总是分最高的, 先精配它 -> 命中即提前停
                std::sort(cands.begin(), cands.end(),
                          [](const GlobalCandidate& a, const GlobalCandidate& b) {
                              return a.score > b.score;
                          });
                // L6: 用多候选初始化假设 (歧义时保留 Top-K)
                std::vector<PoseStamped> hposes;
                std::vector<double> hscores;
                for (const auto& c : cands) { hposes.push_back(c.pose); hscores.push_back(c.score); }
                if (nk > 1) m_hyp.seed(hposes, hscores);
                else m_hyp.reset(cands.front().pose);

                // **多候选一起验**: BBS 只给粗候选(常含"看起来相似的另一处"), 而正确地点
                // 的覆盖率能到 0.99、错地点稳定卡在 ~0.6。所以每个候选都做多起点精配,
                // 取覆盖率最高的那个; 全部都不够高就判定未定位(不给假位姿)。
                double best_fit = -1.0;
                double second_fit = -1.0;   // 次优精配分: 消歧需要 top1-top2 差值
                PoseStamped best_pose;
                for (const auto& cand : cands) {
                m_pose = cand.pose;
                m_pose.time = f.end_time;
                {
                    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                    m_prior->reset(m_pose);
                }
                m_monitor->reset();
                m_mediocre_streak = 0;
                m_initialized = true;
                m_status = LOCALIZED;
                m_coast_count = 0;
                m_reloc_cooldown = 0;
                m_has_cached_target = false;
                m_smoother.reset(m_pose);

                // **BBS 是粗搜(0.3m / 6 层)**: 实机重锁后覆盖率只有 ~0.71, 而用正确位姿
                // 初始化时是 0.99。这里用当前 matcher 在候选位姿处反复精配(gicp),
                // 只接受"覆盖率更高"的结果, 把粗搜的位姿收敛到 ~0.95+。
                // **BBS 是粗搜(0.3m / 6 层)**: 实机重锁后覆盖率只有 ~0.61~0.71, 而正确位姿
                // 初始化时是 0.99。单点精配无效(候选已落在 gicp 的局部极小里, 第一次迭代
                // 覆盖率不升就退出), 所以这里做**小范围多起点**: ±0.4m / ±6°, 共 45 个起点,
                // 每个都跑一次 gicp, 取覆盖率最高者。
                double refined_fit = cand.score;
                {
                    // **两级搜索**: BBS 候选的误差实测可达 ~1.4m / 数度, 只搜 ±0.4m
                    // 会把正确位姿排除在搜索范围外(45 个起点全部停在同一个局部极大)。
                    // 第一级 ±2.0m(步长1.0) × ±12°(步长6°); 第二级在最优处 ±0.5m(0.25) × ±3°。
                    const PoseStamped base_pose = m_pose;
                    auto evalAt = [&](const double dx, const double dy, const double dth_deg,
                                      PoseStamped& best_pose) -> bool {
                        PoseStamped g = base_pose;
                        g.trans = base_pose.trans + V3D(dx, dy, 0.0);
                        g.rot = Eigen::AngleAxisd(dth_deg * M_PI / 180.0, V3D::UnitZ())
                                    .toRotationMatrix() * base_pose.rot;
                        CloudType::Ptr target = buildTarget(g);
                        if (!target || target->empty()) return false;
                        m_matcher->setTarget(target);
                        MatchResult r;
                        if (!m_matcher->match(f.cloud, g, r) || !r.converged) return false;
                        if (r.fitness <= refined_fit) return false;
                        refined_fit = r.fitness;
                        best_pose = r.pose;
                        return true;
                    };

                    PoseStamped best_pose = base_pose;
                    bool have_best = false;
                    double best_dx = 0.0, best_dy = 0.0, best_dth = 0.0;
                    // 墙钟预算 + 达标即停: 全量两级 = 每候选 200 次 GICP, 多候选时
                    // 单帧回调被拖到分钟级, 单线程 spinner 饿死(实机 /relocation
                    // 排队数十秒、输出停更)。正确候选通常前几个起点就到 0.95+。
                    const auto t0 = std::chrono::steady_clock::now();
                    auto budget_left = [&]() {
                        return std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - t0).count() < 3000.0;
                    };
                    bool stop = false;
                    for (double a = -12.0; a <= 12.01 && !stop; a += 6.0) {
                        for (double dx = -2.0; dx <= 2.01 && !stop; dx += 1.0) {
                            for (double dy = -2.0; dy <= 2.01; dy += 1.0) {
                                if (evalAt(dx, dy, a, best_pose)) {
                                    have_best = true;
                                    best_dx = dx; best_dy = dy; best_dth = a;
                                }
                                stop = (refined_fit >= 0.95) || !budget_left();
                            }
                        }
                    }
                    if (have_best) {
                        // 第二级: 在粗搜最优附近细化 (已足够好或预算耗尽则跳过)
                        if (refined_fit < 0.97 && budget_left()) {
                            stop = false;
                            for (double a = best_dth - 3.0; a <= best_dth + 3.01 && !stop; a += 3.0) {
                                for (double dx = best_dx - 0.5; dx <= best_dx + 0.51 && !stop; dx += 0.25) {
                                    for (double dy = best_dy - 0.5; dy <= best_dy + 0.51; dy += 0.25) {
                                        evalAt(dx, dy, a, best_pose);
                                        stop = (refined_fit >= 0.97) || !budget_left();
                                    }
                                }
                            }
                        }
                        m_pose = best_pose;
                        {
                            std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                            m_prior->reset(m_pose);
                        }
                        m_smoother.reset(m_pose);
                    }
                    std::cout << "[LocPipeline] relocalization refine: " << cand.score
                              << " -> " << refined_fit << " (two-stage ±2m/±12deg)" << std::endl;
                }

                    if (refined_fit > best_fit) {
                        second_fit = best_fit;   // 原最优降为次优
                        best_fit = refined_fit;
                        best_pose = m_pose;
                    } else if (refined_fit > second_fit) {
                        second_fit = refined_fit;
                    }
                    // 已锁定(≥0.95)且**已评出第二名** -> 不再精配其余候选, 把 spinner
                    // 还给服务/发布。消歧必须评出第二名: 重复结构下错候选一样能到
                    // 0.99+ (2026-10-17 推行事故: 单候选 0.999 锁进相似车位)。
                    if (best_fit >= 0.95 && second_fit > 0.0) break;
                }   // for (const auto& cand : cands)

                // 全部候选都不够高 -> 判定未定位, 绝不给"看起来对但覆盖率只有 0.6"的假位姿。
                // 接受线 = reloc_accept_score(0.85): 弱重定位宁可保持 LOST(界面如实显示、
                // 人工点选兜底), 也不接受后进入"僵尸-判丢-再弱接受"的循环(2026-10-17 实测)。
                if (best_fit < m_global_cfg.reloc_accept_score) {
                    m_status = LOST;
                    m_initialized = false;
                    // 冷却: 之前这里漏设, 下一帧立刻又是一场全量全局风暴(fpfh top_k=5
                    // × 两级 200 GICP), 单线程 spinner 被连续独占
                    m_reloc_cooldown = std::max(1, m_global_cfg.retry_interval);
                    {
                        std::ostringstream os;
                        os << "relocalization weak: best_fit=" << best_fit
                           << " < " << m_global_cfg.reloc_accept_score << " -> keep LOST";
                        diag::log('W', "LocPipeline", os.str());
                        FlightRecorder::instance().event("RELOC_WEAK", os.str());
                        FlightRecorder::instance().onStatusChange(true);
                    }
                    return;
                }
                m_pose = best_pose;
                m_pose.time = f.end_time;
                {
                    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                    m_prior->reset(m_pose);
                }
                m_smoother.reset(m_pose);
                m_initialized = true;
                m_status = LOCALIZED;
                m_last_match = MatchResult();
                m_last_match.converged = true;
                m_last_match.fitness = best_fit;
                m_last_match.pose = m_pose;
                const double gap = (second_fit > 0.0) ? (best_fit - second_fit) : 1.0;
                if (nk == 1)
                    diag::log('W', "LocPipeline", "single-candidate reloc, no disambiguation "
                              "source, best_fit=" + std::to_string(best_fit));
                {
                    std::ostringstream os;
                    os << "relocalized by " << m_relocalizer_name
                       << " cands=" << nk << " hypotheses=" << m_hyp.size()
                       << " best_fit=" << best_fit << " second=" << second_fit
                       << (gap < m_ambiguity_margin ? " AMBIGUOUS(multi-hyp kept)" : "");
                    diag::log('I', "LocPipeline", os.str());
                    FlightRecorder::instance().event("RELOC", os.str());
                    FlightRecorder::instance().onStatusChange(false);
                }
            } else {
                m_reloc_cooldown = std::max(1, m_global_cfg.retry_interval);
            }
        }
        if (!m_initialized && m_status != LOCALIZED) return;  // 尚无初值
        if (m_status == LOCALIZED) return;
    }

    PoseStamped guess;
    {
        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
        guess = m_prior->predict(f.end_time);
    }

    MatchResult res;
    bool accepted = false;
    PoseStamped out_pose = guess;

    if (m_hyp.ambiguous()) {
        // L6 多假设: 逐假设局部配准并评分, 取最优, 淘汰落后
        const int nh = m_hyp.size();
        for (int i = 0; i < nh; ++i) {
            const PoseStamped g = m_hyp.all()[i].pose;
            if (!m_matcher->usesStore()) m_matcher->setTarget(buildTarget(g));
            MatchResult r;
            if (m_matcher->match(f.cloud, g, r)) m_hyp.update(i, r);
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
        }
        m_has_cached_target = false;
    } else {
        // 局部地图缓存: 位姿小幅移动(或全图)且**叠加层未变**时复用, 避免每帧重建
        // 降采样+KD-tree。之前配了维护层(m_maintainer)就直接放弃缓存 —— 但 overlay
        // 只在真正写入新点时变(revision 自增), 静态环境基本不增长; 而每帧重建 10k 目标
        // 的降采样+建树是把位姿输出拖到 10Hz 的主因之一(雷达 15.8Hz, 2026-10-17 实测)。
        bool rebuild_target = true;
        const uint64_t maint_rev = m_maintainer ? m_maintainer->revision() : 0;
        if (m_has_cached_target && maint_rev == m_cached_target_rev &&
            (m_local_map_name == "global" ||
             (guess.trans - m_cached_target_pose.trans).norm() < m_local_map_cfg.cache_move)) {
            rebuild_target = false;
        }
        if (!m_matcher->usesStore() && rebuild_target) {
            CloudType::Ptr target = buildTarget(guess);
            // **逸出保护 (fail fast)**: 局部地图为空 => 猜测位姿已经在图外(或该处无关键帧)。
            // 以前会拿空目标去匹配: 空 KD-tree 报错、ok=0、然后运动先验继续积分 ->
            // 位姿越漂越远(实机漂到 -96m), 从此永远匹配不上, 界面上角标彻底冻结。
            // 这里回退到最近一次被接受的位姿并复位运动先验, 让下一次仍有匹配机会。
            if (!target || target->empty()) {
                if (m_have_last_accepted) {
                    m_pose = m_last_accepted;
                    {
                        std::lock_guard<std::mutex> prior_lk(m_prior_mu);
                        if (m_prior) m_prior->reset(m_pose);
                    }
                    out_pose = m_last_accepted;
                } else {
                    out_pose = guess;
                }
                out_pose.time = f.end_time;
                return;      // 本次判定为未定位, 让上层报"丢失"而不是静默
            }
            m_cached_target = target;
            m_cached_target_pose = guess;
            m_cached_target_rev = m_maintainer ? m_maintainer->revision() : 0;
            m_has_cached_target = true;
            m_matcher->setTarget(target);
        }
        const bool ok = m_matcher->match(f.cloud, guess, res);
        // [LocProbe] 诊断: 输入扫描点数 / 匹配结果 (NO Effective Points 排查)
        {
            static int probe_n = 0;
            if (++probe_n % 50 == 0) {   // 5s 一条: 100Hz 输出下 1Hz 仍然太吵
                std::cout << "[LocProbe] scan=" << (f.cloud ? f.cloud->size() : 0)
                          << " target=" << (m_cached_target ? m_cached_target->size() : 0)
                          << " ok=" << (ok ? 1 : 0) << " converged=" << (res.converged ? 1 : 0)
                          << " fitness=" << res.fitness << std::endl;
            }
        }
        accepted = ok && acceptMatch(res, f.cloud, guess);
        out_pose = accepted ? res.pose : guess;
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
    // 地面/平面约束 (锁 z/roll/pitch; 坡道跟随地形)
    m_planar.apply(out_pose);
    if (accepted) {
        {
            std::lock_guard<std::mutex> prior_lk(m_prior_mu);
            m_prior->onMatch(out_pose);
        }
        m_last_accepted = out_pose;      // 逸出保护用的回退锚点
        m_have_last_accepted = true;
    }
    m_pose = out_pose;
    m_last_match = res;
    m_last_match.pose = out_pose;
    // [DropProbe] 低fitness/被拒帧逐帧打印 (推行丢定位根因取证; 正常帧仍走10帧一条的LocProbe)。
    //   lag = 匹配修正量|配准-先验猜测|: 推行时逐帧增大 => 预测跟不上(假设A: 常速外推滞后)
    //   corr 骤降 => 动态滤波螺旋(滞后>0.4m 时删掉的恰是真实位置的证据)
    //   tgt 变化 => 目标子图换源(假设B: 关键帧覆盖缺口)
    if (res.fitness < 0.90 || !accepted) {
        const double lag = (out_pose.trans - guess.trans).norm();
        std::cout << "[DropProbe] fit=" << res.fitness
                  << " lag=" << lag
                  << " corr=" << res.correspondences
                  << " inl=" << res.inliers
                  << " tgt=" << (m_has_cached_target && m_cached_target ? m_cached_target->size() : 0)
                  << " acc=" << (accepted ? 1 : 0)
                  << " coast=" << m_coast_count << std::endl;
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

    // 地图维护: 仅在位姿可信(配准被采纳)时写入/雕刻叠加层
    if (m_maintainer) m_maintainer->onLocalized(out_pose, f.cloud, m_last_match, accepted);

    // 过桥计数: 连续未采纳帧数
    m_coast_count = accepted ? 0 : (m_coast_count + 1);

    // 跟踪期歧义: 连续未采纳 N 帧后, 用重定位多候选重播假设 (aliasing 纠错)
    m_low_fit_frames = accepted ? 0 : (m_low_fit_frames + 1);
    if (m_hyp_reseed_frames > 0 && !m_hyp.ambiguous() &&
        m_low_fit_frames >= m_hyp_reseed_frames &&
        ensureRelocalizerMap()) {
        std::vector<GlobalCandidate> cands;
        const int nk = m_relocalizer->localizeTopK(f.cloud, std::max(1, m_global_cfg.top_k), cands);
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
        const double rf = 0.0175 * 0.0175, tf = 0.05 * 0.05;
        for (int k = 0; k < 3; ++k) C(k, k) += rf;
        for (int k = 3; k < 6; ++k) C(k, k) += tf;
        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(C);
        const double d2 = e.dot(ldlt.solve(e));
        if (std::isfinite(d2)) mi.nees = d2;
    }
    MonitorLevel lvl = m_monitor->update(mi);
    // 过桥期(连续未采纳未超 coast_patience): 不判丢失, 保持降级连续外推
    if (lvl == MonitorLevel::LOST && m_coast_count < m_monitor_cfg.coast_patience)
        lvl = MonitorLevel::DEGRADED;
    // 僵尸带直通判丢(管线层, 不受过桥钳制): monitor 的平庸升级会被上面的过桥
    // 条款吞掉 —— acc 偶尔通过(0.6 > min_coverage 0.35)清零 coast, LOST 永远
    // 被降级(2026-10-17 实测: fitness 0.64 僵尸 4 分钟不判丢不恢复)。这里用
    // res.fitness 独立计数: 连续 mediocre_patience 帧上不了 credible 线直接
    // LOST -> 触发冷启动重定位(要么确认要么纠正, 代价一次几秒尝试)。
    m_mediocre_streak =
        (res.converged && res.fitness < m_monitor_cfg.credible_fitness)
            ? (m_mediocre_streak + 1) : 0;
    if (m_monitor_cfg.mediocre_patience > 0 &&
        m_mediocre_streak >= m_monitor_cfg.mediocre_patience) {
        m_status = LOST;
        {
            std::ostringstream os;
            os << "fitness<" << m_monitor_cfg.credible_fitness
               << " sustained " << m_mediocre_streak << " frames -> relocate";
            diag::log('W', "LocPipeline", "zombie-band LOST: " + os.str());
            FlightRecorder::instance().event("LOST", "zombie-band: " + os.str());
            FlightRecorder::instance().onStatusChange(true);
        }
        return;
    }
    switch (lvl) {
        case MonitorLevel::INIT:     m_status = INIT; break;
        case MonitorLevel::CREDIBLE: m_status = LOCALIZED; break;
        case MonitorLevel::DEGRADED: m_status = LOCALIZED; break;
        case MonitorLevel::LOST:     m_status = LOST; break;
    }
}

ILocalization::Status LocalizationPipeline::getStatus() {
    std::lock_guard<std::mutex> lk(m_state_mu);
    return m_status;
}

bool LocalizationPipeline::getPose(PoseStamped& out) {
    std::lock_guard<std::mutex> lk(m_state_mu);
    // 从未初始化/从未匹配则返回 false. 否则 ROS 层会把 setMapData 后残留的
    // 上一张图位姿(或默认 identity)当成"当前位姿"发出去 —— 切图瞬间表现为
    // 初始位姿相对地图零点偏 90°(IMU body 默认 identity 再被 -90° 输出).
    // 定位成功过之后 LOST 仍返回最后有效位姿, 供界面保持位置.
    if (!m_initialized.load()) return false;
    out = m_pose;
    return true;
}

bool LocalizationPipeline::getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) {
    std::lock_guard<std::mutex> lk(m_state_mu);
    // LOST/未初始化必须返回 false：ROS 层的 LostMonitor 用它判断“当前是否可信”。
    // getPose() 在定位成功后 LOST 仍可返回最后有效位姿，供界面保持最后位置显示。
    if (!m_initialized.load() || m_status == LOST) return false;
    out = m_pose;
    cov = m_last_match.cov;
    return true;
}

bool LocalizationPipeline::predictAt(double t, PoseStamped& out) {
    if (!m_initialized || !m_prior) return false;
    std::lock_guard<std::mutex> prior_lk(m_prior_mu);
    out = m_prior->predict(t);   // 运动先验(IMU)从最后校正位姿传播
    return true;
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
