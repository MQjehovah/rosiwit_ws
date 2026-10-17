// src/algorithms/localization/localization_pipeline.cpp
#include "algorithms/localization/localization_pipeline.h"
#include "algorithms/localization/localization_factory.h"
#include "algorithms/localization/localization_utils.h"
#include "io/xzmap_reader.h"
#include "algorithms/map_store/map_store_factory.h"
#include <cmath>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

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
    m_status = INIT;
    m_initialized = false;
    m_has_cached_target = false;
    m_map_path = map_name;
    m_map.reset(new CloudType());
    if (pcl::io::loadPCDFile(map_name, *m_map) == -1) {
        // **没有 PCD 的兼容回退**: 老格式 (xz_3d_slam) 的地图只有 <base>.xzmap + pgm/yaml,
        // 关键帧里自带点 —— 直接从关键帧重建基准点云。
        // 这样"切图/定位"就**不再要求 PCD 文件** (旧实现 localiza_ptr_->run("<dir>/map")
        // 也只是读它自己那份 xzmap, 从来不看 PCD)。
        m_map->clear();
        std::vector<io::XzMapKeyframe> kfs_fb;
        if (io::readXzMap(map_name + ".xzmap", kfs_fb) && !kfs_fb.empty()) {
            for (const auto& kf : kfs_fb) {
                for (const auto& p : kf.points) {
                    PointType pt;   // PCL 1.8 无 (x,y,z) 三参构造, 逐字段赋值
                    pt.x = p.x();
                    pt.y = p.y();
                    pt.z = p.z();
                    m_map->push_back(pt);
                }
            }
            std::cout << "[LocPipeline] no PCD at " << map_name << "; rebuilt base map from "
                      << map_name << ".xzmap (" << m_map->size() << " pts, " << kfs_fb.size()
                      << " kfs)" << std::endl;
        } else {
            std::cerr << "[LocPipeline] failed to load map: " << map_name << std::endl;
            m_map.reset();
            return;
        }
    } else {
        std::cout << "[LocPipeline] loaded map: " << map_name << " (" << m_map->size() << " pts)"
                  << std::endl;
    }
    if (m_local_map) m_local_map->setMap(map_name, m_map);
    m_planar.setMap(m_map);
    if (m_store) {
        m_store->buildFromCloud(m_map);
        if (m_matcher) m_matcher->setStore(m_store.get());
        m_planar.setStore(m_store.get());
    }
    if (m_maintainer) m_maintainer->setBaseMap(map_name, m_map);

    // 尝试载入关键帧 (map.pcd 的伴生文件 map.pcd.xzmap) 供 keyframe_submap 使用
    if (m_local_map) {
        std::vector<io::XzMapKeyframe> kfs;
        if (io::readXzMap(map_name + ".xzmap", kfs) && !kfs.empty()) {
            std::vector<LocalKeyframe> lkfs;
            lkfs.reserve(kfs.size());
            for (const auto& kf : kfs) {
                LocalKeyframe lk;
                lk.pose.trans = V3D(kf.pose.x, kf.pose.y, kf.pose.z);
                lk.pose.rot = Eigen::AngleAxisd(kf.pose.yaw, V3D::UnitZ()).toRotationMatrix() *
                              Eigen::AngleAxisd(kf.pose.pitch, V3D::UnitY()).toRotationMatrix() *
                              Eigen::AngleAxisd(kf.pose.roll, V3D::UnitX()).toRotationMatrix();
                lk.cloud.reset(new CloudType());
                lk.cloud->reserve(kf.points.size());
                // 注意: PCL 1.8 (melodic/18.04) 的 pcl::PointXYZINormal 没有 (x,y,z)
                // 三参构造 (PCL 1.12+ 才有), 因此逐字段赋值。默认构造已把 intensity /
                // curvature / normal_* 置零, data[3] 置 1, 与新版本三参构造语义一致。
                for (const auto& p : kf.points) {
                    PointType pt;
                    pt.x = p.x();
                    pt.y = p.y();
                    pt.z = p.z();
                    lk.cloud->push_back(pt);
                }
                lkfs.push_back(std::move(lk));
            }
            m_local_map->setKeyframes(lkfs);
            std::cout << "[LocPipeline] loaded " << lkfs.size()
                      << " keyframes from " << map_name << ".xzmap" << std::endl;
        }
    }
    if (m_relocalizer && m_relocalizer_name != "none")
        m_relocalizer->setMap(map_name, m_map);
}

void LocalizationPipeline::setInitPose(const PoseStamped& pose) {
    m_pose = pose;
    if (m_prior) m_prior->reset(pose);
    if (m_monitor) m_monitor->reset();
    m_corr_prev_valid = false;
    m_consistent_cnt = 0;
    m_initialized = true;
    m_status = INIT;
    m_smoother.reset(m_pose);
    m_hyp.reset(m_pose);
}

void LocalizationPipeline::onImu(const IMUSample& s) {
    if (m_prior) m_prior->onImu(s);
}

void LocalizationPipeline::requestRelocalization() {
    // 上层 /global_localization 触发: 强制回到未定位状态, 下一次 onLidar 走冷启动分支
    // (relocalizer=bbs 时即全局检索; =none 时退化为 tryBoundedRecovery 的有界局部重锁)
    m_status = LOST;
    m_initialized = false;
    m_reloc_cooldown = 0;
    std::cout << "[LocPipeline] relocalization requested by service" << std::endl;
}

void LocalizationPipeline::onOdom(const PoseStamped& odom) {
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

    const double offs[3] = {-1.5, 0.0, 1.5};
    const double yaws[3] = {-10.0, 0.0, 10.0};
    double best_fit = m_accept_cfg.min_coverage;
    PoseStamped best;
    bool found = false;
    for (double dx : offs)
        for (double dy : offs)
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
            }
    if (!found) return false;
    m_pose = best;
    m_pose.time = best.time;
    m_prior->reset(m_pose);
    m_monitor->reset();
    m_has_cached_target = false;
    m_coast_count = 0;
    std::cout << "[LocPipeline] bounded recovery, fitness=" << best_fit << std::endl;
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
            const int nk = m_relocalizer
                ? m_relocalizer->localizeTopK(f.cloud, std::max(1, m_global_cfg.top_k), cands)
                : 0;
            if (nk > 0) {
                // L6: 用多候选初始化假设 (歧义时保留 Top-K)
                std::vector<PoseStamped> hposes;
                std::vector<double> hscores;
                for (const auto& c : cands) { hposes.push_back(c.pose); hscores.push_back(c.score); }
                if (nk > 1) m_hyp.seed(hposes, hscores);
                else m_hyp.reset(cands.front().pose);

                const GlobalCandidate& cand = cands.front();
                m_pose = cand.pose;
                m_pose.time = f.end_time;
                m_prior->reset(m_pose);
                m_monitor->reset();
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
                    const double dxy[3] = {-0.4, 0.0, 0.4};
                    const double dth[5] = {-6.0, -3.0, 0.0, 3.0, 6.0};
                    MatchResult rr, best_r;
                    bool have_best = false;
                    for (double a : dth) {
                        for (double dx : dxy) {
                            for (double dy : dxy) {
                                PoseStamped g = m_pose;
                                g.trans = m_pose.trans + V3D(dx, dy, 0.0);
                                g.rot = Eigen::AngleAxisd(a * M_PI / 180.0, V3D::UnitZ())
                                            .toRotationMatrix() * m_pose.rot;
                                CloudType::Ptr target = buildTarget(g);
                                if (!target || target->empty()) continue;
                                m_matcher->setTarget(target);
                                if (!m_matcher->match(f.cloud, g, rr) || !rr.converged) continue;
                                if (rr.fitness > refined_fit) {
                                    refined_fit = rr.fitness;
                                    best_r = rr;
                                    have_best = true;
                                }
                            }
                        }
                    }
                    if (have_best) {
                        m_pose = best_r.pose;
                        m_prior->reset(m_pose);
                        m_smoother.reset(m_pose);
                    }
                    std::cout << "[LocPipeline] relocalization refine: " << cand.score
                              << " -> " << refined_fit << " (multi-start 45)" << std::endl;
                }

                m_last_match = MatchResult();
                m_last_match.converged = true;
                m_last_match.fitness = refined_fit;
                m_last_match.pose = m_pose;
                std::cout << "[LocPipeline] relocalized by " << m_relocalizer_name
                          << " cands=" << nk << " hypotheses=" << m_hyp.size()
                          << " score=" << cand.score << std::endl;
            } else {
                m_reloc_cooldown = std::max(1, m_global_cfg.retry_interval);
            }
        }
        if (!m_initialized && m_status != LOCALIZED) return;  // 尚无初值
        if (m_status == LOCALIZED) return;
    }

    const PoseStamped guess = m_prior->predict(f.end_time);

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
        // 局部地图缓存: 无维护层且位姿小幅移动(或全图)时复用, 避免每帧重建 KD-tree/体素
        bool rebuild_target = true;
        if (!m_maintainer && m_has_cached_target &&
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
                    if (m_prior) m_prior->reset(m_pose);
                    out_pose = m_last_accepted;
                } else {
                    out_pose = guess;
                }
                out_pose.time = f.end_time;
                return;      // 本次判定为未定位, 让上层报"丢失"而不是静默
            }
            m_cached_target = target;
            m_cached_target_pose = guess;
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
        m_prior->onMatch(out_pose);
        m_last_accepted = out_pose;      // 逸出保护用的回退锚点
        m_have_last_accepted = true;
    }
    m_pose = out_pose;
    m_last_match = res;
    m_last_match.pose = out_pose;
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
        m_relocalizer && m_relocalizer_name != "none") {
        std::vector<GlobalCandidate> cands;
        const int nk = m_relocalizer->localizeTopK(f.cloud, std::max(1, m_global_cfg.top_k), cands);
        if (nk > 1) {
            std::vector<PoseStamped> hp;
            std::vector<double> hs;
            for (const auto& c : cands) { hp.push_back(c.pose); hs.push_back(c.score); }
            m_hyp.seed(hp, hs);
            m_low_fit_frames = 0;
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
    switch (lvl) {
        case MonitorLevel::INIT:     m_status = INIT; break;
        case MonitorLevel::CREDIBLE: m_status = LOCALIZED; break;
        case MonitorLevel::DEGRADED: m_status = LOCALIZED; break;
        case MonitorLevel::LOST:     m_status = LOST; break;
    }
}

ILocalization::Status LocalizationPipeline::getStatus() { return m_status; }

bool LocalizationPipeline::getPose(PoseStamped& out) {
    if (m_status == LOST) return false;
    out = m_pose;
    return true;
}

bool LocalizationPipeline::getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) {
    if (m_status == LOST) return false;
    out = m_pose;
    cov = m_last_match.cov;
    return true;
}

bool LocalizationPipeline::predictAt(double t, PoseStamped& out) {
    if (!m_initialized || !m_prior) return false;
    out = m_prior->predict(t);   // 运动先验(IMU)从最后校正位姿传播
    return true;
}

void LocalizationPipeline::onAbsolute(const PoseStamped& pose,
                                      const Eigen::Matrix<double, 6, 6>& cov) {
    if (!m_abs_cfg.enabled) return;
    (void)cov;   // 门控互补融合暂只用位姿; 协方差供后续因子图
    m_abs_pose = pose;
    m_abs_time = pose.time;
    m_has_abs = true;
}

} // namespace rosiwit_slam
