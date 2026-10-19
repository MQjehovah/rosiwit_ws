#include "algorithms/pcd_map_manager/pcd_map_manager.h"
#include "algorithms/grid_map/grid_map_factory.h"
#include "algorithms/grid_map/grid_map_io.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {

namespace {
// SE3 组合/求逆: poseMul(a,b) = 先 b 后 a
PoseStamped poseMul(const PoseStamped& a, const PoseStamped& b) {
    PoseStamped c;
    c.time = b.time;
    c.rot = a.rot * b.rot;
    c.trans = a.rot * b.trans + a.trans;
    return c;
}
PoseStamped poseInv(const PoseStamped& p) {
    PoseStamped q;
    q.time = p.time;
    q.rot = p.rot.transpose();
    q.trans = -q.rot * p.trans;
    return q;
}
}  // namespace

PcdMapManager::PcdMapManager()
    : global_map_(new CloudType()) {}

bool PcdMapManager::init(const std::string& config_path) {
    std::ifstream fin(config_path);
    if (!fin.good()) {
        std::cerr << "[PcdMapManager] Config not found: " << config_path
                  << ", using defaults" << std::endl;
        map_dir_ = ".";
        grid_gen_ = GridMapFactory::create("height_projection");
        initialized_ = true;
        return true;
    }

#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["map_directory"])
            map_dir_ = config["map_directory"].as<std::string>();
        if (config["map_save_voxel"])
            save_voxel_ = config["map_save_voxel"].as<double>();
        if (config["grid_map_generator"])
            grid_gen_name_ = config["grid_map_generator"].as<std::string>();
        if (config["grid_min_height"])
            grid_cfg_.min_height = config["grid_min_height"].as<double>();
        if (config["grid_max_height"])
            grid_cfg_.max_height = config["grid_max_height"].as<double>();
        if (config["grid_occupied_thresh"])
            grid_cfg_.occupied_thresh = config["grid_occupied_thresh"].as<int>();
        if (config["grid_dilate_radius"])
            grid_cfg_.dilate_radius = config["grid_dilate_radius"].as<int>();
        if (config["grid_max_range"])
            grid_cfg_.max_range = config["grid_max_range"].as<double>();
        if (config["grid_ground_z"])
            grid_cfg_.ground_z = config["grid_ground_z"].as<double>();
        if (config["grid_ground_method"])
            grid_cfg_.ground_method = config["grid_ground_method"].as<std::string>();
        if (config["grid_ransac_dist_thr"])
            grid_cfg_.ransac_dist_thr = config["grid_ransac_dist_thr"].as<double>();
        if (config["grid_ransac_max_iter"])
            grid_cfg_.ransac_max_iter = config["grid_ransac_max_iter"].as<int>();
        if (config["grid_min_hits"])
            grid_cfg_.min_hits = config["grid_min_hits"].as<int>();
        if (config["grid_min_misses"])
            grid_cfg_.min_misses = config["grid_min_misses"].as<int>();
        if (config["grid_mark_radius"])
            grid_cfg_.mark_radius = config["grid_mark_radius"].as<int>();
        // ---- 2D 后处理管道 ----
        if (config["grid_post_pipeline"])
            grid_post_cfg_.pipeline = parsePostPipeline(config["grid_post_pipeline"].as<std::string>());
        if (config["grid_min_component_cells"])
            grid_post_cfg_.min_component_cells = config["grid_min_component_cells"].as<int>();
        if (config["grid_connectivity"])
            grid_post_cfg_.connectivity = config["grid_connectivity"].as<int>();
        if (config["grid_no_info_is_obstacle"])
            grid_post_cfg_.no_info_is_obstacle = config["grid_no_info_is_obstacle"].as<bool>();
        if (config["grid_close_radius"])
            grid_post_cfg_.close_radius = config["grid_close_radius"].as<int>();
        if (config["grid_open_radius"])
            grid_post_cfg_.open_radius = config["grid_open_radius"].as<int>();
        if (config["grid_fill_max_hole_cells"])
            grid_post_cfg_.max_hole_cells = config["grid_fill_max_hole_cells"].as<int>();
        if (config["grid_dilate_radius"])
            grid_post_cfg_.dilate_radius = config["grid_dilate_radius"].as<int>();
        if (config["grid_skeleton_width"])
            grid_post_cfg_.skeleton_width = config["grid_skeleton_width"].as<int>();
        if (config["grid_line_min_len_m"])
            grid_post_cfg_.line_min_len_m = config["grid_line_min_len_m"].as<double>();
        if (config["grid_axis_snap"])
            grid_post_cfg_.axis_snap = config["grid_axis_snap"].as<bool>();
    } catch (const std::exception& e) {
        std::cerr << "[PcdMapManager] Config parse error: " << e.what() << std::endl;
        return false;
    }
    grid_gen_ = GridMapFactory::create(grid_gen_name_);
    if (!grid_gen_) {
        std::cerr << "[PcdMapManager] Unknown grid_map_generator '" << grid_gen_name_
                  << "', fallback to height_projection" << std::endl;
        grid_gen_name_ = "height_projection";
        grid_gen_ = GridMapFactory::create(grid_gen_name_);
    }
#else
    (void)config_path;
    grid_gen_ = GridMapFactory::create(grid_gen_name_);
#endif

    if (map_dir_.empty()) map_dir_ = ".";
    initialized_ = true;
    return true;
}

bool PcdMapManager::saveMap(const std::string& name) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) {
        std::cerr << "[PcdMapManager] Nothing to save" << std::endl;
        return false;
    }

    std::string path = map_dir_ + "/" + name;
    if (pcl::io::savePCDFileBinary(path, *global_map_) == -1) {
        std::cerr << "[PcdMapManager] Save failed: " << path << std::endl;
        return false;
    }
    std::cout << "[PcdMapManager] Saved: " << path
              << " (" << global_map_->size() << " pts)" << std::endl;
    return true;
}

bool PcdMapManager::loadMap(const std::string& name) {
    std::string path = map_dir_ + "/" + name;
    CloudType::Ptr cloud(new CloudType());
    if (pcl::io::loadPCDFile(path, *cloud) == -1) {
        std::cerr << "[PcdMapManager] Load failed: " << path << std::endl;
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex_);
    global_map_ = cloud;
    observations_.clear();   // 纯 PCD 无逐帧原点, 射线类策略退回整体点云
    ++m_revision_;
    std::cout << "[PcdMapManager] Loaded: " << path
              << " (" << global_map_->size() << " pts)" << std::endl;
    return true;
}

bool PcdMapManager::getGlobalMap(PointVec& out) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) return false;
    out.clear();
    out.reserve(global_map_->size());
    for (const auto& pt : *global_map_) {
        out.push_back(pt);
    }
    return true;
}

bool PcdMapManager::addSubMap(const KeyFrame& kf) {
    if (!kf.cloud || kf.cloud->empty()) return false;

    std::lock_guard<std::mutex> lock(m_mutex_);
    Submap sm;
    sm.pose = kf.pose;
    sm.cloud = kf.cloud;
    submaps_.push_back(sm);

    // 若后端已发生过回环修正, 新关键帧也要平移到修正坐标系下, 保证与重建后的地图连续
    const PoseStamped eff_pose = poseMul(m_frontier_anchor_, kf.pose);

    CloudType::Ptr transformed(new CloudType());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = eff_pose.rot.cast<float>();
    T.block<3,1>(0,3) = eff_pose.trans.cast<float>();
    pcl::transformPointCloud(*kf.cloud, *transformed, T);

    if (!global_map_) global_map_.reset(new CloudType());
    *global_map_ += *transformed;

    // 保存逐帧观测(原点=修正后位置), 供射线投射类栅格策略使用
    Observation ob;
    ob.origin = eff_pose.trans;
    ob.cloud = transformed;
    observations_.push_back(ob);

    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(global_map_);
    vg.setLeafSize(save_voxel_, save_voxel_, save_voxel_);
    CloudType::Ptr filtered(new CloudType());
    vg.filter(*filtered);
    global_map_ = filtered;

    ++m_revision_;
    return true;
}

void PcdMapManager::onBackendOptimized(const std::vector<PoseStamped>& corrected) {
    if (corrected.empty()) return;

    std::lock_guard<std::mutex> lock(m_mutex_);
    if (submaps_.empty()) return;

    // ⚠ **必须按帧对齐, 不能按下标**: 后端是异步的、可能只消化了前 k 个关键帧,
    // 而 submaps_ 是本地全部关键帧。按下标配对会把"第 i 个后端位姿"套到"第 i 个本地点云"上,
    // 一旦后端滞后几帧, 第 i 个其实不是同一帧 -> 整张图错位/旋转叠加。
    // 实机症状正是: 平时正常, **一触发重建就突然全糊、像全部旋转叠加**(重建前后 anchor 跳 5~36m)。
    // 每个关键帧的 pose.time 唯一, 用时间戳匹配 (容差 0.2s)。
    std::vector<int> kf_of_corr(corrected.size(), -1);
    for (size_t i = 0; i < corrected.size(); ++i) {
        int best = -1;
        double best_dt = 1e9;
        for (size_t j = 0; j < submaps_.size(); ++j) {
            const double dt = std::fabs(submaps_[j].pose.time - corrected[i].time);
            if (dt < best_dt) { best_dt = dt; best = static_cast<int>(j); }
        }
        if (best >= 0 && best_dt <= 0.2) kf_of_corr[i] = best;
    }
    // 匹配上的帧数太少 -> 说明本地/后端两套数据没法对齐, 宁可不重建也不能出错图
    size_t matched = 0;
    for (int v : kf_of_corr) if (v >= 0) ++matched;
    if (matched < 2) {
        std::cout << "[PcdMapManager] skip rebuild: only " << matched
                  << "/" << corrected.size() << " backend poses matched local keyframes by time"
                  << std::endl;
        return;
    }

    // 每个本地关键帧的最终位姿: 后端修正优先, 未匹配(后端还没消化)的用原始位姿
    std::vector<PoseStamped> used(submaps_.size());
    for (size_t j = 0; j < submaps_.size(); ++j) used[j] = submaps_[j].pose;
    for (size_t i = 0; i < corrected.size(); ++i) {
        if (kf_of_corr[i] >= 0) used[kf_of_corr[i]] = corrected[i];
    }

    // 前沿锚点 A = G_last * P_last^{-1} (同一帧!), 供 pipeline 把之后的新位姿换到修正系
    const int last_matched_kf = [&]() {
        for (size_t i = corrected.size(); i-- > 0;)
            if (kf_of_corr[i] >= 0) return kf_of_corr[i];
        return -1;
    }();
    if (last_matched_kf < 0) return;
    const PoseStamped& raw_last = submaps_[last_matched_kf].pose;
    PoseStamped anchor_new = poseMul(used[last_matched_kf], poseInv(raw_last));

    // 兜底: 相邻两帧之间的真实修正量不应有多大。超过阈值说明对齐/后端出了问题,
    // 保留上一次的锚点并告警, 避免把整张图瞬间平移旋转出去 (这才是"突然全糊"的直接表现)。
    const double dt_anchor = (anchor_new.trans - m_frontier_anchor_.trans).norm();
    const Eigen::AngleAxisd aa(anchor_new.rot * m_frontier_anchor_.rot.transpose());
    if (dt_anchor > m_max_correction_trans_m || std::fabs(aa.angle()) > m_max_correction_rot_rad) {
        std::cout << "[PcdMapManager] WARN reject correction: d_trans=" << dt_anchor
                  << "m d_rot=" << std::fabs(aa.angle()) * 180.0 / M_PI
                  << "deg (limits " << m_max_correction_trans_m << "m/"
                  << m_max_correction_rot_rad * 180.0 / M_PI << "deg); keep previous anchor"
                  << std::endl;
        return;
    }
    m_frontier_anchor_ = anchor_new;

    // 尾部未匹配的本地帧用原始位姿占位 (见上)
    (void)raw_last;

    CloudType::Ptr rebuilt(new CloudType());
    std::vector<Observation> rebuilt_obs;
    rebuilt_obs.reserve(submaps_.size());
    for (size_t i = 0; i < submaps_.size(); ++i) {
        const PoseStamped* pose = (i < used.size()) ? &used[i] : &submaps_[i].pose;
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = pose->rot.cast<float>();
        T.block<3,1>(0,3) = pose->trans.cast<float>();
        CloudType::Ptr transformed(new CloudType());
        pcl::transformPointCloud(*submaps_[i].cloud, *transformed, T);
        *rebuilt += *transformed;

        Observation ob;
        ob.origin = pose->trans;
        ob.cloud = transformed;
        rebuilt_obs.push_back(ob);
    }
    observations_ = std::move(rebuilt_obs);

    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(rebuilt);
    vg.setLeafSize(save_voxel_, save_voxel_, save_voxel_);
    CloudType::Ptr filtered(new CloudType());
    vg.filter(*filtered);
    global_map_ = filtered;

    std::cout << "[PcdMapManager] Rebuilt map from backend poses: " << used.size() << " kfs"
              << " (matched " << matched << "/" << corrected.size() << " backend poses)"
              << " (" << global_map_->size() << " pts), anchor trans="
              << m_frontier_anchor_.trans.norm() << " m" << std::endl;
    ++m_revision_;
}

bool PcdMapManager::getCorrectionAnchor(PoseStamped& out) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    out = m_frontier_anchor_;
    return true;
}


bool PcdMapManager::generateGridMap(double resolution) {
    // 地图生成不再全程持有 m_mutex_: 先快照观测/点云, 在锁外重建栅格,
    // 最后只短暂加锁写回. 否则 0.05m 栅格重建会阻塞前端 addSubMap,
    // 进而阻塞 SLAM 位姿/TF 发布, 表现成 TF 断树、只看到 costmap.
    std::vector<Observation> observations;
    CloudType::Ptr global_map;
    GridMapConfig cfg;
    GridPostConfig post;
    std::string gen_name;
    double save_voxel = 0.3;
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        if (!global_map_ || global_map_->empty()) return false;
        observations = observations_;
        global_map = global_map_;
        if (observations.empty() && global_map) {
            global_map.reset(new CloudType(*global_map));
        }
        cfg = grid_cfg_;
        post = grid_post_cfg_;
        gen_name = grid_gen_name_;
        save_voxel = save_voxel_;
    }

    auto gen = GridMapFactory::create(gen_name);
    if (!gen) gen = GridMapFactory::create("height_projection");
    if (!gen) return false;

    cfg.resolution = resolution;
    cfg.point_spacing = save_voxel;

    GridInfo info;
    const bool ok = !observations.empty()
        ? gen->buildFromObservations(observations, Eigen::Vector3d::Zero(), cfg, info)
        : gen->build(global_map, Eigen::Vector3d::Zero(), cfg, info);
    if (!ok) return false;

    post.resolution = info.resolution;
    post.point_spacing = save_voxel;
    applyGridPostProcess(info, post);

    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        m_grid_data = info.data;
        m_grid_w = info.width;
        m_grid_h = info.height;
        m_grid_res = info.resolution;
        m_grid_ox = info.origin_x;
        m_grid_oy = info.origin_y;
    }
    std::cout << "[PcdMapManager] Generated grid map (" << gen->name() << "): "
              << info.width << "x" << info.height << std::endl;
    return true;
}

bool PcdMapManager::saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                                  double resolution) {
    return saveGridMap(pgm_path, yaml_path, resolution, "map");
}

bool PcdMapManager::saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                                  double resolution, const std::string& frame_id) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    if (!global_map_ || global_map_->empty()) return false;
    if (!grid_gen_) grid_gen_ = GridMapFactory::create("height_projection");
    if (!grid_gen_) return false;

    GridMapConfig cfg = grid_cfg_;
    cfg.resolution = resolution;
    cfg.point_spacing = save_voxel_;

    GridInfo info;
    bool ok = !observations_.empty()
        ? grid_gen_->buildFromObservations(observations_, Eigen::Vector3d::Zero(), cfg, info)
        : grid_gen_->build(global_map_, Eigen::Vector3d::Zero(), cfg, info);
    if (!ok) return false;

    grid_post_cfg_.resolution = info.resolution;
    grid_post_cfg_.point_spacing = save_voxel_;
    applyGridPostProcess(info, grid_post_cfg_);

    if (!saveGridMapPGM(info, pgm_path, yaml_path, frame_id)) return false;

    m_grid_data = info.data;
    m_grid_w = info.width;
    m_grid_h = info.height;
    m_grid_res = info.resolution;
    m_grid_ox = info.origin_x;
    m_grid_oy = info.origin_y;
    std::cout << "[PcdMapManager] Grid map saved (" << grid_gen_->name() << "): " << pgm_path
              << " (" << m_grid_w << "x" << m_grid_h << ")" << std::endl;
    return true;
}

bool PcdMapManager::loadGridMap(const std::string& pgm_path, const std::string& yaml_path) {
    GridInfo info;
    if (!loadGridMapPGM(pgm_path, yaml_path, info)) return false;
    m_grid_data = info.data;
    m_grid_w = info.width;
    m_grid_h = info.height;
    m_grid_res = info.resolution;
    m_grid_ox = info.origin_x;
    m_grid_oy = info.origin_y;
    ++m_revision_;
    return true;
}

GridInfo PcdMapManager::getGridInfo() const {
    GridInfo info;
    info.width = m_grid_w;
    info.height = m_grid_h;
    info.resolution = m_grid_res;
    info.origin_x = m_grid_ox;
    info.origin_y = m_grid_oy;
    info.data = m_grid_data;
    return info;
}

bool PcdMapManager::setGridMapGenerator(const std::string& name) {
    std::lock_guard<std::mutex> lock(m_mutex_);
    auto gen = GridMapFactory::create(name);
    if (!gen) {
        std::cerr << "[PcdMapManager] Unknown grid generator: " << name << std::endl;
        return false;
    }
    grid_gen_ = std::move(gen);
    grid_gen_name_ = name;
    std::cout << "[PcdMapManager] grid generator -> " << name << std::endl;
    return true;
}

} // namespace rosiwit_slam
