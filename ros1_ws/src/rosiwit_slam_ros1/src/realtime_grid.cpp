// ============================================================================
// 建图模式实时栅格的实现 (设计文档 §5.11 A)
// 逐行复刻核心 PcdMapManager::generateGridMap() (pcd_map_manager.cpp:264-293)
// ============================================================================
#include "realtime_grid.h"

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <vector>

#include "algorithms/grid_map/grid_map_factory.h"      // GridMapFactory
#include "algorithms/grid_map/grid_map_postprocess.h"  // applyGridPostProcess / parsePostPipeline

namespace rosiwit_slam_ros1 {
namespace {

constexpr const char* kDefaultGenerator = "height_projection";

/// 与核心一致: 未知生成器名时回落 height_projection (pcd_map_manager.cpp:109-115)
std::unique_ptr<rosiwit_slam::IGridMapGenerator> createGenerator(const std::string& name) {
    auto gen = rosiwit_slam::GridMapFactory::create(name);
    if (!gen) {
        ROS_WARN_STREAM("[realtime_grid] unknown grid_map_generator '" << name
                        << "', falling back to " << kDefaultGenerator);
        gen = rosiwit_slam::GridMapFactory::create(kDefaultGenerator);
    }
    return gen;
}

} // namespace

bool buildGridFromPoints(const rosiwit_slam::PointVec& pts,
                         const rosiwit_slam::GridMapConfig& cfg,
                         const rosiwit_slam::GridPostConfig& post,
                         const std::string& generator,
                         rosiwit_slam::GridInfo& out) {
    auto gen = createGenerator(generator);
    if (!gen) return false;

    // PointVec 与 CloudType 是同一种点 (pcl::PointXYZINormal), 直接搬进 PCL 容器。
    // build() 对空点云返回 false (height_projection_grid_map.cpp:22), 与这里语义一致。
    rosiwit_slam::CloudType::Ptr cloud(new rosiwit_slam::CloudType());
    cloud->reserve(pts.size());
    for (const auto& p : pts) cloud->push_back(p);

    // 原点取 (0,0,0): 与核心 generateGridMap() 一致 (pcd_map_manager.cpp:277)
    if (!gen->build(cloud, Eigen::Vector3d::Zero(), cfg, out)) return false;

    // 后处理要用**实际**分辨率 (生成器可能因格数上限放粗), 见 pcd_map_manager.cpp:280。
    // 核心是改自己的成员变量, 这里用局部副本, 语义等价。
    rosiwit_slam::GridPostConfig p = post;
    p.resolution = out.resolution;
    rosiwit_slam::applyGridPostProcess(out, p);
    return true;
}

void loadGridConfigFromYaml(const std::string& yaml_path,
                            rosiwit_slam::GridMapConfig& cfg,
                            rosiwit_slam::GridPostConfig& post,
                            std::string& generator) {
    // 先落到与核心 GridMapConfig/GridPostConfig 相同的默认值
    generator = kDefaultGenerator;

    std::ifstream fin(yaml_path);
    if (!fin.good()) {
        ROS_WARN_STREAM("[realtime_grid] grid config not readable, using built-in defaults: "
                        << yaml_path);
        return;
    }
    try {
        const YAML::Node c = YAML::LoadFile(yaml_path);
        if (!c.IsMap()) {
            ROS_WARN_STREAM("[realtime_grid] grid config is not a map, using built-in defaults: "
                            << yaml_path);
            return;
        }

        // ---- 生成策略 + GridMapConfig (pcd_map_manager.cpp:56-81) ----
        if (c["grid_map_generator"]) generator = c["grid_map_generator"].as<std::string>();
        if (c["grid_min_height"]) cfg.min_height = c["grid_min_height"].as<double>();
        if (c["grid_max_height"]) cfg.max_height = c["grid_max_height"].as<double>();
        if (c["grid_occupied_thresh"]) cfg.occupied_thresh = c["grid_occupied_thresh"].as<int>();
        if (c["grid_dilate_radius"]) cfg.dilate_radius = c["grid_dilate_radius"].as<int>();
        if (c["grid_max_range"]) cfg.max_range = c["grid_max_range"].as<double>();
        if (c["grid_ground_z"]) cfg.ground_z = c["grid_ground_z"].as<double>();
        if (c["grid_ground_method"]) cfg.ground_method = c["grid_ground_method"].as<std::string>();
        if (c["grid_ransac_dist_thr"]) cfg.ransac_dist_thr = c["grid_ransac_dist_thr"].as<double>();
        if (c["grid_ransac_max_iter"]) cfg.ransac_max_iter = c["grid_ransac_max_iter"].as<int>();
        if (c["grid_min_hits"]) cfg.min_hits = c["grid_min_hits"].as<int>();
        if (c["grid_min_misses"]) cfg.min_misses = c["grid_min_misses"].as<int>();
        if (c["grid_mark_radius"]) cfg.mark_radius = c["grid_mark_radius"].as<int>();

        // ---- GridPostConfig (pcd_map_manager.cpp:83-104) ----
        if (c["grid_post_pipeline"])
            post.pipeline = rosiwit_slam::parsePostPipeline(c["grid_post_pipeline"].as<std::string>());
        if (c["grid_min_component_cells"])
            post.min_component_cells = c["grid_min_component_cells"].as<int>();
        if (c["grid_connectivity"]) post.connectivity = c["grid_connectivity"].as<int>();
        if (c["grid_no_info_is_obstacle"])
            post.no_info_is_obstacle = c["grid_no_info_is_obstacle"].as<bool>();
        if (c["grid_close_radius"]) post.close_radius = c["grid_close_radius"].as<int>();
        if (c["grid_open_radius"]) post.open_radius = c["grid_open_radius"].as<int>();
        if (c["grid_fill_max_hole_cells"])
            post.max_hole_cells = c["grid_fill_max_hole_cells"].as<int>();
        if (c["grid_dilate_radius"]) post.dilate_radius = c["grid_dilate_radius"].as<int>();
        if (c["grid_skeleton_width"]) post.skeleton_width = c["grid_skeleton_width"].as<int>();
        if (c["grid_line_min_len_m"]) post.line_min_len_m = c["grid_line_min_len_m"].as<double>();
        if (c["grid_axis_snap"]) post.axis_snap = c["grid_axis_snap"].as<bool>();

        // ---- 分辨率与点间距: 核心是在 generateGridMap() 里赋的, 这里在装载时定下来 ----
        // (核心: cfg.resolution = 调用方传入的 resolution; cfg.point_spacing = save_voxel_)
        if (c["map_resolution"]) cfg.resolution = c["map_resolution"].as<double>();
        if (c["map_save_voxel"]) {
            const double voxel = c["map_save_voxel"].as<double>();
            cfg.point_spacing = voxel;
            post.point_spacing = voxel;
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("[realtime_grid] grid config parse error (" << e.what()
                        << "), using built-in defaults: " << yaml_path);
    }
}

RealtimeGrid::RealtimeGrid() = default;

RealtimeGrid::~RealtimeGrid() { stop(); }

void RealtimeGrid::setConfig(const rosiwit_slam::GridMapConfig& cfg,
                             const rosiwit_slam::GridPostConfig& post,
                             const std::string& generator) {
    std::lock_guard<std::mutex> lk(m_mtx_);
    m_cfg_ = cfg;
    m_post_ = post;
    m_generator_ = generator;
}

void RealtimeGrid::setSource(Source s) {
    std::lock_guard<std::mutex> lk(m_mtx_);
    m_source_ = std::move(s);
}

void RealtimeGrid::setGridSource(GridSource s) {
    std::lock_guard<std::mutex> lk(m_mtx_);
    m_grid_source_ = std::move(s);
}

void RealtimeGrid::start(double hz) {
    std::lock_guard<std::mutex> lk(m_mtx_);
    if (!m_stop_) return;  // 已经在跑
    m_hz_ = (hz > 0.0) ? hz : 1.0;
    m_stop_ = false;
    m_thread_ = std::thread(&RealtimeGrid::run, this);
}

void RealtimeGrid::stop() {
    {
        std::lock_guard<std::mutex> lk(m_mtx_);
        m_stop_ = true;
    }
    m_cv_.notify_all();
    if (m_thread_.joinable()) m_thread_.join();
}

bool RealtimeGrid::latest(rosiwit_slam::GridInfo& out) const {
    std::lock_guard<std::mutex> lk(m_out_mtx_);
    if (!m_has_) return false;
    out = m_latest_;
    return true;
}

void RealtimeGrid::clear() {
    std::lock_guard<std::mutex> lk(m_out_mtx_);
    m_latest_ = rosiwit_slam::GridInfo();
    m_has_ = false;
}

void RealtimeGrid::run() {
    std::unique_lock<std::mutex> lk(m_mtx_);
    while (!m_stop_) {
        // 快照配置后放开锁: 重算是 O(点数), 不能占着锁, 否则 setConfig/stop 会被卡住
        const Source src = m_source_;
        const GridSource gsrc = m_grid_source_;
        const rosiwit_slam::GridMapConfig cfg = m_cfg_;
        const rosiwit_slam::GridPostConfig post = m_post_;
        const std::string gen = m_generator_;
        const double hz = m_hz_;
        lk.unlock();

        if (src || gsrc) {
            try {
                rosiwit_slam::GridInfo info;
                bool ok = false;
                if (gsrc) {
                    // 直接取核心生成好的栅格 (与存 pgm 同一条路径, 有逐帧地面处理)
                    ok = gsrc(info);
                } else {
                    rosiwit_slam::PointVec pts;
                    ok = src(pts) && buildGridFromPoints(pts, cfg, post, gen, info);
                }
                if (ok) {
                    std::lock_guard<std::mutex> out_lk(m_out_mtx_);
                    m_latest_ = std::move(info);
                    m_has_ = true;
                }
            } catch (const std::exception& e) {
                ROS_WARN_STREAM_THROTTLE(5.0, "[realtime_grid] build failed: " << e.what());
            }
        }

        lk.lock();
        if (m_stop_) break;
        m_cv_.wait_for(lk, std::chrono::duration<double>(1.0 / hz),
                       [this] { return m_stop_; });
    }
}

} // namespace rosiwit_slam_ros1
