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


void loadGridConfigFromYaml(const std::string& yaml_path,
                            rosiwit_slam::GridMapConfig& cfg,
                            rosiwit_slam::GridPostConfig& post,
                            std::string& generator) {
    // 先落到与核心 GridMapConfig/GridPostConfig 相同的默认值
    generator = "height_projection";

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

        const YAML::Node grid =
            (c["mapping"] && c["mapping"]["grid"]) ? c["mapping"]["grid"] : c;
        auto gv = [&](const char* key) -> YAML::Node {
            if (grid[key]) return grid[key];
            const std::string legacy = (std::string(key) == "generator")
                                           ? "grid_map_generator"
                                           : (std::string("grid_") + key);
            return c[legacy];
        };

        // ---- 生成策略 + GridMapConfig (pcd_map_manager.cpp:56-81) ----
        if (gv("generator")) generator = gv("generator").as<std::string>();
        if (gv("min_height")) cfg.min_height = gv("min_height").as<double>();
        if (gv("max_height")) cfg.max_height = gv("max_height").as<double>();
        if (gv("occupied_thresh")) cfg.occupied_thresh = gv("occupied_thresh").as<int>();
        if (gv("dilate_radius")) cfg.dilate_radius = gv("dilate_radius").as<int>();
        if (gv("max_range")) cfg.max_range = gv("max_range").as<double>();
        if (gv("ground_z")) cfg.ground_z = gv("ground_z").as<double>();
        if (gv("ground_method")) cfg.ground_method = gv("ground_method").as<std::string>();
        if (gv("ransac_dist_thr")) cfg.ransac_dist_thr = gv("ransac_dist_thr").as<double>();
        if (gv("ransac_max_iter")) cfg.ransac_max_iter = gv("ransac_max_iter").as<int>();
        if (gv("min_hits")) cfg.min_hits = gv("min_hits").as<int>();
        if (gv("min_misses")) cfg.min_misses = gv("min_misses").as<int>();
        if (gv("mark_radius")) cfg.mark_radius = gv("mark_radius").as<int>();

        // ---- GridPostConfig (pcd_map_manager.cpp:83-104) ----
        if (gv("post_pipeline"))
            post.pipeline = rosiwit_slam::parsePostPipeline(gv("post_pipeline").as<std::string>());
        if (gv("min_component_cells")) post.min_component_cells = gv("min_component_cells").as<int>();
        if (gv("connectivity")) post.connectivity = gv("connectivity").as<int>();
        if (gv("no_info_is_obstacle")) post.no_info_is_obstacle = gv("no_info_is_obstacle").as<bool>();
        if (gv("close_radius")) post.close_radius = gv("close_radius").as<int>();
        if (gv("open_radius")) post.open_radius = gv("open_radius").as<int>();
        if (gv("fill_max_hole_cells")) post.max_hole_cells = gv("fill_max_hole_cells").as<int>();
        if (gv("dilate_radius")) post.dilate_radius = gv("dilate_radius").as<int>();
        if (gv("skeleton_width")) post.skeleton_width = gv("skeleton_width").as<int>();
        if (gv("line_min_len_m")) post.line_min_len_m = gv("line_min_len_m").as<double>();
        if (gv("axis_snap")) post.axis_snap = gv("axis_snap").as<bool>();

        // ---- 分辨率与点间距: 核心是在 generateGridMap() 里赋的, 这里在装载时定下来 ----
        if (c["map_resolution"]) cfg.resolution = c["map_resolution"].as<double>();
        const double voxel = gv("save_voxel") ? gv("save_voxel").as<double>()
                                              : c["map_save_voxel"].as<double>(0.0);
        if (voxel > 0.0) {
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
        const GridSource gsrc = m_grid_source_;
        const rosiwit_slam::GridMapConfig cfg = m_cfg_;
        const rosiwit_slam::GridPostConfig post = m_post_;
        const std::string gen = m_generator_;
        const double hz = m_hz_;
        lk.unlock();

        if (gsrc) {
            try {
                rosiwit_slam::GridInfo info;
                // 直接取核心生成好的栅格 (与存 pgm 同一条路径, 有逐帧地面处理)
                const bool ok = gsrc(info);
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
