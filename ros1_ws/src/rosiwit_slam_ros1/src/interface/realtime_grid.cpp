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
        const YAML::Node gmap = (c["global"] && c["global"]["map"])
                                    ? c["global"]["map"] : YAML::Node();
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

        // ---- 分辨率与点间距: 最终 2D 图在 mapping.grid.resolution, 3D 在 global.map.voxel_resolution ----
        if (grid && grid["resolution"]) cfg.resolution = grid["resolution"].as<double>();
        const double voxel = (gmap && gmap["save_voxel"])
                                 ? gmap["save_voxel"].as<double>()
                                 : (gv("save_voxel") ? gv("save_voxel").as<double>()
                                                     : c["map_save_voxel"].as<double>(0.0));
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
    if (m_thread_) return;   // 已经在跑
    const double rate = (hz > 0.0) ? hz : 1.0;
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / rate));
    // PeriodicThread: sleep_until(next += period), 单次重算超时不会累积漂移
    m_thread_.reset(new rosiwit_common::PeriodicThread(
        "rt_grid", period, [this] { runOnce(); }));
}

void RealtimeGrid::stop() {
    if (!m_thread_) return;
    m_thread_->requestStop();
    m_thread_->join();
    m_thread_.reset();
}

bool RealtimeGrid::latest(rosiwit_slam::GridInfo& out) const {
    const auto snap = m_slot_.load();
    if (!snap) return false;
    out = *snap;
    return true;
}

void RealtimeGrid::clear() {
    m_slot_.publish(nullptr);   // 等价于"从没算出过"
}

void RealtimeGrid::runOnce() {
    // 快照配置后放开锁: 重算是 O(点数), 不能占着锁, 否则 setConfig/stop 会被卡住
    GridSource gsrc;
    {
        std::lock_guard<std::mutex> lk(m_mtx_);
        gsrc = m_grid_source_;
    }
    if (!gsrc) return;
    try {
        rosiwit_slam::GridInfo info;
        // 直接取核心生成好的栅格 (与存 pgm 同一条路径, 有逐帧地面处理)
        if (gsrc(info)) {
            m_slot_.publish(std::make_shared<const rosiwit_slam::GridInfo>(std::move(info)));
        }
    } catch (const std::exception& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "[realtime_grid] build failed: " << e.what());
    }
}

} // namespace rosiwit_slam_ros1
