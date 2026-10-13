// src/algorithms/grid_map/occupancy_logodds.cpp
#include "algorithms/grid_map/occupancy_logodds.h"
#include "algorithms/grid_map/grid_map_utils.h"
#include "algorithms/grid_map/ground_segmentation.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

namespace rosiwit_slam {

namespace {

inline int cellIndex(int col, int row, int width) { return row * width + col; }

void addHit(std::vector<uint16_t>& hits, int w, int h, int c, int r) {
    if (c < 0 || c >= w || r < 0 || r >= h) return;
    size_t idx = static_cast<size_t>(r) * w + c;
    if (hits[idx] < std::numeric_limits<uint16_t>::max()) hits[idx]++;
}
void addMiss(std::vector<uint16_t>& misses, int w, int h, int c, int r) {
    if (c < 0 || c >= w || r < 0 || r >= h) return;
    size_t idx = static_cast<size_t>(r) * w + c;
    if (misses[idx] < std::numeric_limits<uint16_t>::max()) misses[idx]++;
}

// Bresenham: 途经格计 miss, 端点(及 mark_radius 圆盘)计 hit
void rayCount(std::vector<uint16_t>& hits, std::vector<uint16_t>& misses,
              int w, int h, int c0, int r0, int c1, int r1,
              bool mark_hit, int mark_radius) {
    int dx = std::abs(c1 - c0);
    int dy = -std::abs(r1 - r0);
    int sx = c0 < c1 ? 1 : -1;
    int sy = r0 < r1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        bool endpoint = (c0 == c1 && r0 == r1);
        if (endpoint) {
            if (mark_hit) addHit(hits, w, h, c0, r0);
        } else {
            addMiss(misses, w, h, c0, r0);
        }
        if (endpoint) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; c0 += sx; }
        if (e2 <= dx) { err += dx; r0 += sy; }
    }
    if (mark_hit && mark_radius > 0) {
        int r2 = mark_radius * mark_radius;
        for (int dr = -mark_radius; dr <= mark_radius; ++dr)
            for (int dc = -mark_radius; dc <= mark_radius; ++dc)
                if (dc * dc + dr * dr <= r2) addHit(hits, w, h, c1 + dc, r1 + dr);
    }
}

} // namespace

bool LogOddsGridMap::build(const CloudType::Ptr& cloud,
                           const Eigen::Vector3d& origin,
                           const GridMapConfig& cfg,
                           GridInfo& out) {
    std::vector<Observation> obs;
    Observation o;
    o.origin = origin;
    o.cloud = cloud;
    obs.push_back(o);
    return buildFromObservations(obs, origin, cfg, out);
}

bool LogOddsGridMap::buildFromObservations(const std::vector<Observation>& obs,
                                           const Eigen::Vector3d& /*origin*/,
                                           const GridMapConfig& cfg,
                                           GridInfo& out) {
    // 1. 收集有效点 + 稳健 XY 包围盒(含原点)
    std::vector<double> xs, ys;
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    bool any = false;
    for (const auto& o : obs) {
        if (!o.cloud) continue;
        for (const auto& p : o.cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            any = true;
            xs.push_back(p.x);
            ys.push_back(p.y);
            min_z = std::min(min_z, static_cast<double>(p.z));
            max_z = std::max(max_z, static_cast<double>(p.z));
        }
    }
    if (!any) return false;
    XYBounds bb = robustXYBounds(xs, ys);
    for (const auto& o : obs) {
        bb.min_x = std::min(bb.min_x, o.origin.x());
        bb.max_x = std::max(bb.max_x, o.origin.x());
        bb.min_y = std::min(bb.min_y, o.origin.y());
        bb.max_y = std::max(bb.max_y, o.origin.y());
    }
    double min_x = bb.min_x, max_x = bb.max_x;
    double min_y = bb.min_y, max_y = bb.max_y;

    // 2. 分辨率自适应
    const long long MAX_CELLS = 25LL * 1000 * 1000;
    double res = std::max(1e-3, cfg.resolution);
    double span_x = std::max(max_x - min_x, res);
    double span_y = std::max(max_y - min_y, res);
    double need = std::ceil(span_x / res) * std::ceil(span_y / res);
    if (need > static_cast<double>(MAX_CELLS)) {
        double scale = std::sqrt(need / static_cast<double>(MAX_CELLS));
        res *= scale;
        std::cout << "[OccCount] WARN map too large, resolution coarsened "
                  << cfg.resolution << " -> " << res << std::endl;
    }

    // 3. 逐帧下采样 + 逐帧地面
    const double extent = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 1e-3});
    const double leaf = std::max(0.02, std::max(res, extent / 1024.0));
    std::vector<CloudType::Ptr> scans(obs.size());
    std::vector<GroundModel> grounds(obs.size());
    for (size_t i = 0; i < obs.size(); ++i) {
        scans[i].reset(new CloudType());
        if (!obs[i].cloud) continue;
        CloudType::Ptr finite(new CloudType());
        finite->reserve(obs[i].cloud->size());
        for (const auto& p : obs[i].cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            finite->push_back(p);
        }
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(finite);
        vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
        vg.filter(*scans[i]);
        grounds[i] = fitGround(scans[i], cfg);
    }

    int width  = static_cast<int>(std::ceil(span_x / res)) + 3;
    int height = static_cast<int>(std::ceil(span_y / res)) + 3;
    double origin_x = min_x - res;
    double origin_y = min_y - res;

    std::cout << "[OccCount] ground_method=" << cfg.ground_method
              << " band=[" << cfg.min_height << "," << cfg.max_height << "]"
              << " min_hits=" << cfg.min_hits
              << " res=" << res << " leaf=" << leaf << std::endl;

    // 4. 命中/未命中计数 (对 2D 投影下"远处地面射线穿过墙体格"的情况比 log-odds 更鲁棒)
    std::vector<uint16_t> hits(static_cast<size_t>(width) * height, 0);
    std::vector<uint16_t> misses(static_cast<size_t>(width) * height, 0);
    const double max_range2 = (cfg.max_range > 0.0) ? cfg.max_range * cfg.max_range : -1.0;
    long long rays = 0;
    for (size_t i = 0; i < obs.size(); ++i) {
        const Eigen::Vector3d& o = obs[i].origin;
        int oc = static_cast<int>((o.x() - origin_x) / res);
        int orow = static_cast<int>((o.y() - origin_y) / res);
        for (const auto& pt : scans[i]->points) {
            double dx = pt.x - o.x();
            double dy = pt.y - o.y();
            if (max_range2 >= 0.0 && dx * dx + dy * dy > max_range2) continue;
            int col = static_cast<int>((pt.x - origin_x) / res);
            int row = static_cast<int>((pt.y - origin_y) / res);
            if (col < 0 || col >= width || row < 0 || row >= height) continue;
            float hgt = grounds[i].height(pt);
            bool is_obstacle = (hgt >= cfg.min_height && hgt <= cfg.max_height);
            rayCount(hits, misses, width, height, oc, orow, col, row,
                     is_obstacle, cfg.mark_radius);
            ++rays;
        }
    }

    // 5. 阈值化 → 三态: 命中次数达标判占据; 否则未命中次数达标判自由; 其余未知
    //    单帧输入时命中上限为 1, 故按观测帧数收紧阈值, 保证单帧也能出占据.
    int num_obs = 0;
    for (const auto& o : obs) if (o.cloud) ++num_obs;
    const int need_hits = std::max(1, std::min(cfg.min_hits, num_obs));
    std::vector<int8_t> data(static_cast<size_t>(width) * height, -1);
    int occupied_cells = 0, free_cells = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        if (hits[i] >= need_hits) { data[i] = 100; occupied_cells++; }
        else if (misses[i] >= cfg.min_misses) { data[i] = 0; free_cells++; }
        else data[i] = -1;
    }

    // 6. 收尾去噪/形态学/膨胀 统一交给后处理管道
    std::cout << "[OccCount] " << width << "x" << height
              << " obs=" << obs.size() << " rays=" << rays
              << " occupied=" << occupied_cells << " free=" << free_cells
              << " total=" << data.size()
              << " res=" << res << std::endl;

    out.width = width;
    out.height = height;
    out.resolution = res;
    out.origin_x = origin_x;
    out.origin_y = origin_y;
    out.data = std::move(data);
    return true;
}

} // namespace rosiwit_slam
