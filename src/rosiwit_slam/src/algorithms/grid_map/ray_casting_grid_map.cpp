// src/algorithms/grid_map/ray_casting_grid_map.cpp
#include "algorithms/grid_map/ray_casting_grid_map.h"
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

// Bresenham 2D: 把 (c0,r0)->(c1,r1) 途经格标记为 free(0), 不覆盖 occupied(100).
void traceFree(std::vector<int8_t>& data, int w, int h, int c0, int r0, int c1, int r1) {
    int dx = std::abs(c1 - c0);
    int dy = -std::abs(r1 - r0);
    int sx = c0 < c1 ? 1 : -1;
    int sy = r0 < r1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (c0 >= 0 && c0 < w && r0 >= 0 && r0 < h) {
            int idx = r0 * w + c0;
            if (data[idx] != 100) data[idx] = 0;   // occupied 优先
        }
        if (c0 == c1 && r0 == r1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; c0 += sx; }
        if (e2 <= dx) { err += dx; r0 += sy; }
    }
}

} // namespace

bool RayCastingGridMap::build(const CloudType::Ptr& cloud,
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

bool RayCastingGridMap::buildFromObservations(const std::vector<Observation>& obs,
                                              const Eigen::Vector3d& /*origin*/,
                                              const GridMapConfig& cfg,
                                              GridInfo& out) {
    // 1. 收集有效点并统计包围盒(含所有观测原点, 保证原点格在地图内)
    //    XY 用分位数稳健包围盒, 避免个别远端噪点把地图量程拉大.
    CloudType all;
    std::vector<double> xs, ys;
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();
    for (const auto& o : obs) {
        if (!o.cloud) continue;
        for (const auto& p : o.cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            all.push_back(p);
            xs.push_back(p.x);
            ys.push_back(p.y);
            min_z = std::min(min_z, static_cast<double>(p.z));
            max_z = std::max(max_z, static_cast<double>(p.z));
        }
    }
    if (all.empty()) return false;
    XYBounds bb = robustXYBounds(xs, ys);
    for (const auto& o : obs) {
        bb.min_x = std::min(bb.min_x, o.origin.x());
        bb.max_x = std::max(bb.max_x, o.origin.x());
        bb.min_y = std::min(bb.min_y, o.origin.y());
        bb.max_y = std::max(bb.max_y, o.origin.y());
    }
    double min_x = bb.min_x, max_x = bb.max_x;
    double min_y = bb.min_y, max_y = bb.max_y;

    // 2. 网格分辨率: 超出格数上限则自适应放粗, 避免 int 溢出与内存爆炸
    const long long MAX_CELLS = 25LL * 1000 * 1000;   // 2500 万格
    double res = std::max(1e-3, cfg.resolution);
    double span_x = std::max(max_x - min_x, res);
    double span_y = std::max(max_y - min_y, res);
    double need = std::ceil(span_x / res) * std::ceil(span_y / res);
    if (need > static_cast<double>(MAX_CELLS)) {
        double scale = std::sqrt(need / static_cast<double>(MAX_CELLS));
        res *= scale;
        std::cout << "[RayCast] WARN map too large, resolution coarsened "
                  << cfg.resolution << " -> " << res << std::endl;
    }

    // 3. 每帧自适应体素下采样 leaf: 限制每轴体素数, 避免 VoxelGrid 整数索引溢出
    const double extent = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 1e-3});
    const double leaf = std::max(0.02, std::max(res, extent / 1024.0));

    int width  = static_cast<int>(std::ceil(span_x / res)) + 3;
    int height = static_cast<int>(std::ceil(span_y / res)) + 3;
    double origin_x = min_x - res;
    double origin_y = min_y - res;

    // 4. 逐帧下采样并逐帧估计地面 (RANSAC 抗倾斜/漂移)
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

    std::cout << "[RayCast] ground_method=" << cfg.ground_method
              << " band=[" << cfg.min_height << "," << cfg.max_height << "]"
              << " res=" << res << " leaf=" << leaf << std::endl;

    // 5. 初始化 unknown(-1), 逐帧射线投射
    std::vector<int8_t> data(static_cast<size_t>(width) * height, -1);
    const double max_range2 = (cfg.max_range > 0.0) ? cfg.max_range * cfg.max_range : -1.0;
    int occupied_cells = 0;
    long long rays = 0;
    for (size_t i = 0; i < obs.size(); ++i) {
        const Eigen::Vector3d& o = obs[i].origin;
        int oc = static_cast<int>((o.x() - origin_x) / res);
        int orow = static_cast<int>((o.y() - origin_y) / res);
        if (oc >= 0 && oc < width && orow >= 0 && orow < height) {
            data[cellIndex(oc, orow, width)] = 0;   // 原点格必为 free
        }
        for (const auto& pt : scans[i]->points) {
            double dx = pt.x - o.x();
            double dy = pt.y - o.y();
            if (max_range2 >= 0.0 && dx * dx + dy * dy > max_range2) continue;
            int col = static_cast<int>((pt.x - origin_x) / res);
            int row = static_cast<int>((pt.y - origin_y) / res);
            if (col < 0 || col >= width || row < 0 || row >= height) continue;
            traceFree(data, width, height, oc, orow, col, row);
            float h = grounds[i].height(pt);
            if (h >= cfg.min_height && h <= cfg.max_height) {
                int idx = cellIndex(col, row, width);
                if (data[idx] != 100) { data[idx] = 100; occupied_cells++; }
            }
            ++rays;
        }
    }

    // 5.5 去噪: 剔除小块占据簇 (膨胀前)
    if (cfg.min_component_cells > 1) {
        int before = occupied_cells;
        removeSmallOccupiedComponents(data, width, height, cfg.min_component_cells, -1);
        occupied_cells = 0;
        for (auto v : data) if (v == 100) ++occupied_cells;
        std::cout << "[RayCast] denoise: occupied " << before << " -> " << occupied_cells << std::endl;
    }
    if (cfg.close_radius > 0) {
        morphCloseOccupied(data, width, height, cfg.close_radius);
        occupied_cells = 0;
        for (auto v : data) if (v == 100) ++occupied_cells;
    }

    // 6. 障碍膨胀: 连通稀疏回波, 并加厚以匹配机器人半径
    int dilate = cfg.dilate_radius;
    if (dilate < 0) {
        dilate = (cfg.point_spacing > 0.0)
            ? static_cast<int>(std::floor(cfg.point_spacing / (2.0 * res)))
            : 0;
    }
    if (dilate > 0 && occupied_cells > 0) {
        std::vector<int8_t> dilated = data;
        const int r2 = dilate * dilate;
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                if (data[cellIndex(col, row, width)] != 100) continue;
                for (int dy = -dilate; dy <= dilate; ++dy) {
                    int nr = row + dy;
                    if (nr < 0 || nr >= height) continue;
                    for (int dx = -dilate; dx <= dilate; ++dx) {
                        if (dx * dx + dy * dy > r2) continue;
                        int nc = col + dx;
                        if (nc < 0 || nc >= width) continue;
                        int idx = cellIndex(nc, nr, width);
                        if (dilated[idx] == 0) {          // 只膨胀 free, 保留 unknown
                            dilated[idx] = 100;
                            occupied_cells++;
                        }
                    }
                }
            }
        }
        data.swap(dilated);
    }

    int free_cells = 0;
    for (auto v : data) if (v == 0) ++free_cells;
    std::cout << "[RayCast] " << width << "x" << height
              << " obs=" << obs.size() << " rays=" << rays
              << " occupied=" << occupied_cells << " free=" << free_cells
              << " total=" << data.size() << " dilate=" << dilate
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
