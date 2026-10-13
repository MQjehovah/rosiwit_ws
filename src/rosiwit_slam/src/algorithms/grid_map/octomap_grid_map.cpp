// src/algorithms/grid_map/octomap_grid_map.cpp
#include "algorithms/grid_map/octomap_grid_map.h"
#include "algorithms/grid_map/grid_map_utils.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>

namespace rosiwit_slam {

namespace {
inline int cellIndex(int col, int row, int width) { return row * width + col; }
}

bool OctomapGridMap::build(const CloudType::Ptr& cloud,
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

bool OctomapGridMap::buildFromObservations(const std::vector<Observation>& obs,
                                           const Eigen::Vector3d& /*origin*/,
                                           const GridMapConfig& cfg,
                                           GridInfo& out) {
    // 1. 收集有效点包围盒(含原点) + 地面检测直方图
    //    XY 用分位数稳健包围盒, 避免个别远端噪点把地图量程拉大.
    std::vector<double> xs, ys, zs;   // 所有有限点坐标
    for (const auto& o : obs) {
        if (!o.cloud) continue;
        for (const auto& p : o.cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            xs.push_back(p.x);
            ys.push_back(p.y);
            zs.push_back(p.z);
        }
    }
    if (xs.empty()) return false;
    XYBounds bb = robustXYBounds(xs, ys);
    for (const auto& o : obs) {
        bb.min_x = std::min(bb.min_x, o.origin.x());
        bb.max_x = std::max(bb.max_x, o.origin.x());
        bb.min_y = std::min(bb.min_y, o.origin.y());
        bb.max_y = std::max(bb.max_y, o.origin.y());
    }
    double min_x = bb.min_x, max_x = bb.max_x;
    double min_y = bb.min_y, max_y = bb.max_y;

    // 2. 分辨率自适应防溢出
    const long long MAX_CELLS = 25LL * 1000 * 1000;
    double res = std::max(1e-3, cfg.resolution);
    double span_x = std::max(max_x - min_x, res);
    double span_y = std::max(max_y - min_y, res);
    double need = std::ceil(span_x / res) * std::ceil(span_y / res);
    if (need > static_cast<double>(MAX_CELLS)) {
        double scale = std::sqrt(need / static_cast<double>(MAX_CELLS));
        res *= scale;
        std::cout << "[OctoMap] WARN map too large, resolution coarsened "
                  << cfg.resolution << " -> " << res << std::endl;
    }

    int width  = static_cast<int>(std::ceil(span_x / res)) + 3;
    int height = static_cast<int>(std::ceil(span_y / res)) + 3;
    double origin_x = min_x - res;
    double origin_y = min_y - res;

    // 3. 自适应地面检测
    double ground_z = std::isnan(cfg.ground_z) ? detectGroundZ(zs) : cfg.ground_z;
    double obs_min_z = ground_z + cfg.min_height;
    double obs_max_z = ground_z + cfg.max_height;

    // 4. OctoMap 累积: 逐帧 insertPointCloud (内部完成射线 free 更新)
    octomap::OcTree tree(res);
    const double maxrange = (cfg.max_range > 0.0) ? cfg.max_range : -1.0;
    long long inserted = 0;
    for (const auto& o : obs) {
        if (!o.cloud || o.cloud->empty()) continue;
        octomap::Pointcloud scan;
        for (const auto& p : o.cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            scan.push_back(p.x, p.y, p.z);
        }
        if (scan.size() == 0) continue;
        tree.insertPointCloud(scan,
                              octomap::point3d(o.origin.x(), o.origin.y(), o.origin.z()),
                              maxrange);
        inserted += scan.size();
    }
    tree.updateInnerOccupancy();

    std::cout << "[OctoMap] Ground Z=" << ground_z
              << " obstacle: [" << obs_min_z << ", " << obs_max_z << "]"
              << " res=" << res << std::endl;

    // 5. 3D 占据/自由叶节点投影到 2D
    std::vector<int8_t> data(static_cast<size_t>(width) * height, -1);
    int occupied_cells = 0, free_cells = 0;
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        double x = it.getX(), y = it.getY(), z = it.getZ();
        int col = static_cast<int>((x - origin_x) / res);
        int row = static_cast<int>((y - origin_y) / res);
        if (col < 0 || col >= width || row < 0 || row >= height) continue;
        int idx = cellIndex(col, row, width);
        if (tree.isNodeOccupied(*it)) {
            if (z >= obs_min_z && z <= obs_max_z) {
                if (data[idx] != 100) { data[idx] = 100; occupied_cells++; }
            }
        } else if (z <= obs_max_z) {
            if (data[idx] == -1) { data[idx] = 0; free_cells++; }
        }
    }

    // 5.5 去噪: 剔除小块占据簇 (膨胀前)
    if (cfg.min_component_cells > 1) {
        int before = occupied_cells;
        removeSmallOccupiedComponents(data, width, height, cfg.min_component_cells, -1);
        occupied_cells = 0;
        for (auto v : data) if (v == 100) ++occupied_cells;
        std::cout << "[OctoMap] denoise: occupied " << before << " -> " << occupied_cells << std::endl;
    }
    if (cfg.close_radius > 0) {
        morphCloseOccupied(data, width, height, cfg.close_radius);
        occupied_cells = 0;
        for (auto v : data) if (v == 100) ++occupied_cells;
    }

    // 6. 障碍膨胀
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
                        if (dilated[idx] == 0) { dilated[idx] = 100; occupied_cells++; }
                    }
                }
            }
        }
        data.swap(dilated);
    }

    std::cout << "[OctoMap] " << width << "x" << height
              << " obs=" << obs.size() << " pts=" << inserted
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
