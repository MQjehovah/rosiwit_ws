// src/algorithms/grid_map/height_projection_grid_map.cpp
#include "algorithms/grid_map/height_projection_grid_map.h"
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
}

bool HeightProjectionGridMap::build(const CloudType::Ptr& cloud,
                                    const Eigen::Vector3d& origin,
                                    const GridMapConfig& cfg,
                                    GridInfo& out)
{
    if (!cloud || cloud->empty()) return false;

    // 1. 仅保留有限坐标点并统计包围盒:
    //    建图结果发散时会出现极端离群点, NaN/Inf 会让 VoxelGrid 得到垃圾边界,
    //    进而导致下面 int 网格尺寸溢出、std::vector 分配错乱而段错误。
    //    XY 用分位数稳健包围盒, 避免个别远端噪点把地图量程拉大.
    CloudType::Ptr finite(new CloudType());
    finite->reserve(cloud->size());
    std::vector<double> xs, ys, zs;
    xs.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        finite->push_back(p);
        xs.push_back(p.x);
        ys.push_back(p.y);
        zs.push_back(p.z);
    }
    if (finite->empty()) return false;
    XYBounds bb = robustXYBounds(xs, ys);
    double min_x = std::min(bb.min_x, origin.x());
    double max_x = std::max(bb.max_x, origin.x());
    double min_y = std::min(bb.min_y, origin.y());
    double max_y = std::max(bb.max_y, origin.y());
    auto [zmin_it, zmax_it] = std::minmax_element(zs.begin(), zs.end());
    double min_z = *zmin_it, max_z = *zmax_it;

    // 2. 网格分辨率: 若按请求分辨率会超出格数上限, 自适应放粗, 避免 int 溢出与内存爆炸
    const long long MAX_CELLS = 25LL * 1000 * 1000;   // 2500 万格
    double res = std::max(1e-3, cfg.resolution);
    double span_x = std::max(max_x - min_x, res);
    double span_y = std::max(max_y - min_y, res);
    double need = std::ceil(span_x / res) * std::ceil(span_y / res);
    if (need > static_cast<double>(MAX_CELLS)) {
        double scale = std::sqrt(need / static_cast<double>(MAX_CELLS));
        res *= scale;
        std::cout << "[GridMap] WARN map too large, resolution coarsened "
                  << cfg.resolution << " -> " << res << std::endl;
    }

    // 3. 自适应体素下采样 leaf: 限制每轴体素数, 避免 PCL VoxelGrid 整数索引溢出.
    //    leaf 不细于网格分辨率, 否则会把多个可能落入同一格的原始点提前合并丢失.
    const double extent = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 1e-3});
    const double leaf = std::max(0.02, std::max(res, extent / 1024.0));
    CloudType::Ptr filtered(new CloudType());
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(finite);
    vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
    vg.filter(*filtered);
    if (filtered->empty()) return false;

    double gmin_x = min_x, gmax_x = max_x;
    double gmin_y = min_y, gmax_y = max_y;

    int width  = static_cast<int>(std::ceil((gmax_x - gmin_x) / res)) + 3;
    int height = static_cast<int>(std::ceil((gmax_y - gmin_y) / res)) + 3;
    double origin_x = gmin_x - res;
    double origin_y = gmin_y - res;

    // 自适应地面检测 (RANSAC 平面 / 分位数)
    GroundModel ground = fitGround(finite, cfg);

    std::cout << "[GridMap] ground_method=" << cfg.ground_method
              << " n=[" << ground.n.x() << "," << ground.n.y() << "," << ground.n.z()
              << "] d=" << ground.d << " valid=" << ground.valid
              << " band=[" << cfg.min_height << "," << cfg.max_height << "]"
              << " res=" << res << " leaf=" << leaf << std::endl;

    int total_cells = width * height;

    // 统计每格的障碍物点数 (高度为相对地面的有符号高度)
    std::vector<int> obs_count(total_cells, 0);
    for (const auto& pt : filtered->points) {
        int col = static_cast<int>((pt.x - origin_x) / res);
        int row = static_cast<int>((pt.y - origin_y) / res);
        if (col < 0 || col >= width || row < 0 || row >= height) continue;
        float h = ground.height(pt);
        if (h >= cfg.min_height && h <= cfg.max_height) {
            obs_count[cellIndex(col, row, width)]++;
        }
    }

    // 生成栅格: 默认全部 FREE, 只标记障碍高度范围的点为 OCCUPIED
    std::vector<int8_t> data(total_cells, 0);
    int occupied_cells = 0;
    for (int i = 0; i < total_cells; ++i) {
        if (obs_count[i] >= cfg.occupied_thresh) {
            data[i] = 100;
            occupied_cells++;
        }
    }

    // 去噪: 剔除小块占据簇 (膨胀前)
    if (cfg.min_component_cells > 1) {
        int before = occupied_cells;
        removeSmallOccupiedComponents(data, width, height, cfg.min_component_cells, 0);
        occupied_cells = 0;
        for (auto v : data) if (v == 100) ++occupied_cells;
        std::cout << "[GridMap] denoise: occupied " << before << " -> " << occupied_cells << std::endl;
    }
    if (cfg.close_radius > 0) {
        morphCloseOccupied(data, width, height, cfg.close_radius);
        occupied_cells = 0;
        for (auto v : data) if (v == 100) ++occupied_cells;
    }

    // 障碍膨胀: 建图点云经体素下采样后点间距通常大于网格分辨率,
    // 单点只落在孤立格中会形成"零星点/断续墙". 按点间距自动膨胀(或显式指定)
    // 把这些点连成连续墙体, 同时略微加厚.
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
                        if (dilated[idx] == 0) {
                            dilated[idx] = 100;
                            occupied_cells++;
                        }
                    }
                }
            }
        }
        data.swap(dilated);
    }

    std::cout << "[GridMap] " << width << "x" << height
              << " occupied=" << occupied_cells
              << " total=" << total_cells
              << " dilate=" << dilate << " res=" << res << std::endl;

    out.width = width;
    out.height = height;
    out.resolution = res;
    out.origin_x = origin_x;
    out.origin_y = origin_y;
    out.data = std::move(data);
    return true;
}

} // namespace rosiwit_slam
