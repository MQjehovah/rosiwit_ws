// src/algorithms/grid_map/height_projection_grid_map.cpp
#include "algorithms/grid_map/height_projection_grid_map.h"
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
    CloudType::Ptr finite(new CloudType());
    finite->reserve(cloud->size());
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double min_z = min_x, max_z = max_x;
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        finite->push_back(p);
        min_x = std::min(min_x, static_cast<double>(p.x));
        max_x = std::max(max_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_y = std::max(max_y, static_cast<double>(p.y));
        min_z = std::min(min_z, static_cast<double>(p.z));
        max_z = std::max(max_z, static_cast<double>(p.z));
    }
    if (finite->empty()) return false;

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

    double gmin_x = std::min(min_x, origin.x());
    double gmax_x = std::max(max_x, origin.x());
    double gmin_y = std::min(min_y, origin.y());
    double gmax_y = std::max(max_y, origin.y());

    int width  = static_cast<int>(std::ceil((gmax_x - gmin_x) / res)) + 3;
    int height = static_cast<int>(std::ceil((gmax_y - gmin_y) / res)) + 3;
    double origin_x = gmin_x - res;
    double origin_y = gmin_y - res;

    // 自适应地面高度检测 (基于有限点包围盒)
    std::vector<int> z_hist(100, 0);
    double z_range = max_z - min_z + 0.001;
    for (const auto& pt : filtered->points) {
        int bin = static_cast<int>((pt.z - min_z) / z_range * 99);
        if (bin >= 0 && bin < 100) z_hist[bin]++;
    }
    int ground_bin = 0;
    for (int i = 1; i < 100; ++i) {
        if (z_hist[i] > z_hist[ground_bin]) ground_bin = i;
    }
    double ground_z = min_z + (ground_bin + 0.5) / 100.0 * z_range;

    // 障碍物高度范围: 地面以上 [min_height, max_height]
    double obs_min_z = ground_z + cfg.min_height;
    double obs_max_z = ground_z + cfg.max_height;

    std::cout << "[GridMap] Ground Z=" << ground_z
              << " obstacle: [" << obs_min_z << ", " << obs_max_z << "]"
              << " res=" << res << " leaf=" << leaf << std::endl;

    int total_cells = width * height;

    // 统计每格的障碍物点数
    std::vector<int> obs_count(total_cells, 0);
    for (const auto& pt : filtered->points) {
        int col = static_cast<int>((pt.x - origin_x) / res);
        int row = static_cast<int>((pt.y - origin_y) / res);
        if (col < 0 || col >= width || row < 0 || row >= height) continue;
        if (pt.z >= obs_min_z && pt.z <= obs_max_z) {
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
