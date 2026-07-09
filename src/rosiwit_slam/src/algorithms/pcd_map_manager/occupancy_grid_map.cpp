// src/algorithms/pcd_map_manager/occupancy_grid_map.cpp
#include "algorithms/pcd_map_manager/occupancy_grid_map.h"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

namespace rosiwit_slam {

OccupancyGridMap::OccupancyGridMap(const OccupancyGridConfig& config)
    : m_config(config) {}

int OccupancyGridMap::cellIndex(int col, int row) const {
    return row * m_width + col;
}

bool OccupancyGridMap::buildFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud,
    const Eigen::Vector3d& origin)
{
    if (!cloud || cloud->empty()) return false;

    // 先用 VoxelGrid 降采样, 减少噪点
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::VoxelGrid<pcl::PointXYZINormal> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*filtered);

    pcl::PointXYZINormal min_pt, max_pt;
    pcl::getMinMax3D(*filtered, min_pt, max_pt);

    double min_x = std::min(static_cast<double>(min_pt.x), origin.x());
    double max_x = std::max(static_cast<double>(max_pt.x), origin.x());
    double min_y = std::min(static_cast<double>(min_pt.y), origin.y());
    double max_y = std::max(static_cast<double>(max_pt.y), origin.y());

    m_width  = static_cast<int>(std::ceil((max_x - min_x) / m_config.resolution)) + 3;
    m_height = static_cast<int>(std::ceil((max_y - min_y) / m_config.resolution)) + 3;
    m_origin_x = min_x - m_config.resolution;
    m_origin_y = min_y - m_config.resolution;

    m_data.assign(m_width * m_height, -1);

    // Step 1: 高度直方图 — 自适应检测地面高度
    // 统计 Z 轴分布, 找到地面 (z 值最密集的层)
    std::vector<int> z_hist(100, 0);
    double z_min = static_cast<double>(min_pt.z);
    double z_max = static_cast<double>(max_pt.z);
    double z_range = z_max - z_min + 0.001;
    for (const auto& pt : filtered->points) {
        int bin = static_cast<int>((pt.z - z_min) / z_range * 99);
        if (bin >= 0 && bin < 100) z_hist[bin]++;
    }
    int ground_bin = 0;
    for (int i = 1; i < 100; ++i) {
        if (z_hist[i] > z_hist[ground_bin]) ground_bin = i;
    }
    double ground_z = z_min + (ground_bin + 0.5) / 100.0 * z_range;
    double obs_min_z = ground_z + m_config.min_height;
    double obs_max_z = ground_z + m_config.max_height;

    std::cout << "[GridMap] Ground Z=" << ground_z
              << " obstacle range: [" << obs_min_z << ", " << obs_max_z << "]" << std::endl;

    // Step 2: 统计每个栅格的点数 (仅障碍物高度范围内的点)
    std::vector<int> point_count(m_width * m_height, 0);
    for (const auto& pt : filtered->points) {
        if (pt.z < obs_min_z || pt.z > obs_max_z) continue;
        int col = static_cast<int>((pt.x - m_origin_x) / m_config.resolution);
        int row = static_cast<int>((pt.y - m_origin_y) / m_config.resolution);
        if (col < 0 || col >= m_width || row < 0 || row >= m_height) continue;
        point_count[cellIndex(col, row)]++;
    }

    // Step 3: 标记占据 — 需要足够多的点
    for (int r = 0; r < m_height; ++r) {
        for (int c = 0; c < m_width; ++c) {
            int idx = cellIndex(c, r);
            if (point_count[idx] >= m_config.occupied_thresh) {
                m_data[idx] = 100;
            }
        }
    }

    // Step 4: 形态学开运算 (腐蚀+膨胀) — 去除孤立噪点
    morphologicalOpen();

    // Step 5: 射线追踪标记自由空间
    rayTraceFreeSpace(filtered);

    return true;
}

void OccupancyGridMap::morphologicalOpen() {
    // 腐蚀: 占据点如果周围 8 邻居中占据数 < 3, 则移除 (去孤立噪点)
    std::vector<int8_t> eroded = m_data;
    for (int r = 1; r < m_height - 1; ++r) {
        for (int c = 1; c < m_width - 1; ++c) {
            if (m_data[cellIndex(c, r)] != 100) continue;
            int count = 0;
            for (int dr = -1; dr <= 1; ++dr)
                for (int dc = -1; dc <= 1; ++dc)
                    if (m_data[cellIndex(c + dc, r + dr)] == 100) count++;
            if (count < 4) eroded[cellIndex(c, r)] = -1;  // 孤立点移除
        }
    }
    // 膨胀: 恢复被腐蚀掉的墙体边缘
    std::vector<int8_t> dilated = eroded;
    for (int r = 1; r < m_height - 1; ++r) {
        for (int c = 1; c < m_width - 1; ++c) {
            if (eroded[cellIndex(c, r)] == 100) continue;
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (eroded[cellIndex(c + dc, r + dr)] == 100) {
                        dilated[cellIndex(c, r)] = 100;
                        break;
                    }
                }
            }
        }
    }
    m_data = dilated;
}

void OccupancyGridMap::rayTraceFreeSpace(
    const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud)
{
    double sx = 0, sy = 0;
    for (const auto& p : cloud->points) { sx += p.x; sy += p.y; }
    sx /= cloud->size(); sy /= cloud->size();

    int sensor_col = static_cast<int>((sx - m_origin_x) / m_config.resolution);
    int sensor_row = static_cast<int>((sy - m_origin_y) / m_config.resolution);

    int max_cells = static_cast<int>(m_config.max_range / m_config.resolution);

    for (int angle_idx = 0; angle_idx < m_config.free_thresh_rays; ++angle_idx) {
        double theta = 2.0 * M_PI * angle_idx / m_config.free_thresh_rays;
        double dx = std::cos(theta);
        double dy = std::sin(theta);

        for (int step = 0; step < max_cells; ++step) {
            int c = sensor_col + static_cast<int>(dx * step);
            int r = sensor_row + static_cast<int>(dy * step);
            if (c < 0 || c >= m_width || r < 0 || r >= m_height) break;

            int idx = cellIndex(c, r);
            if (m_data[idx] == 100) break;
            if (m_data[idx] == -1) m_data[idx] = 0;
        }
    }
}

bool OccupancyGridMap::saveToPGM(
    const std::string& pgm_path, const std::string& yaml_path,
    const std::string& frame_id)
{
    if (m_data.empty()) return false;
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    pgm << "P5\n" << m_width << " " << m_height << "\n100\n";
    for (auto v : m_data) {
        uint8_t pixel = 205;
        if (v == 0) pixel = 254;
        else if (v >= 100) pixel = 0;
        else pixel = static_cast<uint8_t>(205 - v * 50 / 100);
        pgm.write(reinterpret_cast<char*>(&pixel), 1);
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) return false;
    yaml << "image: " << pgm_path.substr(pgm_path.find_last_of("/\\") + 1) << "\n"
         << "resolution: " << m_config.resolution << "\n"
         << "origin: [" << m_origin_x << ", " << m_origin_y << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n";
    yaml.close();
    return true;
}

bool OccupancyGridMap::loadFromPGM(
    const std::string& pgm_path, const std::string& yaml_path)
{
    std::ifstream yaml_file(yaml_path);
    if (!yaml_file) return false;
    std::string line, image_name;
    while (std::getline(yaml_file, line)) {
        auto pos = line.find(':');
        if (pos == std::string::npos) continue;
        auto key = line.substr(0, pos);
        auto val = line.substr(pos + 1);
        while (!val.empty() && val[0] == ' ') val = val.substr(1);
        if (key == "image") image_name = val;
        else if (key == "resolution") m_config.resolution = std::stod(val);
    }
    std::ifstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    std::string format; int max_val;
    pgm >> format >> m_width >> m_height >> max_val;
    pgm.get();
    if (format != "P5") return false;
    m_data.resize(m_width * m_height);
    for (int i = 0; i < m_width * m_height; ++i) {
        uint8_t pixel;
        pgm.read(reinterpret_cast<char*>(&pixel), 1);
        if (pixel == 254) m_data[i] = 0;
        else if (pixel == 0) m_data[i] = 100;
        else m_data[i] = -1;
    }
    return true;
}

} // namespace rosiwit_slam
