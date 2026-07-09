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

    // 降采样去噪
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

    // 自适应地面高度检测
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

    // 障碍物高度范围: 地面以上 0.15m ~ 1.8m
    double obs_min_z = ground_z + 0.15;
    double obs_max_z = ground_z + 1.8;

    std::cout << "[GridMap] Ground Z=" << ground_z
              << " obstacle: [" << obs_min_z << ", " << obs_max_z << "]" << std::endl;

    int total_cells = m_width * m_height;

    // 统计每格的障碍物点数和总点数
    std::vector<int> obs_count(total_cells, 0);   // 障碍物高度范围内的点数
    std::vector<int> any_count(total_cells, 0);   // 任意高度的点数(用于判断是否被观测到)

    for (const auto& pt : filtered->points) {
        int col = static_cast<int>((pt.x - m_origin_x) / m_config.resolution);
        int row = static_cast<int>((pt.y - m_origin_y) / m_config.resolution);
        if (col < 0 || col >= m_width || row < 0 || row >= m_height) continue;
        int idx = cellIndex(col, row);
        any_count[idx]++;
        if (pt.z >= obs_min_z && pt.z <= obs_max_z) {
            obs_count[idx]++;
        }
    }

    // 生成栅格: 默认全部 FREE, 只标记有障碍物高度的点为 OCCUPIED
    m_data.assign(total_cells, 0);  // 默认 free
    int occupied_cells = 0;
    for (int i = 0; i < total_cells; ++i) {
        if (obs_count[i] >= m_config.occupied_thresh) {
            m_data[i] = 100;   // 有障碍物高度的点 → 占据
            occupied_cells++;
        }
        // 其余全部 free (包括未扫描到的地面)
    }

    std::cout << "[GridMap] " << m_width << "x" << m_height
              << " occupied=" << occupied_cells
              << " total=" << total_cells
              << std::endl;

    return true;
}

void OccupancyGridMap::rayTraceFreeSpace(
    const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud)
{
    // 不再使用射线追踪, 改为基于观测的直接标记
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
    std::string line;
    while (std::getline(yaml_file, line)) {
        auto pos = line.find(':');
        if (pos == std::string::npos) continue;
        auto key = line.substr(0, pos);
        auto val = line.substr(pos + 1);
        while (!val.empty() && val[0] == ' ') val = val.substr(1);
        if (key == "resolution") m_config.resolution = std::stod(val);
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
