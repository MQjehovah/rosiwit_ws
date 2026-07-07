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

    pcl::PointXYZINormal min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    double min_x = std::min(static_cast<double>(min_pt.x), origin.x());
    double max_x = std::max(static_cast<double>(max_pt.x), origin.x());
    double min_y = std::min(static_cast<double>(min_pt.y), origin.y());
    double max_y = std::max(static_cast<double>(max_pt.y), origin.y());

    m_width  = static_cast<int>(std::ceil((max_x - min_x) / m_config.resolution)) + 3;
    m_height = static_cast<int>(std::ceil((max_y - min_y) / m_config.resolution)) + 3;
    m_origin_x = min_x - m_config.resolution;
    m_origin_y = min_y - m_config.resolution;

    m_data.assign(m_width * m_height, -1);  // 初始化为 unknown

    double ground_low = m_config.ground_height - 0.2;
    double ground_high = m_config.ground_height + m_config.height_thresh;

    // Step 1: 填入占据点
    std::vector<int> point_count(m_width * m_height, 0);
    for (const auto& pt : cloud->points) {
        if (pt.z < ground_low || pt.z > ground_high) continue;
        // 过滤地面点 (只保留地面以上的物体)
        int col = static_cast<int>((pt.x - m_origin_x) / m_config.resolution);
        int row = static_cast<int>((pt.y - m_origin_y) / m_config.resolution);
        if (col < 0 || col >= m_width || row < 0 || row >= m_height) continue;
        point_count[cellIndex(col, row)]++;
    }

    for (int r = 0; r < m_height; ++r) {
        for (int c = 0; c < m_width; ++c) {
            int idx = cellIndex(c, r);
            if (point_count[idx] >= m_config.occupied_thresh) {
                m_data[idx] = 100;  // occupied
            }
        }
    }

    // Step 2: 射线追踪自由空间
    rayTraceFreeSpace(cloud);

    return true;
}

void OccupancyGridMap::rayTraceFreeSpace(
    const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud)
{
    // 取点云中心作为传感器位置
    double sx = 0, sy = 0;
    for (const auto& p : cloud->points) {
        sx += p.x; sy += p.y;
    }
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
            if (m_data[idx] == 100) break;  // 遇到障碍物停止
            if (m_data[idx] == -1) m_data[idx] = 0;  // 标记为 free
        }
    }
}

bool OccupancyGridMap::saveToPGM(
    const std::string& pgm_path, const std::string& yaml_path,
    const std::string& frame_id)
{
    if (m_data.empty()) return false;

    // 写入 PGM (P5 binary format)
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) { std::cerr << "[GridMap] Cannot write: " << pgm_path << std::endl; return false; }
    pgm << "P5\n" << m_width << " " << m_height << "\n100\n";
    for (auto v : m_data) {
        uint8_t pixel = 205;  // unknown=gray
        if (v == 0) pixel = 254;        // free=white
        else if (v >= 100) pixel = 0;   // occupied=black
        else pixel = static_cast<uint8_t>(205 - v * 50 / 100);
        pgm.write(reinterpret_cast<char*>(&pixel), 1);
    }
    pgm.close();

    // 写入 YAML metadata
    std::ofstream yaml(yaml_path);
    if (!yaml) { std::cerr << "[GridMap] Cannot write: " << yaml_path << std::endl; return false; }
    yaml << "image: " << pgm_path.substr(pgm_path.find_last_of("/\\") + 1) << "\n"
         << "resolution: " << m_config.resolution << "\n"
         << "origin: [" << m_origin_x << ", " << m_origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.25\n"
         << "mode: trinary\n";
    yaml.close();
    return true;
}

bool OccupancyGridMap::loadFromPGM(
    const std::string& pgm_path, const std::string& yaml_path)
{
    // 简单实现: 先读取 YAML
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
        else if (key == "origin") {
            // parse "[x, y, z]"
            // simplified: just skip
        }
    }

    // 读取 PGM
    std::ifstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    std::string format;
    int max_val;
    pgm >> format >> m_width >> m_height >> max_val;
    pgm.get(); // newline
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
