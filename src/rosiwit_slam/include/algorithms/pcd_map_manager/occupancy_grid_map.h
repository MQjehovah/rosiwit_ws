// include/algorithms/pcd_map_manager/occupancy_grid_map.h
#pragma once
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Eigen>

namespace rosiwit_slam {

struct OccupancyGridConfig {
    double resolution = 0.05;
    double min_height = 0.1;     // 障碍物最低高度(地面以上 10cm), 兼顾低矮障碍
    double max_height = 2.5;     // 障碍物最高高度
    int    occupied_thresh = 2;  // 每个栅格至少 2 个点就算占据
    int    free_thresh_rays = 360;
    double max_range = 30.0;
};

class OccupancyGridMap {
public:
    OccupancyGridMap(const OccupancyGridConfig& config = OccupancyGridConfig());

    bool buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud,
                             const Eigen::Vector3d& origin = Eigen::Vector3d::Zero());

    bool saveToPGM(const std::string& pgm_path, const std::string& yaml_path,
                   const std::string& frame_id = "map");

    bool loadFromPGM(const std::string& pgm_path, const std::string& yaml_path);

    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    double getResolution() const { return m_config.resolution; }
    double getOriginX() const { return m_origin_x; }
    double getOriginY() const { return m_origin_y; }
    const std::vector<int8_t>& getData() const { return m_data; }

private:
    void rayTraceFreeSpace(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud);
    int cellIndex(int col, int row) const;

    OccupancyGridConfig m_config;
    int m_width = 0;
    int m_height = 0;
    double m_origin_x = 0;
    double m_origin_y = 0;
    std::vector<int8_t> m_data;
};

} // namespace rosiwit_slam
