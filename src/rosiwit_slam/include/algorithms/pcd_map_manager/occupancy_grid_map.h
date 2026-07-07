// include/algorithms/pcd_map_manager/occupancy_grid_map.h
#pragma once
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Eigen>

namespace rosiwit_slam {

struct OccupancyGridConfig {
    double resolution = 0.05;     // 栅格大小 (米/像素)
    double height_thresh = 0.3;   // 高于地面的高度阈值(m)
    double ground_height = -0.5;  // 地面高度(m)
    int occupied_thresh = 3;      // 每个栅格内至少 N 个点才算占据
    int free_thresh_rays = 50;    // 射线采样数
    double max_range = 30.0;      // 最大射线距离
};

// 2D 占据栅格地图生成器
class OccupancyGridMap {
public:
    OccupancyGridMap(const OccupancyGridConfig& config = OccupancyGridConfig());

    // 从点云构建栅格地图
    bool buildFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud,
                             const Eigen::Vector3d& origin = Eigen::Vector3d::Zero());

    // 保存为 PGM + YAML (ROS nav_msgs/OccupancyGrid 标准格式)
    bool saveToPGM(const std::string& pgm_path, const std::string& yaml_path,
                   const std::string& frame_id = "map");

    // 加载已保存的 PGM + YAML
    bool loadFromPGM(const std::string& pgm_path, const std::string& yaml_path);

    // 访问结果
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    double getResolution() const { return m_config.resolution; }
    const std::vector<int8_t>& getData() const { return m_data; }

private:
    void rayTraceFreeSpace(const pcl::PointCloud<pcl::PointXYZINormal>::ConstPtr& cloud);
    int cellIndex(int col, int row) const;

    OccupancyGridConfig m_config;
    int m_width = 0;
    int m_height = 0;
    double m_origin_x = 0;
    double m_origin_y = 0;
    std::vector<int8_t> m_data;  // 0=free, 100=occupied, -1=unknown
};

} // namespace rosiwit_slam
