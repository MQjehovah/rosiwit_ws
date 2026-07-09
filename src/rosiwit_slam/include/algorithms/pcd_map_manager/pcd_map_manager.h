#pragma once
#include <string>
#include "slam_core/i_map_manager.h"
#include "algorithms/pcd_map_manager/occupancy_grid_map.h"

namespace rosiwit_slam {

class PcdMapManager : public IMapManager {
public:
    PcdMapManager();
    ~PcdMapManager() override = default;

    bool init(const std::string& config_path) override;
    bool saveMap(const std::string& name) override;
    bool loadMap(const std::string& name) override;
    bool getGlobalMap(PointVec& out) override;
    bool addSubMap(const KeyFrame& kf) override;

    // 2D 占据栅格地图
    bool generateGridMap(double resolution = 0.05);
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                     double resolution = 0.05, const std::string& frame_id = "map");
    bool loadGridMap(const std::string& pgm_path, const std::string& yaml_path);
    const std::vector<int8_t>& getGridData() const { return m_grid_data; }
    int getGridWidth() const { return m_grid_w; }
    int getGridHeight() const { return m_grid_h; }
    double getGridRes() const { return m_grid_res; }
    double getGridOriginX() const { return m_grid_ox; }
    double getGridOriginY() const { return m_grid_oy; }

private:
    CloudType::Ptr global_map_;
    std::string map_dir_;
    bool initialized_ = false;
    std::vector<int8_t> m_grid_data;
    int m_grid_w = 0, m_grid_h = 0;
    double m_grid_res = 0.05;
    double m_grid_ox = 0, m_grid_oy = 0;
};

} // namespace rosiwit_slam
