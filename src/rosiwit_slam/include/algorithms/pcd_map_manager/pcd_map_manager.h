#pragma once
#include <string>
#include <mutex>
#include <memory>
#include "slam_core/i_map_manager.h"
#include "slam_core/i_grid_map_generator.h"
#include "algorithms/grid_map/grid_map_postprocess.h"

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
    void onBackendOptimized(const std::vector<PoseStamped>& corrected) override;
    bool getCorrectionAnchor(PoseStamped& out) override;

    // 2D 占据栅格地图 (IMapManager 接口覆写)
    bool generateGridMap(double resolution = 0.05) override;
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                     double resolution = 0.05) override;
    GridInfo getGridInfo() const override;
    bool setGridMapGenerator(const std::string& name) override;

    bool loadGridMap(const std::string& pgm_path, const std::string& yaml_path);

    // PcdMapManager 扩展 (额外 frame_id 参数)
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                     double resolution, const std::string& frame_id);

private:
    struct Submap {
        PoseStamped pose;          // 前端原始位姿(与交付后端顺序一致)
        CloudType::Ptr cloud;      // 关键帧点云(保持原始引用, 不被外部修改)
    };

    CloudType::Ptr global_map_;
    std::string map_dir_;
    bool initialized_ = false;
    std::vector<int8_t> m_grid_data;
    int m_grid_w = 0, m_grid_h = 0;
    double m_grid_res = 0.05;
    double m_grid_ox = 0, m_grid_oy = 0;

    std::vector<Submap> submaps_;        // addSubMap 顺序保存的关键帧(原始位姿)
    PoseStamped m_frontier_anchor_;      // 前沿锚点 A = G_last*P_last^{-1}, 新位姿经 A∘R 转到修正系
    std::mutex m_mutex_;                 // 保护 global_map_ / submaps_ / anchor
    double save_voxel_ = 0.3;            // 保存/重建全局地图时的体素大小(m), yaml: map_save_voxel

    // 2D 占据栅格: 生成策略 + 参数 (yaml: grid_map_generator / grid_*)
    std::unique_ptr<IGridMapGenerator> grid_gen_;
    std::string grid_gen_name_ = "height_projection";
    GridMapConfig grid_cfg_;
    GridPostConfig grid_post_cfg_;      // 2D 后处理管道 (可组合/可关闭)
    std::vector<Observation> observations_;   // 逐帧观测(原点+世界系扫描), 供射线类策略使用
};

} // namespace rosiwit_slam
