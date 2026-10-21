#pragma once
#include <string>
#include <mutex>
#include <atomic>
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
    /// 多会话建图: 把另一段 PCD 追加/融合到当前地图（同一 map 坐标系）。
    bool appendMap(const std::string& name);
    /// 多会话对齐: initial 为 session 初始位姿 (time<0=identity)，align=true 时用 ICP 精配准。
    bool appendMap(const std::string& name, const PoseStamped& initial,
                   bool align = true, double max_fitness = 1.0);
    bool getGlobalMap(PointVec& out) override;
    bool addSubMap(const KeyFrame& kf) override;
    void onBackendOptimized(const std::vector<PoseStamped>& corrected) override;
    bool getCorrectionAnchor(PoseStamped& out) override;

    // 2D 占据栅格地图 (IMapManager 接口覆写)
    bool generateGridMap(double resolution = 0.05) override;
    bool saveGridMap(const std::string& pgm_path, const std::string& yaml_path,
                     double resolution = 0.05) override;
    GridInfo getGridInfo() const override;
    uint64_t getRevision() const override { return m_revision_.load(); }
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
    uint64_t m_grid_revision = 0;
    double m_grid_ox = 0, m_grid_oy = 0;

    std::vector<Submap> submaps_;        // addSubMap 顺序保存的关键帧(原始位姿)
    PoseStamped m_frontier_anchor_;      // 前沿锚点 A = G_last*P_last^{-1}, 新位姿经 A∘R 转到修正系
    mutable std::mutex m_mutex_;         // 保护 global_map_ / submaps_ / anchor; const 查询也加锁
    std::atomic<uint64_t> m_revision_{0}; // 地图内容版本号
    double save_voxel_ = 0.3;            // 保存/重建全局地图时的体素大小(m), yaml: map_save_voxel
    // 后端修正量的合理性上限: 相邻两次优化之间的锚点跳变量超过它就拒绝本次修正并告警。
    // 实机教训: 后端滞后导致 corrected/submaps 下标错位时, 锚点会跳 5~36m, 整张图瞬间错位重叠。
    // yaml: mapping.map_manager.max_correction_trans_m / max_correction_rot_deg
    double m_max_correction_trans_m = 1.5;              // m
    double m_max_correction_rot_rad = 15.0 * M_PI / 180.0;  // rad

    // 2D 占据栅格: 生成策略 + 参数 (yaml: grid_map_generator / grid_*)
    std::unique_ptr<IGridMapGenerator> grid_gen_;
    std::string grid_gen_name_ = "height_projection";
    GridMapConfig grid_cfg_;
    GridPostConfig grid_post_cfg_;      // 最终 PGM 的 2D 后处理管道
    GridPostConfig grid_realtime_post_cfg_; // 实时 /map 的后处理; 不继承 0.05 的骨架/大去噪
    std::vector<Observation> observations_;   // 逐帧观测(原点+世界系扫描), 供射线类策略使用
};

} // namespace rosiwit_slam
