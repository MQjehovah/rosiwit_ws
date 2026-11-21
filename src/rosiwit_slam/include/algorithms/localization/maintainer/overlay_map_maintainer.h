// include/algorithms/localization/maintainer/overlay_map_maintainer.h
#pragma once
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_map_maintainer.h"

namespace rosiwit_slam {

/// 非破坏性叠加地图维护:
///   - 原图冻结, 不改写;
///   - fitness 低于阈值(连续 min_confirm 帧) 视为环境变化 -> 当前扫描(按定位位姿转到
///     世界系)并入 overlay; 以体素去重, 以粗分块支持半径查询;
///   - getOverlay() 返回位姿附近 overlay, 由管道叠加到局部地图作为配准目标.
class OverlayMapMaintainer : public IMapMaintainer {
public:
    std::string name() const override { return "overlay"; }
    void configure(const MaintainerConfig& cfg) override { m_cfg = cfg; }
    void setBaseMap(const std::string& map_path, const CloudType::Ptr& base) override;
    void onLocalized(const PoseStamped& pose, const CloudType::Ptr& scan_body,
                     const MatchResult& match, bool pose_confident) override;
    CloudType::Ptr getOverlay(const PoseStamped& pose) override;
    CloudType::Ptr snapshotOverlay() override;
    void popOverlayDelta(CloudType::Ptr& out) override;
    bool save(const std::string& path) override;
    bool load(const std::string& path) override;

    /// A 阶段(影子): 最近一次可信帧的扫描点里, 落在 overlay 体素上的点中
    /// w >= cfg.persistency_anchor 的占比; 只统计, 不参与决策. 无变化结构可见时返回 -1.
    double persistencyCoverage(const PoseStamped& pose,
                         int* anchor_age = nullptr,
                         size_t* session_map_pts = nullptr) const override;

    /// 开始一个新会话(任务): w 按 session 计票时, 每个会话最多 +1。
    /// 由 /start_task (或换图/节点启动) 调用; 超过 persistency_session_hours 也会自动换。
    void beginPersistencySession();
    /// 诊断/测试: 某点所在体素的权重, 不存在返回 -1
    int persistencyAt(const Eigen::Vector3d& p) const;

    /// 诊断: 最近一帧参与"新结构候选"计票的体素数 / 本帧实际沉淀的体素数
    size_t lastNovelVotes() const override { return m_novel_votes; }
    size_t lastNovelCommitted() const override { return m_novel_committed; }

    size_t overlaySize() const { return m_overlay_count; }
    uint64_t revision() const override { return m_revision; }

private:
    static int64_t voxelKey(int cx, int cy, int cz);
    void addPoint(const Eigen::Vector3d& p);
    void rayCarve(const PoseStamped& pose, const CloudType::Ptr& scan_body);
    void observeHits(const PoseStamped& pose, const CloudType::Ptr& scan_body);
    void hitVoxel(int64_t vk);
    void missVoxel(int64_t vk);
    uint32_t currentUnitKey() const;
    void rebuildCells();
    /// 懒建底图索引(降采样点云 + KD树 + 体素粗筛表)
    void ensureBaseIndex();
    /// 新结构沉淀: 与 fitness 解耦, 只看"离底图 > delta_min" + 计票单位 + 最小簇
    void collectNovel(const PoseStamped& pose, const CloudType::Ptr& body);
    void flushBuffer();
    void optimizePoses();   // GTSAM 位姿图平滑缓冲关键帧

    MaintainerConfig m_cfg;
    std::string m_map_path;
    CloudType::Ptr m_base_raw;                  // setBaseMap 原始指针(懒建用)
    CloudType::Ptr m_base;                       // 冻结原图(降采样), 用于增量/雕刻
    pcl::KdTreeFLANN<PointType> m_base_kdtree;
    bool m_has_base = false;
    std::unordered_set<int64_t> m_base_voxels;   // 底图体素(粗筛: 同体素直接跳过 KD)
    bool m_base_indexed = false;                 // 懒建标志
    double m_voxel = 0.3;
    double m_cell = 10.0;
    std::unordered_map<int64_t, long> m_voxel_seen;   // 体素 -> 最近观测帧(去重+衰减)
    // 新结构候选: 与 w 用同一套计票单位 (frame/view/session/day).
    // votes 够 persist_hits 才 addPoint; 单帧同体素只算一次.
    struct CandidateVoxel {
        uint16_t votes = 0;      // 已获得的计票单位数
        uint32_t last_key = 0;   // 上次计票的单位键 (session/day)
        uint32_t last_frame = 0; // 上次观测的帧号 (view 用; 0=从未)
    };
    std::unordered_map<int64_t, CandidateVoxel> m_hits;
    // A 阶段(影子): 体素可信度 w. 计票单位见 cfg.persistency_unit (frame/view/session/day).
    // 只在可信帧写; shadow=true 时不参与决策.
    struct PersistencyVoxel {
        uint8_t  w = 0;          // 可信度
        uint8_t  miss = 0;       // 连续 free 证据帧数 (去抖)
        uint32_t last_key = 0;   // 上次计票的单位键 (session/day)
        uint32_t last_frame = 0; // 上次 hit 的帧号 (view 单位用)
    };
    std::unordered_map<int64_t, PersistencyVoxel> m_persistency;
    uint32_t m_session_id = 1;     // 计票单位 = session
    uint32_t m_day_index = 0;      // 计票单位 = day (pose.time/86400)
    double   m_session_start_s = -1.0;
    CloudType::Ptr m_last_world;   // 最近一次可信帧的扫描点(世界系, 按体素去重, 供 persistencyCoverage)
    std::unordered_map<int64_t, CloudType::Ptr> m_cells;  // 粗分块索引
    Eigen::Vector3d m_last_add = Eigen::Vector3d::Zero();
    bool m_have_last = false;
    int m_low_count = 0;
    long m_frame = 0;
    size_t m_overlay_count = 0;
    size_t m_novel_votes = 0;       // 本帧计票的候选体素数(诊断)
    size_t m_novel_committed = 0;   // 本帧沉淀的体素数(诊断)
    uint64_t m_revision = 0;      // 内容版本号: addPoint/load 自增, 供目标缓存判失效

    // L13 位姿图: 变化关键帧缓冲 (pose + 机体系点云)
    struct KfBuf { PoseStamped pose; CloudType::Ptr body; };
    std::vector<KfBuf> m_buf;
    CloudType::Ptr m_pending_store;   // 新增 overlay 点(供 store 增量插入)
};

} // namespace rosiwit_slam
