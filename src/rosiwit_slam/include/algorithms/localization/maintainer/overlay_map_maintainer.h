// include/algorithms/localization/maintainer/overlay_map_maintainer.h
#pragma once
#include <unordered_map>
#include <functional>
#include <unordered_set>
#include <cstdint>
#include <pcl/kdtree/kdtree_flann.h>
#include "slam_core/i_map_maintainer.h"
#include <mutex>

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
                     const MatchResult& match, bool pose_confident) override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        onLocalizedLocked(pose, scan_body, match, pose_confident);
    }
    void onLocalizedLocked(const PoseStamped& pose, const CloudType::Ptr& scan_body,
                     const MatchResult& match, bool pose_confident);
    CloudType::Ptr getOverlay(const PoseStamped& pose) override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        return getOverlayLocked(pose);
    }
    // ★线程安全 (2026-11-26 实机段错误根因): 本对象被三个线程访问 ——
    //   定位线程 onLocalized/mergeChunk (改 m_cells/m_persistency/m_dead_base)、
    //   IO 发布线程 snapshotOverlay/getOverlay、存盘线程 reclaim。
    //   之前零锁, 并发读写 unordered_map -> 进入定位瞬间 (overlay_map 首发布)
    //   与账本更新相撞 -> SIGSEGV (exit -11)。所有公开入口统一加递归锁。
    CloudType::Ptr snapshotOverlay() override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        return snapshotOverlayLocked();
    }
    void popOverlayDelta(CloudType::Ptr& out) override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        popOverlayDeltaLocked(out);
    }
    bool save(const std::string& path) override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        return saveLocked(path);
    }
    bool load(const std::string& path) override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        return loadLocked(path);
    }

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
    /// 统一工作图 Phase A: 底图体素的 w 账本查询 (目标里剔除已衰减的底图点)
    const std::unordered_set<int64_t>& deadBaseVoxels() const { return m_dead_base; }
    double voxelSize() const { return m_voxel; }
    static int64_t voxelKeyOf(int cx, int cy, int cz) { return voxelKey(cx, cy, cz); }
    // 统一工作图: 工作图文件里 intensity = w
    void setBaseEmbeddedWeights(bool on) override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        m_base_embedded = on;
    }
    void annotateBaseWeightsRange(CloudType& cloud, size_t begin, size_t end) const override;
    CloudType::Ptr filterDeadBase(const CloudType::Ptr& cloud) const override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        return filterDeadBaseLocked(cloud);
    }
    /// 回收已删除体素的点 (w 降到 0 / 硬删的那些): 由落盘时刻调用 (1h 一次)。
    /// 逐条删除要扫整格点云, 太贵; 攒到落盘时全量扫一遍, 成本可忽略。
    void reclaim() override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        compactCells();
    }
    /// w-map 段级融合: 把一块 (位姿+世界系点云) 走完整账目 ——
    /// 命中票 / 自由票(雕刻) / 新结构提交, 一次完成。用于子图关键帧走出覆盖时。
    void mergeChunk(const PoseStamped& pose, const CloudType::Ptr& world) {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        mergeChunkLocked(pose, world);
    }
    /// 提交钩子: 新结构升格时回调 (w-map 模式下管道用它把点插进工作图分块缓存)
    void setCommitHook(std::function<void(const Eigen::Vector3d&, float)> hook) {
        m_commit_sink = std::move(hook);
    }
    uint64_t revision() const override {
        std::lock_guard<std::recursive_mutex> lk(m_mu);
        return m_revision;
    }
    // ---- 加锁实现 (原函数体改名, 由上面的公开入口统一加锁) ----
    CloudType::Ptr snapshotOverlayLocked();
    void popOverlayDeltaLocked(CloudType::Ptr& out);
    bool saveLocked(const std::string& path);
    bool loadLocked(const std::string& path);
    CloudType::Ptr filterDeadBaseLocked(const CloudType::Ptr& cloud) const;
    void mergeChunkLocked(const PoseStamped& pose, const CloudType::Ptr& world);
    CloudType::Ptr getOverlayLocked(const PoseStamped& pose);

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
        uint32_t last_frame = 0; // 上次 hit 的帧号 (frame 单位用)
        uint32_t last_view = 0;       // 上次 hit 的视角编号 (view 单位)
        uint32_t last_miss_view = 0;  // 上次 free 证据的视角编号 (view 单位)
        uint32_t last_miss_frame = 0; // 上次 free 证据的帧号 (miss 与 hit 对称: 每帧一票)
        uint32_t last_miss_key = 0;   // 上次 free 证据的计票单位键 (session/day)
        bool     from_base = false;  // 该体素来自底图 (点不在 m_cells, 死亡时进 m_dead_base)
    };
    std::unordered_map<int64_t, PersistencyVoxel> m_persistency;
    /// 底图云是否已是"工作图"(intensity = w, 已烘焙之前的 overlay)
    bool m_base_embedded = false;
    /// 底图体素 w 衰减到 0 的集合: 匹配目标里要剔除这些体素里的底图点
    /// (墙挪了/拆了 = 射线穿过 -> w 递减为 0)。落盘时也剔除, 跨会话持久。
    std::unordered_set<int64_t> m_dead_base;
    std::atomic<bool> m_carve_enabled{true};   // 静止不雕刻
    // ★视角编号 (2026-11-26 根因修复): view 计票单位必须显式化 —— 位姿移动超过
    // 阈值才产生新视角。静止机器人只有一个视角, hit/miss 都只计一票:
    // 否则 miss 侧的"每帧"语义会把静止时未被当前扫描返回的真实几何逐帧减死
    // (射线抽样间歇 = 假的新视角), 工作图被自更新侵蚀 -> 位姿缓漂。
    mutable std::recursive_mutex m_mu;   // 全对象递归锁 (见公开入口说明)
    uint32_t m_view_id = 1;
    PoseStamped m_view_pose;
    bool m_view_valid = false;
    std::function<void(const Eigen::Vector3d&, float)> m_commit_sink;
    uint32_t m_session_id = 1;     // 计票单位 = session
    uint32_t m_day_index = 0;      // 计票单位 = day (pose.time/86400)
    double   m_session_start_s = -1.0;
    CloudType::Ptr m_last_world;   // 最近一次可信帧的扫描点(世界系, 按体素去重, 供 persistencyCoverage)
    std::unordered_map<int64_t, CloudType::Ptr> m_cells;  // 粗分块索引
    /// 已删除体素(voxel key)的待回收清单: w 降到 0 的体素, 其点仍在 m_cells 里。
    /// 立即回收要扫整格点云(千点级) x 每帧数千次删除 -> 太贵; 攒到阈值批量压缩。
    std::unordered_set<int64_t> m_pending_remove;
    void compactCells();
    Eigen::Vector3d m_last_add = Eigen::Vector3d::Zero();
    bool m_have_last = false;
    int m_low_count = 0;
    long m_frame = 0;
    size_t m_overlay_count = 0;
    size_t m_novel_votes = 0;       // 本帧计票的候选体素数(诊断)
    size_t m_novel_committed = 0;   // 本帧沉淀的体素数(诊断)
    // [诊断] collectNovel 各分支计数, 定位"新结构不进 overlay"
    size_t m_dbg_scan = 0, m_dbg_seen = 0, m_dbg_base = 0, m_dbg_kd = 0, m_dbg_cluster = 0;
    uint64_t m_revision = 0;      // 内容版本号: addPoint/load 自增, 供目标缓存判失效

    // L13 位姿图: 变化关键帧缓冲 (pose + 机体系点云)
    struct KfBuf { PoseStamped pose; CloudType::Ptr body; };
    std::vector<KfBuf> m_buf;
    CloudType::Ptr m_pending_store;   // 新增 overlay 点(供 store 增量插入)
};

} // namespace rosiwit_slam
