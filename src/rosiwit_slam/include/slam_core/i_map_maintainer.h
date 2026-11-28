// include/slam_core/i_map_maintainer.h
#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include "slam_core/localization_stage_types.h"

#include <functional>

namespace rosiwit_slam {

/// 地图维护阶段 (第6阶段): 长期运行时吸收环境变化.
/// 采用"非破坏性叠加": 原始地图保持冻结, 检测到的变化以叠加层(overlay)形式累积;
/// 配准目标 = 原局部地图 + overlay, 从而原图文件与索引永不被改写.
class IMapMaintainer {
public:
    virtual ~IMapMaintainer() = default;

    virtual std::string name() const = 0;

    virtual bool init(const std::string& /*config_path*/) { return true; }
    virtual void configure(const MaintainerConfig& /*cfg*/) {}

    /// 载入原始地图(冻结) 与可选关键帧
    virtual void setBaseMap(const std::string& /*map_path*/, const CloudType::Ptr& /*base*/) {}
    virtual void setKeyframes(const std::vector<LocalKeyframe>& /*keyframes*/) {}

    /// 每帧定位完成后调用.
    /// @param pose_confident 本帧位姿是否可信(由配准采纳判据决定); false(过桥外推)时
    ///        不写入/不雕刻, 避免把变化写到漂移位姿上而污染地图.
    virtual void onLocalized(const PoseStamped& /*pose*/, const CloudType::Ptr& /*scan_body*/,
                             const MatchResult& /*match*/, bool /*pose_confident*/) {}

    /// 返回位姿附近(半径内)的 overlay 点云(世界系), 用于叠加到配准目标
    virtual CloudType::Ptr getOverlay(const PoseStamped& /*pose*/) { return nullptr; }

    /// 返回全部 overlay 点云(世界系), 供刷新全局重定位器 (lifelong)
    virtual CloudType::Ptr snapshotOverlay() { return nullptr; }

    /// 叠加层内容版本号: 任何点写入/删除/载入时自增。
    /// 配准目标缓存用它判断失效 —— 静态环境下 overlay 不增长, 目标(降采样+KD树)
    /// 可跨帧复用, 不必每帧重建(实测每帧重建把位姿输出拖到 10Hz, 雷达是 15.8Hz)。
    virtual uint64_t revision() const { return 0; }

    /// 诊断: 最近一帧参与"新结构候选"计票的体素数 / 本帧实际沉淀的体素数
    virtual size_t lastNovelVotes() const { return 0; }
    virtual size_t lastNovelCommitted() const { return 0; }

    /// 取出并清空"自上次调用以来新增"的 overlay 点 (供存储底座增量插入)
    virtual void popOverlayDelta(CloudType::Ptr& out) { out.reset(); }

    /// 持久化/载入 overlay (原图不变)
    virtual bool save(const std::string& /*path*/) { return false; }
    virtual bool load(const std::string& /*path*/) { return false; }

    // ---- 统一工作图 (Phase A): 底图也进同一套 w 账本 ----
    /// 载入的底图云是否是"工作图"(map.overlay.pcd): 是则 intensity 就是 w。
    /// 必须在 setBaseMap 之前调用 (setBaseMap 用它解释 intensity)。
    virtual void setBaseEmbeddedWeights(bool /*on*/) {}
    /// 分片标注时给"要丢弃的点"打的标记 (合法 w >= 1, 所以 <0 不会与合法值混淆)。
    static constexpr float kAnnotateDrop = -1.0f;
    /// 把 w 写进 intensity 并丢弃 w==0 的点 (整云一次完成: 分片标注 + 压缩)。
    /// 需要账本 —— 只在账本的写者线程 (跟踪线程 / 换图线程) 上调用。
    virtual void annotateBaseWeights(CloudType& cloud) const {
        annotateBaseWeightsRange(cloud, 0, cloud.size());
        compactAnnotated(cloud);
    }
    /// 分片标注: 对 points[begin, end) 逐点写 intensity = min(255, w); w<=0 或非有限点打
    /// kAnnotateDrop 标记。**不压缩、不改点数** —— 调用方可以在自己的时间预算内分多次
    /// 调用 (每片的持锁范围就是这一片), 最后再 compactAnnotated()。
    /// 线程安全: 段内持锁。
    virtual void annotateBaseWeightsRange(CloudType& /*cloud*/, size_t /*begin*/,
                                          size_t /*end*/) const {}
    /// 压缩: 去掉带 kAnnotateDrop 标记的点 (原地, 写指针恒不超过读指针)。
    /// 不需要账本, 不持锁 —— 可以放在后台线程 / 任意时刻做。
    static void compactAnnotated(CloudType& cloud) {
        auto& pts = cloud.points;
        size_t keep = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            if (pts[i].intensity < 0.0f) continue;
            if (keep != i) pts[keep] = pts[i];
            ++keep;
        }
        pts.resize(keep);
        cloud.width = static_cast<uint32_t>(keep);
        cloud.height = 1;
    }
    /// 剔除 w 已降到 0 的底图点 (匹配目标用); 无死亡体素时原样返回。
    virtual CloudType::Ptr filterDeadBase(const CloudType::Ptr& cloud) const { return cloud; }
    /// 稳定的工作图底图快照 (深拷贝, 账本锁内完成)。addPoint 会把新提交的结构
    /// 追加到工作图云尾部 (与管道 m_map 同一指针), 落盘/调试导出直接读它会与
    /// 地图更新线程的 push_back 并发 (2026-11-28 实车 SIGSEGV: 行驶中 map save 的
    /// voxelDownsample 撞上 append)。需要在后台线程使用底图的地方都走这个快照。
    virtual CloudType::Ptr snapshotBaseCloud() const { return CloudType::Ptr(); }
    /// 回收已删除体素的点 (落盘时刻调用一次, 批量压缩)。
    virtual void reclaim() {}
    /// w-map: 新结构升格回调 (管道用它同步分块缓存; 默认无操作)
    virtual void setCommitHook(std::function<void(const Eigen::Vector3d&, float)>) {}
    /// w-map 段级融合: 走出子图覆盖的关键帧一次性「命中+雕刻+新结构」记账
    virtual void mergeChunk(const PoseStamped& /*pose*/, const CloudType::Ptr& /*world*/) {}

    /// A 阶段(影子): 最近一次"位姿可信"帧的扫描点里, 落在 overlay 体素上的点中
    /// w >= maintainer.persistency_anchor 的占比 (0~1)。只统计, 不参与决策。
    /// @param anchor_age 出参: 命中锚点体素中"最久未再观测"的帧龄; 无锚点则 0
    /// @param session_map_pts 出参: 参与统计的点数 (落在 overlay 体素上的扫描点)
    /// @return 占比; 无变化结构可见时返回 -1 (调用方记 n/a)
    virtual double persistencyCoverage(const PoseStamped& /*pose*/,
                                 int* /*anchor_age*/ = nullptr,
                                 size_t* /*session_map_pts*/ = nullptr) const { return -1.0; }
    /// 同上, 但是**账本自己在最近一次写入后算好的缓存值**: 不拿账本的锁, 任何线程都可以调
    /// (定位线程的 1Hz 状态日志用它 —— 账本写入在地图更新线程上, 持锁可达几百 ms)。没有记录时返回 -1。
    virtual double cachedPersistencyCoverage(int* /*anchor_age*/ = nullptr,
                                             size_t* /*session_map_pts*/ = nullptr) const { return -1.0; }
};

} // namespace rosiwit_slam
