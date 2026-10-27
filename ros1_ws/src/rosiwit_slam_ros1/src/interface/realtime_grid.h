// ============================================================================
// 建图模式的实时 2D 占据栅格 (设计文档 §5.11 A)
//
// 为什么不用改核心: 核心的栅格生成器与后处理都在**公开头文件**里
// (slam_core/i_grid_map_generator.h, algorithms/grid_map/grid_map_factory.h,
//  algorithms/grid_map/grid_map_postprocess.h), 且核心被打成单个 rosiwit_slam 库、
// include 目录是 PUBLIC。核心私有的只有 PcdMapManager::generateGridMap() 的封装,
// 这里做的就是**逐行复刻**那个封装 (pcd_map_manager.cpp:264-293)。
//
// 注意: 生成器与后处理都是 O(点数), **绝不能跑在 ROS 回调线程上**
// (会拖累 100Hz 的定位位姿与 map→odom TF), 所以下面用独立 worker 线程。
// ============================================================================
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rosiwit_common/periodic.hpp"
#include "rosiwit_common/snapshot.hpp"

#include "slam_core/slam_types.h"                     // GridInfo / PointVec

namespace rosiwit_slam_ros1 {

/// worker 线程封装: 按 hz 节流, 调 source() 取点云并重算, 结果加锁给发布方。
/// start() 后**立即**先算一张 (避免 1Hz 时首张要等满一秒), 之后按周期重算。
class RealtimeGrid {
public:
    RealtimeGrid();
    ~RealtimeGrid();
    RealtimeGrid(const RealtimeGrid&) = delete;
    RealtimeGrid& operator=(const RealtimeGrid&) = delete;

    /// 改为直接取**已经生成好的栅格** (走核心 generateGridMap): 与存 pgm 完全同一条路径,
    /// 因此实时 /map 不会有地面噪点。设置后 setSource 的点云路径不再使用。
    using GridSource = std::function<bool(rosiwit_slam::GridInfo&)>;
    void setGridSource(GridSource s);

    void start(double hz);  // hz <= 0 时按 1.0 处理; 重复调用无副作用
    void stop();            // 幂等; 析构时自动调用

    /// 有结果就拷到 out 并返回 true; 从没算出过返回 false
    bool latest(rosiwit_slam::GridInfo& out) const;
    /// 零拷贝读取: 拿到一份不可变快照 (发布侧不再整份复制给每个读者)
    std::shared_ptr<const rosiwit_slam::GridInfo> latestPtr() const { return m_slot_.load(); }

    /// 丢掉缓存的最新栅格 (核心重建/重新进建图时调用) —— 否则会一直发上一轮那张。
    void clear();

private:
    /// 一轮: 快照配置 -> 重算 -> 发布不可变快照 (由 PeriodicThread 按周期调用)
    void runOnce();

    mutable std::mutex m_mtx_;          // 保护配置与 source (只在 set*/快照时短暂持有)
    GridSource m_grid_source_;

    std::unique_ptr<rosiwit_common::PeriodicThread> m_thread_;
    rosiwit_common::SnapshotSlot<rosiwit_slam::GridInfo> m_slot_;   // 发布侧无锁
};

} // namespace rosiwit_slam_ros1
