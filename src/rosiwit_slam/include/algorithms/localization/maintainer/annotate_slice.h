// include/algorithms/localization/maintainer/annotate_slice.h
#pragma once
#include <algorithm>
#include <chrono>
#include <cstddef>
#include "slam_core/i_map_maintainer.h"

namespace rosiwit_slam {

/// 在时间预算内推进分片标注 (工作图落盘, 2026-11-26)。
///
/// 账本只有地图更新线程在写 (MapUpdateWorker), 所以需要账本的 w 标注由它在队列空的间隙做; 为了不把
/// 排队的观测任务拖久, 从 begin 起按 sub_chunk 点一段一段调用 IMapMaintainer::annotateBaseWeightsRange,
/// 每段之后看表, 累计耗时达到 budget_ms 即停 —— 下次调用从返回值处继续。
///
/// 返回本次推进到的位置 (== cloud.size() 表示已标完)。
/// 单次调用的持锁范围 = 一段 (<= sub_chunk 点); 总耗时 <= budget_ms + 一段。
inline size_t annotateWithinBudget(const IMapMaintainer& maintainer, CloudType& cloud,
                                   size_t begin, double budget_ms, size_t sub_chunk = 8192) {
    const size_t n = cloud.size();
    if (begin >= n || sub_chunk == 0) return n;
    using clk = std::chrono::steady_clock;
    const auto t0 = clk::now();
    const auto budget = std::chrono::duration<double, std::milli>(budget_ms);
    size_t next = begin;
    while (next < n) {
        const size_t end = std::min(n, next + sub_chunk);
        maintainer.annotateBaseWeightsRange(cloud, next, end);
        next = end;
        if (clk::now() - t0 >= budget) break;
    }
    return next;
}

}  // namespace rosiwit_slam
