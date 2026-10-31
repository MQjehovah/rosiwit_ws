// src/algorithms/fast_livo2/preprocess.h
#pragma once

#include "algorithms/fast_livo2/fast_livo2_config.h"
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// FAST-LIVO2 预处理（ROS 无关）。
///
/// 核心的 LidarFrame 已经由接口层完成字段解析与逐点时间写入（curvature，单位 ms），
/// 因此这里只做与传感器无关的过滤：点抽稀、盲区裁剪、NaN 丢弃。
class FastLivo2Preprocess {
public:
    void configure(const FastLivo2Config& cfg);

    /// 返回 false 表示过滤后没有有效点。
    bool process(const CloudType& in, CloudType& out) const;

private:
    int stride_ = 1;
    double blind_sqr_ = 0.0;
};

}  // namespace rosiwit_slam
