// ============================================================================
// cloud_publisher.cpp — 兼容层的点云 / 栅格输出
//
// 单一职责: 把核心输出变成 legacy 话题 (scan_f_map / curr_scan_point /
// mapping_path / global_map / global_grid_map / /map)。
// 输入回调在 subscribers.cpp; 位姿/健康在 pose_publisher.cpp。
// ============================================================================
#include "ros1_slam_interface.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "convert.h"
#include "internal.h"
#include "map_adapter.h"

using namespace rosiwit_slam;
using namespace rosiwit_slam_ros1::interface_internal;

namespace rosiwit_slam_ros1 {





// ---------------------------------------------------------------- 定时器

void Ros1SlamInterface::timerPublishLocalCloudAndSubmap(const ros::TimerEvent&) {
    startPub();   // legacy.cpp:834
}
void Ros1SlamInterface::timerPublishRealtimeGridMap(const ros::TimerEvent&) {
    // 只触发: 栅格->OccupancyGrid 的转换在 cpu lane, publish 在输出线程
    if (slam_mode_ == SlamMode::MAPPING) scheduleGridJob();   // legacy.cpp:836-838
}

// ---------------------------------------------------------------- 0.1s 分发

void Ros1SlamInterface::startPub() {
    updateModeFromCore();   // 核心是模式的唯一权威

    // **只触发, 不做转换**:
    //   点云抽稀 / PointCloud(2) 构造 -> cpu lane (buildScanMessages)
    //   publish                      -> 输出线程 (drainHeavyOutbox)
    // 这里原来在 IO 线程上跑 30MB/s 的拷贝+序列化, 会把 IMU 200Hz 回调挤掉。
    const SlamMode mode = slam_mode_.load();
    const bool scan_wanted =
        (mode == SlamMode::LOCALIZATION) ||
        (mode == SlamMode::MAPPING && have_imu_.load() && have_laser_.load());
    if (scan_wanted) scheduleScanJob();

    // 整图只在 debug 打开时发, 每个模式一次 (legacy.cpp:936)
    if (debug_publish_ && !debug_map_published_.load() &&
        (mode == SlamMode::LOCALIZATION ||
         (mode == SlamMode::MAPPING && have_imu_.load() && have_laser_.load()))) {
        debug_map_published_.store(true);
        scheduleGlobalMapJob();
    }
}

// ---------------------------------------------------------------- 0.01s 位姿 + slam/info

} // namespace rosiwit_slam_ros1
