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

    // 1Hz 调试出口: 地图自更新 overlay / 会话地图 (latched, 有订阅者才构建)
    if (debug_publish_ && mode == SlamMode::LOCALIZATION &&
        (overlay_map_puber_.getNumSubscribers() > 0 ||
         session_map_puber_.getNumSubscribers() > 0)) {
        const ros::Time now = ros::Time::now();
        if (last_aux_map_pub_.isZero() || (now - last_aux_map_pub_).toSec() >= 1.0) {
            last_aux_map_pub_ = now;
            if (overlay_map_puber_.getNumSubscribers() > 0) {
                rosiwit_slam::PointVec pts;
                if (engine_.getOverlayCloud(pts) && !pts.empty()) {
                    // 只发 x/y/z/intensity, intensity = A 阶段持久度 w(0~255);
                    // 着色交给客户端 (RViz Intensity / CloudCompare 标量场), 这里不烤颜色.
                    pcl::PointCloud<pcl::PointXYZI> cloud;
                    cloud.reserve(pts.size());
                    for (const auto& p : pts) {
                        pcl::PointXYZI q;
                        q.x = p.x;
                        q.y = p.y;
                        q.z = p.z;
                        q.intensity = p.intensity;
                        cloud.push_back(q);
                    }
                    cloud.width = static_cast<uint32_t>(cloud.size());
                    cloud.height = 1;
                    cloud.is_dense = false;
                    sensor_msgs::PointCloud2 msg;
                    pcl::toROSMsg(cloud, msg);
                    msg.header.frame_id = kMapFrame;
                    msg.header.stamp = now;
                    overlay_map_puber_.publish(msg);
                    static int wlog = 0;
                    if (++wlog % 10 == 1) {   // 约 10s 一次, 打 w 的分布
                        float wmin = 1e9f, wmax = -1e9f;
                        std::vector<float> ws;
                        ws.reserve(pts.size());
                        for (const auto& p : pts) {
                            const float w = p.intensity;
                            wmin = std::min(wmin, w);
                            wmax = std::max(wmax, w);
                            ws.push_back(w);
                        }
                        std::nth_element(ws.begin(), ws.begin() + ws.size() / 2, ws.end());
                        const float wmed = ws[ws.size() / 2];
                        size_t confirmed = 0;
                        for (size_t i = 0; i < ws.size(); ++i) {
                            if (ws[i] >= overlay_anchor_) ++confirmed;
                        }
                        std::cout << "[overlay] pts=" << pts.size()
                                  << " w[min/med/max]=" << wmin << "/" << wmed << "/" << wmax
                                  << " w>=" << overlay_anchor_ << ":" << confirmed << std::endl;
                    }
                }
            }
            if (session_map_puber_.getNumSubscribers() > 0) {
                rosiwit_slam::PointVec pts;
                if (engine_.getSessionMapCloud(pts) && !pts.empty()) {
                    CloudType cloud;
                    cloud.reserve(pts.size());
                    for (const auto& p : pts) cloud.push_back(p);
                    ensureCloudDims(cloud);
                    session_map_puber_.publish(toPointCloud2(cloud, kMapFrame, now));
                }
            }
        }
    }

    // 整图只在 debug 打开时发, 每个模式一次 (legacy.cpp:936)
    if (debug_publish_ && !debug_map_published_.load() &&
        (mode == SlamMode::LOCALIZATION ||
         (mode == SlamMode::MAPPING && have_imu_.load() && have_laser_.load()))) {
        // 失败(取不到图) 不能置位, 否则一次失败就永远不再重试 -> 定位模式下 /global_map 永远为空
        const double now_wall = ros::WallTime::now().toSec();
        if (now_wall >= next_debug_map_try_) {
            next_debug_map_try_ = now_wall + 2.0;   // 失败每 2s 重试, 成功由 worker 置位
            scheduleGlobalMapJob();
        }
    }
}

// ---------------------------------------------------------------- 0.01s 位姿 + slam/info

} // namespace rosiwit_slam_ros1
