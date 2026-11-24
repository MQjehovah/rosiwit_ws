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

    // 1Hz 调试出口: 地图自更新 overlay / 已确认 overlay / 会话地图 (latched, 有订阅者才构建)
    if (debug_publish_ && mode == SlamMode::LOCALIZATION &&
        (overlay_map_puber_.getNumSubscribers() > 0 ||
         overlay_confirmed_puber_.getNumSubscribers() > 0 ||
         session_map_puber_.getNumSubscribers() > 0)) {
        const ros::Time now = ros::Time::now();
        if (last_aux_map_pub_.isZero() || (now - last_aux_map_pub_).toSec() >= 1.0) {
            last_aux_map_pub_ = now;
            const bool want_ov = overlay_map_puber_.getNumSubscribers() > 0;
            const bool want_conf = overlay_confirmed_puber_.getNumSubscribers() > 0;
            if (want_ov || want_conf) {
                rosiwit_slam::PointVec pts;
                if (engine_.getOverlayCloud(pts) && !pts.empty()) {
                    // intensity = A 阶段持久度 w(0~255), rgb = 按 w 上色:
                    // 灰=无记录/刚提交, 红=1 票, 橙/黄=中间, 绿>=anchor(已确认), 青=满票.
                    if (want_ov) {
                        CloudType cloud;
                        cloud.reserve(pts.size());
                        for (const auto& p : pts) cloud.push_back(p);
                        ensureCloudDims(cloud);
                        overlay_map_puber_.publish(
                            toPointCloud2Weighted(cloud, kMapFrame, now, overlay_anchor_));
                    }
                    if (want_conf) {
                        // 只发 w >= maintainer.persistency_anchor 的点: 拿掉"只看过一两次"的
                        // 弱证据, 只留跨会话确认过的变化结构 (干净的一片绿).
                        // 即使为空也发 (点数 0): 订阅端能明确区分"没有已确认结构"和"话题没起来".
                        CloudType conf;
                        conf.reserve(pts.size());
                        for (const auto& p : pts) {
                            if (p.intensity >= overlay_anchor_) conf.push_back(p);
                        }
                        ensureCloudDims(conf);
                        overlay_confirmed_puber_.publish(
                            toPointCloud2Weighted(conf, kMapFrame, now, overlay_anchor_));
                    }
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
