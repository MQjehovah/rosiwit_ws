// ============================================================================
// cloud_utils.cpp — 点云工具
//
// 单一职责: ROS 点云 <-> PCL 点云的转换与"机身自过滤"这类纯数据处理,
// 不掺任何状态机、配置或发布逻辑。
// ============================================================================
#include "slam_algorithm.h"

#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include "slam_core/diagnostics.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "slam_core/slam_factory.h"
#include "slam_core/slam_pipeline.h"

namespace rosiwit_slam_ros1 {

using namespace rosiwit_slam;

CloudType::Ptr SlamAlgorithm::ros2Pcl(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                   int filter_num, double min_r, double max_r,
                                   double scan_period_ms) {
    CloudType::Ptr cloud(new CloudType());
    filter_num = std::max(1, filter_num);

    // 万集点云没有 normal_x/y/z 字段；直接 fromROSMsg 到 PointXYZINormal
    // 会每帧打印 "Failed to find match for field 'normal_*'”，并污染日志。
    // 这里按它实际有的 x/y/z/intensity 读取，再拷贝到核心点类型，
    // normal 字段保持默认零值。
    bool has_intensity = false;
    for (const auto& field : msg->fields) {
        if (field.name == "intensity") { has_intensity = true; break; }
    }
    if (has_intensity) {
        pcl::PointCloud<pcl::PointXYZI> tmp;
        pcl::fromROSMsg(*msg, tmp);
        cloud->resize(tmp.size());
        for (size_t i = 0; i < tmp.size(); ++i) {
            PointType& dst = cloud->points[i];
            dst.x = tmp.points[i].x;
            dst.y = tmp.points[i].y;
            dst.z = tmp.points[i].z;
            dst.intensity = tmp.points[i].intensity;
            dst.curvature = 0.0f;   // 无逐点时间时保持 0 (下面按需覆盖)
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
    } else {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        pcl::fromROSMsg(*msg, tmp);
        cloud->resize(tmp.size());
        for (size_t i = 0; i < tmp.size(); ++i) {
            PointType& dst = cloud->points[i];
            dst.x = tmp.points[i].x;
            dst.y = tmp.points[i].y;
            dst.z = tmp.points[i].z;
            dst.curvature = 0.0f;   // 无逐点时间时保持 0 (下面按需覆盖)
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = false;
    }

    // ⚠ **逐点时间必须手动补到 curvature** —— 核心的运动畸变补偿(IMUProcessor::undistort)用的是
    //    point.curvature/1000 = 该点相对帧起始的秒数, 并要求点按时间升序。
    //    pcl::fromROSMsg 不认识万集驱动的 `timestamp` 字段, 会把它丢掉 -> curvature 全 0
    //    -> 去畸变完全不生效。实机表现正是"平移还行, 一旋转就叠图"(一帧 100ms 内转过的角度
    //    没有被补偿)。万集字段实测: x,y,z,intensity,ring, timestamp(FLOAT64, 相对 header.stamp
    //    的秒数, 0 -> 0.0999; **按 ring 分块输出, 单 ring 内升序, 全局不是升序**;
    //    ts_first_point=true 表示 header.stamp 取首点时间。全局排序见本函数末尾。
    bool has_pt_time = false;
    for (const auto& f : msg->fields) {
        if (f.name == "timestamp" && f.datatype == sensor_msgs::PointField::FLOAT64) {
            has_pt_time = true;
            break;
        }
    }
    if (has_pt_time) {
        sensor_msgs::PointCloud2ConstIterator<double> t_it(*msg, "timestamp");
        for (size_t i = 0; i < cloud->size(); ++i, ++t_it) {
            cloud->points[i].curvature = static_cast<float>((*t_it) * 1000.0);   // ms
        }
    } else if (scan_period_ms > 1e-6) {
        // 没有逐点时间戳 (仿真 Gazebo 插件就是这样): 按方位角线性映射到一帧周期。
        // 机械式 360 度雷达一帧内匀速旋转, atan2(y,x) 就对应采样时刻。
        // scan_period_ms = 0 即关闭该推导 (等价于不做去畸变)。
        for (auto& p : cloud->points) {
            const double az = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
            p.curvature = static_cast<float>((az + M_PI) / (2.0 * M_PI) * scan_period_ms);
        }
        static bool warned = false;
        if (!warned) {
            warned = true;
            std::cout << "[slam_algo] no timestamp field: synthesized per-point time "
                         "from azimuth (scan_period_ms=" << scan_period_ms << ")"
                      << std::endl;
        }
    }

    // 万集驱动按 **ring 分块** 输出 (ring1: 0..100ms, ring2: 0..100ms, ...), 全局
    // timestamp 不是升序; 而核心 IMUProcessor::undistort() 是**从点云尾部向前**、按
    // curvature 单调匹配 IMU 位姿 (imu_processor.cpp:176-201), 要求点按时间升序.
    // 不排序的实机表现: 平移还能看, **一旋转就叠图/大块黑区**.
    // 这里按逐点时间稳定排序, 顺带让 filter_num 跳采在时间上均匀.
    if (!cloud->empty()) {
        std::stable_sort(cloud->points.begin(), cloud->points.end(),
                         [](const PointType& a, const PointType& b) {
                             return a.curvature < b.curvature;
                         });
    }

    CloudType::Ptr out(new CloudType());
    out->reserve(cloud->size() / filter_num + 1);
    const double min2 = min_r * min_r, max2 = max_r * max_r;
    double curv_min = 1e9, curv_max = -1e9;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (i % filter_num != 0) continue;
        const auto& p = cloud->points[i];
        if (!pcl::isFinite(p)) continue;
        const double d2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
                          static_cast<double>(p.z) * p.z;
        if (d2 < min2 || d2 > max2) continue;
        if (p.curvature < curv_min) curv_min = p.curvature;
        if (p.curvature > curv_max) curv_max = p.curvature;
        out->push_back(p);
    }
    // 一次性核对日志: 确认逐点时间真的进来了 (0 表示没填上 -> 去畸变失效)
    // backjumps 必须为 0: 核心去畸变要求点按时间升序.
    {
        static int once = 0;
        if (once++ == 0 && !out->empty()) {
            size_t backjumps = 0;
            for (size_t i = 1; i < out->size(); ++i)
                if (out->points[i].curvature < out->points[i - 1].curvature) ++backjumps;
            std::cout << "[slam_algo] point-time check: src=" << msg->width << "x" << msg->height
                      << " kept=" << out->size()
                      << " has_timestamp_field=" << (has_pt_time ? 1 : 0)
                      << " curvature_ms=[" << curv_min << " .. " << curv_max << "]"
                      << " backjumps=" << backjumps << std::endl;
        }
    }
    return out;
}

void SlamAlgorithm::applySelfFilter(CloudType::Ptr& cloud) {
    if (!cfg_.self_filter_enabled || !cloud || cloud->empty()) return;
    const RigidTransform& base_lidar = baseLidar();
    if (!base_lidar.valid) {
        static bool warned = false;
        if (!warned) {
            ROS_WARN("[slam_algo] self_filter enabled but base_link <- lidar TF is invalid; skip");
            warned = true;
        }
        return;
    }

    CloudType::Ptr out(new CloudType());
    out->reserve(cloud->size());
    size_t removed = 0;
    for (const auto& p : cloud->points) {
        const Eigen::Vector3d q = base_lidar.R * Eigen::Vector3d(p.x, p.y, p.z) + base_lidar.t;
        const bool in_body = q.x() >= cfg_.self_x_min && q.x() <= cfg_.self_x_max &&
                             q.y() >= cfg_.self_y_min && q.y() <= cfg_.self_y_max &&
                             q.z() >= cfg_.self_z_min && q.z() <= cfg_.self_z_max;
        if (in_body) { ++removed; continue; }
        out->push_back(p);
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = false;
    cloud = out;

    static int log_i = 0;
    if (++log_i % 100 == 0) {
        std::cout << "[slam_algo] self_filter removed " << removed << "/"
                  << (removed + cloud->size()) << " points" << std::endl;
    }
}

} // namespace rosiwit_slam_ros1
