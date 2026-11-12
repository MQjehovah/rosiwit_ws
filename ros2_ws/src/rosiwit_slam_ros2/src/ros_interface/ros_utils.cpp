// src/ros_interface/ros_utils.cpp
#include "ros_interface/ros_utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cstring>
#include <cmath>
#include <algorithm>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr RosUtils::ros2PCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num,
    double min_range, double max_range, float scan_period_ms)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *tmp_cloud);

    const int point_num = tmp_cloud->size();
    if (filter_num < 1) filter_num = 1;

    // 读取逐点时间字段 (timestamp/offset_time), 单位为 ns 级 epoch 时间。
    // Livox 非重复扫描的逐点时间非线性, 必须用真实时间做去畸变;
    // 若用线性 100ms 假设, 旋转时点云会被严重扭曲导致匹配失败。
    bool have_time = false;
    double time_scale_ms = 1e-6;   // 逐点时间单位 -> ms (ns 默认)
    std::vector<double> pt_times;
    for (const auto& f : msg->fields) {
        if (f.name == "timestamp" || f.name == "offset_time" || f.name == "t") {
            const int ps = msg->point_step;
            // 注意: 必须用 width*height, organized point cloud 的 height>1 时
            // 只用 width 会让 pt_times 只覆盖 1/height 的点, 后面 pt_times[i] 越界读
            // (UB) 得到垃圾 curvature -> 帧 end_time 变成 inf, 同步层永远等不到 IMU.
            const size_t n = static_cast<size_t>(msg->width) * msg->height;
            pt_times.reserve(n);
            const uint8_t* data = msg->data.data();
            for (size_t i = 0; i < n; ++i) {
                uint64_t v = 0;
                if (f.datatype == 8) { double d=0; memcpy(&d,data+i*ps+f.offset,8); pt_times.push_back(d); continue; }
                else if (f.datatype == 7) { float fl=0; memcpy(&fl,data+i*ps+f.offset,4); pt_times.push_back(fl); continue; }
                else if (f.datatype == 6) { uint32_t u=0; memcpy(&u,data+i*ps+f.offset,4); v=u; }
                else if (f.datatype == 4) { uint16_t u=0; memcpy(&u,data+i*ps+f.offset,2); v=u; }
                else if (f.datatype == 2 || f.datatype == 1) { v = data[i*ps+f.offset]; }
                else { memcpy(&v,data+i*ps+f.offset,8); }
                pt_times.push_back(static_cast<double>(v));
            }
            have_time = !pt_times.empty();
            break;
        }
    }
    if (have_time) {
        double lo = *std::min_element(pt_times.begin(), pt_times.end());
        double hi = *std::max_element(pt_times.begin(), pt_times.end());
        double span = hi - lo;
        // 单位自适应: 有的驱动发 epoch ns, 有的发"相对扫描起始的秒"(Vanjee 722 实测 0~0.09995).
        // 旧实现只按 ns 判断, 秒制的时间戳会被误判为无效而回退线性模型 —— 真实逐点时间被丢掉.
        const double period_s = scan_period_ms * 1e-3;
        double time_unit_ms = 0.0;   // 每个单位对应多少 ms
        if (span > 1e5 && span < 1e10) time_unit_ms = 1e-6;        // ns
        else if (span > 1e2 && span <= 1e5) time_unit_ms = 1e-3;   // us
        else if (span > 1.0 && span <= 1e2) time_unit_ms = 1.0;    // ms
        else if (span > 1e-6 && span <= 1.0) time_unit_ms = 1e3;   // s (Vanjee)
        // 单位换算后的跨度必须与扫描周期同量级 (0.2~5 倍), 否则判为不可用
        const double span_ms = span * time_unit_ms;
        if (time_unit_ms <= 0.0 || span_ms < 0.2 * scan_period_ms ||
            span_ms > 5.0 * scan_period_ms) {
            have_time = false;
            (void)period_s;
        }
        // 防御: 时间数组必须覆盖全部点, 否则回退线性模型 (绝不能越界读)
        if (pt_times.size() < static_cast<size_t>(tmp_cloud->size())) have_time = false;
        if (have_time) time_scale_ms = time_unit_ms;
    }
    double t0 = 0.0;
    if (have_time) t0 = *std::min_element(pt_times.begin(), pt_times.end());

    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num) {
        const auto& p = tmp_cloud->points[i];
        // 丢弃无效点: 机械式雷达(如 Vanjee)对无回波点输出 NaN/Inf,
        // 而 NaN 与任何值比较均为 false, 会绕过下面的距离过滤进入建图/GICP KNN 触发断言。
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
        if (d2 < min_range * min_range || d2 > max_range * max_range) continue;
        pcl::PointXYZINormal pt;
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = p.intensity;
        if (have_time) {
            // 相对扫描起始时间: 已按单位换算到 ms
            pt.curvature = static_cast<float>((pt_times[i] - t0) * time_scale_ms);
        } else {
            pt.curvature = scan_period_ms * static_cast<float>(i) / static_cast<float>(point_num);
        }
        cloud->push_back(pt);
    }
    return cloud;
}

double RosUtils::getSec(std_msgs::msg::Header &header) {
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time RosUtils::getTime(const double &sec) {
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
