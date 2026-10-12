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
    std::vector<double> pt_times;
    for (const auto& f : msg->fields) {
        if (f.name == "timestamp" || f.name == "offset_time" || f.name == "t") {
            const int ps = msg->point_step;
            const size_t n = msg->width;
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
        // 判断时间字段是否有效覆盖扫描周期:
        // 若跨度 < 扫描周期 5% (近同时采集或常量字段), 不可用于去畸变, 回退线性模型。
        if (span < 0.05 * scan_period_ms * 1e6) have_time = false;   // ns → 周期*1e6
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
            // 相对扫描起始时间: ns → ms
            pt.curvature = static_cast<float>((pt_times[i] - t0) / 1e6);
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
