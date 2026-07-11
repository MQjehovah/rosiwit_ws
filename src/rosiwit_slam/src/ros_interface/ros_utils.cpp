// src/ros_interface/ros_utils.cpp
#include "ros_interface/ros_utils.h"
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr RosUtils::ros2PCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num,
    double min_range, double max_range, float scan_period_ms)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *tmp_cloud);

    const int point_num = tmp_cloud->size();
    if (filter_num < 1) filter_num = 1;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num) {
        const auto& p = tmp_cloud->points[i];
        const float d2 = p.x * p.x + p.y * p.y + p.z * p.z;
        if (d2 < min_range * min_range || d2 > max_range * max_range) continue;
        pcl::PointXYZINormal pt;
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = p.intensity;
        pt.curvature = scan_period_ms * static_cast<float>(i) / static_cast<float>(point_num);
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
