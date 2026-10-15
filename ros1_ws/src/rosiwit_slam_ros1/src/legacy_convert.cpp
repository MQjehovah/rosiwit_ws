// ============================================================================
// 旧 3d_slam 消息/数值转换的实现 (计划 T5 §5.1)
// ============================================================================
#include "legacy_convert.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cmath>
#include <cstdio>

namespace rosiwit_slam_ros1 {

namespace {

// 与旧实现同值 (json_warning_code.cpp:26-29 / json_warning_code.h:68)
constexpr const char* kModuleSlam = "xz_3d_slam";
constexpr const char* kTypeAlgorithmFault = "algorithm fault";

/// 最小 JSON 字符串转义 (旧实现用 nlohmann, 我们不想为此引一个 json 库)
std::string jsonEscape(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\b': o += "\\b";  break;
            case '\f': o += "\\f";  break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    o += buf;
                } else {
                    o += static_cast<char>(c);
                }
        }
    }
    return o;
}

void fillGridHeader(nav_msgs::OccupancyGrid& g, const rosiwit_slam::GridInfo& gi,
                    const std::string& frame) {
    g.header.frame_id = frame;
    // **不设 stamp**: 旧实现填栅格时压根没设过 header.stamp (legacy.cpp:1717-1731
    // 的 updateRealtimeGridMap、373-455 的 LocalizationSetGridMap 都只设 frame_id/info),
    // 所以旧 /map 与 global_grid_map 的 stamp 一直是 0。要 now() 的调用方自己设。
    g.info.resolution = gi.resolution;
    g.info.width = gi.width;
    g.info.height = gi.height;
    g.info.origin.position.x = gi.origin_x;
    g.info.origin.position.y = gi.origin_y;
    g.info.origin.orientation.w = 1.0;
}

} // namespace

nav_msgs::OccupancyGrid toLegacyGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame) {
    nav_msgs::OccupancyGrid g;
    fillGridHeader(g, gi, frame);
    g.data.assign(gi.data.begin(), gi.data.end());   // 行序原样, 不翻转
    return g;
}

nav_msgs::OccupancyGrid toFlippedGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame) {
    nav_msgs::OccupancyGrid g;
    fillGridHeader(g, gi, frame);
    const int w = gi.width, h = gi.height;
    g.data.resize(static_cast<size_t>(w) * h);
    for (int row = 0; row < h; ++row) {
        const int src = h - 1 - row;   // 首行 = 核心的最后一行(= 最大 y)
        for (int col = 0; col < w; ++col)
            g.data[static_cast<size_t>(row) * w + col] =
                gi.data[static_cast<size_t>(src) * w + col];
    }
    return g;
}

xzrobot_ros_dependency::SlamInfo buildSlamInfo(int8_t slam_state, int8_t location_status,
                                               const std::string& map_name,
                                               double x, double y, double yaw, float map_area) {
    xzrobot_ros_dependency::SlamInfo si;
    si.slam_state = slam_state;
    si.location_status = location_status;
    si.map_name = map_name;
    si.robot_pose.x = x;
    si.robot_pose.y = y;
    si.robot_pose.theta = yaw;
    si.map_area = map_area;
    si.update_request = 0;      // 恒 0
    si.matching_degree = 0.0f;  // D3
    return si;
}

std::string buildWarningJson(const std::string& code, const std::string& level,
                             const std::string& name, const std::string& content) {
    std::string j = "{\"faults\":[{";
    j += "\"code\":\"" + jsonEscape(code) + "\",";
    j += "\"content\":\"" + jsonEscape(content) + "\",";
    j += "\"level\":\"" + jsonEscape(level) + "\",";
    j += "\"module\":\"" + std::string(kModuleSlam) + "\",";
    j += "\"name\":\"" + jsonEscape(name) + "\",";
    j += "\"type\":\"" + std::string(kTypeAlgorithmFault) + "\"";
    j += "}]}";
    return j;
}

sensor_msgs::PointCloud2 toPointCloud2(const rosiwit_slam::CloudType& c,
                                       const std::string& frame, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 m;
    pcl::toROSMsg(c, m);
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    return m;
}

sensor_msgs::PointCloud toOldPointCloud(const rosiwit_slam::CloudType& c,
                                        const std::string& frame, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 m;
    pcl::toROSMsg(c, m);
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(m, pc);
    pc.header = m.header;
    return pc;
}

double legacyYawFromRot(const Eigen::Matrix3d& R) {
    return std::atan2(R(0, 1), R(0, 0));
}

double stdYawFromRot(const Eigen::Matrix3d& R) {
    return std::atan2(R(1, 0), R(0, 0));
}

rosiwit_slam::CloudType::Ptr legacyScanShape(const rosiwit_slam::CloudType& in,
                                             double z_max,
                                             int downsample_multiplier) {
    rosiwit_slam::CloudType::Ptr out(new rosiwit_slam::CloudType());
    const size_t n = in.size();

    int down_sample = 1;
    if (n < static_cast<size_t>(down_sample) * 2000) {
        down_sample = 1;
    } else {
        down_sample = static_cast<int>(std::round(1.0 * n / (down_sample * 2000)));
    }
    if (down_sample < 1) down_sample = 1;
    if (downsample_multiplier > 1) down_sample *= downsample_multiplier;

    out->reserve(n / static_cast<size_t>(down_sample) + 1);
    int scan_count = 1;
    const bool use_z = (z_max > 0.0);
    for (const auto& p : in) {
        if (!use_z || (p.z > 0 && p.z < z_max)) {
            if (scan_count % down_sample == 0) out->push_back(p);
        }
        ++scan_count;
    }
    return out;
}

} // namespace rosiwit_slam_ros1
