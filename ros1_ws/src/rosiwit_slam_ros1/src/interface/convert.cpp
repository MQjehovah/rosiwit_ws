// ============================================================================
// 旧 3d_slam 消息/数值转换的实现 (计划 T5 §5.1)
// ============================================================================
#include "convert.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

nav_msgs::OccupancyGrid toRosGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame) {
    nav_msgs::OccupancyGrid g;
    fillGridHeader(g, gi, frame);
    g.data.assign(gi.data.begin(), gi.data.end());   // 行序原样, 不翻转
    return g;
}

nav_msgs::OccupancyGrid toFlippedRosGrid(const rosiwit_slam::GridInfo& gi, const std::string& frame) {
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
                                               double x, double y, double yaw, float map_area,
                                               float matching_degree) {
    xzrobot_ros_dependency::SlamInfo si;
    si.slam_state = slam_state;
    si.location_status = location_status;
    si.map_name = map_name;
    si.robot_pose.x = x;
    si.robot_pose.y = y;
    si.robot_pose.theta = yaw;
    si.map_area = map_area;
    si.update_request = 0;               // 恒 0
    si.matching_degree = matching_degree;   // 定位模式下的真实 fitness (见 §5.3 / D3 已填实)
    return si;
}

std::string buildWarningJson(const std::string& code, const std::string& level,
                             const std::string& name, const std::string& content) {
    WarningFault f;
    f.code = code; f.level = level; f.name = name; f.content = content;
    return buildWarningJson(std::vector<WarningFault>{f});
}

std::string buildWarningJson(const std::vector<WarningFault>& faults) {
    if (faults.empty()) return "";   // 旧实现: 空列表返回空串 (json_warning_code.cpp:57-59)
    // 键序照 nlohmann 默认的字母序: code, content, level, module, name, type
    std::string j = "{\"faults\":[";
    for (size_t i = 0; i < faults.size(); ++i) {
        const WarningFault& f = faults[i];
        if (i) j += ",";
        j += "{";
        j += "\"code\":\"" + jsonEscape(f.code) + "\",";
        j += "\"content\":\"" + jsonEscape(f.content) + "\",";
        j += "\"level\":\"" + jsonEscape(f.level) + "\",";
        j += "\"module\":\"" + std::string(kModuleSlam) + "\",";
        j += "\"name\":\"" + jsonEscape(f.name) + "\",";
        j += "\"type\":\"" + std::string(kTypeAlgorithmFault) + "\"";
        j += "}";
    }
    j += "]}";
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

namespace {
// 持久度 w(0~255) -> RGB. 分段线性, 刻度与 maintainer.persistency_anchor(默认 128) 对齐:
//   0 灰(无记录) / 8 红(只有 1 票, 刚提交) / 64 橙 / 112 黄 / 128 绿(已确认) / 255 青
// w 每票 +persistency_hit_gain(默认 8), anchor=128 即"16 票才算高可信".
inline void weightColor(float w, uint8_t& r, uint8_t& g, uint8_t& b) {
    struct Stop { float w; uint8_t r, g, b; };
    static const Stop kStops[] = {
        {0.0f, 140, 140, 140}, {8.0f, 225, 40, 40}, {64.0f, 235, 140, 30},
        {112.0f, 240, 220, 50}, {128.0f, 40, 215, 70}, {255.0f, 60, 210, 255}};
    const int n = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));
    if (w <= kStops[0].w) { r = kStops[0].r; g = kStops[0].g; b = kStops[0].b; return; }
    for (int i = 1; i < n; ++i) {
        if (w <= kStops[i].w) {
            const float span = kStops[i].w - kStops[i - 1].w;
            const float u = (span > 1e-6f) ? (w - kStops[i - 1].w) / span : 1.0f;
            auto mix = [u](uint8_t a, uint8_t c2) {
                return static_cast<uint8_t>(a + (static_cast<int>(c2) - static_cast<int>(a)) * u);
            };
            r = mix(kStops[i - 1].r, kStops[i].r);
            g = mix(kStops[i - 1].g, kStops[i].g);
            b = mix(kStops[i - 1].b, kStops[i].b);
            return;
        }
    }
    r = kStops[n - 1].r; g = kStops[n - 1].g; b = kStops[n - 1].b;
}
}  // namespace

sensor_msgs::PointCloud2 toPointCloud2Weighted(const rosiwit_slam::CloudType& c,
                                               const std::string& frame,
                                               const ros::Time& stamp) {
    sensor_msgs::PointCloud2 m;
    sensor_msgs::PointCloud2Modifier mod(m);
    mod.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    mod.resize(c.size());
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.height = 1;
    m.is_dense = true;
    sensor_msgs::PointCloud2Iterator<float> ix(m, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(m, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(m, "z");
    sensor_msgs::PointCloud2Iterator<float> iw(m, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t> ic(m, "rgb");
    for (size_t k = 0; k < c.size(); ++k, ++ix, ++iy, ++iz, ++iw, ++ic) {
        const auto& p = c[k];
        *ix = p.x;
        *iy = p.y;
        *iz = p.z;
        const float w = p.intensity;   // overlay: 由 maintainer 填成持久度 w
        *iw = w;
        uint8_t r = 0, g = 0, b = 0;
        weightColor(w, r, g, b);
        ic[0] = b;   // PCL 的 rgb 字段是 0x00RRGGBB, 小端下字节序为 b,g,r
        ic[1] = g;
        ic[2] = r;
    }
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

double xzYawFromRot(const Eigen::Matrix3d& R) {
    return std::atan2(R(0, 1), R(0, 0));
}

double stdYawFromRot(const Eigen::Matrix3d& R) {
    return std::atan2(R(1, 0), R(0, 0));
}

rosiwit_slam::CloudType::Ptr scanShape(const rosiwit_slam::CloudType& in,
                                             double z_max,
                                             int downsample_multiplier,
                                             double z_ref,
                                             double z_min) {
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
    // ⚠ z_ref = **地面在地图系里的 z**。旧实现假设地面就是 z=0 才写死 `p.z > 0`,
    // 而我们的地图系原点是起始时刻的雷达 -> 地面在 -0.898m。用绝对 z 过滤会把
    // 0~0.9m 的墙面全部砍掉(只剩天花板/上部), 与栅格的 0.1~2.0m 高度带对不上,
    // 表现就是"看不清点云和墙体的贴合情况"。这里改成相对地面过滤, 与栅格同带。
    for (const auto& p : in) {
        const double z = p.z - z_ref;
        // 下界默认 0(旧语义); scan_f_map 传 0.15 —— z_ref 实测可偏差 ~7cm(标定 -0.898
        // vs 实测地面 -0.83), 下界含 0 会把贴地点放进界面(用户报告"非墙体点云")。
        // 0.15 是 2026-11-22 现场确定的显示带下界 (上界 1.5m)。
        if (!use_z || (z > z_min && z < z_max)) {
            if (scan_count % down_sample == 0) out->push_back(p);
        }
        ++scan_count;
    }
    return out;
}

} // namespace rosiwit_slam_ros1
