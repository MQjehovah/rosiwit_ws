// legacy 接口拆分后跨实现文件共享的内部助手 (原单文件匿名 namespace 内容)。
// 仅限 rosiwit_slam_ros1 包内使用。
#pragma once
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "legacy_xz3dslam_interface.h"

namespace rosiwit_slam_ros1 {
namespace legacy_internal {

constexpr const char* kMapFrame = "map";
// 旧实现的 z 过滤上界: 定位 2.0 (legacy.cpp:989) / 建图 1.0 (legacy.cpp:1078)
constexpr double kLocZMax = 2.0;
constexpr double kMapZMax = 1.0;
// scan_f_map 高度带下界(相对地面): 与栅格提取的 0.1~2.0m 对齐。
// 0 会放行贴地点(z_ref 标定偏差 ~7cm 时地面直接漏进界面, 2026-10-17 用户报告)。
constexpr double kScanZMin = 0.1;

inline const char* modeName(SlamMode m) {
    switch (m) {
        case SlamMode::FREE:         return "FREE";
        case SlamMode::MAPPING:      return "MAPPING";
        case SlamMode::LOCALIZATION: return "LOCALIZATION";
        default:                     return "UNKNOWN";
    }
}

/// 从我们刚写出的 `<base>.pgm` 生成 `<base>.jpg` 与 `<dir>/thumb.png`。
/// 照抄旧实现 (multilayer_grid_map.cpp:284-288): jpg 与 pgm 是**同一张图**,
/// thumb 是它的 0.2 倍缩放 (App 用来显示地图缩略图)。
inline void writeMapThumbnails(const std::string& base) {
    const std::string pgm = base + ".pgm";
    const cv::Mat img = cv::imread(pgm, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        ROS_WARN_STREAM("[legacy_iface] cannot read " << pgm << " -> skip jpg/thumb");
        return;
    }
    if (!cv::imwrite(base + ".jpg", img)) {
        ROS_WARN_STREAM("[legacy_iface] imwrite failed: " << base + ".jpg");
    }
    cv::Mat thumb;
    cv::resize(img, thumb, cv::Size(0, 0), 0.2, 0.2);
    const size_t slash = base.find_last_of('/');
    const std::string dir = (slash == std::string::npos) ? std::string(".") : base.substr(0, slash);
    const std::string tpath = dir + "/thumb.png";
    if (!cv::imwrite(tpath, thumb)) {
        ROS_WARN_STREAM("[legacy_iface] imwrite failed: " << tpath);
    }
    std::cout << "[legacy_iface] wrote " << base << ".jpg and " << tpath << " (thumb "
              << thumb.cols << "x" << thumb.rows << ")" << std::endl;
}

/// 读取旧地图的 PGM+YAML。旧 PGM 可能带 `#` 注释行，不能直接用
/// 简单的 `>>` 解析（会把注释当成宽高）。这是 ROS1 兼容层逻辑：
/// 内部栅格 row0=最小 y，文件 row0=最大 y，读入时翻转。
inline bool loadLegacyGridPgm(const std::string& base, rosiwit_slam::GridInfo& out) {
    const cv::Mat img = cv::imread(base + ".pgm", cv::IMREAD_GRAYSCALE);
    if (img.empty()) return false;
    double resolution = 0.05;
    double ox = 0.0, oy = 0.0;
    std::ifstream yaml(base + ".yaml");
    std::string line;
    while (std::getline(yaml, line)) {
        const auto pos = line.find(':');
        if (pos == std::string::npos) continue;
        const std::string key = line.substr(0, pos);
        const std::string val = line.substr(pos + 1);
        try {
            if (key == "resolution") resolution = std::stod(val);
            if (key == "origin") {
                const auto lb = val.find('[');
                const auto rb = val.find(']');
                if (lb != std::string::npos && rb != std::string::npos) {
                    std::string s = val.substr(lb + 1, rb - lb - 1);
                    for (auto& c : s) if (c == ',') c = ' ';
                    std::istringstream iss(s);
                    iss >> ox >> oy;
                }
            }
        } catch (...) {}
    }
    out.width = img.cols;
    out.height = img.rows;
    out.resolution = resolution;
    out.origin_x = ox;
    out.origin_y = oy;
    out.data.resize(static_cast<size_t>(out.width) * out.height);
    for (int row = 0; row < out.height; ++row) {
        const int src = out.height - 1 - row;
        for (int col = 0; col < out.width; ++col) {
            const uint8_t px = img.at<uint8_t>(src, col);
            int8_t v = -1;
            if (px == 254 || px == 255) v = 0;
            else if (px == 0) v = 100;
            out.data[static_cast<size_t>(row) * out.width + col] = v;
        }
    }
    return true;
}

} // namespace legacy_internal
} // namespace rosiwit_slam_ros1
