// ============================================================================
// interface 内部兼容助手：地图缩略图与 PGM/YAML 读取。
// 这些实现需要 OpenCV/fstream；放在 .cpp 里，internal.h 只保留声明。
// ============================================================================
#include "internal.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/opencv.hpp>

namespace rosiwit_slam_ros1 {
namespace interface_internal {

/// 从我们刚写出的 `<base>.pgm` 生成 `<base>.jpg` 与 `<dir>/thumb.png`。
/// 照抄旧实现 (multilayer_grid_map.cpp:284-288): jpg 与 pgm 是**同一张图**,
/// thumb 是它的 0.2 倍缩放 (App 用来显示地图缩略图)。
void writeMapThumbnails(const std::string& base) {
    const std::string pgm = base + ".pgm";
    const cv::Mat img = cv::imread(pgm, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        ROS_WARN_STREAM("[ros1_slam] cannot read " << pgm << " -> skip jpg/thumb");
        return;
    }
    if (!cv::imwrite(base + ".jpg", img)) {
        ROS_WARN_STREAM("[ros1_slam] imwrite failed: " << base + ".jpg");
    }
    cv::Mat thumb;
    cv::resize(img, thumb, cv::Size(0, 0), 0.2, 0.2);
    const size_t slash = base.find_last_of('/');
    const std::string dir = (slash == std::string::npos) ? std::string(".") : base.substr(0, slash);
    const std::string tpath = dir + "/thumb.png";
    if (!cv::imwrite(tpath, thumb)) {
        ROS_WARN_STREAM("[ros1_slam] imwrite failed: " << tpath);
    }
    std::cout << "[ros1_slam] wrote " << base << ".jpg and " << tpath << " (thumb "
              << thumb.cols << "x" << thumb.rows << ")" << std::endl;
}

/// 读取旧地图的 PGM+YAML。旧 PGM 可能带 `#` 注释行，不能直接用
/// 简单的 `>>` 解析（会把注释当成宽高）。这是 ROS1 兼容层逻辑：
/// 内部栅格 row0=最小 y，文件 row0=最大 y，读入时翻转。
bool loadGridPgm(const std::string& base, rosiwit_slam::GridInfo& out) {
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

} // namespace interface_internal
} // namespace rosiwit_slam_ros1
