// src/algorithms/grid_map/grid_map_postprocess.cpp
#include "algorithms/grid_map/grid_map_postprocess.h"
#include "algorithms/grid_map/grid_map_metrics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/ximgproc.hpp>)
#include <opencv2/ximgproc.hpp>
#define HAVE_OPENCV_XIMGPROC 1
#endif

namespace rosiwit_slam {

namespace {

cv::Mat toOccMask(const std::vector<int8_t>& data, int w, int h, bool no_info_is_obstacle) {
    cv::Mat m(h, w, CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < data.size(); ++i) {
        int8_t v = data[i];
        if (v == 100 || (no_info_is_obstacle && v < 0)) m.data[i] = 255;
    }
    return m;
}

// 把占据掩码写回: mask=255 -> occupied; mask=0 且原为 occupied -> 背景(bg)
void writeBack(const cv::Mat& mask, std::vector<int8_t>& data, int8_t bg) {
    for (size_t i = 0; i < data.size(); ++i) {
        if (mask.data[i] == 255) {
            data[i] = 100;
        } else if (data[i] == 100) {
            data[i] = bg;
        }
    }
}

cv::Mat ellipseKernel(int radius) {
    int k = 2 * radius + 1;
    return cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
}

// ---- 算子 ----

void opDenoise(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t bg) {
    if (c.min_component_cells <= 1) return;
    cv::Mat mask = toOccMask(data, w, h, c.no_info_is_obstacle);
    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids,
                                             c.connectivity == 4 ? 4 : 8, CV_32S);
    // 先标记每个连通域是否保留, 再单次遍历回写; 避免按组件重复扫描整幅图 (O(组件数*像素数)).
    std::vector<uint8_t> keep(n, 0);
    for (int i = 1; i < n; ++i)
        keep[i] = (stats.at<int>(i, cv::CC_STAT_AREA) >= c.min_component_cells) ? 1 : 0;
    int removed = 0;
    for (int y = 0; y < h; ++y) {
        const int* lab = labels.ptr<int>(y);
        int8_t* row = data.data() + static_cast<size_t>(y) * w;
        for (int x = 0; x < w; ++x) {
            int lbl = lab[x];
            if (lbl > 0 && !keep[lbl]) { row[x] = bg; ++removed; }
        }
    }
    std::cout << "[Post:denoise] comps=" << (n - 1) << " removed_cells=" << removed
              << " min_group=" << c.min_component_cells << std::endl;
}

void opOpen(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t bg) {
    if (c.open_radius <= 0) return;
    cv::Mat mask = toOccMask(data, w, h, false);
    cv::Mat out;
    cv::morphologyEx(mask, out, cv::MORPH_OPEN, ellipseKernel(c.open_radius));
    writeBack(out, data, bg);
    std::cout << "[Post:open] radius=" << c.open_radius << std::endl;
}

void opClose(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t /*bg*/) {
    if (c.close_radius <= 0) return;
    cv::Mat mask = toOccMask(data, w, h, false);
    cv::Mat out;
    cv::morphologyEx(mask, out, cv::MORPH_CLOSE, ellipseKernel(c.close_radius));
    writeBack(out, data, -1);   // close 只新增, 不会移除占据
    std::cout << "[Post:close] radius=" << c.close_radius << std::endl;
}

void opFillHoles(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t /*bg*/) {
    if (c.max_hole_cells <= 0) return;
    cv::Mat mask = toOccMask(data, w, h, false);
    cv::Mat nonocc(h, w, CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < data.size(); ++i) nonocc.data[i] = (mask.data[i] == 255) ? 0 : 255;

    cv::Mat labels, stats, centroids;
    int n = cv::connectedComponentsWithStats(nonocc, labels, stats, centroids, 8, CV_32S);
    int filled = 0;
    for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > c.max_hole_cells) continue;
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int ww = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int hh = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        bool border = (x == 0 || y == 0 || x + ww >= w || y + hh >= h);
        if (border) continue;   // 外部区域不是空洞
        for (int yy = 0; yy < h; ++yy)
            for (int xx = 0; xx < w; ++xx)
                if (labels.at<int>(yy, xx) == i) data[yy * w + xx] = 100;
        filled += area;
    }
    std::cout << "[Post:fill_holes] filled_cells=" << filled
              << " max_hole=" << c.max_hole_cells << std::endl;
}

void opDilate(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t /*bg*/) {
    int r = c.dilate_radius;
    if (r < 0) {
        r = (c.point_spacing > 0.0 && c.resolution > 0.0)
            ? static_cast<int>(std::floor(c.point_spacing / (2.0 * c.resolution)))
            : 0;
    }
    if (r <= 0) return;
    cv::Mat mask = toOccMask(data, w, h, false);
    cv::Mat out;
    cv::dilate(mask, out, ellipseKernel(r));
    writeBack(out, data, -1);   // 膨胀只新增
    std::cout << "[Post:dilate] radius=" << r << std::endl;
}

void opSkeleton(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t bg) {
    if (c.skeleton_width <= 0) return;
#ifdef HAVE_OPENCV_XIMGPROC
    cv::Mat mask = toOccMask(data, w, h, false);
    cv::Mat thin;
    cv::ximgproc::thinning(mask, thin, cv::ximgproc::THINNING_ZHANGSUEN);
    if (c.skeleton_width > 1) {
        cv::Mat out;
        cv::dilate(thin, out, ellipseKernel(c.skeleton_width / 2));
        thin = out;
    }
    writeBack(thin, data, bg);
    std::cout << "[Post:skeleton] width=" << c.skeleton_width << std::endl;
#else
    (void)data; (void)w; (void)h; (void)c; (void)bg;
    std::cout << "[Post:skeleton] OpenCV ximgproc 不可用, 跳过" << std::endl;
#endif
}

void opLine(std::vector<int8_t>& data, int w, int h, const GridPostConfig& c, int8_t /*bg*/) {
    if (c.line_min_len_m <= 0.0) return;
    cv::Mat mask = toOccMask(data, w, h, false);
    std::vector<cv::Vec4i> lines;
    int min_len = std::max(2, static_cast<int>(c.line_min_len_m / c.resolution));
    int max_gap = std::max(1, static_cast<int>(c.line_max_gap_m / c.resolution));
    cv::HoughLinesP(mask, lines, 1, CV_PI / 180.0, std::max(10, min_len / 2), min_len, max_gap);
    cv::Mat out(h, w, CV_8UC1, cv::Scalar(0));   // 用线段重建 (正则化: 只保留直线)
    for (const auto& l : lines) {
        cv::Point p1(l[0], l[1]), p2(l[2], l[3]);
        if (c.axis_snap) {
            double ang = std::atan2(p2.y - p1.y, p2.x - p1.x);
            double deg = ang * 180.0 / CV_PI;
            if (std::abs(deg) < 15.0 || std::abs(deg) > 165.0) p2.y = p1.y;
            else if (std::abs(std::abs(deg) - 90.0) < 15.0) p2.x = p1.x;
        }
        cv::line(out, p1, p2, cv::Scalar(255), 2);
    }
    writeBack(out, data, -1);
    std::cout << "[Post:line] segments=" << lines.size()
              << " min_len=" << min_len << std::endl;
}

void opMetrics(const std::vector<int8_t>& data, int w, int h, const GridPostConfig& c) {
    GridInfo g;
    g.width = w; g.height = h; g.data = data;
    GridMetrics m = computeGridMetrics(g, c.connectivity,
                                       std::max(2, c.min_component_cells));
    std::cout << "[Post:metrics] occ=" << m.occupied << " free=" << m.free
              << " unknown=" << m.unknown
              << " comps=" << m.occupied_components
              << " largest=" << m.largest_component
              << " continuity=" << m.continuity
              << " noise_comps=" << m.noise_components
              << " free_comps=" << m.free_components
              << " thickness=" << m.mean_thickness << std::endl;
}

} // namespace

std::vector<std::string> parsePostPipeline(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        // trim
        size_t a = tok.find_first_not_of(" \t");
        size_t b = tok.find_last_not_of(" \t");
        if (a == std::string::npos) continue;
        std::string name = tok.substr(a, b - a + 1);
        if (!name.empty() && name != "none") out.push_back(name);
    }
    return out;
}

void applyGridPostProcess(GridInfo& grid, const GridPostConfig& cfg) {
    if (grid.data.empty() || grid.width <= 0 || grid.height <= 0) return;
    const int w = grid.width, h = grid.height;

    bool has_unknown = false;
    for (auto v : grid.data) if (v < 0) { has_unknown = true; break; }
    const int8_t bg = has_unknown ? -1 : 0;

    for (const auto& name : cfg.pipeline) {
        if      (name == "denoise")    opDenoise(grid.data, w, h, cfg, bg);
        else if (name == "open")       opOpen(grid.data, w, h, cfg, bg);
        else if (name == "close")      opClose(grid.data, w, h, cfg, bg);
        else if (name == "fill_holes") opFillHoles(grid.data, w, h, cfg, bg);
        else if (name == "dilate")     opDilate(grid.data, w, h, cfg, bg);
        else if (name == "skeleton")   opSkeleton(grid.data, w, h, cfg, bg);
        else if (name == "line")        opLine(grid.data, w, h, cfg, bg);
        else if (name == "metrics")     opMetrics(grid.data, w, h, cfg);
        else std::cout << "[Post] 未知算子: " << name << " (忽略)" << std::endl;
    }
}

} // namespace rosiwit_slam
