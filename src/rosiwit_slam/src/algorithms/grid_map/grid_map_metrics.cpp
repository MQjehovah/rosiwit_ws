// src/algorithms/grid_map/grid_map_metrics.cpp
#include "algorithms/grid_map/grid_map_metrics.h"
#include <algorithm>
#include <opencv2/imgproc.hpp>
#if __has_include(<opencv2/ximgproc.hpp>)
#include <opencv2/ximgproc.hpp>
#define HAVE_OPENCV_XIMGPROC 1
#endif

namespace rosiwit_slam {

GridMetrics computeGridMetrics(const GridInfo& grid, int connectivity, int noise_threshold) {
    GridMetrics m;
    if (grid.data.empty() || grid.width <= 0 || grid.height <= 0) return m;
    const int w = grid.width, h = grid.height;
    cv::Mat occ(h, w, CV_8UC1, cv::Scalar(0));
    cv::Mat fre(h, w, CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < grid.data.size(); ++i) {
        int8_t v = grid.data[i];
        if (v == 100) { m.occupied++; occ.data[i] = 255; }
        else if (v == 0) { m.free++; fre.data[i] = 255; }
        else m.unknown++;
    }
    const int conn = (connectivity == 4) ? 4 : 8;

    if (m.occupied > 0) {
        cv::Mat labels, stats, cents;
        int n = cv::connectedComponentsWithStats(occ, labels, stats, cents, conn, CV_32S);
        m.occupied_components = n - 1;
        for (int i = 1; i < n; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > m.largest_component) m.largest_component = area;
            if (area < noise_threshold) { m.noise_components++; m.noise_cells += area; }
        }
        m.continuity = static_cast<double>(m.largest_component) / m.occupied;

#ifdef HAVE_OPENCV_XIMGPROC
        cv::Mat thin;
        cv::ximgproc::thinning(occ, thin, cv::ximgproc::THINNING_ZHANGSUEN);
        long skel = cv::countNonZero(thin);
        if (skel > 0) m.mean_thickness = static_cast<double>(m.occupied) / skel;
#endif
    }
    if (m.free > 0) {
        cv::Mat labels, stats, cents;
        int n = cv::connectedComponentsWithStats(fre, labels, stats, cents, conn, CV_32S);
        m.free_components = n - 1;
    }
    return m;
}

} // namespace rosiwit_slam
