// include/algorithms/grid_map/grid_map_utils.h
#pragma once
#include <vector>
#include <algorithm>
#include <cmath>

namespace rosiwit_slam {

/// XY 平面稳健包围盒 (按分位数裁掉稀疏离群点, 避免个别远端噪点把地图量程拉大).
struct XYBounds { double min_x = 0, max_x = 0, min_y = 0, max_y = 0; };

inline XYBounds robustXYBounds(std::vector<double>& xs, std::vector<double>& ys,
                               double p_lo = 0.001, double p_hi = 0.999) {
    XYBounds b;
    if (xs.empty() || ys.empty()) return b;
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    auto P = [](std::vector<double>& v, double p) {
        size_t i = static_cast<size_t>(p * (v.size() - 1));
        return v[i];
    };
    b.min_x = P(xs, p_lo); b.max_x = P(xs, p_hi);
    b.min_y = P(ys, p_lo); b.max_y = P(ys, p_hi);
    return b;
}

/// 去除占据栅格中的小块噪点: 8-邻域连通域面积 < min_cells 的占据簇置为背景值 bg.
/// 应在障碍膨胀之前调用, 否则孤立噪点会被膨胀连到墙体上而无法剔除.
inline void removeSmallOccupiedComponents(std::vector<int8_t>& data, int w, int h,
                                          int min_cells, int8_t bg) {
    if (min_cells <= 1 || w <= 0 || h <= 0) return;
    const size_t n = static_cast<size_t>(w) * h;
    std::vector<char> visited(n, 0);
    std::vector<int> stack, comp;
    for (size_t i = 0; i < n; ++i) {
        if (data[i] != 100 || visited[i]) continue;
        comp.clear();
        stack.clear();
        stack.push_back(static_cast<int>(i));
        visited[i] = 1;
        while (!stack.empty()) {
            int cur = stack.back(); stack.pop_back();
            comp.push_back(cur);
            int r = cur / w, c = cur % w;
            for (int dr = -1; dr <= 1; ++dr) {
                int nr = r + dr;
                if (nr < 0 || nr >= h) continue;
                for (int dc = -1; dc <= 1; ++dc) {
                    if (dr == 0 && dc == 0) continue;
                    int nc = c + dc;
                    if (nc < 0 || nc >= w) continue;
                    int ni = nr * w + nc;
                    if (!visited[ni] && data[ni] == 100) {
                        visited[ni] = 1;
                        stack.push_back(ni);
                    }
                }
            }
        }
        if (static_cast<int>(comp.size()) < min_cells) {
            for (int idx : comp) data[idx] = bg;
        }
    }
}

/// 形态学闭运算 (先膨胀后腐蚀) 填补占据栅格中的小孔洞/缝隙 (墙体因回波稀疏
/// 呈"筛孔状"时的去噪), 不改变墙体整体厚度. radius 为结构元半径(格).
inline void morphCloseOccupied(std::vector<int8_t>& data, int w, int h, int radius) {
    if (radius <= 0 || w <= 0 || h <= 0) return;
    const size_t n = static_cast<size_t>(w) * h;
    std::vector<uint8_t> occ(n, 0);
    for (size_t i = 0; i < n; ++i) occ[i] = (data[i] == 100) ? 1 : 0;

    // 膨胀
    std::vector<uint8_t> dil = occ;
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (!occ[static_cast<size_t>(r) * w + c]) continue;
            for (int dy = -radius; dy <= radius; ++dy) {
                int nr = r + dy;
                if (nr < 0 || nr >= h) continue;
                for (int dx = -radius; dx <= radius; ++dx) {
                    int nc = c + dx;
                    if (nc < 0 || nc >= w) continue;
                    dil[static_cast<size_t>(nr) * w + nc] = 1;
                }
            }
        }
    }
    // 腐蚀
    std::vector<uint8_t> ero = dil;
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (!dil[static_cast<size_t>(r) * w + c]) continue;
            bool all = true;
            for (int dy = -radius; dy <= radius && all; ++dy) {
                int nr = r + dy;
                if (nr < 0 || nr >= h) { all = false; break; }
                for (int dx = -radius; dx <= radius; ++dx) {
                    int nc = c + dx;
                    if (nc < 0 || nc >= w || !dil[static_cast<size_t>(nr) * w + nc]) {
                        all = false; break;
                    }
                }
            }
            if (!all) ero[static_cast<size_t>(r) * w + c] = 0;
        }
    }
    for (size_t i = 0; i < n; ++i) {
        if (ero[i]) data[i] = 100;
    }
}

/// 自适应地面高度估计 (供各 2D 栅格策略共用).
/// 用 z 的低分位数(默认 2%)作为地面, 与点云密度无关:
/// 上视/环视雷达(如 Livox Mid-360)对天花板采样远多于地面, 任何"取最密平面"
/// 的做法都会把天花板误判为地面; 而地面总是观测 z 的最低显著包络, 用低分位数
/// 可稳定取到, 且天然忽略少量远端离群点.
inline double detectGroundZ(const std::vector<double>& zs, double p_low = 0.02) {
    if (zs.empty()) return 0.0;
    std::vector<double> s = zs;
    std::sort(s.begin(), s.end());
    size_t i = static_cast<size_t>(p_low * (s.size() - 1));
    if (i >= s.size()) i = s.size() - 1;
    return s[i];
}

} // namespace rosiwit_slam
