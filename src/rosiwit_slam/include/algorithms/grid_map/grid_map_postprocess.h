// include/algorithms/grid_map/grid_map_postprocess.h
#pragma once
#include <string>
#include <vector>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 2D 栅格后处理管道配置 (每个算子可由参数值/管道列表开关)
struct GridPostConfig {
    // 算子执行顺序 (名字见 grid_map_postprocess.cpp 的 dispatch); 不在列表中的不执行
    std::vector<std::string> pipeline = {"denoise", "close", "dilate"};

    // denoise: 连通域去噪
    int  min_component_cells = 30;     // <该格数的占据连通域按噪点剔除(<=1 关闭)
    int  connectivity = 8;             // 4 或 8
    bool no_info_is_obstacle = false;  // 将 unknown 参与占据(默认否, 保留 unknown)

    // open / close: 形态学 (椭圆核半径, 单位: 格)
    int  open_radius = 0;              // 0 关闭
    int  close_radius = 1;             // 0 关闭

    // fill_holes: 填内部小空洞
    int  max_hole_cells = 0;           // 0 关闭; >0 填充面积<=该值且不接触边界的空洞

    // dilate: 障碍膨胀(圆形/椭圆)
    int  dilate_radius = -1;           // <0 按 point_spacing/resolution 自动

    // refine
    int  skeleton_width = 0;           // >0: 骨架化(细化)后按该宽度膨胀, 得到统一细墙
    double line_min_len_m = 0.0;       // >0: HoughLinesP 线段正则化(最小长度 m)
    double line_max_gap_m = 0.3;
    bool axis_snap = false;            // 线段吸附到水平/垂直

    // 环境 (供自动膨胀)
    double resolution = 0.05;
    double point_spacing = 0.0;
};

/// 依次执行 cfg.pipeline 中列出的算子 (原地修改 grid.data)
void applyGridPostProcess(GridInfo& grid, const GridPostConfig& cfg);

/// 解析 "a,b,c" 为算子列表
std::vector<std::string> parsePostPipeline(const std::string& s);

} // namespace rosiwit_slam
