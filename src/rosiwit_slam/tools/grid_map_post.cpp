// tools/grid_map_post.cpp
// 离线 2D 栅格后处理/评估工具: 载入 PGM, 应用可组合后处理管道, 输出指标并另存.
//
// 用法:
//   grid_map_post <in.pgm> <in.yaml> <out.pgm> [--post denoise,close,...]
//     --close N --dilate N --open N --min-comp N --connectivity 4|8
//     --fill N --skeleton N --line L --snap
//   pipeline 中可含 metrics 算子(仅统计), 例如 "metrics" 或 "denoise,close,skeleton,metrics"
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <string>
#include "algorithms/grid_map/grid_map_io.h"
#include "algorithms/grid_map/grid_map_metrics.h"
#include "algorithms/grid_map/grid_map_postprocess.h"

using namespace rosiwit_slam;

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: grid_map_post <in.pgm> <in.yaml> <out.pgm> [options]\n";
        return 1;
    }
    GridInfo g;
    if (!loadGridMapPGM(argv[1], argv[2], g)) { std::cerr << "load failed\n"; return 2; }

    GridPostConfig c;
    c.resolution = g.resolution;
    c.pipeline = parsePostPipeline("metrics");
    for (int i = 4; i < argc; ++i) {
        std::string a = argv[i];
        auto num = [&](double def) { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        if (a == "--post" && i + 1 < argc) c.pipeline = parsePostPipeline(argv[++i]);
        else if (a == "--close") c.close_radius = static_cast<int>(num(1));
        else if (a == "--dilate") c.dilate_radius = static_cast<int>(num(0));
        else if (a == "--open") c.open_radius = static_cast<int>(num(0));
        else if (a == "--min-comp") c.min_component_cells = static_cast<int>(num(30));
        else if (a == "--connectivity") c.connectivity = static_cast<int>(num(8));
        else if (a == "--fill") c.max_hole_cells = static_cast<int>(num(0));
        else if (a == "--skeleton") c.skeleton_width = static_cast<int>(num(0));
        else if (a == "--line") c.line_min_len_m = num(0.0);
        else if (a == "--snap") c.axis_snap = true;
    }

    std::cout << "[grid_map_post] in=" << argv[1]
              << " ops=" << c.pipeline.size() << " res=" << c.resolution << std::endl;
    applyGridPostProcess(g, c);
    GridMetrics m = computeGridMetrics(g, c.connectivity, std::max(2, c.min_component_cells));
    std::cout << "[grid_map_post] occ=" << m.occupied << " free=" << m.free
              << " unknown=" << m.unknown
              << " comps=" << m.occupied_components << " largest=" << m.largest_component
              << " continuity=" << m.continuity
              << " noise_comps=" << m.noise_components << " noise_cells=" << m.noise_cells
              << " free_comps=" << m.free_components
              << " thickness=" << m.mean_thickness << std::endl;
    if (!saveGridMapPGM(g, argv[3], std::string(argv[3]) + ".yaml", "map")) {
        std::cerr << "save failed\n";
        return 3;
    }
    return 0;
}
