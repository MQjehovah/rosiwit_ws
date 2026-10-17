// src/algorithms/grid_map/grid_map_io.cpp
#include "algorithms/grid_map/grid_map_io.h"
#include <fstream>
#include <sstream>

namespace rosiwit_slam {

bool saveGridMapPGM(const GridInfo& grid,
                    const std::string& pgm_path,
                    const std::string& yaml_path,
                    const std::string& frame_id)
{
    if (grid.data.empty() || grid.width <= 0 || grid.height <= 0) return false;

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    pgm << "P5\n" << grid.width << " " << grid.height << "\n255\n";
    // 写盘时竖直翻转: 图像首行=最大 y (ROS map_server 约定, 与参照项目一致)
    for (int row = grid.height - 1; row >= 0; --row) {
        const int8_t* r = grid.data.data() + static_cast<size_t>(row) * grid.width;
        for (int col = 0; col < grid.width; ++col) {
            int8_t v = r[col];
            uint8_t pixel = 128;            // unknown: 中灰 (pixel/255 与 (255-pixel)/255 两种解算下都判 unknown)
            if (v == 0) pixel = 254;         // free: 白
            else if (v >= 100) pixel = 0;    // occupied: 黑
            pgm.write(reinterpret_cast<char*>(&pixel), 1);
        }
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) return false;
    yaml << "image: " << pgm_path.substr(pgm_path.find_last_of("/\\") + 1) << "\n"
         << "resolution: " << grid.resolution << "\n"
         << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.59\nfree_thresh: 0.49\nmode: trinary\n";
    yaml.close();
    (void)frame_id;
    return true;
}

bool loadGridMapPGM(const std::string& pgm_path,
                    const std::string& yaml_path,
                    GridInfo& grid)
{
    std::ifstream yaml_file(yaml_path);
    if (!yaml_file) return false;
    std::string line;
    while (std::getline(yaml_file, line)) {
        auto pos = line.find(':');
        if (pos == std::string::npos) continue;
        auto key = line.substr(0, pos);
        auto val = line.substr(pos + 1);
        while (!val.empty() && val[0] == ' ') val = val.substr(1);
        if (key == "resolution") grid.resolution = std::stod(val);
        // ⚠ **origin 必须读**。旧实现只读了 resolution, 导致重新载入的 pgm 永远落在 (0,0),
        // 与 yaml 里声明的原点 (例如 [-32.947, -7.591]) 相差几十米 —— 对外发布的地图整体位移。
        if (key == "origin") {
            const auto lb = val.find('[');
            const auto rb = val.find(']');
            if (lb != std::string::npos && rb != std::string::npos) {
                std::string s = val.substr(lb + 1, rb - lb - 1);
                for (auto& c : s) if (c == ',') c = ' ';
                std::istringstream iss(s);
                double ox = 0.0, oy = 0.0, oz = 0.0;
                if (iss >> ox >> oy) { grid.origin_x = ox; grid.origin_y = oy; }
                (void)oz;
            }
        }
    }

    std::ifstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    std::string format;
    int max_val = 0;
    pgm >> format >> grid.width >> grid.height >> max_val;
    pgm.get();
    if (format != "P5") return false;
    grid.data.resize(static_cast<size_t>(grid.width) * grid.height);
    // 读入时反向翻转 (写盘时首行=最大 y), 还原为 row0=最小 y, 与内部约定一致
    for (int row = 0; row < grid.height; ++row) {
        const int src_row = grid.height - 1 - row;   // 文件行 src_row -> 数据行 row
        for (int col = 0; col < grid.width; ++col) {
            uint8_t pixel = 0;
            pgm.read(reinterpret_cast<char*>(&pixel), 1);
            int8_t v = -1;
            if (pixel == 254 || pixel == 255) v = 0;         // free
            else if (pixel == 0) v = 100;                    // occupied
            else v = -1;                                     // unknown
            grid.data[static_cast<size_t>(row) * grid.width + col] = v;
        }
    }
    return true;
}

} // namespace rosiwit_slam
