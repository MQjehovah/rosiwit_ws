// src/algorithms/grid_map/grid_map_io.cpp
#include "algorithms/grid_map/grid_map_io.h"
#include <fstream>

namespace rosiwit_slam {

bool saveGridMapPGM(const GridInfo& grid,
                    const std::string& pgm_path,
                    const std::string& yaml_path,
                    const std::string& frame_id)
{
    if (grid.data.empty() || grid.width <= 0 || grid.height <= 0) return false;

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    pgm << "P5\n" << grid.width << " " << grid.height << "\n100\n";
    for (auto v : grid.data) {
        uint8_t pixel = 205;
        if (v == 0) pixel = 254;
        else if (v >= 100) pixel = 0;
        else pixel = static_cast<uint8_t>(205 - v * 50 / 100);
        pgm.write(reinterpret_cast<char*>(&pixel), 1);
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) return false;
    yaml << "image: " << pgm_path.substr(pgm_path.find_last_of("/\\") + 1) << "\n"
         << "resolution: " << grid.resolution << "\n"
         << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n";
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
    }

    std::ifstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return false;
    std::string format;
    int max_val = 0;
    pgm >> format >> grid.width >> grid.height >> max_val;
    pgm.get();
    if (format != "P5") return false;
    grid.data.resize(static_cast<size_t>(grid.width) * grid.height);
    for (int i = 0; i < grid.width * grid.height; ++i) {
        uint8_t pixel = 0;
        pgm.read(reinterpret_cast<char*>(&pixel), 1);
        if (pixel == 254) grid.data[i] = 0;
        else if (pixel == 0) grid.data[i] = 100;
        else grid.data[i] = -1;
    }
    return true;
}

} // namespace rosiwit_slam
