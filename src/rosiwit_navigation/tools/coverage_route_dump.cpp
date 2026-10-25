// 离线覆盖路线生成工具 (不依赖 ROS/仿真): 读 PGM 地图 + 多边形 -> buildRoute -> CSV
//
// 用法:
//   coverage_route_dump <map.pgm> <out.csv> <poly.txt>
//                      [sweep=0.5] [half_width=0.31] [turn_radius=0.0]
//                      [start_x] [start_y] [start_theta=0]
// poly.txt: 每行 "x,y" (米), 任意凸/凹多边形, 逆时针或顺时针均可.
// turn_radius: 0 = 原地转掉头; >0 = 半圆弧掉头 (上限半径, 实际 R = 行距/2).
// 输出 CSV: x,y,theta,kind   (kind 仅按相邻点几何粗分类, 调试用)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rosiwit_navigation/nav_core/coverage_grid.hpp"

using rosiwit_navigation::core::Pose2D;

namespace
{

struct Pgm
{
  int width = 0;
  int height = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> data;   // OccupancyGrid 语义: >=100 障碍, <0 未知, 其它自由
};

bool loadPgm(const std::string & path, Pgm & out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::string magic;
  f >> magic;
  if (magic != "P5") { std::cerr << "only P5 pgm supported\n"; return false; }
  auto skip = [&f]() {
    while (true) {
      const int c = f.peek();
      if (c == '#') { std::string line; std::getline(f, line); }
      else if (std::isspace(c)) { f.get(); }
      else { break; }
    }
  };
  skip(); int w = 0; f >> w;
  skip(); int h = 0; f >> h;
  skip(); int maxv = 0; f >> maxv;
  f.get();   // 单个空白
  std::vector<unsigned char> raw(static_cast<size_t>(w) * h);
  f.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(raw.size()));
  out.width = w;
  out.height = h;
  out.data.assign(static_cast<size_t>(w) * h, 0);
  // PGM 首行是图像顶部 (最大 y); OccupancyGrid 原点在左下 -> 翻转
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const unsigned char v = raw[static_cast<size_t>(h - 1 - y) * w + x];
      int8_t occ = 0;
      if (v <= 50) occ = 100;            // 占据
      else if (v < 250) occ = -1;        // 未知
      out.data[static_cast<size_t>(y) * w + x] = occ;
    }
  }
  return true;
}

bool loadPolygon(const std::string & path, std::vector<Pose2D> & poly)
{
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') { continue; }
    Pose2D p;
    if (std::sscanf(line.c_str(), "%lf,%lf", &p.x, &p.y) == 2) { poly.push_back(p); }
  }
  return poly.size() >= 3;
}

}  // namespace

using namespace rosiwit_navigation;

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::printf(
      "usage: %s map.pgm out.csv poly.txt [sweep=0.5] [half_width=0.31] "
      "[turn_radius=0] [headland=0] [start_x] [start_y] [start_theta=0]\n", argv[0]);
    return 2;
  }
  Pgm map;
  map.resolution = 0.05;
  map.origin_x = -2.83255;
  map.origin_y = -5.86221;
  if (!loadPgm(argv[1], map)) return 2;

  std::vector<Pose2D> polygon;
  if (!loadPolygon(argv[3], polygon)) { std::cerr << "bad polygon\n"; return 2; }
  const double sweep = (argc > 4) ? std::atof(argv[4]) : 0.5;
  const double half_width = (argc > 5) ? std::atof(argv[5]) : 0.31;
  const double turn_radius = (argc > 6) ? std::atof(argv[6]) : 0.0;
  const double headland = (argc > 7) ? std::atof(argv[7]) : 0.0;

  core::CostmapGridInfo info;
  info.width = static_cast<unsigned int>(map.width);
  info.height = static_cast<unsigned int>(map.height);
  info.resolution = map.resolution;
  info.origin.position.x = map.origin_x;
  info.origin.position.y = map.origin_y;

  core::CoverageConfig cfg;
  cfg.sweep_width = sweep;
  cfg.obstacle_dilation = 0.10;
  cfg.center_inset = 0.0;
  cfg.robot_inset = half_width;
  cfg.turn_radius = turn_radius;
  cfg.headland_width = headland;

  core::CoverageGrid grid;
  if (!grid.initialize(info, polygon, cfg)) { std::cerr << "initialize failed\n"; return 1; }
  grid.updateStaticObstacles(map.data);

  // 起点: 命令行优先, 否则区域质心 (若不在区内就取第一条边的内点)
  core::Pose2D start;
  if (argc > 9) {
    start.x = std::atof(argv[8]);
    start.y = std::atof(argv[9]);
  } else {
    start = grid.centroid();
  }
  start.theta = (argc > 10) ? std::atof(argv[10]) : 0.0;
  if (!grid.contains(start)) {
    const core::Pose2D c = grid.centroid();
    start.x = c.x;
    start.y = c.y;
    if (!grid.contains(start)) {
      std::cerr << "start outside region and centroid not inside\n";
      return 1;
    }
  }

  std::vector<Pose2D> route;
  if (!grid.buildRoute(start, route)) {
    std::cerr << "buildRoute returned false\n";
    return 1;
  }

  std::ofstream out(argv[2]);
  out << "x,y,theta,kind\n";
  double len = 0.0;
  size_t rot = 0;
  for (size_t i = 0; i < route.size(); ++i) {
    const char * kind = "row";
    if (i > 0) {
      const double d = std::hypot(route[i].x - route[i - 1].x, route[i].y - route[i - 1].y);
      len += d;
      if (d < 1e-3) { kind = "rot"; ++rot; }
      else if (std::fabs(std::remainder(route[i].theta - route[i - 1].theta, 2.0 * M_PI)) > 0.1) {
        kind = "link";
      }
    }
    out << route[i].x << "," << route[i].y << "," << route[i].theta << "," << kind << "\n";
  }
  const auto st = grid.stats();
  std::printf("route pts=%zu rot_pts=%zu len=%.2f m  turn_radius=%.2f headland=%.2f\n",
              route.size(), rot, len, turn_radius, headland);
  std::printf("region: total=%.2f m2 obstacle=%.2f blocked=%.2f uncovered=%.2f\n",
              st.total * map.resolution * map.resolution,
              st.obstacle * map.resolution * map.resolution,
              st.blocked * map.resolution * map.resolution,
              st.uncovered * map.resolution * map.resolution);
  std::printf("wrote %s\n", argv[2]);
  return 0;
}
