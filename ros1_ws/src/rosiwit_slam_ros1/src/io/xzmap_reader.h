#pragma once
// xzmap 读取模块 —— writeXzMap 的逆操作, 与 xzmap_writer 同属 io/, 仅依赖 std.
//
// 用于定位侧从设备端导出的 map.xzmap 载入关键帧 (位姿 + 机体系点云), 构建
// 关键帧子图 (keyframe_submap). 采用内置极简 JSON 解析器, 不引入第三方依赖.
//
// 注意: 解析为完整内存树, 大图 (数十万点) 会占用较多内存; 关键帧子图仅需
//       关键帧层级数据, 常规地图规模可接受.
#include <string>
#include <vector>
#include "xzmap_writer.h"

namespace rosiwit_slam {
namespace io {

/// 读取 xzmap 文件; 成功返回 true 并填充 keyframes (位姿为 body->world,
/// 点云为 body 系, 单位 m). out_map_name 可空.
bool readXzMap(const std::string& path,
               std::vector<XzMapKeyframe>& keyframes,
               std::string* out_map_name = nullptr);

} // namespace io
} // namespace rosiwit_slam
