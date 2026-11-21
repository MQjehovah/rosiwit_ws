#pragma once
// xzmap 读取模块 —— writeXzMap 的逆操作, 与 xzmap_writer 同属 io/, 仅依赖 std.
//
// 用于定位侧从设备端导出的 map.xzmap 载入关键帧 (位姿 + 机体系点云), 构建
// 关键帧子图 (keyframe_submap). 采用内置极简 JSON 解析器, 不引入第三方依赖.
//
// 注意: 流式解析, 点云直接写入 keyframes, 不构建 JVal 树; 6M 点的大图
//       峰值从 ~4.5GB 降到 "文件缓冲 + 原始点" 量级.
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
