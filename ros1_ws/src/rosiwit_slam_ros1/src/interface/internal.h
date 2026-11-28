// legacy 接口拆分后跨实现文件共享的内部助手 (原单文件匿名 namespace 内容)。
// 仅限 rosiwit_slam_ros1 包内使用。
#pragma once
#include <ros/ros.h>
#include <string>

#include "ros1_slam_interface.h"

namespace rosiwit_slam_ros1 {
namespace interface_internal {

constexpr const char* kMapFrame = "map";
// /scan_f_map 的高度带已改为 footprint 系 + yaml 可配 (scan_f_map.z_min/z_max,
// 2026-11-28); 旧的 map 系常量 kLocZMax/kMapZMax/kScanZMin 已删除。

inline const char* modeName(SlamMode m) {
    switch (m) {
        case SlamMode::FREE:         return "FREE";
        case SlamMode::MAPPING:      return "MAPPING";
        case SlamMode::LOCALIZATION: return "LOCALIZATION";
        default:                     return "UNKNOWN";
    }
}

/// 地图目录 -> `<base>` (剥掉 map.yaml/.pgm/.xzmap/.pcd/.jpg 后缀再拼 "/map")。
/// 兼容层服务 (/switch_map、/stop_map_operation ...) 都走这里。
std::string xzMapBase(const std::string& map_dir);

/// 从我们刚写出的 `<base>.pgm` 生成 `<base>.jpg` 与 `<dir>/thumb.png`。
/// 实现放在 internal.cpp，避免把 OpenCV/fstream 带给所有 interface 翻译单元。
void writeMapThumbnails(const std::string& base);

/// 读取旧地图的 PGM+YAML；内部栅格 row0=最小 y，文件 row0=最大 y，读入时翻转。
bool loadGridPgm(const std::string& base, rosiwit_slam::GridInfo& out);

} // namespace interface_internal
} // namespace rosiwit_slam_ros1
