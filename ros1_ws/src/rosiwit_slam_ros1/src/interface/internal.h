// legacy 接口拆分后跨实现文件共享的内部助手 (原单文件匿名 namespace 内容)。
// 仅限 rosiwit_slam_ros1 包内使用。
#pragma once
#include <ros/ros.h>
#include <string>

#include "ros1_slam_interface.h"

namespace rosiwit_slam_ros1 {
namespace interface_internal {

constexpr const char* kMapFrame = "map";
// 旧实现的 z 过滤上界: 定位 2.0 (legacy.cpp:989) / 建图 1.0 (legacy.cpp:1078)
constexpr double kLocZMax = 2.0;
constexpr double kMapZMax = 1.0;
// scan_f_map 高度带下界(相对地面): 与栅格提取的 0.1~2.0m 对齐。
// 0 会放行贴地点(z_ref 标定偏差 ~7cm 时地面直接漏进界面, 2026-10-17 用户报告)。
constexpr double kScanZMin = 0.1;

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
