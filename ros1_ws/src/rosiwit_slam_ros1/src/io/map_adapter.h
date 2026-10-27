#pragma once
// ROS1 私有地图格式 (xzmap) <-> 核心中立 MapData 的适配层。
// 核心不再读取/写入 xzmap。
#include <string>
#include "slam_core/map_data.h"

namespace rosiwit_slam_ros1 {
namespace map_adapter {

/// path 可以是 <dir>、<dir>/map 或 <dir>/map.xzmap。
/// 读写互斥: 载图 (切图后台任务, 大图可达 25s) 与存图 (服务线程) 不能同时进行,
/// 否则会出现"读到写了一半的 xzmap"。载图阻塞等待, 存图最多等 wait_ms,
/// 超时返回 false 让上层报 MAP_BUSY, 而不是把 App 的服务调用挂死。
bool loadMapData(const std::string& path, rosiwit_slam::MapData& out);

/// 整段地图写盘的互斥护栏 (xzmap + pgm/yaml + 缩略图 全部包住)。
/// 与 loadMapData / saveMapData 用的是同一把**递归**锁, 所以内部嵌套加锁是安全的;
/// 拿不到锁 (正在载图/存图) 时 ok()==false, 调用方应报"忙"而不是硬写。
class MapIoGuard {
public:
    explicit MapIoGuard(int wait_ms = 2000);
    ~MapIoGuard();
    MapIoGuard(const MapIoGuard&) = delete;
    MapIoGuard& operator=(const MapIoGuard&) = delete;
    bool ok() const { return locked_; }

private:
    bool locked_ = false;
};
bool saveMapData(const rosiwit_slam::MapData& data, const std::string& path,
                 int wait_ms = 2000);

}  // namespace map_adapter
}  // namespace rosiwit_slam_ros1
