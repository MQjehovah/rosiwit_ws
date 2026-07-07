// include/slam_core/i_slam_algorithm.h
#pragma once
#include <string>
#include <functional>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

class ISlamAlgorithm {
public:
    virtual ~ISlamAlgorithm() = default;

    virtual bool        init(const std::string& config_path) = 0;
    virtual void        onImu(const IMUSample& s) = 0;
    virtual void        onLidar(const LidarFrame& f) = 0;
    virtual SlamState   state() const = 0;
    virtual std::string name() const = 0;

    using OutputCallback = std::function<void(const SlamOutput&)>;
    virtual void        setOutputCallback(OutputCallback cb) = 0;

    // 可选:周期性提取全局地图点(供 Node 发布 cloud_map)
    virtual bool getGlobalMap(PointVec& out_points) { (void)out_points; return false; }
};

} // namespace rosiwit_slam
