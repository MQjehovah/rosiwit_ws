#pragma once
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"
#include "slam_core/map_data.h"

namespace rosiwit_slam {

class ILocalization {
public:
    enum Status { INIT, LOCALIZED, LOST };
    virtual ~ILocalization() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void setMap(const std::string& map_name) = 0;
    /// 中立地图数据加载；ROS1 xzmap 适配层使用。
    virtual void setMapData(const MapData& /*data*/) {}
    /// 取消正在进行的 setMap；实现可在阶段间检查并提前返回。
    virtual void cancelSetMap() {}
    virtual void setInitPose(const PoseStamped& pose) = 0;
    virtual void onImu(const IMUSample& s) = 0;
    virtual void onLidar(const LidarFrame& f) = 0;
    /// 可选: 轮式里程计 (运动先验, 默认忽略)
    virtual void onOdom(const PoseStamped& /*odom*/) {}
    virtual Status getStatus() = 0;
    virtual bool getPose(PoseStamped& out) = 0;

    /// 高频预测: 在 t 时刻给出位姿(基于最后校正 + 运动先验传播), 供高频输出.
    virtual bool predictAt(double /*t*/, PoseStamped& /*out*/) { return false; }

    /// 绝对约束 (GNSS/UWB/反光标/二维码): 地图系位姿 + 协方差 [rot(3),trans(3)]. 默认忽略.
    virtual void onAbsolute(const PoseStamped& /*pose*/, const Eigen::Matrix<double, 6, 6>& /*cov*/) {}

    /// 按需触发一次**全局重定位** (上层 /global_localization 用)。
    /// 默认空实现: 不支持全局检索的实现忽略即可。
    virtual void requestRelocalization() {}

    /// 位姿 + 协方差 (order: [rot(3), trans(3)]), 供下游(Nav2 等)使用.
    /// 默认实现忽略协方差, 返回 getPose.
    virtual bool getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& /*cov*/) {
        return getPose(out);
    }

    /// 匹配置信度/覆盖率 [0,1] (跨配准器统一口径). 默认 0 —— 不支持的实现可不管.
    /// 旧 xz_3d_slam 的 slam/info.matching_degree 就是这个 (等价于旧的 getFitness()).
    virtual double confidence() const { return 0.0; }
};

} // namespace rosiwit_slam
