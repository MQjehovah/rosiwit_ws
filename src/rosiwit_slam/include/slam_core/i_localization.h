#pragma once
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class ILocalization {
public:
    enum Status { INIT, LOCALIZED, LOST };
    virtual ~ILocalization() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void setMap(const std::string& map_name) = 0;
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

    /// 位姿 + 协方差 (order: [rot(3), trans(3)]), 供下游(Nav2 等)使用.
    /// 默认实现忽略协方差, 返回 getPose.
    virtual bool getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& /*cov*/) {
        return getPose(out);
    }
};

} // namespace rosiwit_slam
