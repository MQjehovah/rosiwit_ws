#pragma once
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"
#include "slam_core/pipeline_types.h"
#include "slam_core/map_data.h"

namespace rosiwit_slam {

class ILocalization {
public:
    // 定位状态只有三种。原 RECOVERING 语义并入 INIT：未确认/重定位中。
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

    /// 相对位姿观测 (LIO 里程计): 只表达"相对上一帧走了多少", 不含绝对位置。
    /// 默认忽略 —— 非融合实现 (gicp/scan_context) 不受影响。
    virtual void onRelative(const PoseStamped& /*pose*/,
                            const Eigen::Matrix<double, 6, 6>& /*cov*/) {}
    virtual Status getStatus() = 0;
    virtual bool getPose(PoseStamped& out) = 0;

    /// 高频预测: 在 t 时刻给出位姿(基于最后校正 + 运动先验传播), 供高频输出.
    virtual bool predictAt(double /*t*/, PoseStamped& /*out*/) { return false; }

    /// 绝对约束 (GNSS/UWB/反光标/二维码): 地图系位姿 + 协方差 [rot(3),trans(3)]. 默认忽略.
    virtual void onAbsolute(const PoseStamped& /*pose*/, const Eigen::Matrix<double, 6, 6>& /*cov*/) {}

    /// 按需触发一次**全局重定位** (上层 /global_localization 用)。
    /// 默认空实现: 不支持全局检索的实现忽略即可。
    virtual void requestRelocalization() {}

    /// 运动先验的等效不确定度 (米/弧度). false = 实现未提供.
    /// 供上层把"估计器的不确定度"写进对外契约, 而不是用按状态分档的启发值.
    virtual bool priorUncertainty(double& /*xy_sigma*/, double& /*yaw_sigma*/) const {
        return false;
    }

    /// 位姿 + 协方差 (order: [rot(3), trans(3)]), 供下游(Nav2 等)使用.
    /// 默认实现忽略协方差, 返回 getPose.
    virtual bool getPoseWithCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& /*cov*/) {
        return getPose(out);
    }

    /// 匹配置信度/覆盖率 [0,1] (跨配准器统一口径). 默认 0 —— 不支持的实现可不管.
    /// 旧 xz_3d_slam 的 slam/info.matching_degree 就是这个 (等价于旧的 getFitness()).
    virtual double confidence() const { return 0.0; }

    /// 位姿融合诊断 (JSON; 未启用融合时返回空串)
    virtual std::string poseFusionDiagnostics() const { return std::string(); }
};

} // namespace rosiwit_slam
