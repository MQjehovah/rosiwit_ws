// include/slam_core/i_slam_algorithm.h
#pragma once
#include <string>
#include <functional>
#include <vector>
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

    // 管线控制接口 (默认空实现, 仅 SlamPipeline 覆写)
    /// 保存地图
    virtual bool saveMap(const std::string& /*path*/) { return false; }
    /// 加载地图用于定位
    virtual bool loadMap(const std::string& /*path*/) { return false; }
    /// 保存栅格地图 (PGM + YAML)
    virtual bool saveGridMap(const std::string& /*pgm_path*/, const std::string& /*yaml_path*/, double /*resolution*/) { return false; }
    /// 生成并返回 2D 占据栅格 (与 saveGridMap 内部**同一条路径**: 逐帧观测 + 地面处理)。
    /// 供实时 /map 使用, 保证"实时看到的"与"存下来的 pgm"一致。
    virtual bool getRealtimeGrid(double /*resolution*/, rosiwit_slam::GridInfo& /*out*/) { return false; }
    /// 运行时切换 2D 栅格生成策略 (GridMapFactory 注册名); 默认不支持
    virtual bool setGridMapGenerator(const std::string& /*name*/) { return false; }
    /// 设置管线模式: "idle" / "mapping" / "localization"
    virtual bool setPipelineMode(const std::string& /*mode*/) { return false; }
    /// 获取当前管线模式
    virtual std::string getPipelineMode() const { return "mapping"; }
    /// 设置初始位姿 (定位模式使用)
    virtual bool setInitPose(const PoseStamped& /*pose*/) { return false; }
    /// 获取栅格地图信息 (用于定位模式发布 grid_map)
    virtual GridInfo getGridInfo() const { return {}; }

    // 视觉松耦合 (route b, 默认空实现): 外部 VO 库输出注入
    /// VO 绝对里程计 (该帧时刻)
    virtual void onVisualOdometry(double /*time*/, const PoseStamped& /*vo_pose*/) {}
    /// 视觉回环约束
    virtual void onVisualLoop(const std::string& /*from_id*/, const std::string& /*to_id*/,
                              const PoseStamped& /*rel*/, double /*cov*/) {}
    /// 轮式里程计 (可选, 供定位运动先验; 默认忽略)
    virtual void onOdom(const PoseStamped& /*odom*/) {}
    /// 高频预测位姿 (定位模式; 供高频输出/TF; 默认不支持)
    virtual bool getHighRatePose(double /*t*/, PoseStamped& /*out*/) { return false; }
    /// 位姿 + 协方差 (order [rot(3),trans(3)]; 默认不支持)
    virtual bool getPoseCov(PoseStamped& /*out*/, Eigen::Matrix<double, 6, 6>& /*cov*/) { return false; }
    /// 定位匹配度 [0,1] (定位模式下才有意义; 默认 0).
    /// 旧 xz_3d_slam 的 slam/info.matching_degree 就是它 —— 实机实测是真值(如 0.831), 不是 0.
    virtual double getMatchingDegree() const { return 0.0; }
    /// 丢掉当前建图结果并重建内部模块, 用于"退出建图后再进建图"**不残留上一轮地图**。
    /// 旧 xz_3d_slam 每次进建图都会 DELETE_PTR(mapping_ptr_) 重建实例; 我们是单核心实例,
    /// 只能用显式重建达到同样效果。默认不支持。
    virtual bool resetForNewMapping(const std::string& /*config_path*/) { return false; }
    /// 绝对约束输入 (地图系位姿 + 协方差; 默认忽略)
    virtual void onAbsolute(const PoseStamped& /*pose*/, const Eigen::Matrix<double, 6, 6>& /*cov*/) {}
};

} // namespace rosiwit_slam
