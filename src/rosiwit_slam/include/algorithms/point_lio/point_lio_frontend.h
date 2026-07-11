// include/algorithms/point_lio/point_lio_frontend.h
// Point-LIO 风格逐点处理前端
//
// 核心差异 vs FastLio2Frontend:
//   FastLio2Frontend:    攒完一帧 → 去畸变 → 全体 IKEF 更新 → 增量地图
//   PointLioFrontend:    每点到达 → IMU传播到该点时间 → 单点IKEF更新 → 加入地图
//
// 优势:
//   1. 无运动畸变补偿环节 (每点在其真实时间戳处理)
//   2. 更低延迟 (不等攒帧)
//   3. 高动态场景精度更好
//   4. 计算量分布更均匀 (非突发式)
//
// 使用方法 (config/default.yaml):
//   modules:
//     frontend: "point_lio_frontend"
//
// 依赖: 同 FastLIO2 (IESKF, iKD-Tree, Eigen, Sophus)

#pragma once
#include <memory>
#include <deque>
#include "slam_core/i_frontend.h"
#include "algorithms/fast_lio2/map_builder/commons.h"
#include "algorithms/fast_lio2/map_builder/ieskf.h"
#include "algorithms/fast_lio2/map_builder/ikd_Tree.h"

namespace rosiwit_slam {

class PointLioFrontend : public IFrontend {
public:
    PointLioFrontend() = default;
    ~PointLioFrontend() override = default;

    bool init(const std::string& config_path) override;
    void onImu(const IMUSample& s) override;
    void onLidar(const LidarFrame& f) override;
    bool getOdometry(PoseStamped& out) override;
    bool getClouds(CloudType::Ptr& body, CloudType::Ptr& world) override;
    bool getKeyFrame(KeyFrame& out) override;
    bool getGlobalMap(PointVec& out) override;

private:
    // 单点处理核心
    struct PointState {
        Eigen::Vector3d point_body;     // 当前点在 body 系下的坐标
        Eigen::Vector3d point_world;    // 变换到 world 系
        double timestamp;               // 点的时间戳
    };

    // 处理一个点: 传播IMU状态 → 变换 → 搜索近邻 → 构建残差 → 更新
    bool processPoint(const PointType& pt, double point_time);

    // 传播 IESKF 到指定时间
    void propagateToTime(double target_time);

    // 从点云数据中解析点的时间 (存储于 curvature 字段)
    double pointTimeFromCloud(const PointType& pt, double cloud_start_time) const;

    // 配置
    ::Config m_config;

    // IESKF 和地图
    std::shared_ptr<::IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;

    // IMU 缓存
    std::deque<::IMUData> m_imu_buffer;
    double m_last_imu_time = 0.0;
    ::IMUData m_last_imu;

    // 状态
    bool m_inited = false;
    bool m_first_scan = true;
    double m_last_lidar_time = 0.0;
    PoseStamped m_current_pose;

    // 用于关键帧判断
    PoseStamped m_last_kf_pose;
    bool m_has_last_kf = false;
    size_t m_frame_counter = 0;
    double m_keyframe_trans_thresh = 1.0;
    double m_keyframe_rot_thresh = 0.17;

    // 点云缓冲区 (用于 getClouds / getKeyFrame 输出)
    CloudType::Ptr m_current_cloud;

    // IMU 噪声协方差
    ::M12D m_Q;

    // 地图管理参数
    double m_map_resolution = 0.3;
    double m_scan_resolution = 0.15;
    double m_det_range = 60.0;
    double m_cube_len = 300.0;
    double m_move_thresh = 1.5;
};

} // namespace rosiwit_slam
