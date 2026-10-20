// include/slam_core/i_map_store.h
// 统一地图存储底座 (P1): 算法只依赖本接口, 不依赖具体存储实现.
// 目标: 换存储(PCD/KD树 或 体素哈希)不影响算法 —— 只要 nearestK/queryRadius
//       结果等价. 各算法按需取用"能力视图"(点/近邻/统计/地面/维护).
#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <Eigen/Eigen>
#include "slam_core/slam_types.h"

namespace rosiwit_slam {

/// 局部统计 (VGICP/NDT 用)
struct VoxelStats {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
    int n = 0;
    bool valid = false;
};

class IMapStore {
public:
    virtual ~IMapStore() = default;
    virtual std::string name() const = 0;

    /// 从点云构建 (载入底图 / 初始化)
    virtual bool buildFromCloud(const CloudType::Ptr& cloud) = 0;

    /// 半径内点云 (局部地图 / GICP / 覆盖率)
    virtual CloudType::Ptr queryRadius(const Eigen::Vector3d& center, double radius) const = 0;

    /// k 近邻 (pp_icp/p2p: 精确最近点). 返回实际找到数, 结果按距离升序写入 points/sqdists
    virtual int nearestK(const PointType& query, int k,
                         std::vector<PointType>& points, std::vector<float>& sqdists) const = 0;

    /// 最近距离平方 (<0 = 无)
    virtual double nearestDist2(const PointType& query) const = 0;

    /// 局部/体素统计 (VGICP/NDT)
    virtual VoxelStats statsAt(const Eigen::Vector3d& p, double radius) const = 0;

    /// 地面查询 (planar); 无返回 false (默认不支持)
    virtual bool groundAt(double /*x*/, double /*y*/, double& /*z*/, Eigen::Vector3d& /*normal*/) const {
        return false;
    }

    /// 更新 (time <0 表示不记录时间; 供 decay 使用)
    virtual void insert(const CloudType::Ptr& world_points, double time = -1.0) = 0;
    virtual void erase(const CloudType::Ptr& world_points) = 0;
    virtual void carve(const Eigen::Vector3d& /*origin*/, const CloudType::Ptr& /*endpoints*/) {}
    virtual void decay(double /*now*/) {}

    /// 版本号 (缓存失效判断)
    virtual uint64_t revision() const = 0;
    virtual size_t size() const = 0;

    virtual bool save(const std::string& path) const = 0;
    virtual bool load(const std::string& path) = 0;

    /// 全量点云快照（供 MapData 导出 / 通用地图格式转换）
    virtual CloudType::Ptr exportCloud() const { return nullptr; }
};

} // namespace rosiwit_slam
