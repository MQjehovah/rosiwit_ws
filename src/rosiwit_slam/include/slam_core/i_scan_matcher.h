// include/slam_core/i_scan_matcher.h
#pragma once
#include <string>
#include "slam_core/localization_stage_types.h"
#include "slam_core/i_map_store.h"

namespace rosiwit_slam {

/// 扫描-地图配准器接口.
/// 由 LocalizationFactory 按名创建; 输入已降采样扫描 + 目标局部地图 + 初始猜测,
/// 输出统一口径的 MatchResult (fitness 恒为覆盖率[0,1], 便于跨算法比较).
class IScanMatcher {
public:
    virtual ~IScanMatcher() = default;

    virtual std::string name() const = 0;

    /// 读取配置 (config["localization"]["matcher"]); 默认无操作
    virtual bool init(const std::string& /*config_path*/) { return true; }

    /// 设置目标(局部)地图, 内部可建 KD-tree
    virtual void setTarget(const CloudType::Ptr& target) = 0;

    /// 设置统一存储底座 (可选): 支持时匹配器直接从 store 取近邻/统计, 与存储解耦
    virtual void setStore(IMapStore* /*store*/) {}
    virtual bool usesStore() const { return false; }

    /// 以 guess 为初值配准 scan → out
    virtual bool match(const CloudType::Ptr& scan,
                       const PoseStamped& guess,
                       MatchResult& out) = 0;

    /// 在给定位姿下评估点到面平均残差(m); 不适用返回 -1 (供 s2m 残差接受判据)
    virtual double evaluateResidual(const CloudType::Ptr& /*scan*/,
                                    const PoseStamped& /*pose*/) { return -1.0; }

    /// 直接注入配置 (离线评估/测试用), 默认无操作
    virtual void configure(const ScanMatcherConfig& /*cfg*/) {}
};

} // namespace rosiwit_slam
