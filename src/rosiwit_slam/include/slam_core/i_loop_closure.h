#pragma once
#include "slam_core/pipeline_types.h"

namespace rosiwit_slam {

class ILoopClosure {
public:
    virtual ~ILoopClosure() = default;
    virtual bool init(const std::string& config_path) = 0;
    virtual void addKeyFrame(const KeyFrame& kf) = 0;
    virtual bool detect(PoseStamped& relative_pose, double& cov,
                        std::string& matched_kf_id) = 0;
    virtual bool saveDatabase(const std::string& path) { (void)path; return false; }

    // 导出全部关键帧 (按添加顺序, 含描述子生成失败的帧), 供地图格式导出等用途
    virtual bool getKeyframes(std::vector<KeyFrame>& out) { (void)out; return false; }
    /// 后端优化后的位姿回写。后端只把修正结果交给 map_manager 重建点云; 而导出 xzmap
    /// 用的是 loop closure 里的关键帧位姿 (入队时的原始前端位姿)。不回写就会出现
    /// "map.pcd 是修正后的, 而 xzmap 里是回环前的漂移位姿", 重新载入后底图对不上。
    virtual void updatePoses(const std::vector<PoseStamped>& /*corrected*/) {}
};

} // namespace rosiwit_slam
