// ============================================================================
// core_gateway.cpp — 与核心的唯一边界
//
// 单一职责: 所有"碰核心内部对象"的调用都从这里走, 并把锁语义收在一处:
//   * 输入热路径用 core_mu_; 结构操作 (重建/装载/改模式/导出) 用 core_excl_mu_;
//   * lane 投递 (runOnLane / runOnLaneWithDone / runOnLaneAndWait);
//   * 只读访问 (global map / grid / pose cov / matching ...) 的持锁版本。
// 调用方不应再直接使用 algo() 裸指针做结构操作。
// ============================================================================
#include "slam_algorithm.h"

#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include "slam_core/diagnostics.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "slam_core/slam_factory.h"
#include "slam_core/slam_pipeline.h"

namespace rosiwit_slam_ros1 {

using namespace rosiwit_slam;

namespace {
/// 结构锁的统一取法: wait_ms<0 阻塞; >=0 最多等这么久, 拿不到就放弃
bool lockExcl(std::timed_mutex& mu, int wait_ms, const char* what)
{
    if (wait_ms < 0) { mu.lock(); return true; }
    if (wait_ms == 0) {
        if (mu.try_lock()) return true;
    } else if (mu.try_lock_for(std::chrono::milliseconds(wait_ms))) {
        return true;
    }
    ROS_WARN("[slam_algo] core busy, '%s' skipped (structural op in progress)", what);
    return false;
}
}  // namespace

bool SlamAlgorithm::runOnLane(
    const std::string& lane, std::string name,
    std::function<void(rosiwit_common::CancelToken)> job)
{
    if (!executor_) {
        ROS_WARN("[slam_algo] runOnLane('%s'): no executor injected", lane.c_str());
        return false;
    }
    rosiwit_common::JobOptions opt;
    opt.name = name;
    const rosiwit_common::JobId id =
        executor_->submit(lane, std::move(opt), std::move(job),
                          rosiwit_common::DoneFn());
    if (id == rosiwit_common::kInvalidJobId) {
        const auto st = executor_->stats();
        ROS_WARN("[slam_algo] runOnLane('%s', '%s') rejected: lane missing/full/stopping "
                 "(queued=%zu rejected=%lu)",
                 lane.c_str(), name.c_str(), st.queued,
                 static_cast<unsigned long>(st.rejected));
        return false;
    }
    return true;
}

bool SlamAlgorithm::runOnLaneWithDone(
    const std::string& lane, std::string name,
    std::function<void(rosiwit_common::CancelToken)> job,
    std::function<void(bool ok)> done)
{
    if (!executor_) {
        ROS_WARN("[slam_algo] runOnLaneWithDone('%s'): no executor injected", lane.c_str());
        if (done) done(false);
        return false;
    }
    rosiwit_common::JobOptions opt;
    opt.name = name;
    const rosiwit_common::JobId id = executor_->submit(
        lane, std::move(opt), std::move(job),
        [done = std::move(done)](rosiwit_common::JobId, bool ok) {
            if (done) done(ok);
        });
    if (id == rosiwit_common::kInvalidJobId) {
        const auto st = executor_->stats();
        ROS_WARN("[slam_algo] runOnLaneWithDone('%s','%s') rejected (queued=%zu rejected=%lu)",
                 lane.c_str(), name.c_str(), st.queued,
                 static_cast<unsigned long>(st.rejected));
        if (done) done(false);
        return false;
    }
    return true;
}

bool SlamAlgorithm::runOnLaneAndWait(
    const std::string& lane, std::string name,
    std::function<void(rosiwit_common::CancelToken)> job, int timeout_ms)
{
    if (!executor_) {
        ROS_WARN("[slam_algo] runOnLaneAndWait('%s'): no executor injected", lane.c_str());
        return false;
    }
    struct State {
        std::promise<bool> done;
        std::atomic<bool> body_ok{false};
    };
    auto st = std::make_shared<State>();
    auto fut = st->done.get_future();

    rosiwit_common::JobOptions opt;
    opt.name = name;
    const rosiwit_common::JobId id = executor_->submit(
        lane, std::move(opt),
        [st, job = std::move(job)](rosiwit_common::CancelToken token) {
            job(token);
            st->body_ok.store(true);
        },
        [st](rosiwit_common::JobId, bool ok) {
            // 执行完成/被取消/被丢弃 都会进这里, 保证等待方一定被唤醒
            try { st->done.set_value(ok); } catch (...) {}
        });
    if (id == rosiwit_common::kInvalidJobId) {
        const auto stats = executor_->stats();
        ROS_WARN("[slam_algo] runOnLaneAndWait('%s','%s') rejected (queued=%zu rejected=%lu)",
                 lane.c_str(), name.c_str(), stats.queued,
                 static_cast<unsigned long>(stats.rejected));
        return false;
    }
    if (fut.wait_for(std::chrono::milliseconds(timeout_ms)) != std::future_status::ready) {
        ROS_ERROR("[slam_algo] runOnLaneAndWait('%s','%s') timeout after %d ms",
                  lane.c_str(), name.c_str(), timeout_ms);
        return false;
    }
    return fut.get() && st->body_ok.load();
}

bool SlamAlgorithm::loadLocalizationMapData(const rosiwit_slam::MapData& data,
                                            uint64_t expect_generation) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);   // 允许等 (装载本来就是慢操作)
    std::lock_guard<std::mutex> lk(core_mu_);
    if (!algo_raw_) return false;
    if (expect_generation != core_generation_.load()) {
        ROS_WARN("[slam_algo] discard map load: core was reset during parse (gen %lu -> %lu)",
                 static_cast<unsigned long>(expect_generation),
                 static_cast<unsigned long>(core_generation_.load()));
        return false;
    }
    return algo_raw_->loadLocalizationMapData(data);
}

bool SlamAlgorithm::setCorePipelineMode(const std::string& mode, int wait_ms) {
    if (!lockExcl(core_excl_mu_, wait_ms, "setPipelineMode")) return false;
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_, std::adopt_lock);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->setPipelineMode(mode);
}

void SlamAlgorithm::cancelCoreMapLoad() {
    std::lock_guard<std::mutex> lk(core_mu_);
    if (algo_raw_) algo_raw_->cancelMapLoad();
}

bool SlamAlgorithm::exportCoreMapData(rosiwit_slam::MapData& out) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->exportMapData(out);
}

bool SlamAlgorithm::getCoreGridMap(double resolution, rosiwit_slam::GridInfo& out) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->getRealtimeGrid(resolution, out);
}

bool SlamAlgorithm::saveCoreGridMap(const std::string& pgm, const std::string& yaml,
                                    double resolution) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->saveGridMap(pgm, yaml, resolution);
}

bool SlamAlgorithm::setCoreGridMapGenerator(const std::string& name) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->setGridMapGenerator(name);
}

bool SlamAlgorithm::appendCoreMap(const std::string& path) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->appendMap(path);
}

bool SlamAlgorithm::loadCoreMap(const std::string& path) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->loadMap(path);
}

void SlamAlgorithm::requestRelocalization() {
    if (algo_raw_) algo_raw_->requestRelocalization();
}

std::string SlamAlgorithm::dumpFlightRecorder(const std::string& reason) {
    return diag::FlightRecorder::instance().dump(reason);
}

bool SlamAlgorithm::getGlobalMap(PointVec& out) const {
    if (!algo_raw_) return false;
    return algo_raw_->getGlobalMap(out);
}

bool SlamAlgorithm::getGridInfo(GridInfo& out) const {
    if (!algo_raw_) return false;
    out = algo_raw_->getGridInfo();
    return true;
}

bool SlamAlgorithm::getCoreGlobalMap(PointVec& out) {
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_);
    std::lock_guard<std::mutex> lk(core_mu_);
    return algo_raw_ && algo_raw_->getGlobalMap(out);
}

bool SlamAlgorithm::getRealtimeGrid(double resolution, GridInfo& out) {
    if (!algo_raw_) return false;
    return algo_raw_->getRealtimeGrid(resolution, out);
}

bool SlamAlgorithm::getPoseCov(PoseStamped& out, Eigen::Matrix<double, 6, 6>& cov) const {
    if (!algo_raw_) return false;
    return algo_raw_->getPoseCov(out, cov);
}

double SlamAlgorithm::getMatchingDegree() const {
    return algo_raw_ ? algo_raw_->getMatchingDegree() : 0.0;
}

bool SlamAlgorithm::resetCore() {
    // 重建会替换核心内部对象: 必须与"装载地图/导出整图"互斥。
    // 200ms 拿不到就明确失败 (调用方是 ROS 服务线程, 不能在这里挂住 spinner)。
    if (!lockExcl(core_excl_mu_, 200, "resetCore")) return false;
    std::lock_guard<std::timed_mutex> excl(core_excl_mu_, std::adopt_lock);
    std::lock_guard<std::mutex> core_lk(core_mu_);
    if (!algo_raw_) return false;
    ++core_generation_;   // 让"解析中"的地图装载作废
    // 先复位 latch/快照类状态, 免得重建后还带着上一轮的位姿
    {
        std::lock_guard<std::mutex> lk(out_mutex_);
        latest_ = SlamOutput();
        have_output_ = false;
    }
    // 旧实现切模式时会删掉建图/定位实例; 我们重建核心模块达到同样效果。
    const bool ok = algo_raw_->resetForNewMapping(config_path_);
    std::cout << "[slam_algo] resetCore -> " << (ok ? "ok" : "unsupported/failed") << std::endl;
    return ok;
}

std::string SlamAlgorithm::pipelineMode() const {
    return algo_raw_ ? algo_raw_->getPipelineMode() : std::string();
}

rosiwit_slam::ILocalization::Status SlamAlgorithm::localizationStatus() const {
    return algo_raw_ ? algo_raw_->localizationStatus() : rosiwit_slam::ILocalization::LOST;
}

bool SlamAlgorithm::priorUncertainty(double& xy_sigma, double& yaw_sigma) const {
    return algo_raw_ ? algo_raw_->priorUncertainty(xy_sigma, yaw_sigma) : false;
}

} // namespace rosiwit_slam_ros1
