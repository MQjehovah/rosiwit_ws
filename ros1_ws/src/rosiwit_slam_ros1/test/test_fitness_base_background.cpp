// test/test_fitness_base_background.cpp
// fitness_base (只对底图的支撑, 桥接退出/健康判据) 后台探测的契约 (2026-11-29):
//   1. 注入执行器 (生产路径: 线程池的 "pool" 通道)   -> 进入 LOCALIZED 后 1Hz 投递,
//      调试 JSON 里的 fitness_base 从 -1 变成合理值, 且是在**后台线程**上算的;
//   2. 不注入执行器 (兼容路径: 后台线程)             -> 同样的结果;
//   3. 管线析构时在途/待处理的探测被安全收掉 (不挂、不崩);
//   4. 值是"对底图做一次 GICP 后的覆盖率", OPEN 态与 fitness 同参考, 应接近 1。
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>
#include <vector>

#include "algorithms/localization/localization_pipeline.h"
#include "algorithms/localization/registration_metrics.h"
#include "rosiwit_common/thread_pool.hpp"

using namespace rosiwit_slam;

namespace {

PointType mkPt(float x, float y, float z)
{
    PointType q;
    q.x = x; q.y = y; q.z = z;
    q.normal_x = q.normal_y = q.normal_z = 0.0f;
    q.curvature = 0.0f;
    q.intensity = 0.0f;
    return q;
}

// 与 test_working_map_save 同一个 8m x 8m 房间: 地面 + 四面墙
CloudType::Ptr makeRoom()
{
    CloudType::Ptr c(new CloudType());
    const double S = 8.0;
    for (double x = 0; x <= S; x += 0.1)
        for (double y = 0; y <= S; y += 0.1)
            c->push_back(mkPt(static_cast<float>(x), static_cast<float>(y), 0.0f));
    for (double t = 0; t <= S; t += 0.05)
        for (double z = 0.2; z <= 2.0; z += 0.1) {
            c->push_back(mkPt(static_cast<float>(t), 0.0f, static_cast<float>(z)));
            c->push_back(mkPt(static_cast<float>(t), static_cast<float>(S), static_cast<float>(z)));
            c->push_back(mkPt(0.0f, static_cast<float>(t), static_cast<float>(z)));
            c->push_back(mkPt(static_cast<float>(S), static_cast<float>(t), static_cast<float>(z)));
        }
    c->width = static_cast<uint32_t>(c->size());
    c->height = 1;
    return c;
}

PoseStamped mkPose(double x, double y, double t)
{
    PoseStamped p;
    p.time = t;
    p.trans = V3D(x, y, 0.0);
    p.rot = M3D::Identity();
    return p;
}

CloudType::Ptr scanFromMap(const CloudType::Ptr& map, const PoseStamped& T_true)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = T_true.rot.cast<float>();
    T.block<3, 1>(0, 3) = T_true.trans.cast<float>();
    CloudType::Ptr scan(new CloudType());
    pcl::transformPointCloud(*map, *scan, T.inverse());
    return scan;
}

std::string makeTempDir()
{
    const std::string base = "/tmp/fitness_base_test_XXXXXX";
    std::vector<char> buf(base.begin(), base.end());
    buf.push_back('\0');
    const char* d = ::mkdtemp(buf.data());
    return d ? std::string(d) : std::string("/tmp");
}

// 调试 JSON 里 "fitness_base":<值> 的数值
bool fitnessBaseFromJson(const std::string& js, double& out)
{
    const size_t k = js.find("\"fitness_base\":");
    if (k == std::string::npos) return false;
    out = std::atof(js.c_str() + k + 15);
    return true;
}

const char* kConfig =
    "localization:\n"
    "  pipeline:\n"
    "    relocalizer: none\n"
    "    motion_prior: constant_velocity\n"
    "    local_map: radius_submap\n"
    "    matcher: pp_icp\n"
    "    monitor: threshold\n"
    "    map_maintainer: overlay\n"
    "  matcher:\n"
    "    voxel_resolution: 0.2\n"
    "    max_iterations: 50\n"
    "    corr_dist_init: 1.5\n"
    "    corr_dist_min: 0.5\n"
    "    degeneracy_gate: false\n"
    "  accept:\n"
    "    mode: 0\n"
    "    min_coverage: 0.4\n"
    "  monitor:\n"
    "    credible_fitness: 0.6\n"
    "    lost_fitness: 0.4\n"
    "    lost_patience: 30\n"
    "  maintainer:\n"
    "    voxel: 0.3\n"
    "    base_voxel_w: 128\n"
    "    shadow: true\n";

struct Fixture {
    std::string dir, pcd, cfg;
    CloudType::Ptr map = makeRoom();
    Fixture() : dir(makeTempDir()), pcd(dir + "/map.pcd"), cfg(dir + "/cfg.yaml") {
        pcl::io::savePCDFileBinary(pcd, *map);
        std::ofstream f(cfg);
        f << kConfig;
    }
    ~Fixture() {
        std::remove(pcd.c_str());
        std::remove(cfg.c_str());
        std::remove((dir + "/map.overlay.pcd").c_str());
        ::rmdir(dir.c_str());
    }
};

// 喂 8 秒的帧 (真值位姿), 返回: 进入 LOCALIZED 的帧数, 以及喂帧期间单帧的最大墙钟
void feed(LocalizationPipeline& lp, const Fixture& fx, int n, int* localized, double* max_ms)
{
    for (int i = 0; i < n; ++i) {
        const PoseStamped truth = mkPose(3.0 + 0.005 * i, 3.0, i * 0.1 + 0.1);
        LidarFrame f;
        f.cloud = scanFromMap(fx.map, truth);
        f.start_time = i * 0.1;
        f.end_time = i * 0.1 + 0.1;
        const auto t0 = std::chrono::steady_clock::now();
        lp.onLidar(f);
        if (max_ms) *max_ms = std::max(*max_ms, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        if (localized && lp.getStatus() == ILocalization::LOCALIZED) ++*localized;
    }
}

// 等后台把 fitness_base 算出来 (最多 5s 墙钟)
bool waitFitnessBase(LocalizationPipeline& lp, double& out)
{
    for (int i = 0; i < 250; ++i) {
        if (fitnessBaseFromJson(lp.debugStatsJson(), out) && out >= 0.0) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

}  // namespace

TEST(FitnessBaseProbe, ComputedOnExecutorPool)
{
    Fixture fx;
    rosiwit_common::ThreadPool pool({{"pool", 1, 16, rosiwit_common::LaneOverflow::kReject}});
    double cov = -1.0;
    int localized = 0;
    {
        LocalizationPipeline lp;
        ASSERT_TRUE(lp.init(fx.cfg));
        lp.setExecutor(&pool);
        lp.setMap(fx.pcd);
        lp.setInitPose(mkPose(3.0, 3.0, 0.0));
        feed(lp, fx, 80, &localized, nullptr);   // 8 秒: LOCALIZED 后每 1s 投递一次
        ASSERT_TRUE(waitFitnessBase(lp, cov)) << "8 秒后 fitness_base 仍是 -1 (没有被算出来): " << lp.debugStatsJson();
    }
    pool.shutdown(true);
    EXPECT_GE(localized, 10) << "合成序列没有稳定进入 LOCALIZED";
    EXPECT_GE(cov, 0.9) << "在真值位姿上对工作图的覆盖率应接近 1, 实际 " << cov;
    std::cout << "[fitness_base] 执行器路径: fitness_base=" << cov << std::endl;
}

TEST(FitnessBaseProbe, ComputedOnCompatThreadWithoutExecutor)
{
    Fixture fx;
    double cov = -1.0;
    int localized = 0;
    {
        LocalizationPipeline lp;
        ASSERT_TRUE(lp.init(fx.cfg));
        lp.setMap(fx.pcd);
        lp.setInitPose(mkPose(3.0, 3.0, 0.0));
        feed(lp, fx, 80, &localized, nullptr);
        ASSERT_TRUE(waitFitnessBase(lp, cov)) << "8 秒后 fitness_base 仍是 -1: " << lp.debugStatsJson();
    }
    EXPECT_GE(cov, 0.9) << "兼容路径: 覆盖率应接近 1, 实际 " << cov;
    std::cout << "[fitness_base] 兼容路径: fitness_base=" << cov << std::endl;
}

TEST(FitnessBaseProbe, DestructorIsSafeWhileJobInFlight)
{
    Fixture fx;
    rosiwit_common::ThreadPool pool({{"pool", 1, 16, rosiwit_common::LaneOverflow::kReject}});
    {
        LocalizationPipeline lp;
        ASSERT_TRUE(lp.init(fx.cfg));
        lp.setExecutor(&pool);
        lp.setMap(fx.pcd);
        lp.setInitPose(mkPose(3.0, 3.0, 0.0));
        int localized = 0;
        feed(lp, fx, 52, &localized, nullptr);   // 任务多半还在途 -> 立刻析构
    }                                             // 析构: 等在途的 fitness_base 结束 (挂住 / 崩掉 = 失败)
    pool.shutdown(true);
    SUCCEED() << "在途 fitness_base 任务时析构正常";
}

TEST(FitnessBaseProbe, NotComputedWhileNeverLocalized)
{
    Fixture fx;
    double cov = -2.0;
    {
        LocalizationPipeline lp;
        ASSERT_TRUE(lp.init(fx.cfg));
        lp.setMap(fx.pcd);
        // 不给初始位姿, 喂的是离地图很远的扫描: 永远不会 LOCALIZED -> 不应投递, fitness_base 保持 -1
        for (int i = 0; i < 80; ++i) {
            LidarFrame f;
            f.cloud.reset(new CloudType());
            for (int k = 0; k < 400; ++k) f.cloud->push_back(mkPt(20.0f + 0.05f * k, 40.0f, 1.0f));
            f.start_time = i * 0.1;
            f.end_time = i * 0.1 + 0.1;
            lp.onLidar(f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        cov = -2.0;
        ASSERT_TRUE(fitnessBaseFromJson(lp.debugStatsJson(), cov));
    }
    EXPECT_LT(cov, 0.0) << "从未定位时不该有 fitness_base, 实际 " << cov;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
