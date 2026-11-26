// test/test_working_map_save.cpp
// 工作图落盘 (2026-11-26 根因: 每小时整点丢失) 的端到端测试。
//
// 落盘被拆成两个线程: 落盘线程 (降采样 / 压缩 / 合并 / 写盘) 与跟踪线程 (在帧预算内分片
// 查账本写 w + overlay 快照), 两个线程按阶段交接 cloud 指针。本测试走真实管线:
//   1) 定期落盘到期 -> 请求 -> 落盘线程降采样 -> 交接 -> 跟踪线程分片标注 -> 回交 -> 写盘;
//   2) 写出的工作图内容正确: 底图体素 (0.3m, 每体素一点) + intensity = w;
//   3) 管线析构时落盘线程被正常收掉 (不崩不挂)。
// 首次 setMap 会同步生成工作图文件 —— 先删掉它, 再验证它经由定期落盘路径重新出现。
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <thread>
#include <vector>

#include "algorithms/localization/localization_pipeline.h"

using namespace rosiwit_slam;

namespace {

PointType mkPt(float x, float y, float z)
{
    // 本机 PCL 1.8 的 PointXYZINormal 只有默认 / 拷贝构造, 逐字段赋值
    PointType q;
    q.x = x; q.y = y; q.z = z;
    q.normal_x = q.normal_y = q.normal_z = 0.0f;
    q.curvature = 0.0f;
    q.intensity = 0.0f;
    return q;
}

// 8m x 8m 房间: 地面 + 四面墙 (与 test_localization_pipeline 的 makeScene 同构)。
// "每帧只推进有界的一片 / 跨多次调用做完"由 test_overlay_map_maintainer 在 260 万条账本上
// 直接测 annotateWithinBudget; 这里不靠把场景堆大来复现 (那会让断言依赖机器速度)。
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

// 机体坐标系扫描 = T_true^{-1} * map
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
    const std::string base = "/tmp/wm_save_test_XXXXXX";
    std::vector<char> buf(base.begin(), base.end());
    buf.push_back('\0');
    const char* d = ::mkdtemp(buf.data());
    return d ? std::string(d) : std::string("/tmp");
}

// 与管线 voxelDownsample 同一口径的体素计数 (独立重写, 用来核对落盘结果)
size_t countVoxels(const CloudType::Ptr& cloud, float voxel)
{
    std::set<uint64_t> seen;
    const float inv = 1.0f / std::max(0.05f, voxel);
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int32_t ix = static_cast<int32_t>(std::floor(p.x * inv));
        const int32_t iy = static_cast<int32_t>(std::floor(p.y * inv));
        const int32_t iz = static_cast<int32_t>(std::floor(p.z * inv));
        seen.insert((static_cast<uint64_t>(static_cast<uint32_t>(ix)) * 73856093ull) ^
                    (static_cast<uint64_t>(static_cast<uint32_t>(iy)) * 19349663ull) ^
                    (static_cast<uint64_t>(static_cast<uint32_t>(iz)) * 83492791ull));
    }
    return seen.size();
}

}  // namespace

TEST(WorkingMapSaveTest, PeriodicSaveRewritesWorkingMapViaSlicedAnnotation)
{
    const std::string dir = makeTempDir();
    const std::string pcd = dir + "/wm_map.pcd";
    const std::string work = dir + "/wm_map.overlay.pcd";
    CloudType::Ptr map = makeRoom();
    ASSERT_EQ(pcl::io::savePCDFileBinary(pcd, *map), 0);
    const size_t n_vox = countVoxels(map, 0.3f);
    ASSERT_GT(n_vox, 1000u);   // 8m 房间 @0.3m: 地面 729 + 四面墙 ~620

    const std::string cfg = dir + "/cfg.yaml";
    {
        std::ofstream f(cfg);
        f << "localization:\n"
             "  pipeline:\n"
             "    relocalizer: none\n"
             "    motion_prior: constant_velocity\n"
             "    local_map: radius_submap\n"   // 走裁剪建目标路径 (global 分支假定匹配器自带全图 store)
             "    matcher: pp_icp\n"
             "    monitor: threshold\n"
             "    map_maintainer: overlay\n"
             "    working_map_save_min: 0.0001\n"     // 周期取整为 0s: 首个被接受的帧即到期
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
             "    base_voxel_w: 128\n";
    }

    int localized_frames = 0;
    bool written = false;
    double max_frame_ms = 0.0;
    {
        LocalizationPipeline lp;
        ASSERT_TRUE(lp.init(cfg));
        lp.setMap(pcd);                              // 首次载入: 同步生成工作图
        {
            std::ifstream probe(work, std::ios::binary);
            ASSERT_TRUE(probe.good()) << "首次载入应生成工作图 " << work;
        }
        ASSERT_EQ(std::remove(work.c_str()), 0);

        lp.setInitPose(mkPose(3.0, 3.0, 0.0));
        for (int i = 0; i < 600 && !written; ++i) {
            const PoseStamped truth = mkPose(3.0 + 0.01 * i, 3.0, i * 0.1 + 0.1);
            LidarFrame f;
            f.cloud = scanFromMap(map, truth);
            f.start_time = i * 0.1;
            f.end_time = i * 0.1 + 0.1;
            const auto t0 = std::chrono::steady_clock::now();
            lp.onLidar(f);
            max_frame_ms = std::max(max_frame_ms,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
            localized_frames += (lp.getStatus() == ILocalization::LOCALIZED) ? 1 : 0;
            std::ifstream probe(work, std::ios::binary);
            written = probe.good();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        // lp 析构: 停止并 join 落盘线程 (在这里挂住 / 崩掉 = 失败)
    }
    // 循环在文件出现时即退出, 这里只要求确实有过被接受的帧 (落盘的前提)
    EXPECT_GE(localized_frames, 1) << "合成序列没有进入 LOCALIZED, 落盘条件不成立";
    ASSERT_TRUE(written) << "定期落盘没有写出工作图 " << work;

    CloudType::Ptr got(new CloudType());
    ASSERT_EQ(pcl::io::loadPCDFile(work, *got), 0) << "工作图读不回 (写了一半?)";
    // rename 是原子的: 文件存在即完整。内容 = 底图体素 (+ 若有 overlay 则更多), intensity = w
    EXPECT_GE(got->size(), n_vox) << "工作图点数少于底图体素数";
    EXPECT_LT(got->size(), n_vox + map->size()) << "工作图不应是整幅原始底图";
    int bad = 0;
    for (const PointType& p : got->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !(p.intensity >= 1.0f)) ++bad;
    }
    EXPECT_EQ(bad, 0) << "工作图里有非有限点或 w<=0 的点";
    std::cout << "[wm_save] 底图 " << map->size() << " pts / " << n_vox << " 体素 -> 工作图 "
              << got->size() << " pts; localized_frames=" << localized_frames
              << " max_frame_ms=" << max_frame_ms << std::endl;

    std::remove(work.c_str());
    std::remove(pcd.c_str());
    std::remove(cfg.c_str());
    ::rmdir(dir.c_str());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
