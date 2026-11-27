// test/test_working_map_save.cpp
// 工作图落盘 (2026-11-26 根因: 每小时整点丢失) 的端到端测试。
//
// 落盘被拆成两个线程: 落盘线程 (降采样 / 压缩 / 合并 / 写盘) 与跟踪线程 (在帧预算内分片
// 查账本写 w + overlay 快照), 两个线程按阶段交接 cloud 指针。本测试走真实管线:
//   1) 定期落盘到期 -> 请求 -> 落盘线程降采样 -> 交接 -> 跟踪线程分片标注 -> 回交 -> 写盘;
//   2) 写出的工作图内容正确: 底图体素 (0.3m, 每体素一点) + intensity = w;
//   3) 管线析构时落盘线程被正常收掉 (不崩不挂);
//   4) **连续**多次落盘 (2026-11-27 06:47 实机崩溃): 落盘线程第一次落盘后把 unique_lock 解开了却
//      没重新锁, 第二次请求一唤醒就在 lk.unlock() 上抛 system_error -> std::terminate -> 节点 abort。
//      旧测试在第一个文件出现时就退出, 从没触发过第二次。这里用两次"新结构提交"(账本版本号变化,
//      定期落盘的触发条件之一) 逼出第二、第三次落盘; 旧实现下本测试进程直接 SIGABRT。
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

// 地图上没有的柱子 (0.3m 体素下一列约 6 个体素, 满足最小簇 3): 被足够多的视角看到后账本提交它。
CloudType::Ptr makePillar(double x, double y)
{
    CloudType::Ptr c(new CloudType());
    for (double z = 0.2; z <= 1.8; z += 0.05)
        c->push_back(mkPt(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
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
             "    base_voxel_w: 128\n"
             // 默认 shadow=true: 只观察不写, onLocalized 不会被调用 (也就没有新结构提交)。
             // 关掉它才会走完整的账本写入路径 —— 第二、三次落盘要靠新结构提交来触发。
             "    shadow: false\n"
             // 按"帧"计票: view 单位下"连续被看到"不重复计票, 而柱子的有无会影响写图门的放行节奏,
             // 候选可能每次调用都在 —— 票永远涨不到 3。这里只关心"提交 -> 版本号变 -> 再次落盘"。
             "    persistency_unit: 0\n";
    }

    int localized_frames = 0;
    // 新结构: A 从第 10 帧起、B 从第 150 帧起出现在**隔帧**里 —— view 计票单位下连续帧不重复计票,
    // 隔一帧出现一次才算被"再次看到", 3 票 (persist_hits) 后提交 -> revision 变 -> 定期落盘到期。
    const CloudType::Ptr pillarA = makePillar(6.5, 6.5);
    const CloudType::Ptr pillarB = makePillar(1.5, 6.5);
    int saves = 0;
    size_t last_pts = 0;
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
        for (int i = 0; i < 700 && saves < 3; ++i) {
            const PoseStamped truth = mkPose(3.0 + 0.01 * i, 3.0, i * 0.1 + 0.1);
            LidarFrame f;
            f.cloud = scanFromMap(map, truth);
            if (i % 2 == 0) {
                if (i >= 10) *f.cloud += *scanFromMap(pillarA, truth);
                if (i >= 150) *f.cloud += *scanFromMap(pillarB, truth);
            }
            f.start_time = i * 0.1;
            f.end_time = i * 0.1 + 0.1;
            const auto t0 = std::chrono::steady_clock::now();
            lp.onLidar(f);
            max_frame_ms = std::max(max_frame_ms,
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
            localized_frames += (lp.getStatus() == ILocalization::LOCALIZED) ? 1 : 0;
            std::ifstream probe(work, std::ios::binary);
            if (probe.good()) {
                // rename 是原子的: 文件存在即完整。读一遍内容, 再删掉, 等下一次落盘把它重新写出来。
                probe.close();
                CloudType::Ptr got(new CloudType());
                ASSERT_EQ(pcl::io::loadPCDFile(work, *got), 0) << "第 " << saves + 1 << " 次落盘: 工作图读不回";
                int bad = 0;
                for (const PointType& p : got->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                        !(p.intensity >= 1.0f)) ++bad;
                }
                EXPECT_EQ(bad, 0) << "第 " << saves + 1 << " 次落盘: 工作图里有非有限点或 w<=0 的点";
                // 内容 = 底图体素 (+ 若有 overlay 则更多)
                EXPECT_GE(got->size(), n_vox) << "第 " << saves + 1 << " 次落盘: 点数少于底图体素数";
                EXPECT_LT(got->size(), n_vox + map->size()) << "第 " << saves + 1 << " 次落盘: 写成了整幅原始底图";
                std::cout << "[wm_save] 第 " << saves + 1 << " 次落盘 (帧 " << i << "): 底图 " << map->size()
                          << " pts / " << n_vox << " 体素 -> 工作图 " << got->size() << " pts" << std::endl;
                last_pts = got->size();
                ++saves;
                ASSERT_EQ(std::remove(work.c_str()), 0);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        // lp 析构: 停止并 join 落盘线程 (在这里挂住 / 崩掉 = 失败)
    }
    // 第一回合同样验证过的性质: 确实有过被接受的帧 (落盘的前提)
    EXPECT_GE(localized_frames, 1) << "合成序列没有进入 LOCALIZED, 落盘条件不成立";
    // ★第二次落盘就是实机崩溃的那一步 (旧实现下本进程在上面就 abort 了, 走不到这条断言)
    EXPECT_GE(saves, 3) << "连续落盘只完成了 " << saves << " 次 (需要 3 次: 第 2 次走的是崩溃的那条路径)";
    EXPECT_GT(last_pts, n_vox) << "提交的新结构没有进工作图";
    std::cout << "[wm_save] localized_frames=" << localized_frames << " max_frame_ms=" << max_frame_ms << std::endl;

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
