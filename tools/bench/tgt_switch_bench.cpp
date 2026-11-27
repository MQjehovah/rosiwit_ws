// tgt_switch_bench: 目标切换的代价落在哪个线程 —— FastGICP 在新目标上的第一次 align 比之后慢多少。
//   tgt_switch_bench <target.pcd> <scan.pcd> [threads=4]
// 复现 GicpMatcher 的做法: setTarget (降采样 + KD 树) 在前, setInputTarget 在第一次 align 时才调。
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <cstdio>
#include <string>

using P = pcl::PointXYZINormal;
using Cloud = pcl::PointCloud<P>;

static double ms_since(const std::chrono::steady_clock::time_point& t) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

int main(int argc, char** argv) {
    const int th = argc > 3 ? std::atoi(argv[3]) : 4;
    Cloud::Ptr tgt(new Cloud()), scan(new Cloud());
    pcl::io::loadPCDFile(argv[1], *tgt);
    pcl::io::loadPCDFile(argv[2], *scan);
    // 匹配器预处理: 0.3m 体素 + 去地面 (URDF 安装角)
    pcl::VoxelGrid<P> vg; vg.setLeafSize(0.3f, 0.3f, 0.3f); vg.setInputCloud(scan);
    Cloud::Ptr src(new Cloud()); vg.filter(*src);
    {
        const Eigen::Matrix3d R = (Eigen::AngleAxisd(1.569, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.020, Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(0.687, Eigen::Vector3d::UnitX())).toRotationMatrix();
        Cloud::Ptr kept(new Cloud());
        for (const auto& p : src->points) {
            const double z = R(2, 0) * p.x + R(2, 1) * p.y + R(2, 2) * p.z + 0.914;
            if (z > 0.15) kept->push_back(p);
        }
        src = kept;
    }
    std::printf("目标 %zu 点, 源 %zu 点, %d 线程\n", tgt->size(), src->size(), th);
    double t_ds = 0, t_kd = 0, t_a1 = 0, t_a2 = 0, t_a3 = 0;
    int it1 = 0, it2 = 0;
    {
        auto t0 = std::chrono::steady_clock::now();
        fast_gicp::FastGICP<P, P> g;
        g.setNumThreads(th); g.setMaximumIterations(12); g.setMaxCorrespondenceDistance(1.2); g.setTransformationEpsilon(5e-3);
        // setTarget: 降采样 (目标已降过采样, 这里原样) -> setInputCloud(KD 树)
        auto t1 = std::chrono::steady_clock::now(); t_ds = ms_since(t0);
        Cloud::Ptr down = tgt;
        pcl::KdTreeFLANN<P> kd; kd.setInputCloud(down);
        auto t2 = std::chrono::steady_clock::now(); t_kd = ms_since(t1);
        // align: setInputTarget (这里才进 PCL/GICP) + 第一次 align
        g.setInputSource(src); g.setInputTarget(down);
        auto t3 = std::chrono::steady_clock::now();
        Cloud out; Eigen::Matrix4f guess = Eigen::Matrix4f::Identity(); guess(0, 3) = 15.08f; guess(1, 3) = -2.49f; guess(2, 3) = 0.98f;
        g.align(out, guess);
        t_a1 = ms_since(t3); it1 = g.performedIterations();
        auto t4 = std::chrono::steady_clock::now();
        g.align(out, guess);
        t_a2 = ms_since(t4); it2 = g.performedIterations();
        auto t5 = std::chrono::steady_clock::now();
        g.align(out, guess);
        t_a3 = ms_since(t5);
        std::printf("setTarget 里的降采样: %.1f ms; KD 树 setInputCloud: %.1f ms\n", t_ds, t_kd);
        std::printf("第一次 align (含 setInputTarget + 目标协方差懒计算): %.1f ms (%d 轮)\n", t_a1, it1);
        std::printf("之后同一目标: %.1f ms (%d 轮), %.1f ms\n", t_a2, it2, t_a3);
        std::printf("=> 每次换目标, 跟踪线程第一帧多付 %.1f ms\n", t_a1 - t_a2);
    }
    return 0;
}
