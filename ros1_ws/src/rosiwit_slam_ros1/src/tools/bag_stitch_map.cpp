// tools/bag_stitch_map.cpp
// 离线拼图: 用 bag 内旧系统轨迹 (/slam/info, 已验证=标准 yaw) 把 /vanjee_points722
// 逐帧拼成稠密地图, 输出 pcd + xzmap + pgm/yaml (+jpg/thumb)。
//
// 为什么不跑在线 SLAM: 该 bag 的窄区段(150s+)会把在线前端匹配带偏(两版代码同段发散),
// 而旧系统自身轨迹在同段稳定(match 0.79~1.0, LOCALIZED) —— 直接用它拼图,
// 产出与旧系统建图等价的地图, 供本系统定位使用。
//
// 单遍流式设计 (2026-11): rosbag 按 chunk(混合话题)压缩, 任何单话题扫描都要解压全包。
// 本工具一遍读完两个话题: 位姿先入内存, 点云入小缓冲, 帧尾时刻被位姿覆盖后立即处理。
//
// 用法:
//   bag_stitch_map <bag> <out_dir> [--map-name NAME] [--voxel 0.15]
//                  [--kf-dist 1.0] [--min-r 0.5] [--max-r 60]
//                  [--mount-rpy 41.67,1.82,89.89] [--mount-xyz 0,0,0.898]
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <xzrobot_ros_dependency/SlamInfo.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "slam_core/slam_types.h"
#include "io/xzmap_writer.h"

using namespace rosiwit_slam;

namespace {

struct Pose2 {
    double t = 0, x = 0, y = 0, z = 0, yaw = 0;
};

inline int64_t voxelKey(double x, double y, double z, double v) {
    const int64_t ix = static_cast<int64_t>(std::floor(x / v)) & 0x1FFFFF;
    const int64_t iy = static_cast<int64_t>(std::floor(y / v)) & 0x1FFFFF;
    const int64_t iz = static_cast<int64_t>(std::floor(z / v)) & 0x1FFFFF;
    return (ix << 42) | (iy << 21) | iz;
}

struct VoxelPt {
    float x, y, z;
    uint16_t cnt;
};

bool interpPose(const std::vector<Pose2>& traj, double t, Pose2& out) {
    if (traj.empty()) return false;
    if (t <= traj.front().t) { out = traj.front(); return true; }
    if (t >= traj.back().t) { out = traj.back(); return true; }
    size_t lo = 0, hi = traj.size() - 1;
    while (hi - lo > 1) {
        const size_t mid = (lo + hi) / 2;
        if (traj[mid].t <= t) lo = mid; else hi = mid;
    }
    const Pose2& a = traj[lo];
    const Pose2& b = traj[hi];
    const double w = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
    out.t = t;
    out.x = a.x + (b.x - a.x) * w;
    out.y = a.y + (b.y - a.y) * w;
    out.z = a.z + (b.z - a.z) * w;
    double dyaw = b.yaw - a.yaw;
    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    while (dyaw < -M_PI) dyaw += 2 * M_PI;
    out.yaw = a.yaw + dyaw * w;
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, NULL, _IOLBF, 0);
    ros::init(argc, argv, "bag_stitch_map", ros::init_options::AnonymousName);
    auto rss_mb = []() -> long {
        std::FILE* f = std::fopen("/proc/self/statm", "r");
        if (!f) return -1;
        long pages = 0, rss = 0;
        if (std::fscanf(f, "%ld %ld", &pages, &rss) != 2) rss = -1;
        std::fclose(f);
        return rss > 0 ? rss * 4 / 1024 : -1;
    };

    if (argc < 3) {
        std::fprintf(stderr,
            "usage: bag_stitch_map <bag> <out_dir> [--map-name N] [--voxel 0.15]\n"
            "       [--kf-dist 1.0] [--min-r 0.5] [--max-r 60]\n"
            "       [--mount-rpy r,p,y_deg] [--mount-xyz x,y,z]\n");
        return 1;
    }
    const std::string bag_path = argv[1];
    const std::string out_dir = argv[2];
    std::string map_name = "stitched";
    double voxel = 0.15, kf_dist = 1.0, min_r = 0.5, max_r = 60.0;
    double mr = 41.67, mp = 1.82, my = 89.89;
    double mx = 0.0, myz = 0.0, mzz = 0.898;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--map-name" && i + 1 < argc) map_name = argv[++i];
        else if (a == "--voxel" && i + 1 < argc) voxel = std::atof(argv[++i]);
        else if (a == "--kf-dist" && i + 1 < argc) kf_dist = std::atof(argv[++i]);
        else if (a == "--min-r" && i + 1 < argc) min_r = std::atof(argv[++i]);
        else if (a == "--max-r" && i + 1 < argc) max_r = std::atof(argv[++i]);
        else if (a == "--mount-rpy" && i + 1 < argc)
            std::sscanf(argv[++i], "%lf,%lf,%lf", &mr, &mp, &my);
        else if (a == "--mount-xyz" && i + 1 < argc)
            std::sscanf(argv[++i], "%lf,%lf,%lf", &mx, &myz, &mzz);
    }

    const Eigen::Matrix3d R_bl =
        (Eigen::AngleAxisd(my * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(mp * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(mr * M_PI / 180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d t_bl(mx, myz, mzz);

    rosbag::Bag bag;
    try { bag.open(bag_path, rosbag::bagmode::Read); }
    catch (const std::exception& e) { std::fprintf(stderr, "open bag failed: %s\n", e.what()); return 1; }

    std::vector<Pose2> traj;
    traj.reserve(40000);
    std::deque<sensor_msgs::PointCloud2> pending;   // 点云小缓冲
    std::unordered_map<int64_t, VoxelPt> vox;
    vox.reserve(4000000);
    std::vector<io::XzMapKeyframe> kfs;
    Eigen::Vector3d last_kf_pos(1e9, 0, 0);
    long long n_frames = 0, n_pts_in = 0, n_pts_kept = 0;
    const double min2 = min_r * min_r, max2 = max_r * max_r;

    auto process_cloud = [&](const sensor_msgs::PointCloud2& msg) {
        ++n_frames;
        const double t_frame = msg.header.stamp.toSec();
        bool has_ts = false;
        for (const auto& f : msg.fields)
            if (f.name == "timestamp" && f.datatype == sensor_msgs::PointField::FLOAT64) { has_ts = true; break; }

        Pose2 pb;
        const bool have_pb = interpPose(traj, t_frame, pb);
        bool make_kf = false;
        Eigen::Vector3d pb_pos(pb.x, pb.y, 0);
        if (have_pb && (pb_pos - last_kf_pos).norm() >= kf_dist) make_kf = true;
        io::XzMapKeyframe kf;
        std::unordered_map<int64_t, uint8_t> kf_vox;

        const size_t n = static_cast<size_t>(msg.width) * msg.height;
        sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
        std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<double>> it_t;
        if (has_ts) it_t.reset(new sensor_msgs::PointCloud2ConstIterator<double>(msg, "timestamp"));

        Pose2 pp;
        for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
            if (it_t) ++(*it_t);
            const float x = *it_x, y = *it_y, z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            const double d2 = double(x) * x + double(y) * y + double(z) * z;
            if (d2 < min2 || d2 > max2) continue;
            ++n_pts_in;
            const double t_i = t_frame + (it_t ? **it_t : 0.0);
            if (!interpPose(traj, t_i, pp)) continue;
            const Eigen::Matrix3d R_mb =
                Eigen::AngleAxisd(pp.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const Eigen::Vector3d p_base = R_bl * Eigen::Vector3d(x, y, z) + t_bl;
            const Eigen::Vector3d p_map = R_mb * p_base + Eigen::Vector3d(pp.x, pp.y, pp.z);
            ++n_pts_kept;
            const int64_t k = voxelKey(p_map.x(), p_map.y(), p_map.z(), voxel);
            auto ins = vox.emplace(k, VoxelPt{float(p_map.x()), float(p_map.y()), float(p_map.z()), 1});
            if (!ins.second && ins.first->second.cnt < 65535) ++ins.first->second.cnt;
            if (make_kf) {
                const int64_t kk = voxelKey(x, y, z, 0.2);
                if (kf_vox.emplace(kk, 1).second)
                    kf.points.push_back(Eigen::Vector3f(x, y, z));
            }
        }
        if (make_kf && !kf.points.empty() && have_pb) {
            kf.pose.x = pb.x; kf.pose.y = pb.y; kf.pose.z = pb.z;
            kf.pose.roll = 0.0; kf.pose.pitch = 0.0; kf.pose.yaw = pb.yaw;
            kfs.push_back(std::move(kf));
            last_kf_pos = pb_pos;
        }
        if (n_frames % 2000 == 0)
            std::printf("[stitch] frames=%lld kept=%lld voxels=%zu kfs=%zu rss=%ldMB\n",
                        n_frames, n_pts_kept, vox.size(), kfs.size(), rss_mb());
    };

    auto flush = [&](bool force) {
        while (!pending.empty()) {
            const double end_t = pending.front().header.stamp.toSec() + 0.105;
            if (!force && (traj.empty() || traj.back().t < end_t)) break;
            process_cloud(pending.front());
            pending.pop_front();
        }
    };

    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{"/slam/info", "/vanjee_points722"}));
    size_t n_msgs = 0;
    for (const rosbag::MessageInstance& m : view) {
        ++n_msgs;
        const std::string& topic = m.getTopic();
        if (topic == "/slam/info") {
            auto mi = m.instantiate<xzrobot_ros_dependency::SlamInfo>();
            if (mi) {
                Pose2 p;
                p.t = m.getTime().toSec();
                p.x = mi->robot_pose.x;
                p.y = mi->robot_pose.y;
                p.yaw = mi->robot_pose.theta;   // 已验证=标准 yaw (与 TF 合成对拍)
                traj.push_back(p);
                flush(false);
            }
        } else if (topic == "/vanjee_points722") {
            auto msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (msg) {
                pending.push_back(*msg);
                flush(false);
                if (pending.size() > 60) flush(true);   // 位姿断流保护 (~6s)
            }
        }
        if (n_msgs % 50000 == 0)
            std::printf("[stitch] progress: msgs=%zu traj=%zu pending=%zu frames=%lld rss=%ldMB\n",
                        n_msgs, traj.size(), pending.size(), n_frames, rss_mb());
    }
    flush(true);
    bag.close();
    std::printf("[stitch] done: msgs=%zu frames=%lld in=%lld kept=%lld voxels=%zu kfs=%zu rss=%ldMB\n",
                n_msgs, n_frames, n_pts_in, n_pts_kept, vox.size(), kfs.size(), rss_mb());
    if (vox.empty()) return 1;

    std::string cmd = "mkdir -p '" + out_dir + "'";
    if (std::system(cmd.c_str()) != 0) { std::fprintf(stderr, "mkdir failed\n"); return 1; }
    {
        CloudType::Ptr cloud(new CloudType());
        cloud->reserve(vox.size());
        double zmin = 1e9, zmax = -1e9;
        for (const auto& kv : vox) {
            PointType p;
            p.x = kv.second.x; p.y = kv.second.y; p.z = kv.second.z;
            cloud->push_back(p);
            zmin = std::min(zmin, double(p.z)); zmax = std::max(zmax, double(p.z));
        }
        cloud->width = static_cast<uint32_t>(cloud->size());
        cloud->height = 1;
        const std::string pcd = out_dir + "/map.pcd";
        pcl::io::savePCDFileBinary(pcd, *cloud);
        std::printf("[stitch] pcd: %s (%zu pts, z=%.2f..%.2f)\n", pcd.c_str(), cloud->size(), zmin, zmax);
    }
    {
        long long total = 0;
        const std::string xz = out_dir + "/map.xzmap";
        const bool ok = io::writeXzMap(xz, map_name, kfs, &total);
        std::printf("[stitch] xzmap: %s ok=%d kfs=%zu pts=%lld\n", xz.c_str(), ok ? 1 : 0, kfs.size(), total);
    }
    {
        const double res = 0.05;
        double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
        double zmin = 1e18;
        for (const auto& kv : vox) {
            xmin = std::min(xmin, double(kv.second.x)); xmax = std::max(xmax, double(kv.second.x));
            ymin = std::min(ymin, double(kv.second.y)); ymax = std::max(ymax, double(kv.second.y));
            zmin = std::min(zmin, double(kv.second.z));
        }
        std::map<int, int> hist;
        for (const auto& kv : vox)
            ++hist[static_cast<int>(std::floor((kv.second.z - zmin) / 0.01))];
        int best_bin = 0, best_cnt = -1;
        for (const auto& h : hist) if (h.second > best_cnt) { best_cnt = h.second; best_bin = h.first; }
        const double z_ground = zmin + (best_bin + 0.5) * 0.01;
        std::printf("[stitch] ground z=%.2f (hist peak %d)\n", z_ground, best_cnt);

        const int W = static_cast<int>(std::ceil((xmax - xmin) / res)) + 1;
        const int H = static_cast<int>(std::ceil((ymax - ymin) / res)) + 1;
        std::vector<int8_t> occ(static_cast<size_t>(W) * H, 0);
        for (const auto& kv : vox) {
            const double dz = kv.second.z - z_ground;
            if (dz < 0.1 || dz > 2.0) continue;
            const int ix = static_cast<int>((kv.second.x - xmin) / res);
            const int iy = static_cast<int>((kv.second.y - ymin) / res);
            if (ix < 0 || ix >= W || iy < 0 || iy >= H) continue;
            occ[static_cast<size_t>(iy) * W + ix] = 100;
        }
        cv::Mat img(H, W, CV_8UC1, cv::Scalar(254));
        long long occ_cnt = 0;
        for (int iy = 0; iy < H; ++iy) {
            uint8_t* row = img.ptr<uint8_t>(H - 1 - iy);
            for (int ix = 0; ix < W; ++ix)
                if (occ[static_cast<size_t>(iy) * W + ix]) { row[ix] = 0; ++occ_cnt; }
        }
        cv::imwrite(out_dir + "/map.pgm", img);
        cv::Mat thumb;
        cv::resize(img, thumb, cv::Size(0, 0), 0.2, 0.2);
        if (thumb.cols < 1) thumb = img.clone();
        cv::imwrite(out_dir + "/thumb.png", thumb);
        cv::imwrite(out_dir + "/map.jpg", img);
        std::FILE* y = std::fopen((out_dir + "/map.yaml").c_str(), "w");
        if (y) {
            std::fprintf(y,
                "image: map.pgm\nresolution: %.3f\norigin: [%.6f, %.6f, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
                res, xmin, ymin);
            std::fclose(y);
        }
        std::printf("[stitch] grid: %dx%d res=%.2f area=%.0fm2 occupied=%lld (%.1f%%)\n",
                    W, H, res, (xmax - xmin) * (ymax - ymin), occ_cnt,
                    100.0 * occ_cnt / (double(W) * H));
        std::printf("MAP_DONE %s\n", out_dir.c_str());
    }
    return 0;
}
