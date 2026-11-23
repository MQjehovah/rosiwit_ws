// src/algorithms/localization/constraints/planar_constraint.cpp
#include "algorithms/localization/constraints/planar_constraint.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace rosiwit_slam {

int64_t PlanarConstraint::key(int cx, int cy) {
    return (static_cast<int64_t>(cx) << 32) ^ (static_cast<unsigned int>(cy));
}

void PlanarConstraint::setMap(const CloudType::Ptr& base_map) {
    m_cells.clear();
    m_have_sensor_h = false;
    if (!base_map || base_map->empty()) return;
    m_cell = m_cfg.cell > 0 ? m_cfg.cell : 1.0;

    // 1) 每格最低 z
    std::unordered_map<int64_t, double> minz;
    minz.reserve(base_map->size());
    for (const auto& p : base_map->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int cx = static_cast<int>(std::floor(p.x / m_cell));
        const int cy = static_cast<int>(std::floor(p.y / m_cell));
        const int64_t k = key(cx, cy);
        auto it = minz.find(k);
        if (it == minz.end()) minz[k] = p.z;
        else if (p.z < it->second) it->second = p.z;
    }

    // 2) 收集每格地面带点, 求均值 z + 法向(PCA 最小特征向量)
    struct Acc { Eigen::Vector3d sum = Eigen::Vector3d::Zero(); Eigen::Matrix3d sq = Eigen::Matrix3d::Zero(); int n = 0; };
    std::unordered_map<int64_t, Acc> acc;
    const double band = m_cfg.ground_band > 0 ? m_cfg.ground_band : 0.3;
    for (const auto& p : base_map->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const int cx = static_cast<int>(std::floor(p.x / m_cell));
        const int cy = static_cast<int>(std::floor(p.y / m_cell));
        const int64_t k = key(cx, cy);
        auto mit = minz.find(k);
        if (mit == minz.end() || p.z > mit->second + band) continue;
        Acc& a = acc[k];
        const Eigen::Vector3d v(p.x, p.y, p.z);
        a.sum += v;
        a.sq += v * v.transpose();
        ++a.n;
    }

    for (auto& kv : acc) {
        Cell c;
        if (kv.second.n < 3) { c.ok = false; m_cells[kv.first] = c; continue; }
        const Eigen::Vector3d mean = kv.second.sum / kv.second.n;
        Eigen::Matrix3d cov = kv.second.sq / kv.second.n - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Vector3d n = es.eigenvectors().col(0);
        if (n.z() < 0) n = -n;               // 朝上
        c.z = mean.z();
        c.n = n;
        c.ok = (n.z() > std::cos(m_cfg.max_tilt_deg * M_PI / 180.0));
        m_cells[kv.first] = c;
    }
}

const PlanarConstraint::Cell* PlanarConstraint::query(double x, double y) const {
    const int cx = static_cast<int>(std::floor(x / m_cell));
    const int cy = static_cast<int>(std::floor(y / m_cell));
    // 就近搜索 3x3
    const Cell* best = nullptr;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy) {
            auto it = m_cells.find(key(cx + dx, cy + dy));
            if (it == m_cells.end() || !it->second.ok) continue;
            if (!best) best = &it->second;
        }
    return best;
}

void PlanarConstraint::apply(PoseStamped& pose, const CloudType::Ptr& scan) {
    {
        // 启动后打 3 条: 确认到底调没调进来、mode 是什么 (现场排查用)
        static int once = 0;
        if (once < 3) {
            ++once;
            std::cout << "[planar] enter mode=" << m_cfg.mode
                      << " scan=" << (scan ? scan->size() : 0)
                      << " max_tilt=" << m_cfg.max_tilt_deg
                      << " max_correction=" << m_cfg.max_correction_deg << std::endl;
        }
    }
    if (m_cfg.mode == "none") return;

    if (m_cfg.mode == "lock_zrp") {
        // 由核心 LocalizationPipeline::planarizePose() 每帧执行:
        // 先扣安装角, 锁 body 的 z/roll/pitch, 再贴回安装角 (不再在这里抹掉安装角).
        return;
    }

    if (m_cfg.mode == "scan_plane") {
        applyScanPlane(pose, scan);
        return;
    }

    if (m_cfg.mode != "ground_plane") return;

    double gz = 0.0;
    Eigen::Vector3d gn = Eigen::Vector3d::UnitZ();
    bool have = false;
    if (m_store) {
        if (m_store->groundAt(pose.trans.x(), pose.trans.y(), gz, gn)) {
            if (gn.z() < std::cos(m_cfg.max_tilt_deg * M_PI / 180.0)) return;  // 倾角超限
            have = true;
        }
    }
    if (!have) {
        const Cell* c = query(pose.trans.x(), pose.trans.y());
        if (!c) return;
        gz = c->z; gn = c->n; have = true;
    }

    if (!m_have_sensor_h) {
        m_sensor_h = m_cfg.sensor_height > 0 ? m_cfg.sensor_height
                                             : (pose.trans.z() - gz);  // 首帧标定
        m_have_sensor_h = true;
    }
    pose.trans.z() = gz + m_sensor_h;

    // roll/pitch: 把机体上轴对齐到地面法向 (平地=竖直 -> 锁平; 坡道=法向倾斜 -> 跟随)
    const Eigen::Vector3d up = pose.rot * V3D::UnitZ();
    Eigen::Quaterniond qa;
    qa.setFromTwoVectors(up, gn);
    pose.rot = qa.toRotationMatrix() * pose.rot;
}

void PlanarConstraint::applyScanPlane(PoseStamped& pose, const CloudType::Ptr& scan) {
    // 诊断: 每 10 次一条, 所有返回路径都打 why=..., 现场不用再猜卡在哪一步
    float zlo = 0.0f, zhi = 0.0f;
    int cf = 0, cc = 0, floor_sel = 0;
    double up_z = 0.0, nm_z = 0.0, ang_deg = 0.0;
    double gz = 0.0, F = 0.0, dz = 0.0;
    bool have_g = false;
    auto report = [&](const char* why) {
        static int n = 0;
        if (++n % 10 != 0) return;
        std::cout.setf(std::ios::fixed);
        std::cout.precision(3);
        std::cout << "[planar] why=" << why
                  << " n=" << (scan ? scan->size() : 0)
                  << " zlo=" << zlo << " zhi=" << zhi
                  << " cf=" << cf << " cc=" << cc << " floor=" << floor_sel
                  << " up_z=" << up_z << " nm_z=" << nm_z << " ang=" << ang_deg
                  << " rpy=" << (std::atan2(pose.rot(2, 1), pose.rot(2, 2)) * 57.2957795)
                  << "," << (std::asin(std::min(1.0, std::max(-1.0, -pose.rot(2, 0)))) * 57.2957795)
                  << " gz=" << gz << " have_g=" << (have_g ? 1 : 0)
                  << " F=" << F << " cur=" << pose.trans.z() << " dz=" << dz
                  << std::endl;
        std::cout.unsetf(std::ios::fixed);
    };

    if (!scan || scan->size() < 200) { report("no_scan"); return; }

    // 按 z 分位取地板/天花板带 (小房间里天花板通常比地板点更多)
    std::vector<float> zs;
    zs.reserve(scan->size());
    for (const auto& p : scan->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        zs.push_back(p.z);
    }
    if (zs.size() < 200) { report("few_z"); return; }
    const size_t lo_i = zs.size() / 20;             // 5%
    const size_t hi_i = zs.size() * 19 / 20;        // 95%
    std::nth_element(zs.begin(), zs.begin() + lo_i, zs.end());
    zlo = zs[lo_i];
    std::nth_element(zs.begin(), zs.begin() + hi_i, zs.end());
    zhi = zs[hi_i];

    auto fitBand = [&](double lo, double hi, Eigen::Vector3d& n, int& cnt,
                       Eigen::Vector3d& centroid) {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d sq = Eigen::Matrix3d::Zero();
        cnt = 0;
        for (const auto& p : scan->points) {
            if (!std::isfinite(p.z) || p.z < lo || p.z > hi) continue;
            const Eigen::Vector3d v(p.x, p.y, p.z);
            sum += v;
            sq += v * v.transpose();
            ++cnt;
        }
        if (cnt < 50) return false;
        const Eigen::Vector3d mean = sum / static_cast<double>(cnt);
        Eigen::Matrix3d cov = sq / static_cast<double>(cnt) - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        if (es.info() != Eigen::Success) return false;
        n = es.eigenvectors().col(0);
        if (n.z() < 0) n = -n;   // 统一朝上
        centroid = mean;
        return true;
    };

    Eigen::Vector3d nf, nc;
    Eigen::Vector3d meanf = Eigen::Vector3d::Zero(), meanc = Eigen::Vector3d::Zero();
    const bool okf = fitBand(zlo - 0.2, zlo + 0.35, nf, cf, meanf);
    const bool okc = fitBand(zhi - 0.35, zhi + 0.2, nc, cc, meanc);
    Eigen::Vector3d up;
    const bool use_floor = okf && (!okc || cf + 100 >= cc);   // 地板更可靠时优先
    if (okc && !use_floor) up = nc;
    else if (okf) up = nf;
    else { report("no_band"); return; }
    floor_sel = use_floor ? 1 : 0;

    const double cos_tilt = std::cos(m_cfg.max_tilt_deg * M_PI / 180.0);
    up_z = up.z();
    // 只做"它是不是一个水平面"的粗筛(墙面法向≈0), 不用 35° 那么严:
    // 雷达俯装 39° 时, 车体系里的地面法向本来就偏离 z 轴 ~39°(up_z≈0.78),
    // 真正"是否竖直"由 map 系的 nm_z 判断。
    if (up_z < 0.5) { report("band_tilt"); return; }

    // 位姿把它转到 map 系后, 还应该保持竖直
    const Eigen::Vector3d n_m = pose.rot * up;
    nm_z = n_m.z();
    if (nm_z < cos_tilt) { report("map_tilt"); return; }
    const double ang = std::acos(std::min(1.0, std::max(-1.0, nm_z)));
    ang_deg = ang * 180.0 / M_PI;
    if (ang_deg > m_cfg.max_correction_deg) { report("jump"); return; }   // 跳变: 不信
    if (ang >= 1e-4) {
        Eigen::Vector3d axis = n_m.cross(Eigen::Vector3d::UnitZ());
        const double s = axis.norm();
        if (s < 1e-9) { report("axis0"); return; }
        axis /= s;
        pose.rot = Eigen::AngleAxisd(ang, axis).toRotationMatrix() * pose.rot;
    }

    // z: 只要"地板可靠"就修, 与"转动用哪个面"无关 (小房间天花板常比地面点多)
    if (!okf || cf < 50 || nf.z() < 0.5) { report("z_no_floor"); return; }
    if (m_store) {
        Eigen::Vector3d gn;
        if (m_store->groundAt(pose.trans.x(), pose.trans.y(), gz, gn)) have_g = true;
    }
    if (!have_g) {
        const Cell* c = query(pose.trans.x(), pose.trans.y());
        if (c) { gz = c->z; have_g = true; }
    }
    if (!have_g) { report("z_no_ground"); return; }

    // 把地板平面转到 map 系, 求它在机器人 (x,y) 处的高度 F。
    // 用 F 与地图地面 gz 对齐, 不需要"传感器离地高"这种假设量(实机该量会随
    // 安装/外参变化, 且符号极易搞反): 点云地板落到地图地面上, z 就是对的。
    const Eigen::Vector3d c_m = pose.rot * meanf + pose.trans;
    const Eigen::Vector3d n_m_f = pose.rot * nf;
    if (n_m_f.z() < cos_tilt) { report("z_floor_tilt"); return; }   // 地板在 map 系也应水平
    F = c_m.z() - (n_m_f.x() * (pose.trans.x() - c_m.x()) +
                   n_m_f.y() * (pose.trans.y() - c_m.y())) / n_m_f.z();

    dz = gz - F;
    if (!std::isfinite(dz) || std::fabs(dz) > 5.0) { report("z_jump"); return; }   // 数据异常, 别把车甩飞
    const double max_dz = 0.3;
    if (dz > max_dz) dz = max_dz;
    if (dz < -max_dz) dz = -max_dz;
    report("z_apply");
    pose.trans.z() += dz;
}

bool PlanarConstraint::estimateGroundDz(const PoseStamped& pose, const CloudType& scan,
                                        double& dz, double& z_floor_body) const {
    if (scan.size() < 200) return false;

    std::vector<float> zs;
    zs.reserve(scan.size());
    for (const auto& p : scan.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        zs.push_back(p.z);
    }
    if (zs.size() < 200) return false;
    const size_t lo_i = zs.size() / 20;   // 5%
    std::nth_element(zs.begin(), zs.begin() + lo_i, zs.end());
    const float zlo = zs[lo_i];

    // 地面带: 最低 5% 附近, PCA 取法向; 必须本身是水平面(排除墙/货架)
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sq = Eigen::Matrix3d::Zero();
    int cnt = 0;
    for (const auto& p : scan.points) {
        if (!std::isfinite(p.z) || p.z < zlo - 0.2 || p.z > zlo + 0.35) continue;
        const Eigen::Vector3d v(p.x, p.y, p.z);
        sum += v;
        sq += v * v.transpose();
        ++cnt;
    }
    if (cnt < 50) return false;
    const Eigen::Vector3d mean = sum / static_cast<double>(cnt);
    Eigen::Matrix3d cov = sq / static_cast<double>(cnt) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    if (es.info() != Eigen::Success) return false;
    Eigen::Vector3d n = es.eigenvectors().col(0);
    if (n.z() < 0) n = -n;
    if (n.z() < 0.5) return false;
    z_floor_body = mean.z() - (n.x() * mean.x() + n.y() * mean.y()) / n.z();

    // 地面平面在 map 系、机器人 (x,y) 处的高度
    const Eigen::Vector3d c_m = pose.rot * mean + pose.trans;
    const Eigen::Vector3d n_m = pose.rot * n;
    if (n_m.z() < 0.5) return false;
    const double F = c_m.z() - (n_m.x() * (pose.trans.x() - c_m.x()) +
                                n_m.y() * (pose.trans.y() - c_m.y())) / n_m.z();

    // 地图地面 (优先统一底座, 退回本类的地面模型)
    double gz = 0.0;
    bool have = false;
    if (m_store) {
        Eigen::Vector3d gn;
        if (m_store->groundAt(pose.trans.x(), pose.trans.y(), gz, gn)) have = true;
    }
    if (!have) {
        const Cell* c = query(pose.trans.x(), pose.trans.y());
        if (c) { gz = c->z; have = true; }
    }
    if (!have) return false;

    dz = gz - F;
    if (!std::isfinite(dz) || std::fabs(dz) > 5.0) return false;
    return true;
}

} // namespace rosiwit_slam