// src/algorithms/point_lio/point_lio_frontend.cpp
// Point-LIO 风格逐点处理前端 — 实现
//
// 处理流程 (每帧):
//   1. 对新帧中的每个点:
//      a. IESKF 传播到该点时间戳 (IMU 内插)
//      b. 变换到 world 系
//      c. iKD-Tree 搜索近邻
//      d. 平面拟合 → 点到面残差 → IESKF 单点更新
//      e. 加入 iKD-Tree (降采样后)
//   2. 输出当前帧位姿

#include "algorithms/point_lio/point_lio_frontend.h"
#include "algorithms/fast_lio2/map_builder/lidar_processor.h"
#include "algorithms/fast_lio2/map_builder/imu_processor.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace rosiwit_slam {

bool PointLioFrontend::init(const std::string& config_path) {
    if (!::parseFastLio2Config(config_path, m_config)) {
        return false;
    }

    m_map_resolution = m_config.map_resolution;
    m_scan_resolution = m_config.scan_resolution;
    m_det_range = m_config.det_range;
    m_cube_len = m_config.cube_len;
    m_move_thresh = m_config.move_thresh;

    // 初始化 IESKF
    m_kf = std::make_shared<::IESKF>();
    m_kf->setMaxIter(static_cast<size_t>(m_config.ieskf_max_iter));

    // 初始化 iKD-Tree
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_map_resolution);

    // IMU 噪声协方差
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng;
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na;
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg;
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;

    m_inited = true;
    std::cout << "[PointLioFrontend] Initialized (per-point processing)" << std::endl;
    return true;
}

void PointLioFrontend::onImu(const IMUSample& s) {
    if (!m_inited) return;
    double t = s.time;
    m_imu_buffer.emplace_back(s.acc, s.gyro, t);
    m_last_imu_time = s.time;
}

double PointLioFrontend::pointTimeFromCloud(const PointType& pt, double cloud_start_time) const {
    // FAST-LIO2 惯例: curvature 字段编码了点的时间偏移 (ms)
    return cloud_start_time + pt.curvature / 1000.0;
}

void PointLioFrontend::propagateToTime(double target_time) {
    if (m_imu_buffer.size() < 2) return;

    // 丢弃过时的 IMU 数据
    while (m_imu_buffer.size() > 1 && m_imu_buffer[1].time < target_time) {
        m_last_imu = m_imu_buffer.front();
        m_imu_buffer.pop_front();
    }

    if (m_imu_buffer.size() < 2) return;

    // 在两个 IMU 之间线性内插, 传播 IESKF
    const auto& head = m_imu_buffer.front();
    double current_t = m_last_imu.time;

    while (current_t < target_time && m_imu_buffer.size() >= 2) {
        const auto& next = m_imu_buffer[1];
        double dt = next.time - current_t;
        if (dt <= 0) { dt = 0.001; }

        // 中值积分: 用 head 和 next 的平均值
        ::Input inp;
        inp.acc  = 0.5 * (head.acc + next.acc);
        inp.gyro = 0.5 * (head.gyro + next.gyro);

        m_kf->predict(inp, dt, m_Q);

        current_t = next.time;
        m_last_imu = next;
        m_imu_buffer.pop_front();
    }

    // 如果还有剩余时间, 用最后一个 IMU 传播
    double dt = target_time - current_t;
    if (dt > 0.001) {
        const auto& last = m_imu_buffer.front();
        ::Input inp;
        inp.acc  = last.acc;
        inp.gyro = last.gyro;
        m_kf->predict(inp, dt, m_Q);
    }

    m_last_imu_time = target_time;
}

bool PointLioFrontend::processPoint(const PointType& pt, double point_time) {
    // 1. IESKF 传播到该点时间
    propagateToTime(point_time);

    // 2. 变换到 world 系
    const auto& x = m_kf->x();
    Eigen::Vector3d p_body(pt.x, pt.y, pt.z);
    Eigen::Vector3d p_world = x.r_wi * (x.r_il * p_body + x.t_il) + x.t_wi;

    PointType world_pt;
    world_pt.x = p_world.x(); world_pt.y = p_world.y(); world_pt.z = p_world.z();
    world_pt.intensity = pt.intensity;

    // 3. 搜索近邻
    int near_num = m_config.near_search_num;
    std::vector<float> point_sq_dist(near_num);
    PointVec points_near;
    m_ikdtree->Nearest_Search(world_pt, near_num, points_near, point_sq_dist);

    if (points_near.size() < static_cast<size_t>(near_num)) {
        // 近邻不足以拟合平面, 直接加入地图
        PointVec to_add = {world_pt};
        m_ikdtree->Add_Points(to_add, true);
        return true;
    }

    // 4. 拟合平面
    V4D plane_coeffs;
    if (!::esti_plane(points_near, 0.1, plane_coeffs)) {
        // 平面拟合失败, 跳过该点
        return false;
    }

    // 5. 点到面距离作为残差
    double pd2 = plane_coeffs(0) * p_world.x() +
                 plane_coeffs(1) * p_world.y() +
                 plane_coeffs(2) * p_world.z() +
                 plane_coeffs(3);

    // 判断是否为有效约束
    double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(p_body.norm());
    if (s <= 0.9) {
        return false;
    }

    // 6. 构建单点雅可比并更新 IESKF
    // 这里的更新使用 batch 方式 (IESKF::update() 会对所有点迭代)
    // 实际 Point-LIO 是渐进式更新, 但我们把残差存起来
    // 由于 IESKF 的接口是批处理 (updateLossFunc), 我们在这里直接做一阶更新
    //
    // 简化方案: 用迭代的 ESKF 进行单点更新
    // H 矩阵 12x12, 残差 1x1
    Eigen::Vector3d norm_vec(plane_coeffs(0), plane_coeffs(1), plane_coeffs(2));

    // 构建单点雅可比 J (1×12)
    Eigen::Matrix<double, 1, 12> J;
    J.setZero();

    // J 对旋转角度: -n^T * R^T * [R_il * p + t_il]×
    Eigen::Vector3d tmp = x.r_il * p_body + x.t_il;
    J.block<1, 3>(0, 0) = -norm_vec.transpose() * x.r_wi * Sophus::SO3d::hat(tmp);
    // J 对位移: n^T
    J.block<1, 3>(0, 3) = norm_vec.transpose();

    if (m_config.esti_il) {
        Eigen::Vector3d C_val = -norm_vec.transpose() * x.r_wi * x.r_il * Sophus::SO3d::hat(p_body);
        Eigen::Vector3d D_val = norm_vec.transpose() * x.r_wi;
        J.block<1, 3>(0, 6) = C_val;
        J.block<1, 3>(0, 9) = D_val;
    }

    // 协方差
    double lidar_cov = 1.0 / m_config.lidar_cov_inv;

    // 卡尔曼增益: K = P * H^T * (H * P * H^T + R)^{-1},  H = J * H_bar (1×21)
    Eigen::Matrix<double, 21, 1> K;
    {
        Eigen::Matrix<double, 1, 21> H_full = Eigen::Matrix<double, 1, 21>::Zero();
        H_full.block<1, 12>(0, 0) = J;

        double S = (H_full * m_kf->P() * H_full.transpose())(0, 0) + lidar_cov;
        K = m_kf->P() * H_full.transpose() / S;
    }

    // 状态更新
    V21D delta_x = K * pd2;
    m_kf->x() += delta_x;

    // 协方差更新: P = (I - K * H_full) * P, 其中 H_full = J * H_bar (1×21)
    Eigen::Matrix<double, 1, 21> H_full = Eigen::Matrix<double, 1, 21>::Zero();
    H_full.block<1, 12>(0, 0) = J;
    m_kf->P() = (M21D::Identity() - K * H_full) * m_kf->P();

    // 7. 降采样后加入地图
    // 体素滤波: 如果该点所在体素已有近邻, 跳过
    PointType downsample_check;
    downsample_check.x = std::floor(world_pt.x / m_map_resolution) * m_map_resolution + 0.5 * m_map_resolution;
    downsample_check.y = std::floor(world_pt.y / m_map_resolution) * m_map_resolution + 0.5 * m_map_resolution;
    downsample_check.z = std::floor(world_pt.z / m_map_resolution) * m_map_resolution + 0.5 * m_map_resolution;

    bool need_add = true;
    for (size_t ni = 0; ni < points_near.size(); ++ni) {
        if (std::abs(points_near[ni].x - downsample_check.x) < 0.5 * m_map_resolution &&
            std::abs(points_near[ni].y - downsample_check.y) < 0.5 * m_map_resolution &&
            std::abs(points_near[ni].z - downsample_check.z) < 0.5 * m_map_resolution) {
            need_add = false;
            break;
        }
    }

    if (need_add) {
        PointVec to_add = {world_pt};
        m_ikdtree->Add_Points(to_add, true);
    }

    return true;
}

void PointLioFrontend::onLidar(const LidarFrame& f) {
    if (!m_inited || !f.cloud || f.cloud->empty()) return;

    // 首帧: 初始化 IESKF, 将所有点加入地图
    if (m_first_scan) {
        // 初始化重力对齐
        if (m_imu_buffer.size() >= static_cast<size_t>(m_config.imu_init_num)) {
            // 计算平均加速度/角速度
            V3D acc_mean = V3D::Zero();
            V3D gyro_mean = V3D::Zero();
            size_t count = 0;
            for (const auto& imu : m_imu_buffer) {
                acc_mean += imu.acc;
                gyro_mean += imu.gyro;
                ++count;
                if (count >= static_cast<size_t>(m_config.imu_init_num)) break;
            }
            acc_mean /= count;
            gyro_mean /= count;

            m_kf->x().r_il = m_config.r_il;
            m_kf->x().t_il = m_config.t_il;
            m_kf->x().bg = gyro_mean;

            if (m_config.gravity_align) {
                m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors(
                    (-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
                m_kf->x().initGravityDir(V3D(0, 0, -1.0));
            } else {
                m_kf->x().initGravityDir(-acc_mean);
            }

            m_kf->P().setIdentity();
            m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
            m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
            m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
            m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
        }

        // 首帧点加入地图
        PointVec points_to_init;
        points_to_init.reserve(f.cloud->size());
        for (const auto& pt : f.cloud->points) {
            // 按距离过滤
            double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (r < m_config.lidar_min_range || r > m_config.lidar_max_range) continue;

            PointType wp;
            wp.x = pt.x; wp.y = pt.y; wp.z = pt.z;
            wp.intensity = pt.intensity;
            points_to_init.push_back(wp);
        }

        if (!points_to_init.empty()) {
            m_ikdtree->Build(points_to_init);
        }

        m_first_scan = false;
        m_current_pose.rot = m_kf->x().r_wi;
        m_current_pose.trans = m_kf->x().t_wi;
        m_current_pose.time = f.end_time;
        m_last_lidar_time = f.end_time;
        m_current_cloud = f.cloud;
        return;
    }

    // === 正常帧: 逐点处理 ===
    double cloud_start = f.start_time;
    int processed = 0;
    int total = 0;
    double skip = 1;  // 降采样步长 (可配置)

    // 对帧内的每个点逐点处理
    // 实际 Point-LIO 按点到达顺序实时处理, 这里模拟为逐点遍历
    for (const auto& pt : f.cloud->points) {
        double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (r < m_config.lidar_min_range || r > m_config.lidar_max_range) continue;

        ++total;
        if (total % static_cast<int>(m_config.lidar_filter_num) != 0) continue;

        double pt_time = pointTimeFromCloud(pt, cloud_start);
        if (processPoint(pt, pt_time)) {
            ++processed;
        }
    }

    // 更新当前位姿
    m_current_pose.rot = m_kf->x().r_wi;
    m_current_pose.trans = m_kf->x().t_wi;
    m_current_pose.time = f.end_time;
    m_last_lidar_time = f.end_time;
    m_current_cloud = f.cloud;

    // 清理 IMU 缓冲区: 保留最近的一些数据
    while (m_imu_buffer.size() > 100) {
        m_imu_buffer.pop_front();
    }
}

bool PointLioFrontend::getOdometry(PoseStamped& out) {
    if (!m_inited || m_first_scan) return false;
    out = m_current_pose;
    return true;
}

bool PointLioFrontend::getClouds(CloudType::Ptr& body, CloudType::Ptr& world) {
    if (!m_inited || !m_current_cloud) return false;

    const auto& x = m_kf->x();
    body = LidarProcessor::transformCloud(m_current_cloud, x.r_il, x.t_il);

    // world 变换: r_wl = r_wi * r_il, t_wl = t_wi + r_wi * t_il
    M3D r_wl = x.r_wi * x.r_il;
    V3D t_wl = x.t_wi + x.r_wi * x.t_il;
    world = LidarProcessor::transformCloud(m_current_cloud, r_wl, t_wl);
    return true;
}

bool PointLioFrontend::getKeyFrame(KeyFrame& out) {
    if (!m_inited || !m_current_cloud) return false;

    const auto& x = m_kf->x();

    // 关键帧判断: 位移/旋转阈值
    if (m_has_last_kf) {
        double dt = (x.t_wi - m_last_kf_pose.trans).norm();
        double drot = Sophus::SO3d(x.r_wi * m_last_kf_pose.rot.transpose()).log().norm();
        if (dt < m_keyframe_trans_thresh && drot < m_keyframe_rot_thresh) {
            return false;
        }
    }

    out.pose.time  = m_last_lidar_time;
    out.pose.rot   = x.r_wi;
    out.pose.trans = x.t_wi;
    out.pose.vel   = x.v;
    out.cloud      = LidarProcessor::transformCloud(m_current_cloud, x.r_il, x.t_il);
    out.id         = "kf_" + std::to_string(m_frame_counter++);
    out.timestamp  = static_cast<int64_t>(m_last_lidar_time * 1e6);

    m_last_kf_pose = out.pose;
    m_has_last_kf = true;
    return true;
}

bool PointLioFrontend::getGlobalMap(PointVec& out) {
    if (!m_ikdtree || !m_ikdtree->Root_Node) return false;
    m_ikdtree->flatten(m_ikdtree->Root_Node, out, NOT_RECORD);
    return !out.empty();
}

} // namespace rosiwit_slam
