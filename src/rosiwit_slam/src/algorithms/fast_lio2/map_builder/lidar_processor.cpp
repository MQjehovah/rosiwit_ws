#include "lidar_processor.h"

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType);
    m_norm_vec.reset(new CloudType);
    m_effect_cloud_lidar.reset(new CloudType);
    m_effect_norm_vec.reset(new CloudType);
    // 缓冲区将在 process() 中根据实际点云大小动态调整

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([this](const V21D &delta) -> bool {
        if (!m_config.convergence_check) return false;
        const V3D rot_delta = delta.block<3, 1>(0, 0);
        const V3D t_delta = delta.block<3, 1>(3, 0);
        const double rot_deg = rot_delta.norm() * 57.29577951308232;
        return rot_deg < m_config.rot_std_threshold &&
               t_delta.norm() < m_config.pos_std_threshold;
    });
}

void LidarProcessor::trimCloudMap()
{
    m_local_map.cub_to_rm.clear();
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);

    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

long LidarProcessor::voxelKey(const PointType &p, double voxel_size)
{
    const long ix = static_cast<long>(std::floor(p.x / voxel_size));
    const long iy = static_cast<long>(std::floor(p.y / voxel_size));
    const long iz = static_cast<long>(std::floor(p.z / voxel_size));
    const long M = (1L << 21) - 1;
    return ((ix & M) << 42) | ((iy & M) << 21) | (iz & M);
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;

        // 动态一致性门控：与地图重叠但未被稳定观测的点视为动态，不入图
        if (m_config.dynamic_removal && m_config.persist_thresh > 1)
        {
            const long key = voxelKey(m_cloud_down_world->points[i], m_config.map_resolution);
            auto it = m_voxel_last_frame.find(key);
            if (it == m_voxel_last_frame.end() || m_frame_idx - it->second > m_config.persist_window)
            {
                m_voxel_last_frame[key] = m_frame_idx;
                m_voxel_count[key] = 1;
            }
            else
            {
                m_voxel_last_frame[key] = m_frame_idx;
                m_voxel_count[key]++;
            }
            const bool has_near = !m_nearest_points[i].empty() &&
                                  sq_dist(m_nearest_points[i][0], m_cloud_down_world->points[i]) <= 5.0;
            if (has_near && std::fabs(m_point_residual[i]) > m_config.dynamic_residual_thresh)
                continue;
            if (has_near && m_voxel_count[key] < m_config.persist_thresh)
                continue;
        }

        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);

    // L3a: 在线自由空间雕刻 (开关) —— 删除"已被移除物体"的残留地图点
    if (m_config.free_space_carving)
        carveFreeSpace(state);
}

void LidarProcessor::carveFreeSpace(const State &state)
{
    if (m_cloud_down_world->empty())
        return;
    const V3D origin = state.t_wi;
    const double res = m_config.map_resolution;
    const double step = std::max(0.05, res * 0.5);
    const int n = static_cast<int>(m_cloud_down_world->size());
    const int stride = std::max(1, m_config.carve_stride);

    PointVec del;
    for (int i = 0; i < n; i += stride)
    {
        const PointType &e = m_cloud_down_world->points[i];
        const V3D d(e.x - origin(0), e.y - origin(1), e.z - origin(2));
        const double len = d.norm();
        if (len < step * 2.0)
            continue;
        const V3D u = d / len;
        for (double t = step; t < len - res; t += step)
        {
            const V3D q = origin + u * t;
            PointType qp;
            qp.x = static_cast<float>(q(0));
            qp.y = static_cast<float>(q(1));
            qp.z = static_cast<float>(q(2));
            const long key = voxelKey(qp, res);
            // 仅当该体素此前有占据点时才计"空"
            if (m_voxel_last_frame.find(key) == m_voxel_last_frame.end())
                continue;
            int &fc = m_voxel_free[key];
            if (++fc < m_config.carve_free_thresh)
                continue;
            // 该体素判为已移除: 收集并删除其地图点
            PointType center;
            center.x = static_cast<float>((std::floor(q(0) / res) + 0.5) * res);
            center.y = static_cast<float>((std::floor(q(1) / res) + 0.5) * res);
            center.z = static_cast<float>((std::floor(q(2) / res) + 0.5) * res);
            PointVec near;
            m_ikdtree->Radius_Search(center, static_cast<float>(res), near);
            for (auto &p : near)
                del.push_back(p);
            m_voxel_last_frame.erase(key);
            m_voxel_count.erase(key);
            m_voxel_free.erase(key);
        }
    }
    if (!del.empty())
        m_ikdtree->Delete_Points(del);
}

void LidarProcessor::collectGlobalMap(std::vector<PointType, Eigen::aligned_allocator<PointType>> &points)
{
    if (m_ikdtree->Root_Node != nullptr)
        m_ikdtree->flatten(m_ikdtree->Root_Node, points, NOT_RECORD);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    m_frame_idx++;

    // 定期清理过期的体素观测，避免内存无界增长
    if (m_config.persist_window > 0 && m_frame_idx % (m_config.persist_window * 10) == 0)
    {
        for (auto it = m_voxel_last_frame.begin(); it != m_voxel_last_frame.end();)
        {
            if (m_frame_idx - it->second > m_config.persist_window)
            {
                m_voxel_count.erase(it->first);
                m_voxel_free.erase(it->first);
                it = m_voxel_last_frame.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    // m_kf->setLossFunction([&](State &s, SharedState &d)
    //                       { updateLossFunc(s, d); });
    // m_kf->setStopFunction([&](const V21D &delta) -> bool
    //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
    //                         V3D t_delta = delta.block<3, 1>(3, 0);
    //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
    // 先剔除 NaN/Inf/超大坐标, 再进 PCL VoxelGrid。坏点会让 VoxelGrid
    // 的整数索引溢出并破坏堆 (服务器实测 free(): unaligned chunk)。
    CloudType::Ptr finite(new CloudType);
    if (package.cloud) {
        finite->reserve(package.cloud->size());
        for (const auto& p : package.cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (std::fabs(p.x) > 1e4 || std::fabs(p.y) > 1e4 || std::fabs(p.z) > 1e4) continue;
            finite->push_back(p);
        }
    }
    finite->width = static_cast<uint32_t>(finite->size());
    finite->height = 1;
    finite->is_dense = false;
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(finite);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*finite, *m_cloud_down_lidar);
    }

    // 根据实际点云大小动态调整缓冲区，避免硬编码 10000 点限制导致的越界
    {
        size_t n = m_cloud_down_lidar->size();
        if (m_cloud_down_world->size() != n)
            m_cloud_down_world->resize(n);
        if (m_norm_vec->size() != n)
            m_norm_vec->resize(n);
        if (m_effect_cloud_lidar->size() != n)
            m_effect_cloud_lidar->resize(n);
        if (m_effect_norm_vec->size() != n)
            m_effect_norm_vec->resize(n);
        if (m_nearest_points.size() != n)
            m_nearest_points.resize(n);
        if (m_point_selected_flag.size() != n)
            m_point_selected_flag.resize(n, false);
        if (m_robust_weight.size() != n)
            m_robust_weight.resize(n, 1.0);
        if (m_effect_weight.size() != n)
            m_effect_weight.resize(n, 1.0);
        if (m_point_residual.size() != n)
            m_point_residual.resize(n, 0.0);
    }

    trimCloudMap();

    // 单帧更新门限: 空地/重复走廊里点到面最近邻可能关联到错误平面,
    // 一次 IEKF 更新就能把 v 从 0.5m/s 推到 10m/s (实测 430s 转角处).
    // 单帧位置/速度/姿态修正超过物理可能范围时, 丢弃本次更新, 保留 IMU 预测,
    // 等下一帧重新关联, 避免一次误匹配把整张图带飞.
    const State before = m_kf->x();
    const M21D P_before = m_kf->P();
    m_kf->update();
    {
        const State &after = m_kf->x();
        // 门限只看水平: 垂直修正不能因为 z 误差大就被拒, 否则 z 永远回不来。
        const double dv = std::hypot(after.v.x() - before.v.x(),
                                     after.v.y() - before.v.y());
        const double dt = std::hypot(after.t_wi.x() - before.t_wi.x(),
                                     after.t_wi.y() - before.t_wi.y());
        const double dz = std::fabs(after.t_wi.z() - before.t_wi.z());
        const double dvz = std::fabs(after.v.z() - before.v.z());
        const double dr = Sophus::SO3d(after.r_wi * before.r_wi.transpose()).log().norm();
        const double dba = (after.ba - before.ba).norm();
        const double dbg = (after.bg - before.bg).norm();
        // 有轮式 odom 时用它的速度/yaw 速度收紧门限; 没有时退回固定门限.
        // 门限按**实际帧间隔**收紧: 旧实现按固定 0.1s 且系数 2.0, 在 1m/s 时允许
        // 0.35m/帧 (=3.5m/s), 窄道沿轴滑移每帧 0.3m 全部合规, 累计 0.5s 就 6.8m
        // (实测 428s LIO 位移是轮速的 6.8 倍). 这里用实测 dt 和 1.5 倍余量.
        const double frame_dt = (m_last_cloud_end > 0.0)
                                    ? std::max(0.02, std::min(0.3, package.cloud_end_time - m_last_cloud_end))
                                    : 0.1;
        // 速度门限同样收紧: 旧值在 0.5m/s 时允许 1.4m/s 误差, IMU 按错速度积分
        // 2s 就是 3m (实测 436-450s LIO 位移是轮速的 5 倍).
        double kMaxDv = m_odom_prior ? std::max(0.35, 1.2 * m_odom_speed + 0.25) : 2.0;
        double kMaxDt = m_odom_prior
                            ? std::max(0.06, 1.5 * m_odom_speed * frame_dt + 0.04)
                            : 0.5;
        double kMaxDr = m_odom_prior
                            ? std::max(0.12, 1.5 * m_odom_yaw_rate * frame_dt + 0.06)
                            : 0.35;
        // 偏置门控不能只在轮式 odom 模式生效; 阈值放宽, 避免把正常
        // 空洞后重新收敛的 LiDAR 更新全部拒掉, 只拦真正的坏帧。
        const bool bad_bias = (dba > 0.50 || dbg > 0.10);
        if (dv > kMaxDv || dt > kMaxDt || dz > 1.0 || dvz > 1.0 ||
            dr > kMaxDr || bad_bias) {
            m_kf->x() = before;
            m_kf->P() = P_before;
            static int reject_cnt = 0;
            if (++reject_cnt <= 20 || reject_cnt % 20 == 0) {
                std::cout << "[FLIO2] reject update dv=" << dv << " dt=" << dt
                          << " dz=" << dz << " dvz=" << dvz
                          << " dr=" << dr << " dba=" << dba << " dbg=" << dbg
                          << " (total=" << reject_cnt << ")" << std::endl;
            }
        }
        // 无论门控是否通过, 都对状态做物理限幅。正常帧几乎不触发;
        // 坏帧/空洞帧即使漏过门控也不会把 v/ba/bg/z 带到发散区。
        m_kf->x().clampPhysical();
    }
    // 坡道感知的垂直速度软先验: 轮速沿车体 x, 世界 z 分量 = R(2,0)*v_fwd.
    // 没有这个约束时, IMU z 零偏会以 0.8m/s 上限把 z 积分到 -100;
    // 有了它, 平地 v.z≈0, 坡道 v.z≈speed*sin(pitch), 坡度仍然保留.
    // 地面 3-DoF 量测: roll/pitch 由地面法向约束, z 由地面高度约束.
    // 不再直接写状态, 也不再积分 vz; 地面丢失时只用很松的 z 先验兜底.
    if (m_ground_follow) {
        const auto &xs = m_kf->x();
        if (m_ground_valid && m_ground_normal.z() > 0.85) {
            m_kf->updateGround(m_ground_normal, m_ground_z_target, 0.02, 0.10);
            const double nz = m_ground_normal.z();
            const double vz = std::max(-1.0, std::min(1.0,
                -(m_ground_normal.x() * xs.v.x() +
                  m_ground_normal.y() * xs.v.y()) / nz));
            m_kf->updateVerticalVelocity(vz, 0.30);
            m_ground_z_ref = m_ground_z_target;
            m_ground_z_init = true;
        } else if (m_ground_z_init) {
            m_kf->updateZ(m_ground_z_ref, 0.40);
        }
        m_kf->x().clampPhysical();
    }

    // 轮式 odom 紧耦合: 速度模长 + 世界系 yaw 角速度作为 2 维量测.
    if (m_odom_fuse && m_odom_prior) {
        V3D gyro_mean = V3D::Zero();
        int n = 0;
        for (const auto &imu : package.imus) { gyro_mean += imu.gyro; ++n; }
        if (n > 0) {
            gyro_mean /= static_cast<double>(n);
            m_kf->updateOdom(m_odom_v_fwd, m_odom_v_lat, m_odom_yaw_rate, gyro_mean,
                             0.2, 0.10, 0.05);
            static int fuse_log = 0;
            if (++fuse_log % 50 == 0) {
                std::cout << "[FLIO2] odom fuse speed=" << m_odom_speed
                          << " yaw_rate=" << m_odom_yaw_rate
                          << " v=" << m_kf->x().v.transpose()
                          << " bg=" << m_kf->x().bg.transpose() << std::endl;
            }
        }
    }
    m_last_cloud_end = package.cloud_end_time;
    incrCloudMap();
}

double LidarProcessor::robustWeight(double residual) const
{
    double w = 1.0;
    if (m_config.robust_scale > 0.0)
    {
        const double s2 = m_config.robust_scale * m_config.robust_scale;
        if (m_config.robust_kernel == "tukey")
        {
            const double ratio = residual * residual / s2;
            if (ratio >= 1.0)
                w = 0.0;
            else
            {
                const double t = 1.0 - ratio;
                w = t * t;
            }
        }
        else if (m_config.robust_kernel == "cauchy")
        {
            w = 1.0 / (1.0 + residual * residual / s2);
        }
        else if (m_config.robust_kernel == "huber")
        {
            const double a = std::fabs(residual);
            w = (a <= m_config.robust_scale) ? 1.0 : m_config.robust_scale / a;
        }
    }
    if (m_config.robust_floor > 0.0)
        w = std::max(w, m_config.robust_floor);
    return w;
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    int size = m_cloud_down_lidar->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++)
    {
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        m_point_residual[i] = 0.0;
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Vector4d pabcd;
        m_point_selected_flag[i] = false;
        if (esti_plane(points_near, 0.1, pabcd))
        {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            m_point_residual[i] = pd2;
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9)
            {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = pabcd(0);
                m_norm_vec->points[i].y = pabcd(1);
                m_norm_vec->points[i].z = pabcd(2);
                m_norm_vec->points[i].intensity = pd2;
                m_robust_weight[i] = robustWeight(pd2);
            }
        }
    }

    // 轮速一致性门控: LIO 短窗口位移远超轮速 -> 本帧不更新 (只保留 IMU 预测).
    if (!m_odom_consistency_ok) {
        share_data.valid = false;
        return;
    }

    int effect_feat_num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!m_point_selected_flag[i])
            continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        m_effect_weight[effect_feat_num] = m_robust_weight[i];
        effect_feat_num++;
    }
    // 诊断: 有效平面点数 + IEKF 状态, 每 ~1s 一行. 用于定位长航时/转弯发散.
    {
        static int diag_cnt = 0;
        if (++diag_cnt % 50 == 0) {
            const State &xs = m_kf->x();
            Eigen::Vector3d rpy = xs.r_wi.eulerAngles(2, 1, 0);
            std::cout << "[FLIO2] eff=" << effect_feat_num << "/" << size
                      << " min=" << m_config.min_effective_points
                      << " v=" << xs.v.transpose()
                      << " bg=" << xs.bg.transpose()
                      << " ba=" << xs.ba.transpose()
                      << " t=" << xs.t_wi.transpose()
                      << " rpy=" << rpy.transpose() << std::endl;
        }
    }
    if (effect_feat_num < m_config.min_effective_points)
    {
        // 有效平面点不足 -> 本帧不做 EKF 更新(纯预测通过)。legacy 语义: <50 拒绝。
        share_data.valid = false;
        if (effect_feat_num == 0)
            std::cerr << "NO Effective Points!" << std::endl;
        return;
    }
    // 退化检测 (LIO-SAM/FAST-LIO 常见做法): 有效点太少或占比太低时, 点面匹配
    // 只剩下少数方向约束, 强行更新会把 yaw/平移拉偏 (实测急转+开阔段厚墙).
    {
        const double ratio = size > 0 ? static_cast<double>(effect_feat_num) / size : 0.0;
        const bool few = effect_feat_num < m_config.degenerate_min_effective_points;
        const bool low_ratio = ratio < m_config.degenerate_ratio;
        if (few || low_ratio) {
            ++m_degenerate_count;
            share_data.valid = false;
            // 注意: 不做 P *= 1.1 这类逐帧放大的操作 —— 连续退化时会指数放大协方差,
            // 恢复后第一帧就能把状态拉飞 (实测 z 掉到 -66m)。退化期间只跳过更新。
            static int deg_log = 0;
            if (++deg_log % 20 == 0) {
                std::cout << "[FLIO2] degenerate skip eff=" << effect_feat_num
                          << " (" << ratio * 100.0 << "%) consecutive="
                          << m_degenerate_count << std::endl;
            }
            return;
        }
    }
    m_degenerate_count = 0;
    share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &norm_p = m_effect_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_wi);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        share_data.H += J.transpose() * m_config.lidar_cov_inv * m_effect_weight[i] * J;
        share_data.b += J.transpose() * m_config.lidar_cov_inv * m_effect_weight[i] * norm_p.intensity;
    }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}