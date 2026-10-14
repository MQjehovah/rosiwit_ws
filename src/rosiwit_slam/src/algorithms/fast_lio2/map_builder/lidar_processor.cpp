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
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
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
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
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
    m_kf->update();
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
    if (effect_feat_num < 1)
    {
        share_data.valid = false;
        std::cerr << "NO Effective Points!" << std::endl;
        return;
    }
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