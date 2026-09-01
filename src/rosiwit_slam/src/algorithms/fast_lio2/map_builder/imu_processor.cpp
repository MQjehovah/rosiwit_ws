#include "imu_processor.h"

IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng;
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na;
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg;
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;
    m_last_acc.setZero();
    m_last_gyro.setZero();
    m_imu_cache.clear();
    m_poses_cache.clear();
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
        return false;

    // 鲁棒重力对齐: 录包 IMU 存在空洞/微动, 只用近似静止(角速度小)的样本
    // 估计重力方向和陀螺零偏, 避免空洞期间的运动加速度污染初始化。
    V3D acc_sum = V3D::Zero();
    V3D gyro_sum = V3D::Zero();
    int n_static = 0;
    for (const auto &imu : m_imu_cache)
    {
        if (imu.gyro.norm() < 0.15)   // 近似静止
        {
            acc_sum += imu.acc;
            gyro_sum += imu.gyro;
            n_static++;
        }
    }
    if (n_static < 3)
    {
        // 静态样本不足(全程在动), 退化为全样本均值
        for (const auto &imu : m_imu_cache)
        {
            acc_sum += imu.acc;
            gyro_sum += imu.gyro;
            n_static++;
        }
    }
    V3D acc_mean = acc_sum / n_static;
    V3D gyro_mean = gyro_sum / n_static;

    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;
    m_kf->x().bg = gyro_mean;
    if (m_config.gravity_align)
    {
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
    }
    else
        m_kf->x().initGravityDir(-acc_mean);
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
    // IMU 有空洞, 初始化姿态存在不确定性, 放大初始姿态协方差让 lidar 校正
    m_kf->P().block<3, 3>(0, 0) = M3D::Identity() * 0.001;

    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
    return true;
}

void IMUProcessor::undistort(SyncPackage &package)
{

    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // const double imu_time_begin = m_imu_cache.front().time;
    const double imu_time_end = m_imu_cache.back().time;

    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;

    // 退化处理: IMU 空洞(丢帧)跨越了传播基准点。
    // 空洞期间运动未知, 用陈旧状态跨空洞外推会污染位姿。
    // 处理: 重置传播基准到空洞后的 IMU, 速度清零、增大状态协方差,
    // 让后续 lidar scan-to-map 匹配主导收敛, 而不是继承空洞前的速度/姿态误差。
    if (m_imu_cache.size() > 1 && m_last_propagate_end_time > 0) {
        double first_new_imu = m_imu_cache[1].time;   // [0]=m_last_imu, [1]=本帧首个IMU
        if (first_new_imu - m_last_propagate_end_time > 0.3) {
            m_last_propagate_end_time = first_new_imu;
            m_kf->x().v.setZero();                     // 空洞期间速度未知, 清零
            // 增大位置/速度/姿态协方差, 让 lidar 观测重新主导
            m_kf->P().block<3, 3>(0, 0)  = M3D::Identity() * 0.01;
            m_kf->P().block<3, 3>(3, 3)  = M3D::Identity() * 1.0;
            m_kf->P().block<3, 3>(12,12) = M3D::Identity() * 1.0;
        }
    }

    m_poses_cache.clear();
    m_poses_cache.emplace_back(0.0, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);

    V3D acc_val, gyro_val;
    double dt = 0.0;
    Input inp;
    inp.acc = m_imu_cache.back().acc;
    inp.gyro = m_imu_cache.back().gyro;
    // IMU 数据存在周期性的数百毫秒丢帧时, 限制单步传播时间, 避免用陈旧 IMU 长时间积分导致速度发散
    const double max_prop_dt = 0.1;
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); it_imu++)
    {
        IMUData &head = *it_imu;
        IMUData &tail = *(it_imu + 1);
        if (tail.time < m_last_propagate_end_time)
            continue;
        gyro_val = 0.5 * (head.gyro + tail.gyro);
        acc_val = 0.5 * (head.acc + tail.acc);

        if (head.time < m_last_propagate_end_time)
            dt = tail.time - m_last_propagate_end_time;
        else
            dt = tail.time - head.time;

        // 大间隔(丢帧): 用最后有效 IMU 补偿, 限制 dt 防止发散
        if (dt > max_prop_dt) dt = max_prop_dt;

        inp.acc = acc_val;
        inp.gyro = gyro_val;
        m_kf->predict(inp, dt, m_Q);

        m_last_gyro = gyro_val - m_kf->x().bg;
        m_last_acc = m_kf->x().r_wi * (acc_val - m_kf->x().ba) + m_kf->x().g;
        double offset = tail.time - cloud_time_begin;
        m_poses_cache.emplace_back(offset, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);
    }

    dt = propagate_time_end - imu_time_end;
    if (dt > max_prop_dt) dt = max_prop_dt;
    m_kf->predict(inp, dt, m_Q);
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    M3D cur_r_wi = m_kf->x().r_wi;
    V3D cur_t_wi = m_kf->x().t_wi;
    M3D cur_r_il = m_kf->x().r_il;
    V3D cur_t_il = m_kf->x().t_il;
    auto it_pcl = package.cloud->points.end() - 1;

    for (auto it_kp = m_poses_cache.end() - 1; it_kp != m_poses_cache.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->trans;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == package.cloud->points.begin())
                break;
        }
    }
}