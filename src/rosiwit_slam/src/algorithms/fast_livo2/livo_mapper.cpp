// src/algorithms/fast_livo2/livo_mapper.cpp
#include "algorithms/fast_livo2/livo_mapper.h"

#include <cmath>
#include <iostream>

#include <vikit/equidistant_camera.h>
#include <vikit/pinhole_camera.h>

namespace rosiwit_slam {
namespace {

constexpr int kGridSize = 5;
constexpr double kExposureTimeInit = 0.0;
constexpr double kPlotTime = -10.0;
constexpr int kFrameCnt = 6;

}  // namespace

LivoMapper::~LivoMapper() = default;

bool LivoMapper::createCamera(const CameraIntrinsics& cam) {
    if (cam.model == "Pinhole") {
        vio_manager_->cam = new vk::PinholeCamera(
            cam.width, cam.height, cam.scale, cam.fx, cam.fy, cam.cx, cam.cy,
            cam.d0, cam.d1, cam.d2, cam.d3);
    } else if (cam.model == "EquidistantCamera") {
        vio_manager_->cam = new vk::EquidistantCamera(
            cam.width, cam.height, cam.scale, cam.fx, cam.fy, cam.cx, cam.cy,
            cam.d0, cam.d1, cam.d2, cam.d3);
    } else {
        std::cerr << "[LivoMapper] unsupported camera model: " << cam.model << std::endl;
        return false;
    }
    return vio_manager_->cam != nullptr;
}

bool LivoMapper::init(const FastLivo2Config& config) {
    cfg_ = config;
    extT_ = cfg_.lidar_imu_trans;
    extR_ = cfg_.lidar_imu_rot;

    p_imu_.reset(new ImuProcess());
    vio_manager_.reset(new VIOManager());
    if (!createCamera(cfg_.camera)) return false;

    VoxelMapConfig voxel_config;
    loadVoxelConfig(cfg_, voxel_config);
    voxelmap_manager_.reset(new VoxelMapManager(voxel_config, voxel_map_));

    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    pcl_w_wait_pub_.reset(new PointCloudXYZI());
    pcl_wait_pub_.reset(new PointCloudXYZI());
    rgb_cloud_.reset(new PointCloudXYZRGB());

    downSizeFilterSurf_.setLeafSize(cfg_.filter_size_surf, cfg_.filter_size_surf, cfg_.filter_size_surf);

    vio_manager_->grid_size = kGridSize;
    vio_manager_->patch_size = cfg_.patch_size;
    vio_manager_->outlier_threshold = cfg_.outlier_threshold;
    vio_manager_->setImuToLidarExtrinsic(extT_, extR_);

    std::vector<double> camera_r(9), camera_t(3);
    for (int r = 0; r < 3; ++r) {
        camera_t[r] = cfg_.cam_lidar_trans[r];
        for (int c = 0; c < 3; ++c) camera_r[r * 3 + c] = cfg_.cam_lidar_rot(r, c);
    }
    vio_manager_->setLidarToCameraExtrinsic(camera_r, camera_t);
    vio_manager_->state = &state_;
    vio_manager_->state_propagat = &state_propagat_;
    vio_manager_->max_iterations = cfg_.vio_max_iterations;
    vio_manager_->img_point_cov = cfg_.img_point_cov;
    vio_manager_->normal_en = cfg_.normal_en;
    vio_manager_->inverse_composition_en = cfg_.inverse_composition_en;
    vio_manager_->raycast_en = cfg_.raycast_en;
    vio_manager_->grid_n_width = 0;
    vio_manager_->grid_n_height = 17;
    vio_manager_->patch_pyrimid_level = cfg_.patch_pyrimid_level;
    vio_manager_->exposure_estimate_en = cfg_.exposure_estimate_en;
    vio_manager_->colmap_output_en = false;
    vio_manager_->initializeVIO();

    p_imu_->set_extrinsic(extT_, extR_);
    p_imu_->set_gyr_cov_scale(V3D(cfg_.gyr_cov, cfg_.gyr_cov, cfg_.gyr_cov));
    p_imu_->set_acc_cov_scale(V3D(cfg_.acc_cov, cfg_.acc_cov, cfg_.acc_cov));
    p_imu_->set_inv_expo_cov(cfg_.inv_expo_cov);
    p_imu_->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
    p_imu_->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
    p_imu_->set_imu_init_frame_num(cfg_.imu_int_frame);

    if (!cfg_.imu_en) p_imu_->disable_imu();
    if (!cfg_.exposure_estimate_en) p_imu_->disable_exposure_est();

    slam_mode_ = (cfg_.img_en && cfg_.lidar_en) ? LIVO : (cfg_.imu_en ? ONLY_LIO : ONLY_LO);
    inited_ = true;
    std::cout << "[LivoMapper] mode=" << static_cast<int>(slam_mode_)
              << " camera=" << cfg_.camera.model << std::endl;
    return true;
}

void LivoMapper::addImu(const IMUSample& sample) {
    if (!cfg_.imu_en) return;
    if (last_timestamp_lidar_ < 0.0) return;

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    IMUSample s = sample;
    s.time -= cfg_.imu_time_offset;
    if (last_timestamp_imu_ > 0.0 && std::fabs(s.time - last_timestamp_imu_) > 1.0) {
        std::cerr << "[LivoMapper] imu time jump: " << s.time - last_timestamp_imu_ << std::endl;
    }
    last_timestamp_imu_ = s.time;
    imu_buffer_.push_back(s);
    sig_buffer_.notify_all();
}

void LivoMapper::addLidar(const LidarFrame& frame) {
    if (!cfg_.lidar_en || !frame.cloud) return;

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    const double stamp = frame.start_time - cfg_.lidar_time_offset;
    if (last_timestamp_lidar_ > 0.0 && stamp < last_timestamp_lidar_) {
        std::cerr << "[LivoMapper] lidar loop back, clear buffer" << std::endl;
        lid_raw_data_buffer_.clear();
        lid_header_time_buffer_.clear();
    }
    CloudType::Ptr ptr = frame.cloud;
    if (ptr->points.empty()) return;

    lid_raw_data_buffer_.push_back(ptr);
    lid_header_time_buffer_.push_back(stamp);
    last_timestamp_lidar_ = stamp;
    sig_buffer_.notify_all();
}

void LivoMapper::addImage(const ImageFrame& frame) {
    if (!cfg_.img_en || frame.image.empty()) return;

    std::lock_guard<std::mutex> lock(mtx_buffer_);
    const double stamp = frame.time - cfg_.img_time_offset;
    if (last_timestamp_img_ > 0.0 && stamp <= last_timestamp_img_) return;  // duplicate / out of order
    last_timestamp_img_ = stamp;
    img_buffer_.push_back(frame.image.clone());
    img_time_buffer_.push_back(stamp);
    sig_buffer_.notify_all();
}

bool LivoMapper::process() {
    if (!inited_) return false;

    bool updated = false;
    for (int iter = 0; iter < 8; ++iter) {
        if (!sync_packages(LidarMeasures_)) break;
        handleFirstFrame();
        processImu();
        stateEstimationAndMapping();
        updated = true;
        if (state_update_flg_) {
            state_update_flg_ = false;
            updated = true;
        }
    }
    return updated;
}

void LivoMapper::handleFirstFrame() {
    if (!is_first_frame_) {
        first_lidar_time_ = LidarMeasures_.last_lio_update_time;
        p_imu_->first_lidar_time = first_lidar_time_;
        is_first_frame_ = true;
        std::cout << "FIRST LIDAR FRAME!" << std::endl;
    }
}

void LivoMapper::gravityAlignment() {
    if (!p_imu_->imu_need_init && !gravity_align_finished_) {
        V3D ez(0, 0, -1), gz(state_.gravity);
        Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
        M3D G_R_I0 = G_q_I0.toRotationMatrix();
        state_.pos_end = G_R_I0 * state_.pos_end;
        state_.rot_end = G_R_I0 * state_.rot_end;
        state_.vel_end = G_R_I0 * state_.vel_end;
        state_.gravity = G_R_I0 * state_.gravity;
        gravity_align_finished_ = true;
    }
}

void LivoMapper::processImu() {
    p_imu_->Process2(LidarMeasures_, state_, feats_undistort_);
    state_propagat_ = state_;
    voxelmap_manager_->state_ = state_;
    voxelmap_manager_->feats_undistort_ = feats_undistort_;
}

void LivoMapper::stateEstimationAndMapping() {
    switch (LidarMeasures_.lio_vio_flg) {
    case VIO:
        handleVIO();
        break;
    case LIO:
    case LO:
        handleLIO();
        break;
    default:
        break;
    }
}

void LivoMapper::handleVIO() {
    if (!pcl_w_wait_pub_ || pcl_w_wait_pub_->empty()) {
        std::cout << "[ VIO ] No point!!!" << std::endl;
        return;
    }

    vio_manager_->plot_flag =
        std::fabs((LidarMeasures_.last_lio_update_time - first_lidar_time_) - kPlotTime) < (kFrameCnt / 2 * 0.1);

    vio_manager_->processFrame(LidarMeasures_.measures.back().img, pv_list_,
                               voxelmap_manager_->voxel_map_, LidarMeasures_.last_lio_update_time - first_lidar_time_);

    latest_ekf_time_ = LidarMeasures_.last_lio_update_time;
    updatePoseFromState(latest_ekf_time_);
    buildOutputs();
}

void LivoMapper::handleLIO() {
    euler_cur_ = RotMtoEuler(state_.rot_end);

    if (!feats_undistort_ || feats_undistort_->empty()) {
        std::cout << "[ LIO ]: No point!!!" << std::endl;
        return;
    }

    const double t0 = omp_get_wtime();

    downSizeFilterSurf_.setInputCloud(feats_undistort_);
    downSizeFilterSurf_.filter(*feats_down_body_);
    const double t_down = omp_get_wtime();

    feats_down_size_ = static_cast<int>(feats_down_body_->points.size());
    voxelmap_manager_->feats_down_body_ = feats_down_body_;
    transformLidar(state_.rot_end, state_.pos_end, feats_down_body_, feats_down_world_);
    voxelmap_manager_->feats_down_world_ = feats_down_world_;
    voxelmap_manager_->feats_down_size_ = feats_down_size_;

    if (!lidar_map_inited_) {
        lidar_map_inited_ = true;
        voxelmap_manager_->BuildVoxelMap();
    }

    const double t1 = omp_get_wtime();
    voxelmap_manager_->StateEstimation(state_propagat_);
    state_ = voxelmap_manager_->state_;
    pv_list_ = voxelmap_manager_->pv_list_;
    const double t2 = omp_get_wtime();

    latest_ekf_time_ = LidarMeasures_.last_lio_update_time;
    updatePoseFromState(latest_ekf_time_);

    const double t3 = omp_get_wtime();

    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
    transformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    for (size_t i = 0; i < world_lidar->points.size(); ++i) {
        voxelmap_manager_->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        M3D point_crossmat = voxelmap_manager_->cross_mat_list_[i];
        M3D var = voxelmap_manager_->body_cov_list_[i];
        var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
              (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() +
              state_.cov.block<3, 3>(3, 3);
        voxelmap_manager_->pv_list_[i].var = var;
    }
    voxelmap_manager_->UpdateVoxelMap(voxelmap_manager_->pv_list_);
    pv_list_ = voxelmap_manager_->pv_list_;

    const double t4 = omp_get_wtime();

    if (voxelmap_manager_->config_setting_.map_sliding_en) {
        voxelmap_manager_->mapSliding();
    }

    PointCloudXYZI::Ptr laser_cloud_full_res(cfg_.dense_map_en ? feats_undistort_ : feats_down_body_);
    PointCloudXYZI::Ptr laser_cloud_world(new PointCloudXYZI(laser_cloud_full_res->points.size(), 1));
    for (size_t i = 0; i < laser_cloud_full_res->points.size(); ++i) {
        RGBpointBodyToWorld(&laser_cloud_full_res->points[i], &laser_cloud_world->points[i]);
    }
    *pcl_w_wait_pub_ = *laser_cloud_world;

    buildOutputs();

    frame_num_++;
    aver_time_consu_ = aver_time_consu_ * (frame_num_ - 1) / frame_num_ + (t4 - t0) / frame_num_;

    (void)t_down;
    (void)t1;
    (void)t2;
    (void)t3;
}

void LivoMapper::updatePoseFromState(double stamp) {
    PoseStamped p;
    p.time = stamp;
    p.rot = state_.rot_end;
    p.trans = state_.pos_end;
    p.vel = state_.vel_end;

    std::lock_guard<std::mutex> lock(mtx_output_);
    latest_pose_ = p;
    has_pose_ = true;
}

void LivoMapper::buildOutputs() {
    CloudType::Ptr body(new CloudType());
    CloudType::Ptr world(new CloudType());
    if (feats_undistort_) *body = *feats_undistort_;
    if (pcl_w_wait_pub_) *world = *pcl_w_wait_pub_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    bool has_rgb = false;

    if (slam_mode_ == LIVO && LidarMeasures_.lio_vio_flg == VIO && !pcl_w_wait_pub_->empty()) {
        *pcl_wait_pub_ += *pcl_w_wait_pub_;
        const size_t size = pcl_wait_pub_->points.size();
        rgb->reserve(size);
        const cv::Mat& img_rgb = vio_manager_->img_rgb;
        for (size_t i = 0; i < size; ++i) {
            const auto& pi = pcl_wait_pub_->points[i];
            V3D p_w(pi.x, pi.y, pi.z);
            V3D pf(vio_manager_->new_frame_->w2f(p_w));
            if (pf[2] < 0) continue;
            V2D pc(vio_manager_->new_frame_->w2c(p_w));
            if (!vio_manager_->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) continue;

            V3F pixel = vio_manager_->getInterpolatedPixel(img_rgb, pc);
            pcl::PointXYZRGB pointRGB;
            pointRGB.x = pi.x;
            pointRGB.y = pi.y;
            pointRGB.z = pi.z;
            pointRGB.r = pixel[2];
            pointRGB.g = pixel[1];
            pointRGB.b = pixel[0];
            if (pf.norm() > cfg_.blind_rgb_points) rgb->push_back(pointRGB);
        }
        has_rgb = !rgb->empty();
        PointCloudXYZI().swap(*pcl_wait_pub_);
        PointCloudXYZI().swap(*pcl_w_wait_pub_);
    }

    std::lock_guard<std::mutex> lock(mtx_output_);
    out_body_ = body;
    out_world_ = world;
    rgb_cloud_ = rgb;
    has_rgb_ = has_rgb;
}

PoseStamped LivoMapper::pose() const {
    std::lock_guard<std::mutex> lock(mtx_output_);
    return latest_pose_;
}

CloudType::Ptr LivoMapper::bodyCloud() const {
    std::lock_guard<std::mutex> lock(mtx_output_);
    return out_body_;
}

CloudType::Ptr LivoMapper::worldCloud() const {
    std::lock_guard<std::mutex> lock(mtx_output_);
    return out_world_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LivoMapper::rgbCloud() const {
    std::lock_guard<std::mutex> lock(mtx_output_);
    return has_rgb_ ? rgb_cloud_ : nullptr;
}

void LivoMapper::transformLidar(const Eigen::Matrix3d& rot, const Eigen::Vector3d& t,
                                const PointCloudXYZI::Ptr& input_cloud, PointCloudXYZI::Ptr& trans_cloud) {
    PointCloudXYZI().swap(*trans_cloud);
    trans_cloud->points.resize(input_cloud->size());
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        PointType& p = trans_cloud->points[i];
        const PointType& pi = input_cloud->points[i];
        V3D pf(pi.x, pi.y, pi.z);
        V3D pi_imu(extR_ * pf + extT_);
        V3D po(rot * pi_imu + t);
        p.x = po(0);
        p.y = po(1);
        p.z = po(2);
        p.intensity = pi.intensity;
        p.curvature = pi.curvature;
    }
}

void LivoMapper::RGBpointBodyToWorld(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_imu(extR_ * p_body + extT_);
    V3D p_world(state_.rot_end * p_imu + state_.pos_end);
    po->x = p_world(0);
    po->y = p_world(1);
    po->z = p_world(2);
    po->intensity = pi->intensity;
    po->curvature = pi->curvature;
}

bool LivoMapper::sync_packages(LidarMeasureGroup& meas) {
    if (lid_raw_data_buffer_.empty() && cfg_.lidar_en) return false;
    if (img_buffer_.empty() && cfg_.img_en) return false;
    if (imu_buffer_.empty() && cfg_.imu_en) return false;

    switch (slam_mode_) {
    case ONLY_LIO: {
        if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer_.front();
        if (!lidar_pushed_) {
            meas.lidar = lid_raw_data_buffer_.front();
            if (meas.lidar->points.size() <= 1) return false;
            meas.lidar_frame_beg_time = lid_header_time_buffer_.front();
            meas.lidar_frame_end_time =
                meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
            meas.pcl_proc_cur = meas.lidar;
            lidar_pushed_ = true;
        }
        if (cfg_.imu_en && last_timestamp_imu_ < meas.lidar_frame_end_time) return false;

        MeasureGroup m;
        m.imu.clear();
        m.lio_time = meas.lidar_frame_end_time;
        {
            std::lock_guard<std::mutex> lock(mtx_buffer_);
            while (!imu_buffer_.empty()) {
                if (imu_buffer_.front().time > meas.lidar_frame_end_time) break;
                m.imu.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
            }
            lid_raw_data_buffer_.pop_front();
            lid_header_time_buffer_.pop_front();
        }
        sig_buffer_.notify_all();

        meas.lio_vio_flg = LIO;
        meas.measures.push_back(m);
        lidar_pushed_ = false;
        return true;
    }
    case LIVO: {
        const EKF_STATE last_flg = meas.lio_vio_flg;
        switch (last_flg) {
        case WAIT:
        case VIO: {
            const double img_capture_time = img_time_buffer_.front() + kExposureTimeInit;
            if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer_.front();

            const double lid_newest_time =
                lid_header_time_buffer_.back() + lid_raw_data_buffer_.back()->points.back().curvature / double(1000);
            const double imu_newest_time = imu_buffer_.back().time;

            if (img_capture_time < meas.last_lio_update_time + 0.00001) {
                std::lock_guard<std::mutex> lock(mtx_buffer_);
                img_buffer_.pop_front();
                img_time_buffer_.pop_front();
                std::cerr << "[LivoMapper] throw one image frame" << std::endl;
                return false;
            }
            if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time) return false;

            MeasureGroup m;
            m.imu.clear();
            m.lio_time = img_capture_time;
            {
                std::lock_guard<std::mutex> lock(mtx_buffer_);
                while (!imu_buffer_.empty()) {
                    if (imu_buffer_.front().time > m.lio_time) break;
                    if (imu_buffer_.front().time > meas.last_lio_update_time) m.imu.push_back(imu_buffer_.front());
                    imu_buffer_.pop_front();
                }
            }
            sig_buffer_.notify_all();

            *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
            PointCloudXYZI().swap(*meas.pcl_proc_next);

            const int lid_frame_num = static_cast<int>(lid_raw_data_buffer_.size());
            const size_t max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
            meas.pcl_proc_cur->reserve(max_size);
            meas.pcl_proc_next->reserve(max_size);

            while (!lid_raw_data_buffer_.empty()) {
                if (lid_header_time_buffer_.front() > img_capture_time) break;
                auto pcl = lid_raw_data_buffer_.front()->points;
                const double frame_header_time = lid_header_time_buffer_.front();
                const float max_offs_time_ms = static_cast<float>((m.lio_time - frame_header_time) * 1000.0);

                for (auto& pt : pcl) {
                    if (pt.curvature < max_offs_time_ms) {
                        pt.curvature += static_cast<float>((frame_header_time - meas.last_lio_update_time) * 1000.0);
                        meas.pcl_proc_cur->points.push_back(pt);
                    } else {
                        pt.curvature += static_cast<float>((frame_header_time - m.lio_time) * 1000.0);
                        meas.pcl_proc_next->points.push_back(pt);
                    }
                }
                std::lock_guard<std::mutex> lock(mtx_buffer_);
                lid_raw_data_buffer_.pop_front();
                lid_header_time_buffer_.pop_front();
            }

            meas.measures.push_back(m);
            meas.lio_vio_flg = LIO;
            return true;
        }
        case LIO: {
            const double img_capture_time = img_time_buffer_.front() + kExposureTimeInit;
            meas.lio_vio_flg = VIO;
            meas.measures.clear();

            MeasureGroup m;
            m.vio_time = img_capture_time;
            m.lio_time = meas.last_lio_update_time;
            m.img = img_buffer_.front();
            {
                std::lock_guard<std::mutex> lock(mtx_buffer_);
                img_buffer_.pop_front();
                img_time_buffer_.pop_front();
            }
            sig_buffer_.notify_all();
            meas.measures.push_back(m);
            lidar_pushed_ = false;
            return true;
        }
        default:
            return false;
        }
    }
    case ONLY_LO: {
        if (!lidar_pushed_) {
            if (lid_raw_data_buffer_.empty()) return false;
            meas.lidar = lid_raw_data_buffer_.front();
            meas.lidar_frame_beg_time = lid_header_time_buffer_.front();
            meas.lidar_frame_end_time =
                meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_pushed_ = true;
        }
        MeasureGroup m;
        m.lio_time = meas.lidar_frame_end_time;
        {
            std::lock_guard<std::mutex> lock(mtx_buffer_);
            lid_raw_data_buffer_.pop_front();
            lid_header_time_buffer_.pop_front();
        }
        sig_buffer_.notify_all();
        lidar_pushed_ = false;
        meas.lio_vio_flg = LO;
        meas.measures.push_back(m);
        return true;
    }
    default:
        return false;
    }
}

}  // namespace rosiwit_slam
