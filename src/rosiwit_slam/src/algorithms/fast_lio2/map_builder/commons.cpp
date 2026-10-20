#include "commons.h"

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

bool parseFastLio2Config(const std::string& config_path, Config& out_cfg) {
#ifndef YAML_CPP_DISABLED
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config) return false;
        // 建图参数标准层级：mapping.frontend.*；旧配置/ROS2 配置仍回退根节点。
        const YAML::Node frontend =
            (config["mapping"] && config["mapping"]["frontend"])
                ? config["mapping"]["frontend"] : config;
        auto get = [&](const char* key) -> YAML::Node {
            if (frontend[key]) return frontend[key];
            return config[key];
        };
        auto get_root_first = [&](const char* key) -> YAML::Node {
            if (config[key]) return config[key];   // ROS1 注入的 r_il/t_il/esti_il 在根节点
            return frontend[key];
        };
        (void)get_root_first;

        out_cfg.lidar_filter_num = get("lidar_filter_num").as<int>(out_cfg.lidar_filter_num);
        out_cfg.lidar_min_range  = get("lidar_min_range").as<double>(out_cfg.lidar_min_range);
        out_cfg.lidar_max_range  = get("lidar_max_range").as<double>(out_cfg.lidar_max_range);
        out_cfg.scan_resolution  = get("scan_resolution").as<double>(out_cfg.scan_resolution);
        out_cfg.map_resolution   = get("map_resolution").as<double>(out_cfg.map_resolution);
        out_cfg.cube_len         = get("cube_len").as<double>(out_cfg.cube_len);
        out_cfg.det_range        = get("det_range").as<double>(out_cfg.det_range);
        out_cfg.move_thresh      = get("move_thresh").as<double>(out_cfg.move_thresh);
        out_cfg.na               = get("na").as<double>(out_cfg.na);
        out_cfg.ng               = get("ng").as<double>(out_cfg.ng);
        out_cfg.nba              = get("nba").as<double>(out_cfg.nba);
        out_cfg.nbg              = get("nbg").as<double>(out_cfg.nbg);
        out_cfg.acc_scale        = get("acc_scale").as<double>(out_cfg.acc_scale);
        out_cfg.imu_init_num     = get("imu_init_num").as<int>(out_cfg.imu_init_num);
        out_cfg.near_search_num  = get("near_search_num").as<int>(out_cfg.near_search_num);
        out_cfg.ieskf_max_iter   = get("ieskf_max_iter").as<int>(out_cfg.ieskf_max_iter);
        out_cfg.gravity_align    = get("gravity_align").as<bool>(out_cfg.gravity_align);
        out_cfg.init_z           = get("init_z").as<double>(out_cfg.init_z);
        out_cfg.undistort_enable = get("undistort_enable").as<bool>(out_cfg.undistort_enable);
        out_cfg.esti_il          = get_root_first("esti_il").as<bool>(out_cfg.esti_il);

        auto t_il = get_root_first("t_il").as<std::vector<double>>(std::vector<double>{0,0,0});
        auto r_il = get_root_first("r_il").as<std::vector<double>>(std::vector<double>{1,0,0,0,1,0,0,0,1});
        if (t_il.size() >= 3 && r_il.size() >= 9) {
            out_cfg.t_il << t_il[0], t_il[1], t_il[2];
            out_cfg.r_il << r_il[0], r_il[1], r_il[2],
                            r_il[3], r_il[4], r_il[5],
                            r_il[6], r_il[7], r_il[8];
        }

        out_cfg.lidar_cov_inv = get("lidar_cov_inv").as<double>(out_cfg.lidar_cov_inv);
        out_cfg.robust_kernel = get("robust_kernel").as<std::string>(out_cfg.robust_kernel);
        out_cfg.robust_scale  = get("robust_scale").as<double>(out_cfg.robust_scale);
        out_cfg.robust_floor  = get("robust_floor").as<double>(out_cfg.robust_floor);
        out_cfg.dynamic_removal = get("dynamic_removal").as<bool>(out_cfg.dynamic_removal);
        out_cfg.persist_thresh  = get("persist_thresh").as<int>(out_cfg.persist_thresh);
        out_cfg.persist_window  = get("persist_window").as<int>(out_cfg.persist_window);
        out_cfg.dynamic_residual_thresh = get("dynamic_residual_thresh").as<double>(out_cfg.dynamic_residual_thresh);
        out_cfg.free_space_carving = get("free_space_carving").as<bool>(out_cfg.free_space_carving);
        out_cfg.carve_free_thresh = get("carve_free_thresh").as<int>(out_cfg.carve_free_thresh);
        out_cfg.carve_stride = get("carve_stride").as<int>(out_cfg.carve_stride);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
#else
    (void)config_path;
    (void)out_cfg;
    // yaml-cpp 不可用时使用 Config 结构体默认值，不阻止 FastLIO2 初始化
    return true;
#endif
}

bool esti_plane(PointVec &points, const double &thresh, V4D &out)
{
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::MatrixXd b(points.size(), 1);
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (size_t i = 0; i < points.size(); i++)
    {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;
    }
    V3D normvec = A.colPivHouseholderQr().solve(b);
    double norm = normvec.norm();
    out[0] = normvec(0) / norm;
    out[1] = normvec(1) / norm;
    out[2] = normvec(2) / norm;
    out[3] = 1.0 / norm;
    for (size_t j = 0; j < points.size(); j++)
    {
        if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
        {
            return false;
        }
    }
    return true;
}

float sq_dist(const PointType &p1, const PointType &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}