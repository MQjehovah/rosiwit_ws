// src/algorithms/fast_livo2/parse_config.cpp
#include "algorithms/fast_livo2/parse_config.h"

#include <cmath>
#include <sstream>

#ifndef YAML_CPP_DISABLED
#include <yaml-cpp/yaml.h>
#endif

namespace rosiwit_slam {
namespace {

#ifdef YAML_CPP_DISABLED
#else
bool readVector(const YAML::Node& node, int expected, double* out) {
    if (!node || !node.IsSequence() || static_cast<int>(node.size()) != expected) return false;
    for (int i = 0; i < expected; ++i) out[i] = node[i].as<double>();
    return true;
}

bool readMatrix3(const YAML::Node& node, Eigen::Matrix3d& m) {
    double v[9];
    if (!readVector(node, 9, v)) return false;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) m(r, c) = v[r * 3 + c];
    return true;
}

bool readVector3(const YAML::Node& node, Eigen::Vector3d& v) {
    double d[3];
    if (!readVector(node, 3, d)) return false;
    v = Eigen::Vector3d(d[0], d[1], d[2]);
    return true;
}

double getDouble(const YAML::Node& n, const char* key, double def) {
    return (n && n[key]) ? n[key].as<double>() : def;
}
int getInt(const YAML::Node& n, const char* key, int def) {
    return (n && n[key]) ? n[key].as<int>() : def;
}
bool getBool(const YAML::Node& n, const char* key, bool def) {
    return (n && n[key]) ? n[key].as<bool>() : def;
}
#endif

}  // namespace

bool parseFastLivo2Config(const std::string& yaml_path, FastLivo2Config& out, std::string& err) {
#ifdef YAML_CPP_DISABLED
    (void)yaml_path;
    (void)out;
    err = "yaml-cpp is not available (built with YAML_CPP_DISABLED)";
    return false;
#else
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        err = std::string("cannot load yaml: ") + e.what();
        return false;
    }
    if (!root.IsMap()) {
        err = "yaml root is not a map";
        return false;
    }

    const YAML::Node cam = root["mapping"]["camera"];
    if (!cam || !cam.IsMap()) {
        err = "missing mapping.camera (camera intrinsics are required for LIVO)";
        return false;
    }
    FastLivo2Config cfg;

    cfg.camera.model = cam["model"] ? cam["model"].as<std::string>() : "Pinhole";
    if (!cam["width"] || !cam["height"] || !cam["fx"] || !cam["fy"] || !cam["cx"] || !cam["cy"]) {
        err = "mapping.camera requires width/height/fx/fy/cx/cy";
        return false;
    }
    cfg.camera.width = cam["width"].as<int>();
    cfg.camera.height = cam["height"].as<int>();
    cfg.camera.scale = getDouble(cam, "scale", 1.0);
    cfg.camera.fx = cam["fx"].as<double>();
    cfg.camera.fy = cam["fy"].as<double>();
    cfg.camera.cx = cam["cx"].as<double>();
    cfg.camera.cy = cam["cy"].as<double>();
    cfg.camera.d0 = getDouble(cam, "d0", 0.0);
    cfg.camera.d1 = getDouble(cam, "d1", 0.0);
    cfg.camera.d2 = getDouble(cam, "d2", 0.0);
    cfg.camera.d3 = getDouble(cam, "d3", 0.0);

    // ---- extrinsics: yaml first, root (TF) overrides ----
    const YAML::Node ext = root["mapping"]["extrin_calib"];
    if (ext) {
        readMatrix3(ext["lidar_imu_r"], cfg.lidar_imu_rot);
        readVector3(ext["lidar_imu_t"], cfg.lidar_imu_trans);
        // Accept the standalone package's names as well.
        if (ext["camera_lidar_r"]) readMatrix3(ext["camera_lidar_r"], cfg.cam_lidar_rot);
        else if (ext["Rcl"]) readMatrix3(ext["Rcl"], cfg.cam_lidar_rot);
        if (ext["camera_lidar_t"]) readVector3(ext["camera_lidar_t"], cfg.cam_lidar_trans);
        else if (ext["Pcl"]) readVector3(ext["Pcl"], cfg.cam_lidar_trans);
    }
    if (root["r_il"]) {
        if (!readMatrix3(root["r_il"], cfg.lidar_imu_rot)) {
            err = "root r_il must be a 9-element sequence";
            return false;
        }
    }
    if (root["t_il"]) {
        if (!readVector3(root["t_il"], cfg.lidar_imu_trans)) {
            err = "root t_il must be a 3-element sequence";
            return false;
        }
    }

    const YAML::Node fl = root["mapping"]["fast_livo2"];
    cfg.img_en = getBool(fl, "img_en", cfg.img_en);
    cfg.lidar_en = getBool(fl, "lidar_en", cfg.lidar_en);
    cfg.imu_en = getBool(fl, "imu_en", cfg.imu_en);
    cfg.lidar_type = getInt(fl, "lidar_type", cfg.lidar_type);
    cfg.point_filter_num = getInt(fl, "point_filter_num", cfg.point_filter_num);
    cfg.scan_line = getInt(fl, "scan_line", cfg.scan_line);
    cfg.blind = getDouble(fl, "blind", cfg.blind);
    cfg.filter_size_surf = getDouble(fl, "filter_size_surf", cfg.filter_size_surf);

    cfg.imu_int_frame = getInt(fl, "imu_int_frame", cfg.imu_int_frame);
    cfg.acc_cov = getDouble(fl, "acc_cov", cfg.acc_cov);
    cfg.gyr_cov = getDouble(fl, "gyr_cov", cfg.gyr_cov);
    cfg.b_acc_cov = getDouble(fl, "b_acc_cov", cfg.b_acc_cov);
    cfg.b_gyr_cov = getDouble(fl, "b_gyr_cov", cfg.b_gyr_cov);

    cfg.lio_max_iterations = getInt(fl, "lio_max_iterations", cfg.lio_max_iterations);
    cfg.dept_err = getDouble(fl, "dept_err", cfg.dept_err);
    cfg.beam_err = getDouble(fl, "beam_err", cfg.beam_err);
    cfg.min_eigen_value = getDouble(fl, "min_eigen_value", cfg.min_eigen_value);
    cfg.voxel_size = getDouble(fl, "voxel_size", cfg.voxel_size);
    cfg.max_layer = getInt(fl, "max_layer", cfg.max_layer);
    cfg.max_points_num = getInt(fl, "max_points_num", cfg.max_points_num);

    cfg.vio_max_iterations = getInt(fl, "vio_max_iterations", cfg.vio_max_iterations);
    cfg.outlier_threshold = getDouble(fl, "outlier_threshold", cfg.outlier_threshold);
    cfg.img_point_cov = getDouble(fl, "img_point_cov", cfg.img_point_cov);
    cfg.patch_size = getInt(fl, "patch_size", cfg.patch_size);
    cfg.patch_pyrimid_level = getInt(fl, "patch_pyrimid_level", cfg.patch_pyrimid_level);
    cfg.normal_en = getBool(fl, "normal_en", cfg.normal_en);
    cfg.raycast_en = getBool(fl, "raycast_en", cfg.raycast_en);
    cfg.inverse_composition_en = getBool(fl, "inverse_composition_en", cfg.inverse_composition_en);
    cfg.exposure_estimate_en = getBool(fl, "exposure_estimate_en", cfg.exposure_estimate_en);
    cfg.inv_expo_cov = getDouble(fl, "inv_expo_cov", cfg.inv_expo_cov);

    cfg.dense_map_en = getBool(fl, "dense_map_en", cfg.dense_map_en);
    cfg.pub_scan_num = getInt(fl, "pub_scan_num", cfg.pub_scan_num);
    cfg.blind_rgb_points = getDouble(fl, "blind_rgb_points", cfg.blind_rgb_points);
    cfg.pub_effect_point_en = getBool(fl, "pub_effect_point_en", cfg.pub_effect_point_en);
    cfg.pub_plane_en = getBool(fl, "pub_plane_en", cfg.pub_plane_en);
    cfg.map_sliding_en = getBool(fl, "map_sliding_en", cfg.map_sliding_en);
    cfg.half_map_size = getDouble(fl, "half_map_size", cfg.half_map_size);
    cfg.sliding_thresh = getDouble(fl, "sliding_thresh", cfg.sliding_thresh);

    cfg.keyframe_trans_thresh = getDouble(fl, "keyframe_trans_thresh", cfg.keyframe_trans_thresh);
    cfg.keyframe_rot_thresh = getDouble(fl, "keyframe_rot_thresh", cfg.keyframe_rot_thresh);

    const YAML::Node t = fl["time_offset"];
    cfg.lidar_time_offset = getDouble(t, "lidar", cfg.lidar_time_offset);
    cfg.img_time_offset = getDouble(t, "img", cfg.img_time_offset);
    cfg.imu_time_offset = getDouble(t, "imu", cfg.imu_time_offset);

    out = cfg;
    return true;
#endif
}

}  // namespace rosiwit_slam
