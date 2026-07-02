/**
 * @file map_persistence.cpp
 * @brief FAST-LIO2 SLAM - 地图持久化模块实现
 */

#include "fast_lio2_slam/map_manager/map_persistence.h"

#include <sstream>

namespace fast_lio2_slam {

MapPersistence::MapPersistence() {}

MapPersistence::MapPersistence(const PersistenceConfig& config)
    : config_(config) {}

void MapPersistence::initialize(const PersistenceConfig& config) {
    config_ = config;
}

bool MapPersistence::savePointCloud(
    const PointCloudPtr& cloud,
    const std::string& path,
    const std::string& format) {

    if (!cloud || cloud->empty()) return false;

    if (format == "pcd") {
        return savePcd(cloud, path, config_.binary_mode);
    } else if (format == "ply") {
        return savePly(cloud, path, config_.binary_mode);
    } else if (format == "bin") {
        return saveBin(cloud, path);
    }

    return false;
}

bool MapPersistence::savePcd(
    const PointCloudPtr& cloud,
    const std::string& path,
    bool binary) {

    pcl::PCDWriter writer;
    if (binary) {
        return writer.writeBinary(path, *cloud) == 0;
    } else {
        return writer.writeASCII(path, *cloud) == 0;
    }
}

bool MapPersistence::savePly(
    const PointCloudPtr& cloud,
    const std::string& path,
    bool binary) {

    pcl::PLYWriter writer;
    if (binary) {
        return writer.write(path, *cloud, true) == 0;
    } else {
        return writer.write(path, *cloud, false) == 0;
    }
}

bool MapPersistence::saveBin(
    const PointCloudPtr& cloud,
    const std::string& path) {

    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) return false;

    // 写入点数
    size_t num_points = cloud->size();
    file.write(reinterpret_cast<const char*>(&num_points), sizeof(size_t));

    // 写入点云数据
    for (const auto& point : cloud->points) {
        file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
        file.write(reinterpret_cast<const char*>(&point.intensity), sizeof(float));
    }

    file.close();
    return true;
}

PointCloudPtr MapPersistence::loadPointCloud(const std::string& path) {
    std::string format = parseFormat(path);

    if (format == "pcd") {
        return loadPcd(path);
    } else if (format == "ply") {
        return loadPly(path);
    } else if (format == "bin") {
        return loadBin(path);
    }

    return nullptr;
}

PointCloudPtr MapPersistence::loadPcd(const std::string& path) {
    PointCloudPtr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile(path, *cloud) == 0) {
        return cloud;
    }
    return nullptr;
}

PointCloudPtr MapPersistence::loadPly(const std::string& path) {
    PointCloudPtr cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPLYFile(path, *cloud) == 0) {
        return cloud;
    }
    return nullptr;
}

PointCloudPtr MapPersistence::loadBin(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) return nullptr;

    PointCloudPtr cloud(new pcl::PointCloud<PointType>);

    // 读取点数
    size_t num_points;
    file.read(reinterpret_cast<char*>(&num_points), sizeof(size_t));

    cloud->resize(num_points);

    // 读取点云数据
    for (size_t i = 0; i < num_points; ++i) {
        float x, y, z, intensity;
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        file.read(reinterpret_cast<char*>(&intensity), sizeof(float));

        cloud->points[i].x = x;
        cloud->points[i].y = y;
        cloud->points[i].z = z;
        cloud->points[i].intensity = intensity;
    }

    file.close();
    return cloud;
}

bool MapPersistence::saveMetadata(
    const MapMetadata& metadata,
    const std::string& path) {

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "map";
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << metadata.map_name;
    out << YAML::Key << "version" << YAML::Value << metadata.version;
    out << YAML::Key << "created_time" << YAML::Value << metadata.created_time;
    out << YAML::Key << "modified_time" << YAML::Value << metadata.modified_time;
    out << YAML::EndMap;

    out << YAML::Key << "size";
    out << YAML::BeginMap;
    out << YAML::Key << "total_points" << YAML::Value << metadata.total_points;
    out << YAML::Key << "total_submaps" << YAML::Value << metadata.total_submaps;
    out << YAML::Key << "total_sessions" << YAML::Value << metadata.total_sessions;
    out << YAML::EndMap;

    out << YAML::Key << "center";
    writeYamlNode(out, "center", metadata.map_center);

    out << YAML::Key << "bounds";
    out << YAML::BeginMap;
    Eigen::Vector3d min_bound = metadata.map_center - metadata.map_size / 2;
    Eigen::Vector3d max_bound = metadata.map_center + metadata.map_size / 2;
    writeYamlNode(out, "min", min_bound);
    writeYamlNode(out, "max", max_bound);
    out << YAML::EndMap;

    out << YAML::Key << "quality";
    out << YAML::BeginMap;
    out << YAML::Key << "avg_density" << YAML::Value << metadata.avg_point_density;
    out << YAML::Key << "quality_score" << YAML::Value << metadata.map_quality_score;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream file(path);
    if (!file.is_open()) return false;
    file << out.c_str();
    file.close();

    return true;
}

MapMetadata MapPersistence::loadMetadata(const std::string& path) {
    MapMetadata metadata;

    try {
        YAML::Node yaml = YAML::LoadFile(path);

        if (yaml["map"]) {
            metadata.map_name = yaml["map"]["name"].as<std::string>();
            metadata.version = yaml["map"]["version"].as<std::string>();
            metadata.created_time = yaml["map"]["created_time"].as<double>();
            metadata.modified_time = yaml["map"]["modified_time"].as<double>();
        }

        if (yaml["size"]) {
            metadata.total_points = yaml["size"]["total_points"].as<int>();
            metadata.total_submaps = yaml["size"]["total_submaps"].as<int>();
            metadata.total_sessions = yaml["size"]["total_sessions"].as<int>();
        }

        if (yaml["center"]) {
            metadata.map_center = readYamlVector(yaml["center"]);
        }

        if (yaml["bounds"]) {
            Eigen::Vector3d min_bound = readYamlVector(yaml["bounds"]["min"]);
            Eigen::Vector3d max_bound = readYamlVector(yaml["bounds"]["max"]);
            metadata.map_size = max_bound - min_bound;
        }

        if (yaml["quality"]) {
            metadata.avg_point_density = yaml["quality"]["avg_density"].as<double>();
            metadata.map_quality_score = yaml["quality"]["quality_score"].as<double>();
        }
    } catch (const YAML::Exception& e) {
        // 解析失败，返回默认值
    }

    return metadata;
}

void MapPersistence::writeYamlNode(
    YAML::Emitter& out,
    const std::string& key,
    const Eigen::Vector3d& vec) {

    out << YAML::Key << key;
    out << YAML::BeginSeq;
    out << vec.x();
    out << vec.y();
    out << vec.z();
    out << YAML::EndSeq;
}

Eigen::Vector3d MapPersistence::readYamlVector(const YAML::Node& node) {
    Eigen::Vector3d vec;
    if (node.IsSequence() && node.size() >= 3) {
        vec.x() = node[0].as<double>();
        vec.y() = node[1].as<double>();
        vec.z() = node[2].as<double>();
    }
    return vec;
}

void MapPersistence::writeYamlSE3(YAML::Emitter& out, const SE3d& pose) {
    out << YAML::BeginMap;

    // 平移
    out << YAML::Key << "translation";
    out << YAML::BeginSeq;
    out << pose.translation().x();
    out << pose.translation().y();
    out << pose.translation().z();
    out << YAML::EndSeq;

    // 旋转 (四元数)
    out << YAML::Key << "rotation";
    out << YAML::BeginSeq;
    out << pose.so3().unit_quaternion().x();
    out << pose.so3().unit_quaternion().y();
    out << pose.so3().unit_quaternion().z();
    out << pose.so3().unit_quaternion().w();
    out << YAML::EndSeq;

    out << YAML::EndMap;
}

SE3d MapPersistence::readYamlSE3(const YAML::Node& node) {
    Eigen::Vector3d trans = readYamlVector(node["translation"]);

    auto q_node = node["rotation"];
    Eigen::Quaterniond quat;
    if (q_node.IsSequence() && q_node.size() >= 4) {
        quat.x() = q_node[0].as<double>();
        quat.y() = q_node[1].as<double>();
        quat.z() = q_node[2].as<double>();
        quat.w() = q_node[3].as<double>();
    }

    return SE3d(quat, trans);
}

bool MapPersistence::saveSession(
    const SessionInfo& session,
    const std::string& path) {

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "session_id" << YAML::Value << session.session_id;
    out << YAML::Key << "name" << YAML::Value << session.name;
    out << YAML::Key << "start_time" << YAML::Value << session.start_time;
    out << YAML::Key << "end_time" << YAML::Value << session.end_time;
    out << YAML::Key << "frame_count" << YAML::Value << session.frame_count;

    out << YAML::Key << "start_pose";
    writeYamlSE3(out, session.start_pose);

    out << YAML::Key << "end_pose";
    writeYamlSE3(out, session.end_pose);

    out << YAML::Key << "submap_ids";
    out << YAML::BeginSeq;
    for (int id : session.submap_ids) {
        out << id;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "is_merged" << YAML::Value << session.is_merged;

    out << YAML::EndMap;

    std::ofstream file(path);
    if (!file.is_open()) return false;
    file << out.c_str();
    file.close();

    return true;
}

SessionInfo MapPersistence::loadSession(const std::string& path) {
    SessionInfo session;

    try {
        YAML::Node yaml = YAML::LoadFile(path);

        session.session_id = yaml["session_id"].as<std::string>();
        session.name = yaml["name"].as<std::string>();
        session.start_time = yaml["start_time"].as<double>();
        session.end_time = yaml["end_time"].as<double>();
        session.frame_count = yaml["frame_count"].as<int>();

        if (yaml["start_pose"]) {
            session.start_pose = readYamlSE3(yaml["start_pose"]);
        }
        if (yaml["end_pose"]) {
            session.end_pose = readYamlSE3(yaml["end_pose"]);
        }

        if (yaml["submap_ids"]) {
            for (const auto& id : yaml["submap_ids"]) {
                session.submap_ids.push_back(id.as<int>());
            }
        }

        session.is_merged = yaml["is_merged"].as<bool>();
    } catch (const YAML::Exception& e) {
        // 解析失败
    }

    return session;
}

bool MapPersistence::savePoseGraph(
    const std::vector<SE3d>& poses,
    const std::vector<std::tuple<int, int, SE3d>>& constraints,
    const std::string& path) {

    std::ofstream file(path);
    if (!file.is_open()) return false;

    // G2O格式
    // 顶点: VERTEX_SE3:QUAT id x y z qx qy qz qw
    // 边: EDGE_SE3:QUAT id1 id2 x y z qx qy qz qw info_xx ...

    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& pose = poses[i];
        file << "VERTEX_SE3:QUAT " << i << " "
             << pose.translation().x() << " "
             << pose.translation().y() << " "
             << pose.translation().z() << " "
             << pose.so3().unit_quaternion().x() << " "
             << pose.so3().unit_quaternion().y() << " "
             << pose.so3().unit_quaternion().z() << " "
             << pose.so3().unit_quaternion().w() << "\n";
    }

    // 信息矩阵 (简化)
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity() * 100;

    for (const auto& [from, to, rel_pose] : constraints) {
        file << "EDGE_SE3:QUAT " << from << " " << to << " "
             << rel_pose.translation().x() << " "
             << rel_pose.translation().y() << " "
             << rel_pose.translation().z() << " "
             << rel_pose.so3().unit_quaternion().x() << " "
             << rel_pose.so3().unit_quaternion().y() << " "
             << rel_pose.so3().unit_quaternion().z() << " "
             << rel_pose.so3().unit_quaternion().w();

        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                file << " " << info(i, j);
            }
        }
        file << "\n";
    }

    file.close();
    return true;
}

std::pair<std::vector<SE3d>, std::vector<std::tuple<int, int, SE3d>>>
MapPersistence::loadPoseGraph(const std::string& path) {
    std::vector<SE3d> poses;
    std::vector<std::tuple<int, int, SE3d>> constraints;

    std::ifstream file(path);
    if (!file.is_open()) return {poses, constraints};

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "VERTEX_SE3:QUAT") {
            int id;
            double x, y, z, qx, qy, qz, qw;
            iss >> id >> x >> y >> z >> qx >> qy >> qz >> qw;

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Vector3d trans(x, y, z);
            poses.push_back(SE3d(quat, trans));
        } else if (type == "EDGE_SE3:QUAT") {
            int from, to;
            double x, y, z, qx, qy, qz, qw;
            iss >> from >> to >> x >> y >> z >> qx >> qy >> qz >> qw;

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Vector3d trans(x, y, z);
            constraints.push_back({from, to, SE3d(quat, trans)});
        }
    }

    file.close();
    return {poses, constraints};
}

bool MapPersistence::saveTrajectory(
    const std::vector<SE3d>& poses,
    const std::vector<double>& timestamps,
    const std::string& path) {

    // 保存位姿
    std::ofstream pose_file(path + "/poses.txt");
    if (!pose_file.is_open()) return false;

    for (size_t i = 0; i < poses.size(); ++i) {
        const auto& pose = poses[i];
        pose_file << i << " "
                  << pose.translation().x() << " "
                  << pose.translation().y() << " "
                  << pose.translation().z() << " "
                  << pose.so3().unit_quaternion().x() << " "
                  << pose.so3().unit_quaternion().y() << " "
                  << pose.so3().unit_quaternion().z() << " "
                  << pose.so3().unit_quaternion().w() << "\n";
    }
    pose_file.close();

    // 保存时间戳
    std::ofstream time_file(path + "/timestamps.txt");
    if (!time_file.is_open()) return false;

    for (size_t i = 0; i < timestamps.size(); ++i) {
        time_file << i << " " << timestamps[i] << "\n";
    }
    time_file.close();

    return true;
}

std::pair<std::vector<SE3d>, std::vector<double>>
MapPersistence::loadTrajectory(const std::string& path) {
    std::vector<SE3d> poses;
    std::vector<double> timestamps;

    // 加载位姿
    std::ifstream pose_file(path + "/poses.txt");
    if (pose_file.is_open()) {
        std::string line;
        while (std::getline(pose_file, line)) {
            std::istringstream iss(line);
            int id;
            double x, y, z, qx, qy, qz, qw;
            iss >> id >> x >> y >> z >> qx >> qy >> qz >> qw;

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Vector3d trans(x, y, z);
            poses.push_back(SE3d(quat, trans));
        }
        pose_file.close();
    }

    // 加载时间戳
    std::ifstream time_file(path + "/timestamps.txt");
    if (time_file.is_open()) {
        std::string line;
        while (std::getline(time_file, line)) {
            std::istringstream iss(line);
            int id;
            double ts;
            iss >> id >> ts;
            timestamps.push_back(ts);
        }
        time_file.close();
    }

    return {poses, timestamps};
}

bool MapPersistence::createProjectDirectory(const std::string& directory) {
    try {
        std::filesystem::create_directories(directory);
        std::filesystem::create_directories(directory + "/" + config_.submap_directory);
        std::filesystem::create_directories(directory + "/" + config_.session_directory);
        std::filesystem::create_directories(directory + "/" + config_.trajectory_directory);
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        return false;
    }
}

bool MapPersistence::fileExists(const std::string& path) const {
    return std::filesystem::exists(path);
}

bool MapPersistence::directoryExists(const std::string& path) const {
    return std::filesystem::is_directory(path);
}

std::string MapPersistence::parseFormat(const std::string& path) const {
    size_t pos = path.find_last_of('.');
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return "";
}

std::vector<std::string> MapPersistence::getSupportedFormats() const {
    return {"pcd", "ply", "bin"};
}

size_t MapPersistence::getFileSize(const std::string& path) const {
    try {
        return std::filesystem::file_size(path);
    } catch (const std::filesystem::filesystem_error& e) {
        return 0;
    }
}

bool MapPersistence::deleteFile(const std::string& path) {
    try {
        return std::filesystem::remove(path);
    } catch (const std::filesystem::filesystem_error& e) {
        return false;
    }
}

std::string MapPersistence::createBackup(const std::string& source) {
    std::string timestamp = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    std::string backup_path = config_.backup_directory + "/backup_" + timestamp;

    try {
        std::filesystem::create_directories(backup_path);
        std::filesystem::copy(source, backup_path,
                             std::filesystem::copy_options::recursive);
        return backup_path;
    } catch (const std::filesystem::filesystem_error& e) {
        return "";
    }
}

bool MapProject::checkIntegrity() const {
    // 检查必要文件是否存在
    std::filesystem::path base(path);

    return std::filesystem::exists(base / metadata_file) &&
           std::filesystem::exists(base / global_map_file);
}

MapProject MapPersistence::exportMapProject(
    std::shared_ptr<MapManager> map_manager,
    const std::string& directory) {

    MapProject project;
    project.name = "fast_lio2_map";
    project.path = directory;
    project.version = "1.0";

    if (!map_manager) return project;

    // 创建目录结构
    createProjectDirectory(directory);

    // 保存全局地图
    auto cloud = map_manager->getFullMap();
    if (cloud && !cloud->empty()) {
        project.global_map_file = "global_map.pcd";
        savePointCloud(cloud, directory + "/" + project.global_map_file, "pcd");
    }

    // 保存元数据
    auto metadata = map_manager->getMetadata();
    project.metadata_file = config_.metadata_file;
    saveMetadata(metadata, directory + "/" + project.metadata_file);

    // 保存子地图
    auto submaps = map_manager->getAllSubmaps();
    for (size_t i = 0; i < submaps.size(); ++i) {
        std::string filename = "submap_" + std::to_string(i) + ".pcd";
        project.submap_files.push_back(filename);
        saveSubmap(submaps[i],
                   directory + "/" + config_.submap_directory + "/" + filename);
    }

    // 保存会话
    // MapManager不提供getAllSessions方法，直接返回空列表
    std::vector<SessionInfo> sessions;  // 暂时返回空列表
    for (const auto& session : sessions) {
        std::string filename = session.session_id + ".yaml";
        project.session_files.push_back(filename);
        saveSession(session,
                    directory + "/" + config_.session_directory + "/" + filename);
    }

    return project;
}

bool MapPersistence::importMapProject(
    const std::string& directory,
    std::shared_ptr<MapManager> map_manager) {

    if (!map_manager) return false;

    // 加载元数据
    std::string metadata_path = directory + "/" + config_.metadata_file;
    if (!fileExists(metadata_path)) return false;

    auto metadata = loadMetadata(metadata_path);

    // 加载全局地图
    std::string map_path = directory + "/global_map.pcd";
    if (fileExists(map_path)) {
        auto cloud = loadPointCloud(map_path);
        if (cloud && !cloud->empty()) {
            // 添加到地图管理器
            map_manager->addPointCloud(cloud, SE3d(), 0);
        }
    }

    // 加载子地图
    std::string submap_dir = directory + "/" + config_.submap_directory;
    if (directoryExists(submap_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(submap_dir)) {
            if (entry.path().extension() == ".pcd") {
                auto submap = loadSubmap(entry.path().string());
                // 子地图加载逻辑
            }
        }
    }

    // 加载会话
    std::string session_dir = directory + "/" + config_.session_directory;
    if (directoryExists(session_dir)) {
        for (const auto& entry : std::filesystem::directory_iterator(session_dir)) {
            if (entry.path().extension() == ".yaml") {
                auto session = loadSession(entry.path().string());
                // 会话加载逻辑
            }
        }
    }

    return true;
}

bool MapPersistence::saveSubmap(const Submap& submap, const std::string& path) {
    // 保存点云
    if (!savePointCloud(submap.cloud, path + ".pcd", "pcd")) {
        return false;
    }

    // 保存元数据
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << submap.id;
    out << YAML::Key << "session_id" << YAML::Value << submap.session_id;
    out << YAML::Key << "is_active" << YAML::Value << submap.is_active;
    out << YAML::Key << "timestamp_start" << YAML::Value << submap.timestamp_start;
    out << YAML::Key << "timestamp_end" << YAML::Value << submap.timestamp_end;

    out << YAML::Key << "center_pose";
    writeYamlSE3(out, submap.center_pose);

    out << YAML::Key << "bounds";
    out << YAML::BeginMap;
    writeYamlNode(out, "min", submap.min_bound);
    writeYamlNode(out, "max", submap.max_bound);
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream file(path + ".meta");
    if (!file.is_open()) return false;
    file << out.c_str();
    file.close();

    return true;
}

Submap MapPersistence::loadSubmap(const std::string& path) {
    Submap submap;

    // 加载点云
    submap.cloud = loadPointCloud(path);

    // 加载元数据
    std::string meta_path = path.substr(0, path.find_last_of('.')) + ".meta";
    if (fileExists(meta_path)) {
        try {
            YAML::Node yaml = YAML::LoadFile(meta_path);

            submap.id = yaml["id"].as<int>();
            submap.session_id = yaml["session_id"].as<std::string>();
            submap.is_active = yaml["is_active"].as<bool>();
            submap.timestamp_start = yaml["timestamp_start"].as<double>();
            submap.timestamp_end = yaml["timestamp_end"].as<double>();

            if (yaml["center_pose"]) {
                submap.center_pose = readYamlSE3(yaml["center_pose"]);
            }

            if (yaml["bounds"]) {
                submap.min_bound = readYamlVector(yaml["bounds"]["min"]);
                submap.max_bound = readYamlVector(yaml["bounds"]["max"]);
            }
        } catch (const YAML::Exception& e) {
            // 解析失败
        }
    }

    return submap;
}

bool MapPersistence::validateMapFile(const std::string& path) const {
    if (!fileExists(path)) return false;

    std::string format = parseFormat(path);
    if (format != "pcd" && format != "ply" && format != "bin") {
        return false;
    }

    // 检查文件大小是否合理
    size_t size = getFileSize(path);
    if (size < 100) return false;  // 太小可能是空文件

    return true;
}

} // namespace fast_lio2_slam
