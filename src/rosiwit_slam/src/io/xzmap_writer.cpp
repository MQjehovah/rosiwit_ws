#include "io/xzmap_writer.h"

#include <cstdio>
#include <cmath>

namespace rosiwit_slam {
namespace io {

namespace {

inline long long quant(double v) {
    return std::llround(v * 1000.0);
}

inline std::string sanitizeName(const std::string& name) {
    std::string out;
    out.reserve(name.size());
    for (char c : name) {
        if (c == '"' || c == '\\') out += ' ';
        else out += c;
    }
    return out;
}

} // namespace

bool writeXzMap(const std::string& path,
                const std::string& map_name,
                const std::vector<XzMapKeyframe>& keyframes,
                long long* out_point_count) {
    if (keyframes.empty()) return false;

    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) return false;

    bool failed = false;
    std::string buf;
    buf.reserve(1u << 20);

    auto flushBuf = [&]() {
        if (failed || buf.empty()) return;
        if (std::fwrite(buf.data(), 1, buf.size(), fp) != buf.size()) failed = true;
        buf.clear();
    };
    auto emit = [&](const char* s) {
        if (failed) return;
        buf += s;
        if (buf.size() >= (1u << 20)) flushBuf();
    };
    auto emitNum = [&](long long v) {
        if (failed) return;
        buf += std::to_string(v);
    };

    long long total_points = 0;
    for (const auto& kf : keyframes) {
        total_points += static_cast<long long>(kf.points.size());
    }

    emit("{\n");
    emit("  \"map_name\": \"");
    buf += sanitizeName(map_name);
    emit("\",\n");
    emit("  \"points_size\": ");      emitNum(total_points);            emit(",\n");
    emit("  \"key_frame_size\": ");   emitNum((long long)keyframes.size()); emit(",\n");

    // ---- kf_frame_poses ----
    emit("  \"kf_frame_poses\": [\n");
    for (size_t i = 0; i < keyframes.size(); ++i) {
        const XzMapPose& p = keyframes[i].pose;
        emit("    {\"key_frame_id\": "); emitNum((long long)(i + 1));
        emit(", \"x\": ");     emitNum(quant(p.x));
        emit(", \"y\": ");     emitNum(quant(p.y));
        emit(", \"z\": ");     emitNum(quant(p.z));
        emit(", \"roll\": ");  emitNum(quant(p.roll));
        emit(", \"pitch\": "); emitNum(quant(p.pitch));
        emit(", \"yaw\": ");   emitNum(quant(p.yaw));
        emit(i + 1 < keyframes.size() ? "},\n" : "}\n");
        flushBuf();
    }
    emit("  ],\n");

    // ---- key_frame_points ----
    emit("  \"key_frame_points\": [\n");
    for (size_t i = 0; i < keyframes.size(); ++i) {
        const XzMapKeyframe& kf = keyframes[i];
        emit("    {\"key_frame_id\": "); emitNum((long long)(i + 1));
        emit(", \"key_frame_point\": [");
        for (size_t j = 0; j < kf.points.size(); ++j) {
            const Eigen::Vector3f& pt = kf.points[j];
            emit(j ? ",{\"x\":" : "{\"x\":");
            emitNum(quant(pt.x()));
            emit(",\"y\":"); emitNum(quant(pt.y()));
            emit(",\"z\":"); emitNum(quant(pt.z()));
            emit("}");
        }
        emit(i + 1 < keyframes.size() ? "]},\n" : "]}\n");
        flushBuf();
    }
    emit("  ],\n");

    // ---- key_frame_points_2d: 每帧空数组占位 (对齐设备端保存行为) ----
    emit("  \"key_frame_points_2d\": [\n");
    for (size_t i = 0; i < keyframes.size(); ++i) {
        emit("    {\"key_frame_id\": "); emitNum((long long)(i + 1));
        emit(i + 1 < keyframes.size()
                 ? ", \"key_frame_point_2d\": []},\n"
                 : ", \"key_frame_point_2d\": []}\n");
    }
    emit("  ]\n}\n");

    flushBuf();
    if (std::fflush(fp) != 0) failed = true;
    std::fclose(fp);
    if (out_point_count) *out_point_count = total_points;
    return !failed;
}

} // namespace io
} // namespace rosiwit_slam
