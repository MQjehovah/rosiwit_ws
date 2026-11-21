#include "xzmap_reader.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

namespace rosiwit_slam {
namespace io {

namespace {

// ---------------------------------------------------------------------------
// 流式 JSON 扫描器
//
// 历史实现把整份 xzmap 解析成 JVal 树: 每个点 {x,y,z} 都是一个带
// std::string/vector 的节点. 6M 点的图实测峰值 ~4.5GB (设备 202MB xzmap),
// 释放后 glibc arena 还会留住这些 free chunk, 常驻 4.6GB.
//
// 这里只做"按 key 定位 + 直接消费", 点云不进树:
//   * 点对象按 x/y/z 单字符 key 就地解析, 不做 std::string 分配
//   * 关键帧对象只认 key_frame_point, 其它 key 用 skipValue 跳过
//   * 位姿/地图名很小, 允许用 std::string
// ---------------------------------------------------------------------------
class Scanner {
public:
    Scanner(const char* b, const char* e) : p_(b), e_(e) {}

    const char* ptr() const { return p_; }

    void skipWs() {
        while (p_ < e_ && (*p_ == ' ' || *p_ == '\t' || *p_ == '\n' || *p_ == '\r')) ++p_;
    }

    char peek() {
        skipWs();
        return p_ < e_ ? *p_ : '\0';
    }

    bool consume(char c) {
        skipWs();
        if (p_ < e_ && *p_ == c) { ++p_; return true; }
        return false;
    }

    bool expect(char c) { return consume(c); }

    /// 读 JSON 字符串到 out (key/地图名用, 调用次数很少).
    bool parseString(std::string& out) {
        skipWs();
        if (p_ >= e_ || *p_ != '"') return false;
        ++p_;
        out.clear();
        while (p_ < e_ && *p_ != '"') {
            if (*p_ == '\\' && p_ + 1 < e_) {
                ++p_;
                switch (*p_) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    default: out += *p_; break;
                }
                ++p_;
            } else {
                out += *p_++;
            }
        }
        if (p_ >= e_) return false;
        ++p_;
        return true;
    }

    /// 读单字符 key (点对象的 x/y/z), 不分配内存.
    bool parseShortKey(char& c) {
        skipWs();
        if (p_ >= e_ || *p_ != '"') return false;
        ++p_;
        if (p_ >= e_ || *p_ == '"') return false;
        c = *p_++;
        while (p_ < e_ && *p_ != '"') ++p_;
        if (p_ >= e_) return false;
        ++p_;
        return true;
    }

    bool parseNumber(double& v) {
        skipWs();
        if (p_ >= e_) return false;
        char* end = nullptr;
        v = std::strtod(p_, &end);
        if (end == p_ || end > e_) return false;
        p_ = end;
        return true;
    }

    /// 跳过任意 JSON 值 (对象/数组/字符串/数字/true/false/null).
    bool skipValue() {
        skipWs();
        if (p_ >= e_) return false;
        if (*p_ == '"') {
            ++p_;
            while (p_ < e_ && *p_ != '"') {
                if (*p_ == '\\' && p_ + 1 < e_) ++p_;
                ++p_;
            }
            if (p_ >= e_) return false;
            ++p_;
            return true;
        }
        if (*p_ == '{') {
            ++p_;
            while (true) {
                skipWs();
                if (p_ < e_ && *p_ == '}') { ++p_; return true; }
                std::string k;
                if (!parseString(k)) return false;
                skipWs();
                if (p_ >= e_ || *p_ != ':') return false;
                ++p_;
                if (!skipValue()) return false;
                skipWs();
                if (p_ < e_ && *p_ == ',') { ++p_; continue; }
                if (p_ < e_ && *p_ == '}') { ++p_; return true; }
                return false;
            }
        }
        if (*p_ == '[') {
            ++p_;
            while (true) {
                skipWs();
                if (p_ < e_ && *p_ == ']') { ++p_; return true; }
                if (!skipValue()) return false;
                skipWs();
                if (p_ < e_ && *p_ == ',') { ++p_; continue; }
                if (p_ < e_ && *p_ == ']') { ++p_; return true; }
                return false;
            }
        }
        if (*p_ == 't' || *p_ == 'f' || *p_ == 'n') {
            while (p_ < e_ && ((*p_ >= 'a' && *p_ <= 'z') || (*p_ >= 'A' && *p_ <= 'Z'))) ++p_;
            return true;
        }
        double v;
        return parseNumber(v);
    }

    /// 从当前位置向后找 "key": (JSON 字符串 + 冒号). 找到后指针停在值前.
    bool findKey(const char* key) {
        const size_t n = std::strlen(key);
        while (p_ < e_) {
            if (*p_ == '"') {
                const char* save = p_;
                ++p_;
                const char* s = p_;
                while (p_ < e_ && *p_ != '"') {
                    if (*p_ == '\\' && p_ + 1 < e_) ++p_;
                    ++p_;
                }
                if (p_ < e_ && static_cast<size_t>(p_ - s) == n &&
                    std::memcmp(s, key, n) == 0) {
                    ++p_;
                    skipWs();
                    if (p_ < e_ && *p_ == ':') { ++p_; return true; }
                }
                if (p_ < e_) ++p_;
                else p_ = save;
            } else {
                ++p_;
            }
        }
        return false;
    }

private:
    const char* p_;
    const char* e_;
};

inline double dequant(double v) { return v / 1000.0; }

bool parsePoint(Scanner& sc, float& x, float& y, float& z) {
    if (!sc.expect('{')) return false;
    bool hx = false, hy = false, hz = false;
    while (true) {
        sc.skipWs();
        if (sc.consume('}')) break;
        if (sc.peek() != '"') return false;
        char k = 0;
        if (!sc.parseShortKey(k)) return false;
        if (!sc.expect(':')) return false;
        if (k == 'x' || k == 'y' || k == 'z') {
            double v = 0.0;
            if (!sc.parseNumber(v)) return false;
            if (k == 'x') { x = static_cast<float>(dequant(v)); hx = true; }
            else if (k == 'y') { y = static_cast<float>(dequant(v)); hy = true; }
            else { z = static_cast<float>(dequant(v)); hz = true; }
        } else if (!sc.skipValue()) {
            return false;
        }
        sc.skipWs();
        if (sc.consume(',')) continue;
        sc.skipWs();
        if (sc.consume('}')) break;
        return false;
    }
    return hx && hy && hz;
}

bool parsePointArray(Scanner& sc, std::vector<Eigen::Vector3f>& out) {
    if (!sc.expect('[')) return false;
    while (true) {
        sc.skipWs();
        if (sc.consume(']')) return true;
        if (sc.peek() == '{') {
            float x = 0.f, y = 0.f, z = 0.f;
            if (!parsePoint(sc, x, y, z)) return false;
            out.emplace_back(x, y, z);
        } else if (!sc.skipValue()) {
            return false;
        }
        sc.skipWs();
        if (sc.consume(',')) continue;
        sc.skipWs();
        if (sc.consume(']')) return true;
        return false;
    }
}

bool parsePose(Scanner& sc, XzMapPose& p) {
    if (!sc.expect('{')) return false;
    double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
    while (true) {
        sc.skipWs();
        if (sc.consume('}')) break;
        std::string k;
        if (!sc.parseString(k)) return false;
        if (!sc.expect(':')) return false;
        if (k == "x" || k == "y" || k == "z" || k == "roll" || k == "pitch" || k == "yaw") {
            double v = 0.0;
            if (!sc.parseNumber(v)) return false;
            if (k == "x") x = dequant(v);
            else if (k == "y") y = dequant(v);
            else if (k == "z") z = dequant(v);
            else if (k == "roll") roll = dequant(v);
            else if (k == "pitch") pitch = dequant(v);
            else yaw = dequant(v);
        } else if (!sc.skipValue()) {
            return false;
        }
        sc.skipWs();
        if (sc.consume(',')) continue;
        sc.skipWs();
        if (sc.consume('}')) break;
        return false;
    }
    p.x = x; p.y = y; p.z = z;
    p.roll = roll; p.pitch = pitch; p.yaw = yaw;
    return true;
}

} // namespace

bool readXzMap(const std::string& path,
               std::vector<XzMapKeyframe>& keyframes,
               std::string* out_map_name) {
    keyframes.clear();

    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) return false;
    std::fseek(fp, 0, SEEK_END);
    const long sz = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    if (sz <= 0) { std::fclose(fp); return false; }
    std::string buf(static_cast<size_t>(sz), '\0');
    const size_t rd = std::fread(&buf[0], 1, static_cast<size_t>(sz), fp);
    std::fclose(fp);
    if (rd == 0) return false;
    buf.resize(rd);

    const char* begin = buf.data();
    const char* end = buf.data() + buf.size();

    // ---- map_name (很小) ----
    if (out_map_name) {
        Scanner sc(begin, end);
        std::string name;
        if (sc.findKey("map_name") && sc.parseString(name)) *out_map_name = name;
    }

    // ---- 位姿: 先定关键帧数量, 再逐点流式填 ----
    std::vector<XzMapPose> poses;
    {
        Scanner sc(begin, end);
        if (sc.findKey("kf_frame_poses") && sc.expect('[')) {
            while (true) {
                sc.skipWs();
                if (sc.consume(']')) break;
                if (sc.peek() == '{') {
                    XzMapPose p;
                    if (!parsePose(sc, p)) return false;
                    poses.push_back(p);
                } else if (!sc.skipValue()) {
                    return false;
                }
                sc.skipWs();
                if (sc.consume(',')) continue;
                sc.skipWs();
                if (sc.consume(']')) break;
                return false;
            }
        }
    }
    if (poses.empty()) return false;
    keyframes.resize(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) keyframes[i].pose = poses[i];

    // ---- 点云: 不再建 JVal 树, 逐帧直接写入 keyframes[i].points ----
    Scanner sc(begin, end);
    if (!sc.findKey("key_frame_points") || !sc.expect('[')) return false;
    size_t idx = 0;
    while (true) {
        sc.skipWs();
        if (sc.consume(']')) break;
        if (sc.peek() != '{') {
            if (!sc.skipValue()) return false;
        } else {
            sc.expect('{');
            while (true) {
                sc.skipWs();
                if (sc.consume('}')) break;
                std::string k;
                if (!sc.parseString(k)) return false;
                if (!sc.expect(':')) return false;
                if (k == "key_frame_point" && idx < keyframes.size()) {
                    if (!parsePointArray(sc, keyframes[idx].points)) return false;
                } else if (!sc.skipValue()) {
                    return false;
                }
                sc.skipWs();
                if (sc.consume(',')) continue;
                sc.skipWs();
                if (sc.consume('}')) break;
                return false;
            }
            ++idx;
        }
        sc.skipWs();
        if (sc.consume(',')) continue;
        sc.skipWs();
        if (sc.consume(']')) break;
        return false;
    }

    return true;
}

} // namespace io
} // namespace rosiwit_slam
