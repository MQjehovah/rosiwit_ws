#include "io/xzmap_reader.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace rosiwit_slam {
namespace io {

namespace {

// ---- 极简 JSON 解析 (对象/数组/数值/字符串/布尔/null), 足够解析 xzmap ----
struct JVal {
    enum T { NUL, BOOL, NUM, STR, ARR, OBJ } t = NUL;
    double num = 0.0;
    bool b = false;
    std::string str;
    std::vector<JVal> arr;
    std::vector<std::pair<std::string, JVal>> obj;

    const JVal* get(const std::string& k) const {
        if (t != OBJ) return nullptr;
        for (const auto& p : obj) if (p.first == k) return &p.second;
        return nullptr;
    }
};

class Parser {
public:
    Parser(const char* b, const char* e) : p_(b), e_(e) {}

    bool parse(JVal& out) {
        skip();
        if (!parseValue(out)) return false;
        skip();
        return true;
    }

private:
    const char* p_;
    const char* e_;

    void skip() { while (p_ < e_ && (*p_ == ' ' || *p_ == '\t' || *p_ == '\n' || *p_ == '\r')) ++p_; }

    bool parseValue(JVal& v) {
        skip();
        if (p_ >= e_) return false;
        switch (*p_) {
            case '{': return parseObject(v);
            case '[': return parseArray(v);
            case '"': v.t = JVal::STR; return parseString(v.str);
            case 't': case 'f': return parseBool(v);
            case 'n':
                if (e_ - p_ >= 4 && std::strncmp(p_, "null", 4) == 0) { p_ += 4; v.t = JVal::NUL; return true; }
                return false;
            default: return parseNumber(v);
        }
    }

    bool parseBool(JVal& v) {
        if (e_ - p_ >= 4 && std::strncmp(p_, "true", 4) == 0) { p_ += 4; v.t = JVal::BOOL; v.b = true; return true; }
        if (e_ - p_ >= 5 && std::strncmp(p_, "false", 5) == 0) { p_ += 5; v.t = JVal::BOOL; v.b = false; return true; }
        return false;
    }

    bool parseNumber(JVal& v) {
        char* end = nullptr;
        const double d = std::strtod(p_, &end);
        if (end == p_) return false;
        p_ = end;
        v.t = JVal::NUM;
        v.num = d;
        return true;
    }

    bool parseString(std::string& s) {
        if (p_ >= e_ || *p_ != '"') return false;
        ++p_;
        s.clear();
        while (p_ < e_ && *p_ != '"') {
            if (*p_ == '\\' && p_ + 1 < e_) {
                ++p_;
                switch (*p_) {
                    case 'n': s += '\n'; break;
                    case 't': s += '\t'; break;
                    case 'r': s += '\r'; break;
                    case 'b': s += '\b'; break;
                    case 'f': s += '\f'; break;
                    default: s += *p_; break;  // \" \\ \/ \u 简化处理
                }
                ++p_;
            } else {
                s += *p_++;
            }
        }
        if (p_ >= e_ || *p_ != '"') return false;
        ++p_;
        return true;
    }

    bool parseArray(JVal& v) {
        v.t = JVal::ARR;
        ++p_;  // '['
        skip();
        if (p_ < e_ && *p_ == ']') { ++p_; return true; }
        while (p_ < e_) {
            JVal item;
            if (!parseValue(item)) return false;
            v.arr.push_back(std::move(item));
            skip();
            if (p_ < e_ && *p_ == ',') { ++p_; skip(); continue; }
            if (p_ < e_ && *p_ == ']') { ++p_; return true; }
            return false;
        }
        return false;
    }

    bool parseObject(JVal& v) {
        v.t = JVal::OBJ;
        ++p_;  // '{'
        skip();
        if (p_ < e_ && *p_ == '}') { ++p_; return true; }
        while (p_ < e_) {
            std::string key;
            if (!parseString(key)) return false;
            skip();
            if (p_ >= e_ || *p_ != ':') return false;
            ++p_;
            JVal val;
            if (!parseValue(val)) return false;
            v.obj.emplace_back(std::move(key), std::move(val));
            skip();
            if (p_ < e_ && *p_ == ',') { ++p_; skip(); continue; }
            if (p_ < e_ && *p_ == '}') { ++p_; return true; }
            return false;
        }
        return false;
    }
};

inline double dequant(const JVal* n) { return n ? n->num / 1000.0 : 0.0; }

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
    buf.resize(rd);

    JVal root;
    Parser parser(buf.data(), buf.data() + buf.size());
    if (!parser.parse(root)) return false;

    if (out_map_name) {
        const JVal* mn = root.get("map_name");
        if (mn && mn->t == JVal::STR) *out_map_name = mn->str;
    }

    const JVal* poses = root.get("kf_frame_poses");
    const JVal* pts = root.get("key_frame_points");
    if (!poses || poses->t != JVal::ARR) return false;

    const size_t n = poses->arr.size();
    keyframes.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const JVal& po = poses->arr[i];
        XzMapPose& p = keyframes[i].pose;
        p.x = dequant(po.get("x"));
        p.y = dequant(po.get("y"));
        p.z = dequant(po.get("z"));
        p.roll = dequant(po.get("roll"));
        p.pitch = dequant(po.get("pitch"));
        p.yaw = dequant(po.get("yaw"));
    }

    if (pts && pts->t == JVal::ARR) {
        const size_t m = std::min(n, pts->arr.size());
        for (size_t i = 0; i < m; ++i) {
            const JVal* parr = pts->arr[i].get("key_frame_point");
            if (!parr || parr->t != JVal::ARR) continue;
            auto& out = keyframes[i].points;
            out.reserve(parr->arr.size());
            for (const JVal& ptn : parr->arr) {
                out.emplace_back(static_cast<float>(dequant(ptn.get("x"))),
                                 static_cast<float>(dequant(ptn.get("y"))),
                                 static_cast<float>(dequant(ptn.get("z"))));
            }
        }
    }

    return !keyframes.empty();
}

} // namespace io
} // namespace rosiwit_slam
