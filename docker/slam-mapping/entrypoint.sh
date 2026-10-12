#!/usr/bin/env bash
# rosiwit_slam 离线建图入口:
#   1. 启动 rosiwit_slam (mapping 模式, use_sim_time=true)
#   2. 播放 rosbag2 (--clock), 每秒打印建图进度
#   3. 播完调用 /save_map 保存 PCD (可选保存 2D 栅格), 正常退出
set -eo pipefail
# 注意: 不能加 -u, ROS 的 setup.bash 会引用未定义变量

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

BAG_PATH="${BAG_PATH:-/data/bag}"
CONFIG_FILE="${CONFIG_FILE:-livox.yaml}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
RATE="${RATE:-1.0}"
OUTPUT_DIR="${OUTPUT_DIR:-/output}"
MAP_NAME="${MAP_NAME:-map}"
GRID_MAP="${GRID_MAP:-1}"

mkdir -p "$OUTPUT_DIR"

# ---------- 0. 录包归一化 ----------
# 支持: rosbag2目录 / *.zip / 裸*.db3(自动生成metadata.yaml) / *.bag(ROS1, 自动转换)
# 注意: 必须写进 if 条件, 否则 set -e 会在脚本捕获错误前直接退出
if ! python3 /ws/scripts/normalize_bag.py "$BAG_PATH" > /tmp/bag_dir 2> /tmp/bag_err; then
    echo "[MAP-ERROR] 录包归一化失败:"
    cat /tmp/bag_err >&2
    exit 1
fi
BAG_DIR=$(cat /tmp/bag_dir)

# 从 metadata.yaml 读取包总时长(秒)
DURATION=$(python3 - "$BAG_DIR/metadata.yaml" <<'PYEOF'
import sys, yaml
info = yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]
print(info["duration"]["nanoseconds"] / 1e9)
PYEOF
)

echo "[MAP-START] bag=$BAG_DIR duration=${DURATION}s rate=${RATE} config=${CONFIG_FILE}"

# ---------- 1. 启动 SLAM 节点 ----------
ros2 launch rosiwit_slam slam.launch.py \
    config_file:="$CONFIG_FILE" \
    lidar_topic:="$LIDAR_TOPIC" \
    imu_topic:="$IMU_TOPIC" \
    use_sim_time:=true \
    use_rviz:=false >/tmp/slam_node.log 2>&1 &
SLAM_PID=$!
trap 'kill "$SLAM_PID" 2>/dev/null || true' EXIT

for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q '^/save_map$'; then break; fi
    if ! kill -0 "$SLAM_PID" 2>/dev/null; then
        echo "[MAP-ERROR] slam 节点启动失败:"
        tail -n 50 /tmp/slam_node.log
        exit 1
    fi
    sleep 1
done
echo "[MAP-READY] slam 节点就绪 (运行日志: /tmp/slam_node.log)"

# ---------- 2. 播包 + 进度上报 ----------
ros2 bag play "$BAG_DIR" --clock --rate "$RATE" &
PLAY_PID=$!

START=$SECONDS
LAST_PCT=-1
while kill -0 "$PLAY_PID" 2>/dev/null; do
    ELAPSED=$((SECONDS - START))
    PCT=$(awk -v e="$ELAPSED" -v r="$RATE" -v d="$DURATION" \
        'BEGIN{p = (d > 0) ? e * r * 100 / d : 0; if (p > 100) p = 100; printf "%d", p}')
    if [ "$PCT" != "$LAST_PCT" ]; then
        echo "[MAP-PROGRESS] ${PCT}% (已播放 ${ELAPSED}s / 包时长 ${DURATION}s)"
        LAST_PCT="$PCT"
    fi
    sleep 1
done
wait "$PLAY_PID" || true

echo "[MAP-PROGRESS] 100% 播包完成, 等待 SLAM 收尾..."
sleep 8

# ---------- 3. 保存地图 ----------
# 注意: /save_map 的参数是"文件名"(节点会拼上 map_directory, 默认为节点工作目录 /ws);
#       /save_grid_map 的参数是完整路径。两者语义不同。
PCD_NAME="$MAP_NAME.pcd"
echo "[MAP-SAVE] 调用 /save_map (文件名: $PCD_NAME)"
SAVE_OUT=$(timeout 300 ros2 service call /save_map rosiwit_slam/srv/SaveMap "{path: '$PCD_NAME'}" 2>&1) || {
    echo "[MAP-ERROR] /save_map 调用超时或失败"
    tail -n 30 /tmp/slam_node.log
    exit 1
}
if echo "$SAVE_OUT" | grep -q "success=True"; then
    cp "/ws/$PCD_NAME" "$OUTPUT_DIR/$PCD_NAME"
    cp "/ws/$PCD_NAME.scdb" "$OUTPUT_DIR/" 2>/dev/null || true
    echo "[MAP-SAVED] PCD: $OUTPUT_DIR/$PCD_NAME"
    if [ -f "/ws/$MAP_NAME.xzmap" ]; then
        cp "/ws/$MAP_NAME.xzmap" "$OUTPUT_DIR/"
        echo "[MAP-SAVED] XZMAP: $OUTPUT_DIR/$MAP_NAME.xzmap"
    else
        echo "[MAP-WARN] 未找到 $MAP_NAME.xzmap"
    fi
else
    echo "[MAP-ERROR] save_map 返回失败: $SAVE_OUT"
    tail -n 30 /tmp/slam_node.log
    exit 1
fi

if [ "$GRID_MAP" = "1" ]; then
    PGM_PATH="$OUTPUT_DIR/$MAP_NAME.pgm"
    YAML_PATH="$OUTPUT_DIR/$MAP_NAME.yaml"
    echo "[MAP-SAVE] 调用 /save_grid_map -> $PGM_PATH (res 0.05)"
    GRID_OUT=$(timeout 300 ros2 service call /save_grid_map rosiwit_slam/srv/SaveGridMap \
        "{pgm_path: '$PGM_PATH', yaml_path: '$YAML_PATH', resolution: 0.05}" 2>&1) || true
    if echo "$GRID_OUT" | grep -q "success=True"; then
        echo "[MAP-SAVED] 2D栅格: $PGM_PATH"
    else
        echo "[MAP-WARN] 2D栅格保存失败(不影响PCD): $GRID_OUT"
    fi
fi

# ---------- 4. PNG 预览 + manifest.json ----------
python3 - "$OUTPUT_DIR" "$MAP_NAME" <<'PYEOF' || echo "[MAP-WARN] PNG 预览生成失败(不影响其他产物)"
import sys, os
from PIL import Image, ImageOps
out, name = sys.argv[1], sys.argv[2]
pgm, png = os.path.join(out, name + ".pgm"), os.path.join(out, name + ".png")
img = Image.open(pgm)
if max(img.size) > 2048:
    s = 2048 / max(img.size)
    img = img.resize((max(1, int(img.width * s)), max(1, int(img.height * s))), Image.BOX)
ImageOps.autocontrast(img.convert("L")).save(png)
print("[MAP-SAVED] PNG:", png)
PYEOF

python3 - "$OUTPUT_DIR" "$MAP_NAME" "$BAG_PATH" "$DURATION" "$RATE" "$CONFIG_FILE" <<'PYEOF' || echo "[MAP-WARN] manifest 生成失败(不影响其他产物)"
import sys, os, re, json, yaml, datetime
out, name, bag, dur, rate, cfg = sys.argv[1:7]
xz = os.path.join(out, name + ".xzmap")
kf = pts = None
if os.path.exists(xz):
    with open(xz, "rb") as f:
        head = f.read(512).decode("utf-8", "ignore")
    kf = re.search(r'"key_frame_size":\s*(\d+)', head)
    pts = re.search(r'"points_size":\s*(\d+)', head)
res = org = None
yml = os.path.join(out, name + ".yaml")
if os.path.exists(yml):
    gy = yaml.safe_load(open(yml))
    res, org = gy.get("resolution"), gy.get("origin")
manifest = {
    "format": "rosiwit_offline_map", "version": 1,
    "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
    "source": {"bag": bag, "duration_s": float(dur), "rate": float(rate), "config": cfg},
    "map": {
        "name": name,
        "pcd": name + ".pcd",
        "scdb": name + ".pcd.scdb",
        "xzmap": {"file": name + ".xzmap",
                  "key_frame_size": int(kf.group(1)) if kf else None,
                  "points_size": int(pts.group(1)) if pts else None},
        "grid": {"pgm": name + ".pgm", "yaml": name + ".yaml",
                 "resolution": res, "origin": org},
        "png_preview": name + ".png" if os.path.exists(os.path.join(out, name + ".png")) else None,
    },
}
with open(os.path.join(out, "manifest.json"), "w") as f:
    json.dump(manifest, f, ensure_ascii=False, indent=2)
print("[MAP-SAVED] manifest.json")
PYEOF

cp /tmp/slam_node.log "$OUTPUT_DIR/slam_node.log"
echo "[MAP-FINISH] 建图完成, 产物在 $OUTPUT_DIR"
