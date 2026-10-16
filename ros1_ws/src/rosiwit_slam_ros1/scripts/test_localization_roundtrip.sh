#!/bin/bash
# ============================================================================
# 定位回路自测: 建图 -> 存图 -> 加载这张图 -> 定位, 全程不需要真雷达/真地图
#
# 这是目前唯一能在本机把**定位链路**走完的办法 (用自己建出来的地图回喂定位)。
# 覆盖: core.saveMap 落盘 -> core.loadMap 成功路径 -> 重定位 -> 位姿输出
#       -> /slam/info(location_status=0) -> /curr_localization_pose(frame_id=map)
#
# 用法: test_localization_roundtrip.sh [地图目录]    (默认 /tmp/rosiwit_loc_test)
# 退出: 0 = 没有 FAIL; 1 = 有 FAIL
# ============================================================================
set -u

PKG="$(cd "$(dirname "$0")/.." && pwd)"
MAPDIR="${1:-/tmp/rosiwit_loc_test}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11399}"

PASS=0; FAIL=0
ok() { PASS=$((PASS+1)); echo "  [PASS] $*"; }
no() { FAIL=$((FAIL+1)); echo "  [FAIL] $*"; }

LP=""; SP=""
cleanup() {
    rosnode kill -a >/dev/null 2>&1
    for p in $LP $SP; do [ -n "$p" ] && kill $p >/dev/null 2>&1; done
    sleep 2
    pkill -f "rosmaster --core -p $(echo "$ROS_MASTER_URI" | sed 's#.*:##')" >/dev/null 2>&1
    for p in $LP $SP; do [ -n "$p" ] && kill -9 $p >/dev/null 2>&1; done
}
trap cleanup EXIT

rm -rf "$MAPDIR"; mkdir -p "$MAPDIR"

echo "=== 起节点 (input_profile=legacy) ==="
roslaunch rosiwit_slam_ros1 legacy_dev.launch > /tmp/lorun_node.log 2>&1 &
LP=$!
sleep 12
rosnode list 2>/dev/null | grep -q "/xz_3d_slam" && ok "节点起来了" || { no "节点没起来"; exit 1; }

echo
echo "=== 1) 建图 + 合成数据 (30s) ==="
timeout 10 rosservice call /set_slam_mode "mode: 'mapping'" >/dev/null 2>&1
rosrun rosiwit_slam_ros1 synthetic_publisher.py \
    _imu_topic:=/livox/imu _lidar_topic:=/livox/lidar > /tmp/lorun_syn1.log 2>&1 &
SP=$!
sleep 30
area=$(timeout 10 rostopic echo -n1 /slam/info 2>/dev/null | awk '/^map_area/{print $2}')
if [ -n "${area:-}" ] && awk "BEGIN{exit !($area > 0)}" 2>/dev/null; then
    ok "建图产出地图 map_area=$area"
else
    no "建图没有产出地图 (map_area=$area)"
fi

echo
echo "=== 2) 存图 (/stop_map_operation -> core.saveMap(\"$MAPDIR/map\")) ==="
timeout 30 rosservice call /stop_map_operation \
    "{map_name: 'synth', map_dir: '$MAPDIR', map_resolution: 0.05, include_unfinished_submaps: true}" \
    > /tmp/lorun_stop.log 2>&1
if grep -q "success: True" /tmp/lorun_stop.log; then ok "stop_map_operation 成功"; else no "stop_map_operation 失败"; fi
if [ -f "$MAPDIR/map.xzmap" ]; then ok "落盘 $MAPDIR/map.xzmap"; else no "缺 map.xzmap"; fi
echo "       落盘文件: $(ls "$MAPDIR" 2>/dev/null | tr '\n' ' ')"
if [ -f "$MAPDIR/map.pgm" ] && [ -f "$MAPDIR/map.yaml" ]; then
    ok "map.pgm / map.yaml 也在"
else
    no "**缺 map.pgm / map.yaml** —— 旧实现有且旧定位会去读它们 (见计划 §13.2)"
fi

echo
echo "=== 3) 切定位 + 加载这张图 (/switch_map) ==="
kill $SP >/dev/null 2>&1; SP=""
sleep 2
timeout 30 rosservice call /switch_map \
    "{map_name: 'synth', map_dir: '$MAPDIR', map_resolution: 0.05, map_origin: [0,0,0], map_negate: false, map_occupied_thresh: 0.65, map_free_thresh: 0.196, initial_pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}, vir_obs: ''}" \
    > /tmp/lorun_switch.log 2>&1
if grep -q "loaded map:" /tmp/lorun_node.log; then ok "core.loadMap 成功 (日志有 'loaded map:')"; else no "core.loadMap 没成功"; fi
echo "       (/switch_map 的 success 取决于 /set_detect_mode —— 该服务不在本仓库, 失败是预期的)"

echo
echo "=== 4) 再喂同样的数据, 看定位能不能定上 ==="
rosrun rosiwit_slam_ros1 synthetic_publisher.py \
    _imu_topic:=/livox/imu _lidar_topic:=/livox/lidar > /tmp/lorun_syn2.log 2>&1 &
SP=$!
# 定位收敛要几十秒 (BBS 重定位 + 监测窗口), 所以轮询而不是单次 echo ——
# 单次 echo 很容易在"还没定上"的空窗里超时, 造成假 FAIL。
echo "  (轮询 /slam/info, 最多约 2 分钟等定位收敛)"
info=""; ls_=""; ss=""
for i in $(seq 1 8); do
    sleep 2
    # 先落到文件再解析: 直接把 rostopic echo 的输出塞进 $(...) 在本机偶发拿到不完整的
    # 文本(slam_state 有、location_status 没有) —— 落文件读取更稳。
    timeout 15 rostopic echo -n1 /slam/info > /tmp/lorun_info.txt 2>&1
    info="$(cat /tmp/lorun_info.txt)"
    ss=$(awk '/^slam_state/{print $2}' /tmp/lorun_info.txt)
    ls_=$(awk '/^location_status/{print $2}' /tmp/lorun_info.txt)
    if [ "${ls:-}" = "0" ]; then break; fi
    echo "    t+$((i*2))s: slam_state=${ss:-<无>} location_status=${ls:-<无>}"
done
if [ "${ls:-}" = "0" ] && [ "${ss:-}" = "1" ]; then
    ok "/slam/info 出现: slam_state=1, location_status=0 (定位成功)"
else
    no "/slam/info: slam_state=${ss:-?} location_status=${ls:-?} (期望 1/0)"
    echo "      原始输出(便于诊断):"
    sed 's/^/        | /' /tmp/lorun_info.txt 2>/dev/null | head -12
fi
pose="$(timeout 12 rostopic echo -n1 /curr_localization_pose 2>/dev/null)"
if echo "$pose" | grep -q 'frame_id: "map"'; then
    ok "/curr_localization_pose frame_id=map"
else
    no "/curr_localization_pose 没出来或 frame_id 不对"
fi

echo
echo "============================================"
echo "PASS=$PASS  FAIL=$FAIL"
echo "============================================"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
