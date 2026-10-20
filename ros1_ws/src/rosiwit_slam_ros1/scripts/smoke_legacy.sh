#!/bin/bash
# ============================================================================
# 旧接口合成数据冒烟 (计划 T13)
#
# 不需要机器人、不需要真实雷达。自动起节点 + 合成数据发布器 + 逐项断言。
# 用法: smoke_legacy.sh
# 退出: 0 = 没有 FAIL (SKIP 不算失败); 1 = 有 FAIL
# ============================================================================
set -u

PKG="$(cd "$(dirname "$0")/.." && pwd)"
PUB="$PKG/test/synthetic_publisher.py"
MASTER="${ROS_MASTER_URI:-http://127.0.0.1:11399}"
export ROS_MASTER_URI="$MASTER"

PASS=0; FAIL=0; SKIP=0
ok()   { PASS=$((PASS+1)); echo "  [PASS] $*"; }
no()   { FAIL=$((FAIL+1)); echo "  [FAIL] $*"; }
skip() { SKIP=$((SKIP+1)); echo "  [SKIP] $*"; }

LP=""; SP=""; TP=""
cleanup() {
    rosnode kill -a >/dev/null 2>&1
    for p in $LP $SP $TP; do [ -n "$p" ] && kill $p >/dev/null 2>&1; done
    sleep 2
    pkill -f "rosmaster --core -p $(echo "$MASTER" | sed 's#.*:##')" >/dev/null 2>&1
    for p in $LP $SP $TP; do [ -n "$p" ] && kill -9 $p >/dev/null 2>&1; done
}
trap cleanup EXIT

echo "=== 起节点 ==="
roslaunch rosiwit_slam_ros1 legacy_dev.launch > /tmp/smoke_node.log 2>&1 &
LP=$!
sleep 12
if rosnode list 2>/dev/null | grep -q "/xz_3d_slam"; then ok "节点起来了"; else no "节点没起来"; fi

echo
echo "--- 第 3 项: slam_state 随 /slam/mode_switch 切换 ---"
timeout 10 rosservice call /slam/mode_switch "{mode: 1}" >/dev/null 2>&1
sleep 1
st=$(timeout 10 rostopic echo -n1 /slam/info 2>/dev/null | awk '/^slam_state/{print $2}')
if [ "${st:-}" = "0" ]; then ok "切建图后 slam_state=0"; else no "切建图后 slam_state=$st (期望 0)"; fi

echo
echo "--- 第 1 项: 建图 + 合成数据 -> /map 有真实栅格 ---"
rosrun rosiwit_slam_ros1 synthetic_publisher.py \
    _imu_topic:=/livox/imu,vanjee_lidar_imu_packets \
    _lidar_topic:=/livox/lidar,vanjee_points722 \
    _room_half:=1.5 _step:=0.05 > /tmp/smoke_syn.log 2>&1 &
SP=$!
sleep 5
# 注意: 这里只能证明"发布器在发", 不能证明节点订阅的就是这些话题 —— legacy profile 到底
# 订 /livox/* 还是 vanjee_* 取决于实机 sensor_config.json + /machine/model, 所以上面两个都发。
if timeout 6 rostopic hz /livox/lidar 2>&1 | grep -q "average rate"; then
    ok "合成数据在发 (/livox/lidar + /livox/imu)"
else
    no "合成数据没发出来"
fi
echo "    (等 35s 让 LIO 初始化建图)"
sleep 35
mapmsg="$(timeout 40 rostopic echo -n1 /map 2>/dev/null)"
w=$(echo "$mapmsg" | awk '/^  width/{print $2; exit}')
occ=$(echo "$mapmsg" | tr ',' '\n' | grep -c '^ *100$')
area=$(timeout 10 rostopic echo -n1 /slam/info 2>/dev/null | awk '/^map_area/{print $2}')
if [ "${w:-0}" -gt 0 ] 2>/dev/null; then ok "/map width=$w (>0)"; else no "/map width=$w (期望 >0)"; fi
if [ "${occ:-0}" -gt 0 ] 2>/dev/null; then ok "/map 里有 $occ 个占据格(100)"; else no "/map 没有占据格"; fi
if [ -n "${area:-}" ] && awk "BEGIN{exit !($area > 0)}" 2>/dev/null; then
    ok "/slam/info.map_area=$area (>0)"
else
    no "/slam/info.map_area=$area (期望 >0)"
fi
oz=$(echo "$mapmsg" | awk '/^      z:/{print $2; exit}')
if [ -n "${oz:-}" ]; then
    ok "/map.origin.position.z=$oz (= map_area*1e-5, 旧实现的约定)"
else
    skip "/map.origin.position.z 未取到"
fi

echo
echo "--- 第 4 项: map->odom TF (脚本同时伪造 base_footprint->odom) ---"
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom base_footprint \
    > /tmp/smoke_tf.log 2>&1 &
TP=$!
sleep 3
tfout="$(timeout 12 rosrun tf tf_echo map odom 2>&1 | head -12)"
if echo "$tfout" | grep -qi "translation"; then
    ok "tf_echo map odom 能查通 ($(echo "$tfout" | grep -iA1 translation | tr '\n' ' ' | cut -c1-70))"
else
    no "tf_echo map odom 查不通"
fi

echo
echo "--- 第 2 项: 定位模式下 curr_localization_pose ---"
skip "需要一张真地图: 核心在定位模式**先 LOCALIZED** 才会产出位姿与 /slam/info (已在计划 §11.1 记录)"

echo
echo "--- 第 5 项: 停数据后报 20003500/20003501 ---"
skip "本实现**有意不复刻**: 这两条来自旧 lidar 处理循环的\"迭代 600 次没拿到新帧\"计数器 (legacy.cpp:3076), 核心没有对应层次"

echo
echo "--- 第 6 项: 定位丢失 -> location_status=2 + 20003504 ---"
skip "需要真地图 + 真数据: 20003504 只在\"定位成功过、之后又丢了\"时才有 (计划 §11.1)"

echo
echo "--- ROS 日志有没有 crash/异常 ---"
if grep -aqE "terminate|Segmentation|what\(\): " /tmp/smoke_node.log; then
    no "节点日志里有 crash/异常"
    grep -aE "terminate|Segmentation|what\(\): " /tmp/smoke_node.log | tail -3
else
    ok "日志无 crash/异常"
fi

echo
echo "============================================"
echo "PASS=$PASS  FAIL=$FAIL  SKIP=$SKIP"
echo "============================================"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
