#!/bin/bash
# ============================================================================
# robot_movebase 兼容层隔离冒烟 (不会碰实车)
#
# 起一个私有 roscore (127.0.0.1:$PORT) 和 /navtest 命名空间下的本节点,
# 由 movebase_smoke_driver.py 造 /map + /odom + TF, 验证:
#   action 可用 / 不支持命令失败码 / 旋转 / 倒车 / 两点规划 / 路径跟踪 /
#   cancel -> PREEMPTED / 取消后不自行恢复
#
# 前置: source /opt/ros/melodic/setup.bash + 本工作空间 devel/setup.bash
# 用法: smoke_movebase.sh [port]
# 退出: 0 = 全 PASS; 1 = 有 FAIL; 2 = 环境不可用
# ============================================================================
set -u

here="$(cd "$(dirname "$0")" && pwd)"
port="${1:-${PORT:-11420}}"

if ! command -v rosparam >/dev/null 2>&1; then
    echo "找不到 ROS 命令, 请先 source /opt/ros/melodic/setup.bash" >&2
    exit 2
fi
if ! rospack find rosiwit_navigation_ros1 >/dev/null 2>&1; then
    echo "找不到 rosiwit_navigation_ros1, 请先 source 工作空间 devel/setup.bash" >&2
    exit 2
fi

export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_IP=127.0.0.1
export ROS_HOME="${ROS_HOME:-/tmp/rosiwit_movebase_smoke}"

rm -rf "$ROS_HOME"
mkdir -p "$ROS_HOME"

cleanup() {
    [ -n "${NODE_PID:-}" ] && kill "$NODE_PID" 2>/dev/null
    [ -n "${CORE_PID:-}" ] && kill "$CORE_PID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT

roscore -p "$port" >/tmp/movebase_smoke_core.log 2>&1 &
CORE_PID=$!
for _ in $(seq 1 40); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.5
done

# 用真实部署配置 (movebase_compat.yaml) 跑, 只把两个输出话题改到 /navtest,
# 避免碰到实车 (私有 master 本身也不会影响实车).
CONFIG="$(rospack find rosiwit_navigation_ros1)/config/movebase_compat.yaml"
rosparam load "$CONFIG" /navtest/navtest
rosparam set /navtest/navtest/cmd_vel_topic /navtest/cmd_vel
rosparam set /navtest/navtest/movebase_cmd_vel_topic /navtest/movebase_cmd_vel

rosrun rosiwit_navigation_ros1 rosiwit_navigation_ros1_node \
    __name:=navtest __ns:=/navtest \
    >/tmp/movebase_smoke_node.log 2>&1 &
NODE_PID=$!

for _ in $(seq 1 120); do
    rostopic list 2>/dev/null | grep -q '^/navtest/RobotNaviCmdAction/goal$' && break
    sleep 0.5
done

python "$here/movebase_smoke_driver.py"
rc=$?
echo "节点日志: /tmp/movebase_smoke_node.log"
exit $rc
