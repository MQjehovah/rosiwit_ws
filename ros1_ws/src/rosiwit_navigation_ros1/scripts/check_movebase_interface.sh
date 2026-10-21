# ============================================================================
# robot_movebase 接口一致性校验
#
# 在节点运行时把 ROS 图与 scripts/movebase_interface.golden 逐项比对:
#   * robot_movebase 兼容话题的名字与类型
#   * /RobotNaviCmdAction 的 5 个 actionlib 端点
#   * 保留的原生接口 (/cmd_vel /global_path /local_path /nav_stop)
#
# 前置: roslaunch rosiwit_navigation_ros1 movebase_compat.launch (或机器人上已跑)
# 用法: check_movebase_interface.sh [golden 路径]
# 退出: 0 = 全部对上; 1 = 缺项/类型不符; 2 = 环境不可用
# ============================================================================
set -u

here="$(cd "$(dirname "$0")" && pwd)"
GOLDEN="${1:-$here/movebase_interface.golden}"

if [ ! -r "$GOLDEN" ]; then
    echo "找不到 golden 文件: $GOLDEN" >&2
    exit 2
fi

if ! rostopic list >/dev/null 2>&1; then
    echo "连不上 ROS master。请先 source setup.bash 并确认节点在跑" >&2
    exit 2
fi

norm() { case "$1" in /*) echo "$1" ;; *) echo "/$1" ;; esac; }

section=""
checked=0
bad=0

while read -r name want; do
    [ -z "${name:-}" ] && continue
    case "$name" in '#'*) continue ;; esac
    case "$name" in
        "[topics]")   section=topics;   continue ;;
        "[services]") section=services; continue ;;
    esac
    [ -z "${section:-}" ] && continue

    full="$(norm "$name")"
    checked=$((checked + 1))

    case "$section" in
        topics)
            have="$(rostopic type "$full" 2>/dev/null)"
            if [ "$have" != "$want" ]; then
                printf '  [BAD] topic   %-32s 期望 %s, 实际 %s\n' "$full" "$want" "${have:-<缺失>}"
                bad=$((bad + 1))
            fi
            ;;
        services)
            have="$(rosservice type "$full" 2>/dev/null)"
            if [ "$have" != "$want" ]; then
                printf '  [BAD] service %-32s 期望 %s, 实际 %s\n' "$full" "$want" "${have:-<缺失>}"
                bad=$((bad + 1))
            fi
            ;;
    esac
done < "$GOLDEN"

echo
echo "检查项 $checked 个, 不符 $bad 个"
if [ "$bad" -eq 0 ]; then
    echo "robot_movebase 接口一致性: OK"
    exit 0
fi
echo "robot_movebase 接口一致性: FAILED"
exit 1
