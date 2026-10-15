#!/bin/bash
# ============================================================================
# 旧 xz_3d_slam 接口一致性校验 (计划 T12)
#
# 把"对上"变成机器可判的检查: 拿运行中的 ROS 图与 scripts/legacy_interface.golden
# 逐项比对 (14 个话题 / 8 个服务 / 1 个参数的名字与类型)。
#
# 前置: 已经有节点在跑 —— roslaunch rosiwit_slam_ros1 legacy_dev.launch,
#       或机器人上的 xz_3d_slam。本脚本**只读** ROS 图, 不启停任何东西。
# 用法: check_legacy_interface.sh [golden 路径]
# 退出: 0 = 全部对上; 1 = 有缺项或类型不符; 2 = 环境不可用
# ============================================================================
set -u

here="$(cd "$(dirname "$0")" && pwd)"
GOLDEN="${1:-$here/legacy_interface.golden}"

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
        "[params]")   section=params;   continue ;;
    esac
    [ -z "${section:-}" ] && continue

    full="$(norm "$name")"
    checked=$((checked + 1))

    case "$section" in
        topics)
            have="$(rostopic type "$full" 2>/dev/null)"
            if [ "$have" != "$want" ]; then
                printf '  [BAD] topic   %-26s 期望 %s, 实际 %s\n' "$full" "$want" "${have:-<缺失>}"
                bad=$((bad + 1))
            fi
            ;;
        services)
            have="$(rosservice type "$full" 2>/dev/null)"
            if [ "$have" != "$want" ]; then
                printf '  [BAD] service %-26s 期望 %s, 实际 %s\n' "$full" "$want" "${have:-<缺失>}"
                bad=$((bad + 1))
            fi
            ;;
        params)
            value="$(rosparam get "$full" 2>/dev/null | head -1)"
            if [ -z "${value:-}" ]; then
                printf '  [BAD] param   %-26s 缺失\n' "$full"
                bad=$((bad + 1))
            fi
            ;;
    esac
done < "$GOLDEN"

echo
echo "检查项 $checked 个, 不符 $bad 个"
if [ "$bad" -eq 0 ]; then
    echo "旧接口一致性: OK"
    exit 0
fi
echo "旧接口一致性: FAILED"
exit 1
