#!/bin/bash
# SLAM 部署: ros1_ws/devel 里的构建产物 + 配置 -> /opt/ros/melodic, 然后 (默认) 重启**一次** xzrobot 服务。
#
#   tools/deploy_slam.sh                校验 -> 备份 -> 替换 4 个产物 + default.yaml -> systemctl restart xzrobot
#   tools/deploy_slam.sh --no-restart   只替换文件 (运行中的节点继续用旧文件, 下次重启服务才生效)
#   tools/deploy_slam.sh --check        只做校验和显示差异, 不动任何文件
#
# 三条硬约束:
#  1. 不原地覆盖正在被进程映射/执行的文件: 截断映射中的 .so 会让运行中的节点立刻 SIGBUS。
#     做法: 先拷到暂存目录, 再 rename 原子替换 —— 运行中的进程继续用旧 inode, 直到服务重启。
#  2. 只用一次 `systemctl restart xzrobot`, 绝不 stop + start: debug-bridge.service 是
#     PartOf=xzrobot.service, systemd 只传播 restart 不传播 start, 分开的 stop/start 会把 debug-bridge 停掉不再拉起。
#  3. 替换前把当前已部署版本拷到 /home/xzrobot/slam_deploy/backup_<时间戳>/。回滚: 把备份里的文件按同样的
#     rename 方式放回去, 再 restart。
#
# 前置校验 (任何失败都发生在动文件之前): 产物都在, 且比它依赖的源码新 (否则是没有重新构建)。
# /opt/ros/melodic/lib 是 root 属主目录 (不可写): 原子替换 (rename) 也要 sudo -n,
# 与重启服务同一权限。文件本身是 xzrobot 属主, 备份不需要 sudo。
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
WS=$ROOT/ros1_ws
DST=/opt/ros/melodic
RESTART=1
CHECK=0
for a in "$@"; do
    case "$a" in
        --no-restart) RESTART=0 ;;
        --check) CHECK=1 ;;
        *) echo "未知参数: $a" >&2; exit 2 ;;
    esac
done

# 相对 $DST 的产物: 核心库 / ROS 层库 / 节点 / fast_gicp (核心库依赖它的符号, 漏部署 = 加载旧实现或缺符号)
ART=(lib/librosiwit_slam.so lib/librosiwit_slam_ros1_lib.so lib/rosiwit_slam_ros1/rosiwit_slam_ros1_node lib/libfast_gicp.so)
YAML_SRC=$WS/src/rosiwit_slam_ros1/config/default.yaml
YAML_DST=$DST/share/rosiwit_slam_ros1/config/default.yaml

# ============ 0. 前置校验 ============
for d in "$ROOT/src/rosiwit_slam" "$WS/src/rosiwit_slam_ros1/src" "$WS/src/fast_gicp/src" "$WS/src/fast_gicp/include"; do
    [ -d "$d" ] || { echo "源码目录不存在: $d" >&2; exit 1; }
done
for f in "${ART[@]}"; do
    [ -f "$WS/devel/$f" ] || { echo "缺少构建产物: devel/$f" >&2; exit 1; }
done
newest() {   # 这些目录里最新的一个源文件
    find "$@" \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \) -not -path '*/test/*' -printf '%T@ %p\n' 2>/dev/null \
        | sort -rn | sed -n '1p' | cut -d' ' -f2-
}
NEWEST_CORE=$(newest "$ROOT/src/rosiwit_slam" "$WS/src/rosiwit_slam_ros1/src" "$WS/src/fast_gicp/src" "$WS/src/fast_gicp/include")
NEWEST_FG=$(newest "$WS/src/fast_gicp/src" "$WS/src/fast_gicp/include")
[ -n "$NEWEST_CORE" ] && [ -n "$NEWEST_FG" ] || { echo "找不到源码, 无法校验产物新旧" >&2; exit 1; }
echo "最新源码: $NEWEST_CORE ($(date -r "$NEWEST_CORE" '+%m-%d %H:%M:%S'))"
fresh() {   # $1 = 产物  $2 = 它依赖的源码范围里最新的一个
    if [ ! "$WS/devel/$1" -nt "$2" ]; then
        echo "产物比源码旧, 没有重新构建: devel/$1 ($(date -r "$WS/devel/$1" '+%m-%d %H:%M:%S'))" >&2
        exit 1
    fi
}
fresh lib/librosiwit_slam.so "$NEWEST_CORE"
fresh lib/librosiwit_slam_ros1_lib.so "$NEWEST_CORE"
fresh lib/rosiwit_slam_ros1/rosiwit_slam_ros1_node "$NEWEST_CORE"
fresh lib/libfast_gicp.so "$NEWEST_FG"
echo "前置校验通过"
for f in "${ART[@]}"; do
    if cmp -s "$WS/devel/$f" "$DST/$f"; then s="相同"; else s="**不同**"; fi
    echo "  $f: 已部署 $(date -r "$DST/$f" '+%m-%d %H:%M:%S') / 新构建 $(date -r "$WS/devel/$f" '+%m-%d %H:%M:%S') -> $s"
done
if cmp -s "$YAML_SRC" "$YAML_DST"; then echo "  default.yaml: 相同"; else echo "  default.yaml: **不同**"; diff "$YAML_DST" "$YAML_SRC" | sed -n '1,40p' | cut -c1-150 || true; fi   # diff 有差异返回 1, 不能触发 set -e
if [ "$CHECK" = "1" ]; then
    echo "(--check: 不改动任何文件)"
    exit 0
fi

# ============ 1. 备份当前已部署版本 ============
STAMP=$(date +%m%d-%H%M%S)
BK=/home/xzrobot/slam_deploy/backup_$STAMP
STG=/home/xzrobot/slam_deploy/staging_$STAMP
mkdir -p "$BK/lib/rosiwit_slam_ros1" "$STG/lib/rosiwit_slam_ros1"
for f in "${ART[@]}"; do cp -p "$DST/$f" "$BK/$f"; done
cp -p "$YAML_DST" "$BK/default.yaml"
echo "备份: $BK"

# ============ 2. 暂存 + rename 替换 ============
for f in "${ART[@]}"; do cp -p "$WS/devel/$f" "$STG/$f"; done
cp -p "$YAML_SRC" "$STG/default.yaml"
# /opt/ros/melodic/lib 属主是 root (目录不可写): rename 必须 sudo。文件本身虽是
# xzrobot, 但 rename 需要目标目录的写权限。sudo -n 与下面的 restart 同一权限。
for f in "${ART[@]}"; do sudo -n mv -f "$STG/$f" "$DST/$f"; echo "已部署: $(date -r "$DST/$f" '+%m-%d %H:%M:%S') $DST/$f"; done
sudo -n mv -f "$STG/default.yaml" "$YAML_DST"
echo "已部署: default.yaml"
rmdir "$STG/lib/rosiwit_slam_ros1" "$STG/lib" "$STG" 2>/dev/null || true

# ============ 3. 重启服务 (只一次) ============
if [ "$RESTART" = "1" ]; then
    sudo -n systemctl restart xzrobot
    sleep 3
    echo "xzrobot 服务已 restart: $(systemctl is-active xzrobot)  debug-bridge: $(systemctl is-active debug-bridge)"
else
    echo "--no-restart: 文件已替换, 下次重启服务后生效"
fi
echo "回滚: 把 $BK 里的文件 mv 回 $DST 对应位置 (和 $YAML_DST), 再 sudo systemctl restart xzrobot"
