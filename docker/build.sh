#!/usr/bin/env bash
# 构建 docker/ 下的某个镜像。构建上下文固定为仓库根目录(见根 .dockerignore)。
#
# 用法:
#   ./docker/build.sh <镜像目录名> [tag]
# 示例:
#   ./docker/build.sh slam-mapping           # -> rosiwit/slam-mapping:humble
#   ./docker/build.sh slam-mapping latest    # -> rosiwit/slam-mapping:latest
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"

if [ $# -lt 1 ]; then
    echo "用法: $0 <镜像目录名> [tag]"
    echo "可用镜像:"
    for d in "$here"/*/; do
        [ -f "${d}Dockerfile" ] && echo "  - $(basename "$d")"
    done
    exit 1
fi

name="$1"
tag="${2:-humble}"
dockerfile="docker/$name/Dockerfile"
image="rosiwit/$name:$tag"

if [ ! -f "$root/$dockerfile" ]; then
    echo "找不到 $dockerfile"
    exit 1
fi

echo ">>> docker build -f $dockerfile -t $image .   (context: $root)"
cd "$root"
exec docker build -f "$dockerfile" -t "$image" .
