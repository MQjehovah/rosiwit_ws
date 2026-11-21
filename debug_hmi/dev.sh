#!/usr/bin/env bash
# 在 WSL 里运行 HMI 的辅助脚本（Windows 侧通过 UNC 路径跑 npm 会踩 9P/esbuild 的坑）。
#
#   ./dev.sh install   安装依赖
#   ./dev.sh dev       vite 开发服务器（0.0.0.0:5173）
#   ./dev.sh build     类型检查 + 生产构建
#   ./dev.sh typecheck 只做类型检查
#   ./dev.sh test      纯数学/解码层单测
#   ./dev.sh bridge-test 桥接端到端测试（自动起 demo bridge）
#
# 如果没有把 node 装进 WSL：
#   curl -fsSL https://npmmirror.com/mirrors/node/v18.20.4/node-v18.20.4-linux-x64.tar.xz -o /tmp/node.tar.xz
#   mkdir -p ~/.local && tar -xJf /tmp/node.tar.xz -C ~/.local
#   mv ~/.local/node-v18.20.4-linux-x64 ~/.local/node
set -euo pipefail

export PATH="$HOME/.local/node/bin:$PATH"
cd "$(dirname "$0")"

if ! command -v node >/dev/null 2>&1; then
  echo "找不到 node。请把 Node 解压到 ~/.local/node（见脚本头部注释），或自行安装 Node >= 18。" >&2
  exit 1
fi

mode="${1:-dev}"
case "$mode" in
  install)   npm install --no-audit --no-fund ;;
  dev)       npm run dev ;;
  build)     npm run build ;;
  typecheck) npm run typecheck ;;
  preview)   npm run preview ;;
  test)      ./scripts/test-ros.sh ;;
  bridge-test) ./scripts/test-bridge.sh ;;
  *)
    echo "用法: $0 [install|dev|build|typecheck|preview|test|bridge-test]" >&2
    exit 2
    ;;
esac
