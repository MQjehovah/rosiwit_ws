#!/usr/bin/env bash
# 桥接端到端测试：启动 demo 模式（假地图 + 扫描），用 WebSocket + protobuf 解码。
# 不依赖 ROS / 设备 / 浏览器。
set -euo pipefail

cd "$(dirname "$0")/.."
export PATH="$HOME/.local/node/bin:$PATH"

if [ ! -x ./node_modules/.bin/esbuild ]; then
  echo "先执行 ./dev.sh install" >&2
  exit 1
fi

if ! python3 -c 'import tornado' >/dev/null 2>&1; then
  echo "python3 缺少 tornado，先执行: python3 -m pip install --user tornado" >&2
  exit 1
fi

port="${BRIDGE_PORT:-19100}"
base="ws://127.0.0.1:${port}/"

python3 ../debug_bridge/debug_bridge.py --demo --port "$port" --quiet \
  >/tmp/debug_bridge_demo.log 2>&1 &
pid=$!
trap 'kill "$pid" 2>/dev/null || true' EXIT

for _ in $(seq 1 40); do
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "demo bridge 进程已退出" >&2
    exit 1
  fi
  if python3 - "$port" <<'PY' >/dev/null 2>&1
import socket, sys
s = socket.create_connection(("127.0.0.1", int(sys.argv[1])), timeout=0.5)
s.close()
PY
  then
    break
  fi
  sleep 0.25
done

BRIDGE_URL="$base" node tests/bridge.e2e.mjs
