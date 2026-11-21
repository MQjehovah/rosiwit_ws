#!/usr/bin/env bash
# 桥接端到端测试：启动 demo bridge（假地图 + 扫描），用浏览器同款客户端解码。
# 不依赖 ROS / 设备 / 浏览器。
set -euo pipefail

cd "$(dirname "$0")/.."
export PATH="$HOME/.local/node/bin:$PATH"

if [ ! -x ./node_modules/.bin/esbuild ]; then
  echo "先执行 ./dev.sh install" >&2
  exit 1
fi

port="${BRIDGE_PORT:-19100}"
base="http://127.0.0.1:${port}"

python3 bridge/hmi_bridge.py --demo --port "$port" --quiet &
pid=$!
trap 'kill "$pid" 2>/dev/null || true' EXIT

for _ in $(seq 1 40); do
  if curl -sf --max-time 1 "$base/api/v1/health" >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

if ! curl -sf --max-time 2 "$base/api/v1/health" >/dev/null 2>&1; then
  echo "demo bridge 没起来（端口 $port）" >&2
  exit 1
fi

out="$(mktemp -d)"
trap 'kill "$pid" 2>/dev/null || true; rm -rf "$out"' EXIT

./node_modules/.bin/esbuild src/ros/bridge.ts --bundle --format=esm --outfile="$out/bridge.mjs" >/dev/null
cp tests/bridge.test.mjs "$out/"
BRIDGE_URL="$base" node "$out/bridge.test.mjs"
