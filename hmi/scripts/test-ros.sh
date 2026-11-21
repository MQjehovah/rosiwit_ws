#!/usr/bin/env bash
# 纯数学/解码层单测（不依赖浏览器、不连设备）：
#   tf 变换、PointCloud2 / OccupancyGrid / LaserScan 解码
set -euo pipefail

cd "$(dirname "$0")/.."
export PATH="$HOME/.local/node/bin:$PATH"

if [ ! -x ./node_modules/.bin/esbuild ]; then
  echo "先执行 ./dev.sh install" >&2
  exit 1
fi

out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

./node_modules/.bin/esbuild src/ros/tf.ts --bundle --format=esm --outfile="$out/tf.mjs" >/dev/null
./node_modules/.bin/esbuild src/ros/decode.ts --bundle --format=esm --outfile="$out/decode.mjs" >/dev/null
./node_modules/.bin/esbuild src/ros/cbor.ts --bundle --format=esm --outfile="$out/cbor.mjs" >/dev/null
./node_modules/.bin/esbuild src/three/gridImage.ts --bundle --format=esm --outfile="$out/gridImage.mjs" >/dev/null
cp tests/ros.test.mjs "$out/"
cp -r tests/fixtures "$out/" 2>/dev/null || true
node "$out/ros.test.mjs"
