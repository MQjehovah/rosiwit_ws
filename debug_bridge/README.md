# debug_bridge

给浏览器 [debug_hmi](../debug_hmi) 用的 **WebSocket + protobuf** 数据桥。

TF、定位状态、日志、控制服务仍然走 rosbridge；只有点云/栅格这些“原始体积太大”的话题走这里。
机器人侧先降采样/量化/压缩，再通过 protobuf 推给浏览器。

## 为什么

| 话题 | 原始 | 问题 |
|---|---|---|
| `/map` | 43 MB / 帧，1 Hz | rosbridge 用 JSON 要 >100MB 文本，实测 2 分钟收不完；CBOR 也要 1~2 分钟（Python 打包 4300 万格） |
| `/vanjee_points722` | 1 MB / 帧，10 Hz | 10 MB/s，远程隧道不现实 |
| `/scan_f_map` | 37 KB / 帧，10 Hz | 能接受，但 JSON 仍是 4 倍膨胀 |

桥接在本地做三件事：

1. **降采样**：点云按体素（默认 5~10cm）
2. **量化**：坐标转 int16 厘米（3 个点 = 6 字节）
3. **zlib**：再塞进 protobuf `bytes`

### 设备实测（aarch64 / Ubuntu 18.04 / Melodic）

| 数据 | 原始 | 传输 | 耗时 |
|---|---|---|---|
| `/map`（7453×5782 = 43,093,246 格） | 43 MB | **1.4 MB** | 机器人生成 6.4s；隧道约 15s |
| `/scan_f_map`（1002 点） | ~37 KB | **4.0 KB/帧** | 5 ms，4 Hz 推送 |
| `/vanjee_points722`（5cm 体素后 5242 点） | ~1 MB | **17 KB** | 0.3 s |
| `/global_map` | latched | 503（`debug_publish=false` 时无数据） | — |

## 协议

协议定义在 [`proto/debug_bridge.proto`](proto/debug_bridge.proto)，包名 `debug_bridge`。
浏览器端用 `protobufjs/light` + 由该 proto 生成的 `debug_hmi/src/ros/proto/debug_bridge.json`
（`node scripts/gen-proto.mjs`）。

```
服务器 -> 浏览器：Envelope { hello | grid | cloud | status | error }
浏览器 -> 服务器：ClientCommand { action, stream, rate_hz, voxel, max_points, refresh }
```

- `SUBSCRIBE`：持续推送。目前支持 `scan`、`points_2d`，`rate_hz` 由客户端指定（0.5~20）。
- `UNSUBSCRIBE`：停止推送。
- `REQUEST`：一次性抓取。`grid`（`/map`）、`global_map`、`vanjee_points722`，也支持 `scan`/`points_2d` 单帧。
- `CloudFrame.xyz_cm` 是 `int16 LE [count*3]`（厘米），`compression=ZLIB` 时先解压。
- `GridFrame.data` 是 `int8[width*height]`，`zlib` 后放在 `bytes` 里。

Python 2.7（Melodic）没有 `google.protobuf` runtime，所以服务端按 protobuf wire format
手工编码；字段编号与 `.proto` 一一对应，浏览器用生成好的 descriptor 解码。

## 部署

```bash
# 1) 放文件（设备能访问 GitHub 时最省事）
mkdir -p ~/debug_bridge
curl -fsSL https://raw.githubusercontent.com/MQjehovah/rosiwit_ws/master/debug_bridge/debug_bridge.py \
  -o ~/debug_bridge/debug_bridge.py

# 2) systemd
sudo cp debug-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now debug-bridge
systemctl status --no-pager debug-bridge

# 3) 看日志
journalctl -u debug-bridge -f
```

单元里的 `PartOf=xzrobot.service` 保证 xzrobot 重启时桥接跟着重启；`Restart=always` 兜底。
只依赖 `rospy` + `tornado`（rosbridge 已经带了），Python 2.7。

远程访问：隧道目标端口用 **9100**：

```
wss://<host>/remote-terminal/tunnel/<SN>/<TOKEN>/127.0.0.1/9100/
```

debug_hmi 会自动从 rosbridge 地址推导（`8089 -> 9100`），也可以在「数据源 → debug_bridge」里手填。

## 本地联调（不需要 ROS/设备）

```bash
python3 debug_bridge.py --demo --port 9100
```

会生成一块带房间的假地图和一个绕圈扫描，协议与真实模式完全一致。
`debug_hmi/scripts/test-bridge.sh` 就是用这个模式做端到端测试（WebSocket + protobufjs 解码）。

## 限制

- 订阅是推送（服务器按 `rate_hz` 发），不是轮询；慢客户端由 TCP 背压自然降频。
- 栅格是快照，不做 1 Hz 连续更新（连续 43MB 依然不划算）；更新就再 `REQUEST` 一次。
- 没有鉴权，默认绑定 `0.0.0.0:9100`。建议只走隧道/内网，必要时 `--host 127.0.0.1`。
- 服务端编码是手写 protobuf wire format；改 `.proto` 时记得同步
  `debug_bridge.py` 的字段编号，并重新生成 `debug_bridge.json`。
