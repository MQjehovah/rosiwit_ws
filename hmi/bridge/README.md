# hmi_bridge

给浏览器 HMI 用的数据桥。**不是** rosbridge 的替代品：TF、状态、日志仍走 rosbridge；
只有点云/栅格这些“原始体积太大”的话题走这里。

## 为什么

| 话题 | 原始 | 问题 |
|---|---|---|
| `/map` | 43 MB / 帧，1 Hz | rosbridge 用 JSON 要 >100MB 文本，实测 2 分钟收不完；CBOR 也要 1~2 分钟（Python 打包 4300 万格） |
| `/vanjee_points722` | 1 MB / 帧，10 Hz | 10 MB/s，远程隧道不现实 |
| `/scan_f_map` | 37 KB / 帧，10 Hz | 可以接受，但 JSON 仍是 4 倍膨胀 |

桥接在**机器人本地**完成三件事，然后只发渲染需要的量：

1. **降采样**：点云按体素（默认 5~10cm），栅格可选。
2. **量化**：坐标转 int16 厘米（3 个点 = 6 字节）。
3. **压缩**：zlib。

结果：栅格通常几百 KB（占位数据高度重复），点云每帧几十 KB。浏览器用
`fetch` + `DecompressionStream('deflate')` 解码，不依赖任何 JS 库。

## 运行

只依赖 `rospy`，Python 2.7 / 3 都能跑（设备上的 ROS Melodic 是 Python 2.7）。

```bash
# 机器人上
source /opt/ros/melodic/setup.bash
source ~/rosiwit_build/ros1_ws/devel/setup.bash   # 按实际工作空间
python /path/to/hmi_bridge.py --port 9100
```

systemd 示例（`/etc/systemd/system/hmi-bridge.service`）：

```ini
[Unit]
Description=rosiwit HMI bridge
After=network.target

[Service]
User=xzrobot
Environment=PYTHONUNBUFFERED=1
ExecStart=/bin/bash -lc 'source /opt/ros/melodic/setup.bash && exec python /opt/xzrobot/hmi_bridge/hmi_bridge.py --port 9100'
Restart=always

[Install]
WantedBy=multi-user.target
```

远程访问：隧道目标端口改成 **9100**，例如
`https://<host>/remote-terminal/tunnel/<SN>/<TOKEN>/127.0.0.1/9100/`。
HMI 会自动从 rosbridge 地址推导（`8089 -> 9100`、`ws(s) -> http(s)`），也可以在
「数据源 → 二进制桥接」里手动填。

## 接口

```
GET /api/v1/health
GET /api/v1/grid[?refresh=1]
GET /api/v1/cloud?topic=scan_f_map&voxel=0.05&max=200000&wait=1
GET /api/v1/cloud?topic=global_map&voxel=0.2&max=500000
GET /api/v1/cloud?topic=vanjee_points722&voxel=0.05
```

- `scan_f_map` 常驻订阅（很小）。
- `/map`、`/global_map`、`/vanjee_points722` 都是**按需抓一帧就退订**，不会常驻 43MB。
- `wait` 是服务端等待新消息的秒数；轮询客户端传小一点，快照传大一点。

### 二进制帧

小端。`flags bit0 = zlib`。

```
grid : 'HMG1' | ver u8 | flags u8 | step u16 | res f32 | w u32 | h u32 |
       ox f64 | oy f64 | oth f32 | frame(u8 len+utf8) | stamp f64 |
       u32 len + zlib(int8[w*h])
cloud: 'HMC1' | ver u8 | flags u8 | count u32 | frame(u8 len+utf8) | stamp f64 |
       u32 len + zlib(int16[count*3])   # 单位 cm
```

## 本地联调（不需要 ROS/设备）

```bash
python3 hmi_bridge.py --demo --port 9100
```

会生成一块带房间的假地图和一个绕圈扫描。HMI 里把桥接地址填成
`http://127.0.0.1:9100` 即可看到效果（`scripts/test-bridge.sh` 也是用这个模式做端到端测试）。

## 限制

- 轮询而不是推送：HMI 端 3~4 Hz 拉取，网络慢时自然降频（请求串行 + 超时），不会积压。
- 栅格是快照，不做 1 Hz 连续更新（连续 43MB 依然不划算）；更新就点一次「抓取」。
- 没有鉴权。只给内网/隧道用，必要时 `--host 127.0.0.1` 让隧道自己转发。
