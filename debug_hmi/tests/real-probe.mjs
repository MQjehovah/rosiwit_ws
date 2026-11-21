import fs from 'node:fs'
import protobuf from 'protobufjs'
import WebSocket from 'ws'

/** 真机探针：只读，不调用任何服务。
 *   BRIDGE_URL='wss://.../9100/' node tests/real-probe.mjs
 */
const url = process.env.BRIDGE_URL || 'ws://127.0.0.1:9100/'
const descriptor = JSON.parse(
  fs.readFileSync(new URL('../src/ros/proto/debug_bridge.json', import.meta.url), 'utf8'),
)
const root = protobuf.Root.fromJSON(descriptor)
const Envelope = root.lookupType('debug_bridge.Envelope')
const ClientCommand = root.lookupType('debug_bridge.ClientCommand')

const ws = new WebSocket(url, { perMessageDeflate: false })
ws.binaryType = 'arraybuffer'
const waiters = []
ws.on('message', (data) => {
  const msg = Envelope.decode(new Uint8Array(data))
  for (const w of [...waiters]) {
    if (w.match(msg)) {
      waiters.splice(waiters.indexOf(w), 1)
      w.resolve(msg)
    }
  }
})

function waitFor(match, timeout = 10000) {
  return new Promise((resolve, reject) => {
    const w = { match, resolve }
    waiters.push(w)
    setTimeout(() => {
      const i = waiters.indexOf(w)
      if (i >= 0) waiters.splice(i, 1)
      reject(new Error('timeout'))
    }, timeout)
  })
}
const send = (cmd) => ws.send(ClientCommand.encode(ClientCommand.create(cmd)).finish())

ws.on('open', async () => {
  console.log('connected', url)
  const hello = await waitFor((m) => m.hello)
  console.log('hello', hello.hello.version, hello.hello.streams.map((s) => s.name).join(','))

  if (process.env.ONLY_GLOBAL) {
    send({ action: 2, stream: '/global_map', voxel: 0.1, max_points: 0 })
    try {
      const cloud = await waitFor((m) => m.cloud && m.cloud.stream === '/global_map', 300000)
      console.log('global_map:', cloud.cloud.count, 'points,', cloud.cloud.xyz_cm.byteLength, 'bytes zlib')
    } catch {
      console.log('global_map: no frame')
    }
    ws.close()
    process.exit(0)
  }

  send({ action: 0, stream: 'status', rate_hz: 10 })
  const slam = await waitFor((m) => m.slam)
  console.log('slam:', JSON.stringify({
    map_name: slam.slam.map_name,
    location_status: slam.slam.location_status,
    slam_state: slam.slam.slam_state,
    matching: slam.slam.matching_degree,
    pose: slam.slam.robot_pose,
  }))

  send({ action: 0, stream: '/curr_scan_point', rate_hz: 4, voxel: 0.05 })
  const scan = await waitFor((m) => m.cloud && m.cloud.stream === '/curr_scan_point')
  console.log('scan:', scan.cloud.count, 'points,', scan.cloud.xyz_cm.byteLength, 'bytes zlib')

  send({ action: 0, stream: '/scan', rate_hz: 5 })
  const laser = await waitFor((m) => m.laser)
  console.log('laser:', laser.laser.stream, laser.laser.ranges.byteLength, 'bytes ranges')

  if (!process.env.SKIP_MAP) {
    send({ action: 2, stream: '/map', refresh: false })
    const grid = await waitFor((m) => m.grid, 90000)
    console.log('grid:', grid.grid.stream, grid.grid.width, 'x', grid.grid.height, grid.grid.data.byteLength, 'bytes zlib')
  }

  send({ action: 2, stream: '/global_map', voxel: 0.05, max_points: 0 })
  try {
    const cloud = await waitFor((m) => m.cloud && m.cloud.stream === '/global_map', 300000)
    console.log('global_map:', cloud.cloud.count, 'points,', cloud.cloud.xyz_cm.byteLength, 'bytes zlib')
  } catch {
    console.log('global_map: no frame (topic never published / not latched)')
  }

  ws.close()
  process.exit(0)
})

ws.on('error', (e) => {
  console.log('WS error', e.message)
  process.exit(1)
})
setTimeout(() => {
  console.log('overall timeout')
  process.exit(1)
}, 360000)
