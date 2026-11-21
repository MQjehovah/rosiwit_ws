import fs from 'node:fs'
import zlib from 'node:zlib'
import protobuf from 'protobufjs'
import WebSocket from 'ws'

const url = process.env.BRIDGE_URL || 'ws://127.0.0.1:19100/'
const descriptor = JSON.parse(
  fs.readFileSync(new URL('../src/ros/proto/debug_bridge.json', import.meta.url), 'utf8'),
)
const root = protobuf.Root.fromJSON(descriptor)
const Envelope = root.lookupType('debug_bridge.Envelope')
const ClientCommand = root.lookupType('debug_bridge.ClientCommand')

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg)
    process.exitCode = 1
  } else {
    console.log('ok  :', msg)
  }
}

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

const opened = new Promise((resolve, reject) => {
  ws.once('open', resolve)
  ws.once('error', reject)
})

function waitFor(match, timeout = 8000) {
  return new Promise((resolve, reject) => {
    const w = { match, resolve }
    waiters.push(w)
    setTimeout(() => {
      const i = waiters.indexOf(w)
      if (i >= 0) waiters.splice(i, 1)
      reject(new Error('timeout waiting for frame'))
    }, timeout)
  })
}

function send(cmd) {
  ws.send(ClientCommand.encode(ClientCommand.create(cmd)).finish())
}

function subscribe(stream, rate, voxel = 0, max = 0) {
  send({ action: 0, stream, rate_hz: rate, voxel, max_points: max })
}

await opened
console.log('connected', url)

const hello = await waitFor((m) => m.hello)
assert(hello.hello && hello.hello.name === 'debug_bridge', 'hello name')
console.log('      streams:', hello.hello.streams.map((s) => s.name).join(','))

subscribe('tf', 10)
const tf = await waitFor((m) => m.tf)
assert(tf.tf && tf.tf.transforms.length > 0, 'tf transforms')
console.log('      tf', tf.tf.transforms.length, 'transforms, static=', tf.tf.is_static)

subscribe('status', 10)
const slam = await waitFor((m) => m.slam)
assert(slam.slam && slam.slam.map_name === 'demo', 'slam info')
console.log('      slam', slam.slam.map_name, slam.slam.matching_degree)

subscribe('diag', 1)
const diag = await waitFor((m) => m.diag)
assert(diag.diag && diag.diag.text.length > 0, 'diag text')

subscribe('logs', 5)
const log = await waitFor((m) => m.log, 15000)
assert(log.log && log.log.msg.length > 0, 'log entry')

subscribe('laser', 10)
const laser = await waitFor((m) => m.laser)
assert(laser.laser && laser.laser.ranges.byteLength > 0, 'laser ranges')
const laserBytes = Uint8Array.from(laser.laser.ranges)
const ranges = new Float32Array(laserBytes.buffer)
assert(ranges.length > 100, 'laser range count')

subscribe('paths', 2)
const path = await waitFor((m) => m.path)
assert(path.path && path.path.poses.length > 0, 'path poses')

subscribe('footprint', 1)
const poly = await waitFor((m) => m.polygon)
assert(poly.polygon && poly.polygon.xy.byteLength >= 24, 'footprint')

subscribe('scan', 4, 0.05, 200000)
const cloud = await waitFor((m) => m.cloud && m.cloud.stream === 'scan')
assert(cloud.cloud.count > 0, 'scan cloud count')
const raw = zlib.inflateSync(Buffer.from(cloud.cloud.xyz_cm))
assert(raw.length === cloud.cloud.count * 6, 'scan xyz_cm length')
console.log(`      scan count=${cloud.cloud.count} bytes=${cloud.cloud.xyz_cm.length}`)

const grid = (() => {
  send({ action: 2, stream: 'grid', refresh: true })
  return waitFor((m) => m.grid, 30000)
})()
const g = await grid
assert(g.grid.width > 0 && g.grid.height > 0, 'grid dims')
const cells = zlib.inflateSync(Buffer.from(g.grid.data))
assert(cells.length === g.grid.width * g.grid.height, 'grid cells')
console.log(`      grid ${g.grid.width}x${g.grid.height} res=${g.grid.resolution}`)

send({ action: 3, x: 1, y: 2, yaw: 0.5 })
await waitFor((m) => m.status && m.status.stream === 'initialpose')

send({ action: 5 })
const topics = await waitFor((m) => m.topics)
assert(topics.topics.topics.length > 0, 'topic list')
console.log('      topics', topics.topics.topics.length)

ws.close()
if (!process.exitCode) console.log('\nall bridge tests passed')
