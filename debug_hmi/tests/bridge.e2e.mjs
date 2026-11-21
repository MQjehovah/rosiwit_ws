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
  const buf = new Uint8Array(data)
  let msg
  try {
    msg = Envelope.decode(buf)
  } catch (e) {
    console.log('decode fail', e.message)
    return
  }
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
  const bytes = ClientCommand.encode(ClientCommand.create(cmd)).finish()
  ws.send(bytes)
}

await opened
console.log('connected', url)

const hello = await waitFor((m) => m.hello)
assert(hello.hello && hello.hello.name === 'debug_bridge', 'hello name')
console.log('      streams:', hello.hello.streams.map((s) => s.name).join(','))

send({ action: 0, stream: 'scan', rate_hz: 4, voxel: 0.05, max_points: 200000 })
const cloud = await waitFor((m) => m.cloud && m.cloud.stream === 'scan')
assert(cloud.cloud.count > 0, 'scan cloud count')
const raw = zlib.inflateSync(Buffer.from(cloud.cloud.xyz_cm))
assert(raw.length === cloud.cloud.count * 6, 'scan xyz_cm length')
assert(cloud.cloud.compression === 1, 'scan zlib compression')
console.log(`      scan count=${cloud.cloud.count} bytes=${cloud.cloud.xyz_cm.length}`)

send({ action: 2, stream: 'grid', refresh: true })
const grid = await waitFor((m) => m.grid, 30000)
assert(grid.grid.width === 800 && grid.grid.height === 600, 'grid dims')
assert(grid.grid.compression === 1, 'grid zlib compression')
const cells = zlib.inflateSync(Buffer.from(grid.grid.data))
assert(cells.length === grid.grid.width * grid.grid.height, 'grid cells')
console.log(`      grid ${grid.grid.width}x${grid.grid.height} res=${grid.grid.resolution}`)

send({ action: 2, stream: 'vanjee_points722', voxel: 0.05, max_points: 200000 })
const v = await waitFor((m) => m.cloud && m.cloud.stream === 'vanjee_points722')
assert(v.cloud.count > 0, 'vanjee cloud count')
console.log(`      vanjee count=${v.cloud.count} bytes=${v.cloud.xyz_cm.length}`)

ws.close()
if (!process.exitCode) console.log('\nall bridge tests passed')
