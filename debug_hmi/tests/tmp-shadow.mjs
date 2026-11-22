import fs from 'node:fs'
import protobuf from 'protobufjs'
import WebSocket from 'ws'

const descriptor = JSON.parse(fs.readFileSync(new URL('../src/ros/proto/debug_bridge.json', import.meta.url), 'utf8'))
const root = protobuf.Root.fromJSON(descriptor)
const Envelope = root.lookupType('debug_bridge.Envelope')
const ClientCommand = root.lookupType('debug_bridge.ClientCommand')
const ws = new WebSocket(process.env.BRIDGE_URL)
await new Promise((res, rej) => { ws.once('open', res); ws.once('error', rej) })
ws.binaryType = 'arraybuffer'
const frames = []
ws.on('message', (d) => {
  const m = Envelope.decode(new Uint8Array(d))
  frames.push(m)
})
const send = (o) => ws.send(ClientCommand.encode(ClientCommand.create(o)).finish())
send({ action: 0, stream: '/device/robot_shadow', rate_hz: 1 })
const deadline = Date.now() + 15000
while (Date.now() < deadline) {
  const raw = frames.find((m) => m.raw && m.raw.topic === '/device/robot_shadow')
  if (raw) {
    const outer = JSON.parse(raw.raw.json)
    const inner = JSON.parse(outer.data)
    console.log('raw bytes', raw.raw.json.length)
    console.log(JSON.stringify({ online: inner.online, device: inner.product_id, bms: inner.property?.bms_soc, clean: inner.property?.clean_height, dirty: inner.property?.dirty_height, state: inner.robot_state, faults: inner.fault?.length }))
    ws.close(); process.exit(0)
  }
  await new Promise((r) => setTimeout(r, 200))
}
console.log('timeout; frames', frames.length)
ws.close(); process.exit(1)
