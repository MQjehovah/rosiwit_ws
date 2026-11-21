import { BridgeClient, deriveBridgeUrl } from './bridge.mjs'

const base = process.env.BRIDGE_URL || 'http://127.0.0.1:9100'
const client = new BridgeClient(base)

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg)
    process.exitCode = 1
  } else {
    console.log('ok  :', msg)
  }
}

const health = await client.health()
assert(health.ok === true, 'health ok')
console.log('     ', JSON.stringify(health))

const grid = await client.grid()
assert(grid.width > 0 && grid.height > 0, 'grid dims')
assert(grid.data.length === grid.width * grid.height, 'grid data length')
assert(grid.resolution > 0 && grid.frame === 'map', 'grid meta')
console.log(
  `      grid ${grid.width}x${grid.height} res=${grid.resolution.toFixed(3)} ` +
    `origin=(${grid.origin.x.toFixed(1)},${grid.origin.y.toFixed(1)}) cells=${grid.data.length}`,
)

const cloud = await client.cloud('scan_f_map', { voxel: 0.08, max: 200000 })
assert(cloud.count > 0, 'cloud has points')
assert(cloud.positions.length === cloud.count * 3, 'cloud positions length')
assert(cloud.frame === 'map', 'cloud frame')
let finite = true
for (let i = 0; i < cloud.positions.length; i++) {
  if (!Number.isFinite(cloud.positions[i])) {
    finite = false
    break
  }
}
assert(finite, 'cloud positions finite')
console.log(`      cloud count=${cloud.count} first=(${cloud.positions.slice(0, 3).join(', ')})`)

assert(deriveBridgeUrl('ws://192.168.1.100:8089') === 'http://192.168.1.100:9100', 'derive plain')
assert(
  deriveBridgeUrl('wss://test.rosiwit.com/remote-terminal/tunnel/XZ-SC50/TOK/127.0.0.1/8089/') ===
    'https://test.rosiwit.com/remote-terminal/tunnel/XZ-SC50/TOK/127.0.0.1/9100',
  'derive tunnel',
)

if (!process.exitCode) console.log('\nall bridge tests passed')
