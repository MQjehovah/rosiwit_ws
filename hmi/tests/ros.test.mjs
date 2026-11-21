import { TfTree, quatToYaw } from './tf.mjs'
import { decodePointCloud2, decodeOccupancyGrid, decodeLaserScan, decodePointCloud } from './decode.mjs'
import { decodeCbor } from './cbor.mjs'
import { gridToImage } from './gridImage.mjs'
import fs from 'node:fs'

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg)
    process.exitCode = 1
  } else {
    console.log('ok  :', msg)
  }
}

function near(a, b, eps = 1e-6) {
  return Math.abs(a - b) <= eps
}

// ------------------------------------------------------------------ TF

const tree = new TfTree()
tree.ingest({
  transforms: [
    {
      header: { frame_id: 'map' },
      child_frame_id: 'odom',
      transform: { translation: { x: 5, y: 0, z: 0 }, rotation: { x: 0, y: 0, z: 0, w: 1 } },
    },
    {
      header: { frame_id: 'odom' },
      child_frame_id: 'base_link',
      transform: {
        translation: { x: 1, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: Math.sin(Math.PI / 4), w: Math.cos(Math.PI / 4) },
      },
    },
  ],
})

const fwd = tree.lookup('map', 'base_link')
assert(fwd && near(fwd.t[0], 6) && near(fwd.t[1], 0), 'chain map<-base_link translation')
assert(fwd && near(quatToYaw(fwd.q), Math.PI / 2), 'chain map<-base_link yaw')

const inv = tree.lookup('base_link', 'map')
assert(inv && near(inv.t[0], 0) && near(inv.t[1], 6), 'inverse base_link<-map translation')
assert(inv && near(quatToYaw(inv.q), -Math.PI / 2), 'inverse base_link<-map yaw')

assert(tree.lookup('odom', 'odom') !== null, 'identity lookup')
assert(tree.lookup('map', 'nope') === null, 'missing frame returns null')
assert(tree.frames().sort().join(',') === 'base_link,map,odom', 'frame list')

// ------------------------------------------------------------------ PointCloud2

const pts = [
  [1, 2, 3, 10],
  [-1, 0.5, 0.2, 20],
  [0, 0, 0, 0],
]
const buf = new ArrayBuffer(pts.length * 16)
const dv = new DataView(buf)
pts.forEach((p, i) => {
  dv.setFloat32(i * 16, p[0], true)
  dv.setFloat32(i * 16 + 4, p[1], true)
  dv.setFloat32(i * 16 + 8, p[2], true)
  dv.setFloat32(i * 16 + 12, p[3], true)
})
const b64 = Buffer.from(buf).toString('base64')
const cloud = decodePointCloud2({
  header: { frame_id: 'map' },
  width: 3,
  height: 1,
  point_step: 16,
  row_step: 48,
  is_bigendian: false,
  fields: [
    { name: 'x', offset: 0, datatype: 7, count: 1 },
    { name: 'y', offset: 4, datatype: 7, count: 1 },
    { name: 'z', offset: 8, datatype: 7, count: 1 },
    { name: 'intensity', offset: 12, datatype: 7, count: 1 },
  ],
  data: b64,
})
assert(cloud && cloud.count === 3, 'pointcloud point count')
assert(cloud && near(cloud.positions[0], 1) && near(cloud.positions[4], 0.5), 'pointcloud xyz')
assert(cloud && cloud.colors && cloud.colors.length === 9, 'pointcloud intensity colors')
assert(cloud && cloud.frame === 'map', 'pointcloud frame')

// 全 0 字节 = 三个 0 坐标，属于有效点
assert(
  decodePointCloud2({
    width: 1,
    height: 1,
    point_step: 12,
    fields: [
      { name: 'x', offset: 0, datatype: 7, count: 1 },
      { name: 'y', offset: 4, datatype: 7, count: 1 },
      { name: 'z', offset: 8, datatype: 7, count: 1 },
    ],
    data: Buffer.from(new Uint8Array(12)).toString('base64'),
  })?.count === 1,
  'pointcloud keeps zero points',
)

// ------------------------------------------------------------------ OccupancyGrid

const grid = decodeOccupancyGrid({
  header: { frame_id: 'map' },
  info: {
    width: 2,
    height: 2,
    resolution: 0.5,
    origin: {
      position: { x: -1, y: -2, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
  },
  data: [0, 100, -1, 40],
})
assert(grid && grid.width === 2 && grid.resolution === 0.5, 'grid dims')
assert(grid && grid.data[1] === 100 && grid.data[2] === -1, 'grid int8 values')
assert(grid && near(grid.origin.x, -1) && near(grid.origin.y, -2), 'grid origin')

// ------------------------------------------------------------------ LaserScan

const scan = decodeLaserScan({
  header: { frame_id: 'laser' },
  angle_min: 0,
  angle_max: Math.PI,
  angle_increment: Math.PI / 2,
  range_min: 0.1,
  range_max: 10,
  ranges: [1, 2, null, 0.01],
})
assert(scan && scan.count === 2, 'laser scan filters invalid ranges')
assert(scan && near(scan.positions[0], 1) && near(scan.positions[3], 0), 'laser scan xy')

// ------------------------------------------------------------------ CBOR

// tag 72 = int8[]；验证 typed-array tag 能还原成 Int8Array
const typed = decodeCbor(new Uint8Array([0xa1, 0x61, 0x61, 0xd8, 0x48, 0x43, 0x00, 0x64, 0xff]))
assert(typed.a instanceof Int8Array, 'cbor tag72 -> Int8Array')
assert(typed.a.length === 3 && typed.a[1] === 100 && typed.a[2] === -1, 'cbor int8 values')

// 真实 rosbridge cbor 帧（/scan_f_map，压缩后 61KB）
const fixture = new URL('./fixtures/scan_f_map.cbor', import.meta.url)
if (fs.existsSync(fixture)) {
  const frame = decodeCbor(new Uint8Array(fs.readFileSync(fixture)))
  assert(frame?.op === 'publish', 'cbor frame op')
  assert(frame?.topic === '/scan_f_map', 'cbor frame topic')
  assert(frame?.msg?.header?.frame_id === 'map', 'cbor frame_id = map')
  assert(Array.isArray(frame?.msg?.points) && frame.msg.points.length > 500, 'cbor PointCloud points')

  const cloud = decodePointCloud(frame.msg, { maxPoints: 200000 })
  assert(cloud && cloud.count === frame.msg.points.length, 'PointCloud decoded count')
  assert(cloud && cloud.frame === 'map', 'PointCloud frame')
  assert(cloud && cloud.colors && cloud.colors.length === cloud.count * 3, 'PointCloud height colors')
  assert(
    cloud && Number.isFinite(cloud.positions[0]) && Number.isFinite(cloud.positions[2]),
    'PointCloud positions finite',
  )

  // 同一份数据用 OccupancyGrid 的 int8[] 路径也能吃 Int8Array
  const grid = decodeOccupancyGrid({
    header: { frame_id: 'map' },
    info: { width: 2, height: 2, resolution: 0.5, origin: { position: {}, orientation: {} } },
    data: new Int8Array([0, 100, -1, 40]),
  })
  assert(grid && grid.data[1] === 100 && grid.data[2] === -1, 'OccupancyGrid accepts Int8Array')
} else {
  console.log('skip: tests/fixtures/scan_f_map.cbor 不存在（可用设备重新抓一份）')
}

// ------------------------------------------------------------------ 栅格降采样

{
  // 4×4，限 4 个像素 → step=2；每 2×2 取最大占据值
  const gi = gridToImage(
    {
      width: 4,
      height: 4,
      resolution: 0.05,
      origin: { x: 0, y: 0, theta: 0 },
      frame: 'map',
      data: new Int8Array([
        0, 100, -1, -1,
        0, 60, -1, -1,
        -1, -1, 0, 0,
        -1, -1, 0, 50,
      ]),
    },
    4,
  )
  assert(gi.width === 2 && gi.height === 2 && gi.step === 2, 'grid image dims')
  assert(gi.pixels[3] === 235, 'grid block max -> occupied')
  assert(gi.pixels[7] === 0, 'grid block all unknown -> transparent')
  assert(gi.pixels[11] === 0, 'grid block all unknown (bottom-left) -> transparent')
  assert(gi.pixels[15] === 140, 'grid block 50 -> semi occupied')
}

if (!process.exitCode) console.log('\nall tests passed')
