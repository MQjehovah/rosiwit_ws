import { TfTree, quatToRpy, quatToYaw } from './tf.mjs'
import { decodePointCloud2, decodeOccupancyGrid, decodeLaserScan, decodePointCloud, floatIntensityColors } from './decode.mjs'
import { gridToImage } from './gridImage.mjs'

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

// ------------------------------------------------------------------ RPY

{
  const roll = 0.12
  const pitch = -0.24
  const yaw = 0.6
  const cr = Math.cos(roll / 2)
  const sr = Math.sin(roll / 2)
  const cp = Math.cos(pitch / 2)
  const sp = Math.sin(pitch / 2)
  const cy = Math.cos(yaw / 2)
  const sy = Math.sin(yaw / 2)
  const q = [
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  ]
  const rpy = quatToRpy(q)
  assert(near(rpy[0], roll) && near(rpy[1], pitch) && near(rpy[2], yaw), 'quatToRpy round-trip')
  assert(near(quatToYaw(q), yaw), 'quatToYaw matches rpy[2]')

  const pureYaw = quatToRpy([0, 0, Math.sin(0.4), Math.cos(0.4)])
  assert(near(pureYaw[0], 0) && near(pureYaw[1], 0) && near(pureYaw[2], 0.8), 'pure yaw rpy')
}

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

// PCD 的 float intensity: 0~255 用固定色标，超出范围按 min/max 拉伸
const fixed = floatIntensityColors(new Float32Array([0, 128, 255]), 3)
assert(fixed && fixed.length === 9, 'float intensity colors')
assert(
  fixed && fixed[0] < 0.4 && fixed[2] > 0.4 && fixed[6] > 0.8 && fixed[8] < 0.2,
  'float intensity fixed 0..255 ramp',
)
const stretched = floatIntensityColors(new Float32Array([100, 200, 300]), 3)
assert(
  fixed &&
    stretched &&
    Math.abs(stretched[0] - fixed[0]) < 1e-6 &&
    Math.abs(stretched[6] - fixed[6]) < 1e-6,
  'float intensity out-of-range auto stretch',
)
assert(floatIntensityColors(null, 3) === null, 'float intensity missing -> null')

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

// ------------------------------------------------------------------ PointCloud (旧版)

const pcloud = decodePointCloud(
  {
    header: { frame_id: 'map' },
    points: [
      { x: 1, y: 2, z: 0.5 },
      { x: -1, y: 0, z: 1 },
    ],
  },
  { maxPoints: 200000 },
)
assert(pcloud && pcloud.count === 2, 'PointCloud decoded count')
assert(pcloud && pcloud.frame === 'map', 'PointCloud frame')
assert(pcloud && pcloud.colors && pcloud.colors.length === 6, 'PointCloud height colors')
assert(
  pcloud && near(pcloud.positions[0], 1) && near(pcloud.positions[3], -1),
  'PointCloud positions',
)

// OccupancyGrid 的 int8[] 路径
const gridInt8 = decodeOccupancyGrid({
  header: { frame_id: 'map' },
  info: { width: 2, height: 2, resolution: 0.5, origin: { position: {}, orientation: {} } },
  data: new Int8Array([0, 100, -1, 40]),
})
assert(gridInt8 && gridInt8.data[1] === 100 && gridInt8.data[2] === -1, 'OccupancyGrid accepts Int8Array')

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
