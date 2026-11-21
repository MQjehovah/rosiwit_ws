/**
 * rosbridge JSON 消息解码：PointCloud2 / OccupancyGrid / LaserScan / Path / Polygon。
 * 只保留渲染需要的字段，避免把整个 ROS 消息常驻内存。
 */

export interface Vec3Box {
  min: [number, number, number]
  max: [number, number, number]
}

export interface CloudData {
  positions: Float32Array
  colors: Float32Array | null
  count: number
  total: number
  frame: string
  bbox: Vec3Box | null
  stamp?: number
}

export interface GridData {
  stream?: string
  width: number
  height: number
  resolution: number
  origin: { x: number; y: number; theta: number }
  data: Int8Array
  frame: string
  stamp?: number
  step?: number
}

export type ColorMode = 'height' | 'intensity' | 'flat'

// ------------------------------------------------------------------ base64

export function base64ToBytes(input: string): Uint8Array {
  const bin = atob(input)
  const out = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
  return out
}

function toBytes(data: unknown): Uint8Array | null {
  if (typeof data === 'string') {
    try {
      return base64ToBytes(data)
    } catch {
      return null
    }
  }
  if (Array.isArray(data)) return Uint8Array.from(data as number[])
  if (data instanceof Uint8Array) return data
  // CBOR 的 int8[] 走 tag 72，解出来是 Int8Array
  if (data instanceof Int8Array) return new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
  return null
}

// ------------------------------------------------------------------ color

function hsvToRgb(h: number, s: number, v: number): [number, number, number] {
  const i = Math.floor(h * 6)
  const f = h * 6 - i
  const p = v * (1 - s)
  const q = v * (1 - f * s)
  const t = v * (1 - (1 - f) * s)
  switch (i % 6) {
    case 0:
      return [v, t, p]
    case 1:
      return [q, v, p]
    case 2:
      return [p, v, t]
    case 3:
      return [p, q, v]
    case 4:
      return [t, p, v]
    default:
      return [v, p, q]
  }
}

/** 蓝 → 青 → 绿 → 黄 → 红 */
function rampColor(t: number): [number, number, number] {
  const c = Math.min(1, Math.max(0, t))
  return hsvToRgb(0.66 * (1 - c), 0.9, 0.45 + 0.55 * c)
}

// ------------------------------------------------------------------ PointCloud2

export function decodePointCloud2(
  msg: any,
  opts: { maxPoints?: number; colorMode?: ColorMode } = {},
): CloudData | null {
  if (!msg || !Array.isArray(msg.fields)) return null

  const width = Number(msg.width) || 0
  const height = Number(msg.height) || 1
  const pointStep = Number(msg.point_step) || 0
  const rowStep = Number(msg.row_step) || pointStep * width
  if (!width || !pointStep) return null

  const bytes = toBytes(msg.data)
  if (!bytes || bytes.length < pointStep) return null

  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
  const little = !msg.is_bigendian

  const field = (name: string) =>
    (msg.fields as any[]).find((f) => f.name === name && f.datatype)

  const fx = field('x')
  const fy = field('y')
  const fz = field('z')
  if (!fx || !fy || !fz) return null
  const fi = field('intensity') ?? field('i')
  const colorMode: ColorMode =
    opts.colorMode ?? (fi ? 'intensity' : 'height')

  const read = (base: number, f: any): number => {
    const o = base + f.offset
    switch (f.datatype) {
      case 1:
        return view.getInt8(o)
      case 2:
        return view.getUint8(o)
      case 3:
        return view.getInt16(o, little)
      case 4:
        return view.getUint16(o, little)
      case 5:
        return view.getInt32(o, little)
      case 6:
        return view.getUint32(o, little)
      case 7:
        return view.getFloat32(o, little)
      case 8:
        return view.getFloat64(o, little)
      default:
        return 0
    }
  }

  const total = width * height
  const maxPoints = opts.maxPoints ?? 250000
  const step = Math.max(1, Math.ceil(total / maxPoints))
  const capacity = Math.ceil(total / step)

  const positions = new Float32Array(capacity * 3)
  const values = colorMode === 'flat' ? null : new Float32Array(capacity)

  let n = 0
  let minX = Infinity,
    minY = Infinity,
    minZ = Infinity,
    maxX = -Infinity,
    maxY = -Infinity,
    maxZ = -Infinity
  let minV = Infinity,
    maxV = -Infinity

  for (let i = 0; i < total; i += step) {
    const row = Math.floor(i / width)
    const col = i - row * width
    const base = row * rowStep + col * pointStep
    if (base + pointStep > bytes.length) break
    const x = read(base, fx)
    const y = read(base, fy)
    const z = read(base, fz)
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue
    const o = n * 3
    positions[o] = x
    positions[o + 1] = y
    positions[o + 2] = z
    if (x < minX) minX = x
    if (y < minY) minY = y
    if (z < minZ) minZ = z
    if (x > maxX) maxX = x
    if (y > maxY) maxY = y
    if (z > maxZ) maxZ = z
    if (values) {
      const v = colorMode === 'intensity' && fi ? read(base, fi) : z
      values[n] = v
      if (v < minV) minV = v
      if (v > maxV) maxV = v
    }
    n++
  }

  if (!n) return null

  let colors: Float32Array | null = null
  if (values && colorMode !== 'flat') {
    colors = new Float32Array(n * 3)
    const span = maxV - minV || 1
    for (let i = 0; i < n; i++) {
      const [r, g, b] = rampColor((values[i] - minV) / span)
      colors[i * 3] = r
      colors[i * 3 + 1] = g
      colors[i * 3 + 2] = b
    }
  }

  const count = n
  return {
    positions: positions.slice(0, count * 3),
    colors,
    count,
    total,
    frame: msg.header?.frame_id ?? '',
    bbox: {
      min: [minX, minY, minZ],
      max: [maxX, maxY, maxZ],
    },
  }
}

// ------------------------------------------------------------------ sensor_msgs/PointCloud (旧版)

/**
 * legacy `sensor_msgs/PointCloud`（/scan_f_map 用的就是它）：
 * points 是 [{x,y,z}] 数组。定位模式下这些点已经是 map 系下配准后的当前帧。
 */
export function decodePointCloud(
  msg: any,
  opts: { maxPoints?: number; colorMode?: ColorMode } = {},
): CloudData | null {
  const pts: any[] = msg?.points
  if (!Array.isArray(pts) || pts.length === 0) return null

  const total = pts.length
  const maxPoints = opts.maxPoints ?? 250000
  const step = Math.max(1, Math.ceil(total / maxPoints))
  const capacity = Math.ceil(total / step)
  const positions = new Float32Array(capacity * 3)

  let n = 0
  let minZ = Infinity
  let maxZ = -Infinity
  let minX = Infinity
  let minY = Infinity
  let maxX = -Infinity
  let maxY = -Infinity

  for (let i = 0; i < total; i += step) {
    const p = pts[i]
    const x = Number(p?.x)
    const y = Number(p?.y)
    const z = Number(p?.z)
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue
    const o = n * 3
    positions[o] = x
    positions[o + 1] = y
    positions[o + 2] = z
    if (x < minX) minX = x
    if (x > maxX) maxX = x
    if (y < minY) minY = y
    if (y > maxY) maxY = y
    if (z < minZ) minZ = z
    if (z > maxZ) maxZ = z
    n++
  }
  if (!n) return null

  let colors: Float32Array | null = null
  if ((opts.colorMode ?? 'height') === 'height') {
    colors = new Float32Array(n * 3)
    const span = maxZ - minZ || 1
    for (let i = 0; i < n; i++) {
      const [r, g, b] = rampColor((positions[i * 3 + 2] - minZ) / span)
      colors[i * 3] = r
      colors[i * 3 + 1] = g
      colors[i * 3 + 2] = b
    }
  }

  return {
    positions: positions.slice(0, n * 3),
    colors,
    count: n,
    total,
    frame: msg.header?.frame_id ?? '',
    bbox: { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] },
  }
}

// ------------------------------------------------------------------ OccupancyGrid

function quatYaw(q: any): number {
  if (!q) return 0
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

export function decodeOccupancyGrid(msg: any): GridData | null {
  const info = msg?.info
  if (!info) return null
  const width = Number(info.width) || 0
  const height = Number(info.height) || 0
  if (!width || !height) return null

  const bytes = toBytes(msg.data)
  let data: Int8Array | null = null
  if (bytes) {
    data = new Int8Array(bytes.buffer, bytes.byteOffset, Math.min(bytes.byteLength, width * height))
  } else if (Array.isArray(msg.data)) {
    const arr = Int8Array.from(msg.data as number[], (v) => Math.max(-1, Math.min(100, v)))
    data = arr
  }
  if (!data) return null

  const origin = info.origin ?? {}
  return {
    width,
    height,
    resolution: Number(info.resolution) || 0.05,
    origin: {
      x: Number(origin.position?.x) || 0,
      y: Number(origin.position?.y) || 0,
      theta: quatYaw(origin.orientation),
    },
    data,
    frame: msg.header?.frame_id ?? '',
  }
}

// ------------------------------------------------------------------ LaserScan

export function decodeLaserScan(msg: any): CloudData | null {
  const ranges = msg?.ranges
  if (!Array.isArray(ranges) || ranges.length === 0) return null
  const angleMin = Number(msg.angle_min) || 0
  const inc = Number(msg.angle_increment) || 0
  const rangeMin = Number(msg.range_min) || 0
  const rangeMax = Number(msg.range_max) || Infinity

  const positions = new Float32Array(ranges.length * 3)
  let n = 0
  let minX = Infinity,
    maxX = -Infinity,
    minY = Infinity,
    maxY = -Infinity
  for (let i = 0; i < ranges.length; i++) {
    const r = Number(ranges[i])
    if (!Number.isFinite(r) || r < rangeMin || r > rangeMax) continue
    const a = angleMin + inc * i
    const x = Math.cos(a) * r
    const y = Math.sin(a) * r
    positions[n * 3] = x
    positions[n * 3 + 1] = y
    positions[n * 3 + 2] = 0
    if (x < minX) minX = x
    if (x > maxX) maxX = x
    if (y < minY) minY = y
    if (y > maxY) maxY = y
    n++
  }
  if (!n) return null
  return {
    positions: positions.slice(0, n * 3),
    colors: null,
    count: n,
    total: ranges.length,
    frame: msg.header?.frame_id ?? '',
    bbox: { min: [minX, minY, 0], max: [maxX, maxY, 0] },
  }
}

// ------------------------------------------------------------------ Path / Polygon

export interface PolylineData {
  positions: Float32Array
  frame: string
  count: number
}

export function decodePath(msg: any): PolylineData | null {
  const poses = msg?.poses
  if (!Array.isArray(poses) || poses.length < 2) return null
  const positions = new Float32Array(poses.length * 3)
  for (let i = 0; i < poses.length; i++) {
    const p = poses[i]?.pose?.position
    positions[i * 3] = Number(p?.x) || 0
    positions[i * 3 + 1] = Number(p?.y) || 0
    positions[i * 3 + 2] = Number(p?.z) || 0
  }
  return {
    positions,
    count: poses.length,
    frame: msg.header?.frame_id ?? '',
  }
}

export function decodePolygon(msg: any): PolylineData | null {
  const pts = msg?.polygon?.points
  if (!Array.isArray(pts) || pts.length < 3) return null
  const positions = new Float32Array(pts.length * 3)
  for (let i = 0; i < pts.length; i++) {
    positions[i * 3] = Number(pts[i]?.x) || 0
    positions[i * 3 + 1] = Number(pts[i]?.y) || 0
    positions[i * 3 + 2] = 0
  }
  return {
    positions,
    count: pts.length,
    frame: msg.header?.frame_id ?? '',
  }
}
