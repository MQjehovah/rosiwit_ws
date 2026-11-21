/**
 * hmi_bridge 客户端。
 *
 * 走 HTTP 二进制：robot 侧已经降采样 + zlib，浏览器 fetch 后 inflate。
 * 帧格式见 bridge/hmi_bridge.py 顶部注释；这里只做解码。
 */

import type { CloudData, GridData } from './decode'

export interface BridgeHealth {
  ok: boolean
  version: number
  uptime: number
  grid: { w: number; h: number } | null
  grid_age: number | null
  clouds: Record<string, number | null>
}

export interface CloudOptions {
  /** 体素边长（m），0 = 不降采样 */
  voxel?: number
  max?: number
  /** 服务端最多等待新消息的时间（秒） */
  wait?: number
  timeout?: number
}

const MAGIC_GRID = [0x48, 0x4d, 0x47, 0x31] // HMG1
const MAGIC_CLOUD = [0x48, 0x4d, 0x43, 0x31] // HMC1

export class BridgeClient {
  baseUrl: string
  lastError = ''

  constructor(baseUrl: string) {
    this.baseUrl = baseUrl.replace(/\/+$/, '')
  }

  setBaseUrl(url: string): void {
    this.baseUrl = url.replace(/\/+$/, '')
  }

  get ready(): boolean {
    return !!this.baseUrl
  }

  async health(timeout = 3500): Promise<BridgeHealth> {
    const text = await this.fetchText('/api/v1/health', timeout)
    const json = JSON.parse(text)
    if (!json || json.ok !== true) throw new Error(json?.error || 'bridge health failed')
    return json as BridgeHealth
  }

  async grid(refresh = false): Promise<GridData> {
    const path = `/api/v1/grid${refresh ? '?refresh=1' : ''}`
    const bytes = await this.fetchBin(path, 90000)
    return decodeGrid(bytes)
  }

  async cloud(topic: string, opts: CloudOptions = {}): Promise<CloudData> {
    const params = new URLSearchParams({ topic })
    if (opts.voxel !== undefined) params.set('voxel', String(opts.voxel))
    if (opts.max !== undefined) params.set('max', String(opts.max))
    if (opts.wait !== undefined) params.set('wait', String(opts.wait))
    const bytes = await this.fetchBin(`/api/v1/cloud?${params.toString()}`, opts.timeout ?? 20000)
    return decodeCloud(bytes)
  }

  private async fetchBin(path: string, timeout: number): Promise<Uint8Array> {
    const controller = new AbortController()
    const timer = setTimeout(() => controller.abort(), timeout)
    try {
      const res = await fetch(this.baseUrl + path, {
        signal: controller.signal,
        cache: 'no-store',
      })
      if (!res.ok) {
        let detail = `${res.status}`
        try {
          const json = await res.json()
          detail = json?.error || detail
        } catch {
          /* ignore */
        }
        throw new Error(detail)
      }
      return new Uint8Array(await res.arrayBuffer())
    } catch (e) {
      const err = e as Error
      const text =
        err.name === 'AbortError' ? `请求超时（${Math.round(timeout / 1000)}s）` : err.message
      this.lastError = text
      throw new Error(text)
    } finally {
      clearTimeout(timer)
    }
  }

  private async fetchText(path: string, timeout: number): Promise<string> {
    const controller = new AbortController()
    const timer = setTimeout(() => controller.abort(), timeout)
    try {
      const res = await fetch(this.baseUrl + path, { signal: controller.signal, cache: 'no-store' })
      const text = await res.text()
      if (!res.ok) throw new Error(text.slice(0, 200) || `${res.status}`)
      return text
    } finally {
      clearTimeout(timer)
    }
  }
}

/** 从 rosbridge 地址推导桥接地址：端口 8089 -> 9100，ws(s) -> http(s)。 */
export function deriveBridgeUrl(rosUrl: string): string {
  try {
    const u = new URL(rosUrl)
    u.protocol = u.protocol === 'wss:' ? 'https:' : 'http:'
    if (u.port === '8089') {
      u.port = '9100'
    } else if (u.pathname.includes('/8089/')) {
      u.pathname = u.pathname.replace('/8089/', '/9100/')
    } else if (/\/8089\/?$/.test(u.pathname)) {
      u.pathname = u.pathname.replace(/8089(\/?)$/, '9100$1')
    }
    u.search = ''
    u.hash = ''
    return u.toString().replace(/\/+$/, '')
  } catch {
    return ''
  }
}

// ------------------------------------------------------------------ decode

function hasMagic(bytes: Uint8Array, magic: number[]): boolean {
  if (bytes.length < magic.length) return false
  for (let i = 0; i < magic.length; i++) if (bytes[i] !== magic[i]) return false
  return true
}

async function inflate(bytes: Uint8Array): Promise<Uint8Array> {
  const DS = (globalThis as any).DecompressionStream
  if (!DS) throw new Error('浏览器不支持 DecompressionStream（需要 Chrome/Edge 80+）')
  const stream = new Blob([bytes]).stream().pipeThrough(new DS('deflate'))
  return new Uint8Array(await new Response(stream).arrayBuffer())
}

async function decodeGrid(bytes: Uint8Array): Promise<GridData> {
  if (!hasMagic(bytes, MAGIC_GRID)) throw new Error('不是 HMG1 栅格帧')
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
  let o = 4
  const version = view.getUint8(o)
  o += 1
  const flags = view.getUint8(o)
  o += 1
  if (version !== 1) throw new Error(`不支持的桥接协议版本 ${version}`)
  const step = view.getUint16(o, true)
  o += 2
  const resolution = view.getFloat32(o, true)
  o += 4
  const width = view.getUint32(o, true)
  o += 4
  const height = view.getUint32(o, true)
  o += 4
  const ox = view.getFloat64(o, true)
  o += 8
  const oy = view.getFloat64(o, true)
  o += 8
  const oth = view.getFloat32(o, true)
  o += 4
  const frameLen = view.getUint8(o)
  o += 1
  const frame = new TextDecoder().decode(bytes.subarray(o, o + frameLen))
  o += frameLen
  const stamp = view.getFloat64(o, true)
  o += 8
  const bodyLen = view.getUint32(o, true)
  o += 4
  const body = bytes.subarray(o, o + bodyLen)
  const raw = flags & 1 ? await inflate(body) : body
  const total = width * height
  if (raw.byteLength < total) throw new Error(`栅格数据不足：${raw.byteLength} < ${total}`)
  const data = new Int8Array(raw.buffer, raw.byteOffset, total)
  return {
    width,
    height,
    resolution,
    origin: { x: ox, y: oy, theta: oth },
    data,
    frame: frame || 'map',
    stamp,
    step,
  }
}

async function decodeCloud(bytes: Uint8Array): Promise<CloudData> {
  if (!hasMagic(bytes, MAGIC_CLOUD)) throw new Error('不是 HMC1 点云帧')
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
  let o = 4
  const version = view.getUint8(o)
  o += 1
  const flags = view.getUint8(o)
  o += 1
  if (version !== 1) throw new Error(`不支持的桥接协议版本 ${version}`)
  const count = view.getUint32(o, true)
  o += 4
  const frameLen = view.getUint8(o)
  o += 1
  const frame = new TextDecoder().decode(bytes.subarray(o, o + frameLen))
  o += frameLen
  const stamp = view.getFloat64(o, true)
  o += 8
  const bodyLen = view.getUint32(o, true)
  o += 4
  const body = bytes.subarray(o, o + bodyLen)
  const raw = flags & 1 ? await inflate(body) : body
  if (raw.byteLength < count * 6) throw new Error(`点云数据不足：${raw.byteLength} < ${count * 6}`)

  const src = new DataView(raw.buffer, raw.byteOffset, raw.byteLength)
  const positions = new Float32Array(count * 3)
  let minX = Infinity
  let minY = Infinity
  let minZ = Infinity
  let maxX = -Infinity
  let maxY = -Infinity
  let maxZ = -Infinity
  for (let i = 0; i < count; i++) {
    const x = src.getInt16(i * 6, true) / 100
    const y = src.getInt16(i * 6 + 2, true) / 100
    const z = src.getInt16(i * 6 + 4, true) / 100
    positions[i * 3] = x
    positions[i * 3 + 1] = y
    positions[i * 3 + 2] = z
    if (x < minX) minX = x
    if (x > maxX) maxX = x
    if (y < minY) minY = y
    if (y > maxY) maxY = y
    if (z < minZ) minZ = z
    if (z > maxZ) maxZ = z
  }
  return {
    positions,
    colors: null,
    count,
    total: count,
    frame: frame || 'map',
    bbox: count ? { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] } : null,
    stamp,
  }
}
