/**
 * debug_bridge 客户端：WebSocket 长连接 + protobuf 二进制帧。
 *
 * 协议见 debug_bridge/proto/debug_bridge.proto（当前文件生成到
 * src/ros/proto/debug_bridge.json）。机器人侧负责体素降采样、int16 厘米量化和
 * zlib，浏览器只做解码和渲染。
 */

import protobuf from 'protobufjs/light'
import descriptor from './proto/debug_bridge.json'
import type { CloudData, GridData } from './decode'

const root = protobuf.Root.fromJSON(descriptor as any)
const Envelope = root.lookupType('debug_bridge.Envelope')
const ClientCommand = root.lookupType('debug_bridge.ClientCommand')

const ACTION_SUBSCRIBE = 0
const ACTION_UNSUBSCRIBE = 1
const ACTION_REQUEST = 2

export interface StreamInfo {
  name: string
  available: boolean
  rate_hz: number
}

export interface BridgeHello {
  name: string
  version: string
  streams: StreamInfo[]
}

export interface BridgeStatus {
  stream: string
  ok: boolean
  message: string
  stamp_ms: number
}

export interface SubscribeOptions {
  /** 推送频率（Hz） */
  rate: number
  /** 体素边长（m） */
  voxel?: number
  max?: number
}

export interface RequestOptions {
  voxel?: number
  max?: number
  refresh?: boolean
  timeout?: number
}

interface Waiter<T> {
  resolve: (value: T) => void
  reject: (err: Error) => void
  timer: ReturnType<typeof setTimeout>
}

interface HelloWaiter {
  resolve: () => void
  reject: (err: Error) => void
  timer: ReturnType<typeof setTimeout>
}

type CloudListener = (stream: string, cloud: CloudData) => void
type HelloListener = (hello: BridgeHello) => void
type StatusListener = (status: BridgeStatus) => void
type ConnectionListener = (connected: boolean) => void
type ErrorListener = (message: string) => void

export class BridgeClient {
  url: string
  ws: WebSocket | null = null
  hello: BridgeHello | null = null
  lastError = ''
  autoReconnect = true

  private subs = new Map<string, SubscribeOptions>()
  private gridWaiters: Waiter<GridData>[] = []
  private cloudWaiters = new Map<string, Waiter<CloudData>[]>()
  private helloWaiters: HelloWaiter[] = []
  private cloudListeners = new Set<CloudListener>()
  private helloListeners = new Set<HelloListener>()
  private statusListeners = new Set<StatusListener>()
  private connectionListeners = new Set<ConnectionListener>()
  private errorListeners = new Set<ErrorListener>()
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  private manualClose = false
  private connecting: Promise<void> | null = null

  constructor(url: string) {
    this.url = normalizeBridgeUrl(url)
  }

  get connected(): boolean {
    return !!this.hello
  }

  setUrl(url: string): void {
    const normalized = normalizeBridgeUrl(url)
    if (normalized === this.url) return
    this.url = normalized
    if (this.ws) {
      this.disconnect()
      void this.connect().catch(() => {
        /* 状态由回调处理 */
      })
    }
  }

  // ---------------------------------------------------------------- lifecycle

  connect(timeout = 5000): Promise<void> {
    if (this.connected) return Promise.resolve()
    if (this.connecting) return this.connecting
    this.manualClose = false
    this.clearReconnect()
    if (!this.url) {
      return Promise.reject(new Error('未配置桥接地址'))
    }

    const ws = new WebSocket(this.url)
    ws.binaryType = 'arraybuffer'
    this.ws = ws

    this.connecting = new Promise<void>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.dropHelloWaiters(new Error('连接超时'))
        try {
          ws.close()
        } catch {
          /* ignore */
        }
        reject(new Error('连接超时'))
      }, timeout)
      this.helloWaiters.push({
        resolve: () => {
          clearTimeout(timer)
          resolve()
        },
        reject: (err) => {
          clearTimeout(timer)
          reject(err)
        },
        timer,
      })
    })

    ws.onopen = () => {
      this.lastError = ''
    }
    ws.onerror = () => {
      this.lastError = 'WebSocket 连接错误'
    }
    ws.onclose = () => {
      this.ws = null
      this.connecting = null
      const wasConnected = !!this.hello
      this.hello = null
      this.lastError = this.lastError || '连接已断开'
      this.dropHelloWaiters(new Error(this.lastError))
      this.dropAllWaiters(new Error(this.lastError))
      if (wasConnected) this.emitConnection(false)
      if (this.autoReconnect && !this.manualClose) this.scheduleReconnect()
    }
    ws.onmessage = (ev: MessageEvent) => {
      if (ev.data instanceof ArrayBuffer) void this.handleFrame(new Uint8Array(ev.data))
    }

    return this.connecting
  }

  disconnect(): void {
    this.manualClose = true
    this.clearReconnect()
    this.subs.clear()
    this.hello = null
    this.connecting = null
    this.dropHelloWaiters(new Error('已断开'))
    this.dropAllWaiters(new Error('已断开'))
    this.emitConnection(false)
    if (this.ws) {
      const ws = this.ws
      this.ws = null
      ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null
      try {
        ws.close()
      } catch {
        /* ignore */
      }
    }
  }

  private scheduleReconnect(): void {
    this.clearReconnect()
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null
      if (this.manualClose) return
      void this.connect().catch(() => {
        /* onclose 会继续重连 */
      })
    }, 2500)
  }

  private clearReconnect(): void {
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
  }

  // ------------------------------------------------------------- subscriptions

  setSubscription(stream: string, opts: SubscribeOptions | null): void {
    if (!opts) {
      if (!this.subs.has(stream)) return
      this.subs.delete(stream)
      if (this.connected) this.sendCommand({ action: ACTION_UNSUBSCRIBE, stream })
      return
    }
    this.subs.set(stream, opts)
    if (this.connected) this.sendSubscribe(stream, opts)
  }

  clearSubscriptions(): void {
    for (const stream of this.subs.keys()) {
      this.sendCommand({ action: ACTION_UNSUBSCRIBE, stream })
    }
    this.subs.clear()
  }

  private sendSubscribe(stream: string, opts: SubscribeOptions): void {
    this.sendCommand({
      action: ACTION_SUBSCRIBE,
      stream,
      rate_hz: opts.rate,
      voxel: opts.voxel ?? 0,
      max_points: opts.max ?? 0,
    })
  }

  // ------------------------------------------------------------------ requests

  requestGrid(refresh = true, timeout = 90000): Promise<GridData> {
    return this.request(this.gridWaiters, timeout, {
      action: ACTION_REQUEST,
      stream: 'grid',
      refresh,
    })
  }

  requestCloud(stream: string, opts: RequestOptions = {}): Promise<CloudData> {
    const bucket = this.cloudWaiters.get(stream) ?? []
    this.cloudWaiters.set(stream, bucket)
    return this.request(bucket, opts.timeout ?? 60000, {
      action: ACTION_REQUEST,
      stream,
      voxel: opts.voxel ?? 0,
      max_points: opts.max ?? 0,
      refresh: !!opts.refresh,
    })
  }

  private request<T>(bucket: Waiter<T>[], timeout: number, command: any): Promise<T> {
    if (!this.connected) return Promise.reject(new Error('桥接未连接'))
    return new Promise<T>((resolve, reject) => {
      const timer = setTimeout(() => {
        const i = bucket.indexOf(waiter)
        if (i >= 0) bucket.splice(i, 1)
        reject(new Error(`请求超时（${Math.round(timeout / 1000)}s）`))
      }, timeout)
      const waiter: Waiter<T> = { resolve, reject, timer }
      bucket.push(waiter)
      if (!this.sendCommand(command)) {
        clearTimeout(timer)
        const i = bucket.indexOf(waiter)
        if (i >= 0) bucket.splice(i, 1)
        reject(new Error('发送失败'))
      }
    })
  }

  private sendCommand(command: any): boolean {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false
    try {
      const bytes = ClientCommand.encode(ClientCommand.create(command)).finish()
      this.ws.send(bytes)
      return true
    } catch (e) {
      this.lastError = (e as Error).message
      return false
    }
  }

  // -------------------------------------------------------------------- events

  onCloud(fn: CloudListener): () => void {
    this.cloudListeners.add(fn)
    return () => this.cloudListeners.delete(fn)
  }

  onHello(fn: HelloListener): () => void {
    this.helloListeners.add(fn)
    return () => this.helloListeners.delete(fn)
  }

  onStatus(fn: StatusListener): () => void {
    this.statusListeners.add(fn)
    return () => this.statusListeners.delete(fn)
  }

  onConnection(fn: ConnectionListener): () => void {
    this.connectionListeners.add(fn)
    return () => this.connectionListeners.delete(fn)
  }

  onError(fn: ErrorListener): () => void {
    this.errorListeners.add(fn)
    return () => this.errorListeners.delete(fn)
  }

  private emitConnection(connected: boolean): void {
    for (const fn of this.connectionListeners) fn(connected)
  }

  // -------------------------------------------------------------------- frames

  private async handleFrame(bytes: Uint8Array): Promise<void> {
    let msg: any
    try {
      msg = Envelope.decode(bytes)
    } catch (e) {
      this.lastError = `protobuf 解码失败：${(e as Error).message}`
      this.emitError(this.lastError)
      return
    }

    if (msg.hello) {
      this.hello = {
        name: msg.hello.name,
        version: msg.hello.version,
        streams: (msg.hello.streams || []).map((s: any) => ({
          name: s.name,
          available: !!s.available,
          rate_hz: s.rate_hz || 0,
        })),
      }
      this.lastError = ''
      this.connecting = null
      for (const waiter of this.helloWaiters.splice(0)) {
        clearTimeout(waiter.timer)
        waiter.resolve()
      }
      for (const fn of this.helloListeners) fn(this.hello)
      this.emitConnection(true)
      // 断线重连后恢复订阅
      for (const [stream, opts] of this.subs) this.sendSubscribe(stream, opts)
      return
    }

    if (msg.status) {
      const status: BridgeStatus = {
        stream: msg.status.stream,
        ok: !!msg.status.ok,
        message: msg.status.message,
        stamp_ms: Number(msg.status.stamp_ms || 0),
      }
      for (const fn of this.statusListeners) fn(status)
      if (!status.ok) this.rejectStream(status.stream, new Error(status.message))
      return
    }

    if (msg.error) {
      const text = msg.error.message || 'bridge error'
      this.lastError = text
      this.emitError(text)
      this.dropAllWaiters(new Error(text))
      return
    }

    if (msg.grid) {
      try {
        const grid = await decodeGrid(msg.grid)
        const waiter = this.gridWaiters.shift()
        if (waiter) {
          clearTimeout(waiter.timer)
          waiter.resolve(grid)
        }
      } catch (e) {
        this.failWaiter(this.gridWaiters, new Error((e as Error).message))
      }
      return
    }

    if (msg.cloud) {
      try {
        const cloud = await decodeCloud(msg.cloud)
        const stream: string = msg.cloud.stream || ''
        const bucket = this.cloudWaiters.get(stream)
        const waiter = bucket?.shift()
        if (waiter) {
          clearTimeout(waiter.timer)
          waiter.resolve(cloud)
        }
        if (this.subs.has(stream)) {
          for (const fn of this.cloudListeners) fn(stream, cloud)
        }
      } catch (e) {
        this.failWaiter(this.cloudWaiters.get(msg.cloud.stream) ?? [], new Error((e as Error).message))
      }
    }
  }

  private emitError(message: string): void {
    for (const fn of this.errorListeners) fn(message)
  }

  private rejectStream(stream: string, err: Error): void {
    const bucket = this.cloudWaiters.get(stream)
    if (bucket?.length) {
      this.failWaiter(bucket, err)
      return
    }
    if (stream === 'grid' && this.gridWaiters.length) this.failWaiter(this.gridWaiters, err)
  }

  private failWaiter<T>(bucket: Waiter<T>[], err: Error): void {
    const waiter = bucket.shift()
    if (!waiter) return
    clearTimeout(waiter.timer)
    waiter.reject(err)
  }

  private dropAllWaiters(err: Error): void {
    for (const waiter of this.gridWaiters.splice(0)) {
      clearTimeout(waiter.timer)
      waiter.reject(err)
    }
    for (const bucket of this.cloudWaiters.values()) {
      for (const waiter of bucket.splice(0)) {
        clearTimeout(waiter.timer)
        waiter.reject(err)
      }
    }
    this.cloudWaiters.clear()
  }

  private dropHelloWaiters(err: Error): void {
    for (const waiter of this.helloWaiters.splice(0)) {
      clearTimeout(waiter.timer)
      waiter.reject(err)
    }
  }
}

// ------------------------------------------------------------------ url

/** 从 rosbridge 地址推导桥接地址：8089 -> 9100，ws(s) 保持。 */
export function deriveBridgeUrl(rosUrl: string): string {
  try {
    const u = new URL(rosUrl)
    u.protocol = u.protocol === 'wss:' ? 'wss:' : 'ws:'
    if (u.port === '8089') {
      u.port = '9100'
    } else if (u.pathname.includes('/8089/')) {
      u.pathname = u.pathname.replace('/8089/', '/9100/')
    } else if (/\/8089\/?$/.test(u.pathname)) {
      u.pathname = u.pathname.replace(/8089(\/?)$/, '9100$1')
    }
    u.search = ''
    u.hash = ''
    let out = u.toString()
    if (!out.endsWith('/')) out += '/'
    return out
  } catch {
    return ''
  }
}

/** 兼容旧配置里的 http(s) 地址。 */
export function normalizeBridgeUrl(url: string): string {
  const trimmed = url.trim()
  if (!trimmed) return ''
  try {
    const u = new URL(trimmed)
    if (u.protocol === 'http:') u.protocol = 'ws:'
    if (u.protocol === 'https:') u.protocol = 'wss:'
    let out = u.toString()
    if (!out.endsWith('/')) out += '/'
    return out
  } catch {
    return trimmed
  }
}

// ------------------------------------------------------------------ decode

async function inflate(bytes: Uint8Array): Promise<Uint8Array> {
  const DS = (globalThis as any).DecompressionStream
  if (!DS) throw new Error('浏览器不支持 DecompressionStream（需要 Chrome/Edge 80+）')
  const stream = new Blob([bytes]).stream().pipeThrough(new DS('deflate'))
  return new Uint8Array(await new Response(stream).arrayBuffer())
}

async function decodeGrid(frame: any): Promise<GridData> {
  const width = Number(frame.width) || 0
  const height = Number(frame.height) || 0
  if (!width || !height) throw new Error('栅格尺寸为 0')
  const raw = Number(frame.compression) === 1 ? await inflate(frame.data) : frame.data
  const total = width * height
  if (raw.byteLength < total) throw new Error(`栅格数据不足：${raw.byteLength} < ${total}`)
  return {
    width,
    height,
    resolution: Number(frame.resolution) || 0.05,
    origin: {
      x: Number(frame.origin?.x) || 0,
      y: Number(frame.origin?.y) || 0,
      theta: Number(frame.origin?.theta) || 0,
    },
    data: new Int8Array(raw.buffer, raw.byteOffset, total),
    frame: frame.header?.frame_id || 'map',
    stamp: Number(frame.header?.stamp) || 0,
  }
}

async function decodeCloud(frame: any): Promise<CloudData> {
  const count = Number(frame.count) || 0
  if (!count) throw new Error('点云为空')
  const raw = Number(frame.compression) === 1 ? await inflate(frame.xyz_cm) : frame.xyz_cm
  if (raw.byteLength < count * 6) throw new Error(`点云数据不足：${raw.byteLength} < ${count * 6}`)

  const view = new DataView(raw.buffer, raw.byteOffset, raw.byteLength)
  const positions = new Float32Array(count * 3)
  let minX = Infinity
  let minY = Infinity
  let minZ = Infinity
  let maxX = -Infinity
  let maxY = -Infinity
  let maxZ = -Infinity
  for (let i = 0; i < count; i++) {
    const x = view.getInt16(i * 6, true) / 100
    const y = view.getInt16(i * 6 + 2, true) / 100
    const z = view.getInt16(i * 6 + 4, true) / 100
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
    frame: frame.header?.frame_id || 'map',
    bbox: count ? { min: [minX, minY, minZ], max: [maxX, maxY, maxZ] } : null,
    stamp: Number(frame.header?.stamp) || 0,
  }
}
