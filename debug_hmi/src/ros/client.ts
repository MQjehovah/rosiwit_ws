/**
 * 极简 rosbridge (ROS1) 客户端。
 *
 * 只实现 HMI 需要的部分：话题订阅/发布、服务调用、连接状态、自动重连。
 * 不依赖 roslib，方便之后替换成 ROS2 的 rosbridge / zenoh 通道。
 *
 * 大消息（点云、栅格）走 `compression: "cbor"`：rosbridge 下发二进制 CBOR 帧，
 * 比 JSON 少 2~4 倍体积，也避免了 4300 万个数组成的 JSON 文本。
 */

import { decodeCbor } from './cbor'

export type RosStatus = 'idle' | 'connecting' | 'open' | 'closed' | 'error'

export interface RosLogEvent {
  level: 'info' | 'warn' | 'error'
  text: string
  time: number
}

export interface SubscribeOptions {
  /** 两条消息之间的最小间隔（ms），0 = 不限 */
  throttle?: number
  /** 队列长度，0 = 不限；实时点云建议 1 */
  queue?: number
  /** 消息类型，通常可省略（rosbridge 会自己查） */
  type?: string
  /** rosbridge 传输编码；大点云/栅格用 'cbor'（二进制） */
  compression?: 'none' | 'cbor'
}

export interface TopicStat {
  count: number
  lastSize: number
  lastTime: number
}

interface Waiter {
  resolve: (value: unknown) => void
  reject: (err: Error) => void
  timer: number
}

type LogHandler = (event: RosLogEvent) => void
type StatusHandler = (status: RosStatus, detail?: string) => void
type TopicHandler = (msg: any) => void

export class RosBridgeClient {
  url: string
  status: RosStatus = 'idle'
  lastError = ''
  autoReconnect = true

  /** 每个话题的收发统计，供 UI 显示 Hz / 体积 */
  readonly stats = new Map<string, TopicStat>()

  private ws: WebSocket | null = null
  private subs = new Map<string, SubscribeOptions>()
  private handlers = new Map<string, Set<TopicHandler>>()
  private statusHandlers = new Set<StatusHandler>()
  private logHandlers = new Set<LogHandler>()
  private waiters = new Map<string, Waiter>()
  private advertised = new Set<string>()
  private seq = 0
  private reconnectTimer: number | null = null
  private failures = 0
  private manualClose = false
  /** rosbridge 在客户端未主动断开时收到 close 的错误文本 */
  private closeDetail = ''

  constructor(url: string) {
    this.url = url
  }

  // ---------------------------------------------------------------- lifecycle

  connect(url?: string): void {
    if (url) this.url = url
    this.manualClose = false
    this.closeDetail = ''
    if (this.ws && (this.status === 'open' || this.status === 'connecting')) return
    this.open()
  }

  disconnect(): void {
    this.manualClose = true
    this.clearReconnect()
    for (const [, waiter] of this.waiters) {
      clearTimeout(waiter.timer)
      waiter.reject(new Error('连接已断开'))
    }
    this.waiters.clear()
    this.advertised.clear()
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
    this.setStatus('closed')
  }

  private open(): void {
    this.clearReconnect()
    this.setStatus('connecting')
    let ws: WebSocket
    try {
      ws = new WebSocket(this.url)
    } catch (e) {
      this.fail(`无法创建 WebSocket：${(e as Error).message}`)
      return
    }
    this.ws = ws
    ws.binaryType = 'arraybuffer'

    ws.onopen = () => {
      this.failures = 0
      this.lastError = ''
      this.setStatus('open')
      this.log('info', `已连接 ${this.url}`)
      // 断线重连后把订阅恢复
      for (const [topic, opts] of this.subs) this.sendSubscribe(topic, opts)
    }

    ws.onmessage = (ev: MessageEvent) => {
      if (typeof ev.data === 'string') {
        this.handleRaw(ev.data)
      } else if (ev.data instanceof ArrayBuffer) {
        this.handleBinary(new Uint8Array(ev.data))
      }
    }

    ws.onerror = () => {
      // 具体原因在 onclose 里补；这里只记录一次
      this.lastError = 'WebSocket 连接错误：检查地址/端口、防火墙，以及是否被 https 页面拦截 ws://'
    }

    ws.onclose = (ev: CloseEvent) => {
      this.ws = null
      this.advertised.clear()
      const wasOpen = this.status === 'open'
      this.setStatus(this.manualClose ? 'closed' : 'error', this.closeDetail)
      if (!this.manualClose) {
        const detail = `${ev.code ? `code=${ev.code} ` : ''}${ev.reason || this.lastError || '连接被关闭'}`
        this.log('error', `连接断开：${detail}`)
      } else if (wasOpen) {
        this.log('info', '已断开')
      }
      for (const [, waiter] of this.waiters) {
        clearTimeout(waiter.timer)
        waiter.reject(new Error('连接已断开'))
      }
      this.waiters.clear()
      if (this.autoReconnect && !this.manualClose) this.scheduleReconnect()
    }
  }

  private scheduleReconnect(): void {
    this.clearReconnect()
    this.failures++
    const delay = Math.min(15000, 800 * Math.pow(1.6, this.failures - 1))
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null
      if (!this.manualClose && this.status !== 'open') this.open()
    }, delay)
  }

  private clearReconnect(): void {
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
  }

  private fail(text: string): void {
    this.lastError = text
    this.log('error', text)
    this.setStatus('error', text)
    if (this.autoReconnect && !this.manualClose) this.scheduleReconnect()
  }

  private setStatus(status: RosStatus, detail?: string): void {
    this.status = status
    if (detail) this.lastError = detail
    for (const fn of this.statusHandlers) fn(status, detail)
  }

  // ------------------------------------------------------------------ sending

  private send(payload: unknown): boolean {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false
    try {
      this.ws.send(JSON.stringify(payload))
      return true
    } catch (e) {
      this.log('error', `发送失败：${(e as Error).message}`)
      return false
    }
  }

  // -------------------------------------------------------------- subscriptions

  subscribe(topic: string, opts: SubscribeOptions = {}): void {
    this.subs.set(topic, opts)
    if (this.status === 'open') this.sendSubscribe(topic, opts)
  }

  unsubscribe(topic: string): void {
    this.subs.delete(topic)
    this.handlers.delete(topic)
    this.send({ op: 'unsubscribe', topic })
  }

  isSubscribed(topic: string): boolean {
    return this.subs.has(topic)
  }

  /** 不丢回调地重新订阅（用于 latched 话题强制重发） */
  resubscribe(topic: string): void {
    const opts = this.subs.get(topic)
    if (!opts) return
    this.send({ op: 'unsubscribe', topic })
    this.sendSubscribe(topic, opts)
  }

  private sendSubscribe(topic: string, opts: SubscribeOptions): void {
    const msg: Record<string, unknown> = {
      op: 'subscribe',
      id: topic,
      topic,
      throttle_rate: opts.throttle ?? 0,
      queue_length: opts.queue ?? 0,
    }
    if (opts.type) msg.type = opts.type
    if (opts.compression && opts.compression !== 'none') msg.compression = opts.compression
    this.send(msg)
  }

  /** 订阅并注册回调；返回取消该回调的函数（不会退订话题） */
  onTopic(topic: string, handler: TopicHandler): () => void {
    let set = this.handlers.get(topic)
    if (!set) {
      set = new Set()
      this.handlers.set(topic, set)
    }
    set.add(handler)
    return () => {
      set?.delete(handler)
      if (set && set.size === 0) this.handlers.delete(topic)
    }
  }

  // ------------------------------------------------------------------ publish

  /** 发布前会自动 advertise（rosbridge 要求先声明类型） */
  publish(topic: string, msg: unknown, type?: string): boolean {
    if (type && !this.advertised.has(topic)) {
      this.advertised.add(topic)
      this.send({ op: 'advertise', topic, type })
    }
    return this.send({ op: 'publish', topic, msg })
  }

  // ------------------------------------------------------------------ service

  callService(name: string, args: Record<string, unknown> = {}, timeout = 10000): Promise<any> {
    return this.callServiceInternal(`svc-${++this.seq}`, name, args, timeout)
  }

  private callServiceInternal(
    id: string,
    name: string,
    args: Record<string, unknown>,
    timeout: number,
  ): Promise<any> {
    if (this.status !== 'open' || !this.ws) {
      return Promise.reject(new Error('尚未连接 rosbridge'))
    }
    return new Promise((resolve, reject) => {
      const timer = window.setTimeout(() => {
        this.waiters.delete(id)
        reject(new Error(`服务 ${name} 超时（${timeout}ms）`))
      }, timeout)
      this.waiters.set(id, { resolve, reject, timer })
      const ok = this.send({ op: 'call_service', id, service: name, args })
      if (!ok) {
        clearTimeout(timer)
        this.waiters.delete(id)
        reject(new Error('发送失败'))
      }
    })
  }

  // ------------------------------------------------------------------- events

  onStatus(handler: StatusHandler): () => void {
    this.statusHandlers.add(handler)
    return () => this.statusHandlers.delete(handler)
  }

  onLog(handler: LogHandler): () => void {
    this.logHandlers.add(handler)
    return () => this.logHandlers.delete(handler)
  }

  log(level: RosLogEvent['level'], text: string): void {
    const event: RosLogEvent = { level, text, time: Date.now() }
    for (const fn of this.logHandlers) fn(event)
  }

  // ------------------------------------------------------------------- recv

  private handleRaw(raw: string): void {
    let payload: any
    try {
      payload = JSON.parse(raw)
    } catch {
      return
    }
    if (Array.isArray(payload)) {
      for (const item of payload) this.dispatch(item, raw.length)
      return
    }
    this.dispatch(payload, raw.length)
  }

  private handleBinary(bytes: Uint8Array): void {
    let payload: any
    try {
      payload = decodeCbor(bytes)
    } catch (e) {
      this.log('error', `CBOR 解码失败：${(e as Error).message}`)
      return
    }
    this.dispatch(payload, bytes.byteLength)
  }

  private dispatch(payload: any, rawSize: number): void {
    const op = payload?.op
    if (op === 'publish') {
      const topic = payload.topic as string
      if (topic === '/rosout_agg' || topic === '/rosout') {
        // 日志本身由 app 层转成控制台，不再刷屏
      }
      const stat = this.stats.get(topic)
      if (stat) {
        stat.count++
        stat.lastSize = rawSize
        stat.lastTime = Date.now()
      } else {
        this.stats.set(topic, { count: 1, lastSize: rawSize, lastTime: Date.now() })
      }
      const set = this.handlers.get(topic)
      if (set) {
        for (const fn of set) {
          try {
            fn(payload.msg)
          } catch (e) {
            this.log('error', `${topic} 回调异常：${(e as Error).message}`)
          }
        }
      }
      return
    }

    if (op === 'service_response') {
      const id = String(payload.id ?? '')
      const waiter = this.waiters.get(id)
      if (!waiter) return
      clearTimeout(waiter.timer)
      this.waiters.delete(id)
      if (payload.result === false) {
        const detail =
          typeof payload.values === 'string' ? payload.values : JSON.stringify(payload.values)
        waiter.reject(new Error(`${payload.service || '服务'} 调用失败：${detail}`))
      } else {
        waiter.resolve(payload.values)
      }
      return
    }

    if (op === 'png') {
      // rosbridge 的 op:png 不携带 topic/id，按 topic 分发无法可靠还原，这里直接忽略。
      this.log('warn', '收到 PNG 压缩消息（当前 HMI 不支持），请把 compression 设为 none')
      return
    }

    if (op === 'fragment') {
      // 需要 fragment_size 才会出现；HMI 不设置 fragment_size。
      return
    }

    if (op === 'status') {
      const level = Number(payload.level)
      const text = String(payload.msg ?? '')
      this.closeDetail = text
      this.log(level >= 2 ? 'error' : level === 1 ? 'warn' : 'info', `rosbridge: ${text}`)
    }
  }
}
