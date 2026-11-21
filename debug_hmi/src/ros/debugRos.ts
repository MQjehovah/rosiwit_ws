/**
 * 兼容层：把旧的 rosbridge 客户端 API 映射到 debug_bridge 的 WebSocket + protobuf。
 *
 * app.ts / SceneView / Inspector 仍然按“话题”工作；这里把话题映射成桥接的 stream，
 * 并把 protobuf 消息还原成原 ROS 消息的 JSON 形状，方便复用现有解码器和面板。
 */

import { BridgeClient, normalizeBridgeUrl, type TopicInfoData } from './bridge'

export type RosStatus = 'idle' | 'connecting' | 'open' | 'closed' | 'error'

export interface RosLogEvent {
  level: 'info' | 'warn' | 'error'
  text: string
  time: number
}

export interface SubscribeOptions {
  throttle?: number
  queue?: number
  type?: string
  compression?: string
}

type TopicHandler = (msg: any) => void

interface TopicStat {
  count: number
  lastSize: number
  lastTime: number
}

interface StreamMap {
  stream: string
  opts: SubscribeOptions
}

const TOPIC_STREAM: Record<string, StreamMap> = {
  '/tf': { stream: 'tf', opts: { throttle: 50, queue: 1 } },
  '/tf_static': { stream: 'tf', opts: { throttle: 50, queue: 1 } },
  '/slam/info': { stream: 'status', opts: { throttle: 100, queue: 1 } },
  '/slam/fusion_diag': { stream: 'diag', opts: { throttle: 1000, queue: 1 } },
  '/rosout_agg': { stream: 'logs', opts: { throttle: 200, queue: 50 } },
  '/scan': { stream: 'laser', opts: { throttle: 100, queue: 1 } },
  '/global_path': { stream: 'paths', opts: { throttle: 500, queue: 1 } },
  '/local_path': { stream: 'paths', opts: { throttle: 500, queue: 1 } },
  '/robot_footprint': { stream: 'footprint', opts: { throttle: 1000, queue: 1 } },
  '/scan_f_map': { stream: 'scan', opts: { throttle: 250, queue: 1 } },
  '/curr_scan_point': { stream: 'scan', opts: { throttle: 250, queue: 1 } },
  '/points_2d': { stream: 'points_2d', opts: { throttle: 500, queue: 1 } },
}

const REQUEST_ONLY = new Set(['/global_map', '/vanjee_points722', '/map', '/initialpose'])

const STREAM_DEFAULT_RATE: Record<string, number> = {
  tf: 20,
  status: 10,
  diag: 1,
  logs: 5,
  laser: 10,
  paths: 2,
  footprint: 1,
  scan: 4,
  points_2d: 2,
}

function rateOf(opts: SubscribeOptions): number {
  if (opts.throttle && opts.throttle > 0) return Math.min(20, Math.max(0.5, 1000 / opts.throttle))
  return 5
}

function stampObj(sec: number): { secs: number; nsecs: number } {
  const secs = Math.floor(sec || 0)
  return { secs, nsecs: Math.round(((sec || 0) - secs) * 1e9) }
}

function yawQuat(yaw: number): { x: number; y: number; z: number; w: number } {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
}

export class DebugRosClient {
  readonly bridge: BridgeClient
  url: string
  status: RosStatus = 'idle'
  lastError = ''
  readonly stats = new Map<string, TopicStat>()

  private topicHandlers = new Map<string, Set<TopicHandler>>()
  private statusHandlers = new Set<(status: RosStatus, detail?: string) => void>()
  private logHandlers = new Set<(event: RosLogEvent) => void>()
  private topicRefs = new Map<string, number>()
  private streamRefs = new Map<string, number>()
  private rawTopics = new Set<string>()

  constructor(url: string) {
    this.url = normalizeBridgeUrl(url)
    this.bridge = new BridgeClient(this.url)
    this.bind()
  }

  private bind(): void {
    this.bridge.onConnection((connected) => {
      this.setStatus(connected ? 'open' : 'closed')
    })
    this.bridge.onHello((hello) => {
      this.setStatus('open')
      this.emitLog('info', `debug_bridge v${hello.version} 已连接（${hello.streams.length} 个流）`)
    })
    this.bridge.onError((message) => {
      this.lastError = message
      this.emitLog('error', `debug_bridge: ${message}`)
    })
    this.bridge.onTf((update) => {
      const msg = {
        transforms: update.transforms.map((t) => ({
          header: { frame_id: t.parent, stamp: stampObj(t.stamp) },
          child_frame_id: t.child,
          transform: { translation: t.translation, rotation: t.rotation },
        })),
      }
      this.dispatch(update.is_static ? '/tf_static' : '/tf', msg)
    })
    this.bridge.onSlam((s) => {
      this.dispatch('/slam/info', {
        slam_state: s.slam_state,
        location_status: s.location_status,
        matching_degree: s.matching_degree,
        update_request: s.update_request,
        map_name: s.map_name,
        map_area: s.map_area,
        robot_pose: s.robot_pose,
        header: { stamp: stampObj(s.stamp) },
      })
    })
    this.bridge.onLog((l) => {
      this.dispatch('/rosout_agg', {
        level: l.level,
        name: l.name,
        msg: l.msg,
        header: { stamp: stampObj(l.stamp) },
      })
    })
    this.bridge.onDiag((d) => {
      this.dispatch('/slam/fusion_diag', { data: d.text, header: { stamp: stampObj(d.stamp) } })
    })
    this.bridge.onPath((p) => {
      const topic = '/' + (p.stream || 'global_path')
      this.dispatch(topic, {
        header: { frame_id: p.frame, stamp: stampObj(p.stamp) },
        poses: p.poses.map((q) => ({
          pose: {
            position: { x: q.x, y: q.y, z: q.z },
            orientation: yawQuat(q.yaw),
          },
        })),
      })
    })
    this.bridge.onLaser((l) => {
      this.dispatch('/scan', {
        header: { frame_id: l.frame, stamp: stampObj(l.stamp) },
        angle_min: l.angle_min,
        angle_max: l.angle_max,
        angle_increment: l.angle_increment,
        range_min: l.range_min,
        range_max: l.range_max,
        ranges: Array.from(l.ranges),
      })
    })
    this.bridge.onPolygon((p) => {
      const points: Array<{ x: number; y: number; z: number }> = []
      for (let i = 0; i + 1 < p.xy.length; i += 2) points.push({ x: p.xy[i], y: p.xy[i + 1], z: 0 })
      this.dispatch('/robot_footprint', {
        header: { frame_id: p.frame, stamp: stampObj(p.stamp) },
        polygon: { points },
      })
    })
    this.bridge.onStat((stat) => {
      this.stats.set(stat.topic, {
        count: stat.count,
        lastSize: stat.bytes,
        lastTime: Date.now() - stat.age_ms,
      })
    })
    this.bridge.onRaw((raw) => {
      try {
        this.dispatch(raw.topic, JSON.parse(raw.json))
      } catch {
        /* ignore */
      }
    })
  }

  // ---------------------------------------------------------------- lifecycle

  connect(url?: string): void {
    if (url) {
      this.url = normalizeBridgeUrl(url)
      this.bridge.setUrl(this.url)
    }
    this.setStatus('connecting')
    this.bridge.autoReconnect = true
    this.bridge.connect().catch((e) => {
      this.lastError = (e as Error).message
      this.setStatus('error', this.lastError)
    })
  }

  disconnect(): void {
    this.bridge.disconnect()
    this.setStatus('closed')
  }

  private setStatus(status: RosStatus, detail?: string): void {
    if (this.status === status && !detail) return
    this.status = status
    if (detail) this.lastError = detail
    for (const fn of this.statusHandlers) fn(status, detail)
  }

  onStatus(fn: (status: RosStatus, detail?: string) => void): () => void {
    this.statusHandlers.add(fn)
    return () => this.statusHandlers.delete(fn)
  }

  onLog(fn: (event: RosLogEvent) => void): () => void {
    this.logHandlers.add(fn)
    return () => this.logHandlers.delete(fn)
  }

  private emitLog(level: RosLogEvent['level'], text: string): void {
    const event: RosLogEvent = { level, text, time: Date.now() }
    for (const fn of this.logHandlers) fn(event)
  }

  // -------------------------------------------------------------- subscriptions

  subscribe(topic: string, opts: SubscribeOptions = {}): void {
    const existing = this.topicRefs.get(topic) ?? 0
    this.topicRefs.set(topic, existing + 1)
    if (existing > 0) return

    const mapped = TOPIC_STREAM[topic]
    if (mapped) {
      const count = this.streamRefs.get(mapped.stream) ?? 0
      this.streamRefs.set(mapped.stream, count + 1)
      if (count === 0) {
        this.bridge.setSubscription(mapped.stream, {
          rate: rateOf(opts) || STREAM_DEFAULT_RATE[mapped.stream] || 5,
          voxel: mapped.stream === 'scan' ? 0.05 : mapped.stream === 'points_2d' ? 0.08 : 0,
          max: 200000,
        })
      }
      return
    }
    if (REQUEST_ONLY.has(topic)) return
    if (!this.rawTopics.has(topic)) {
      this.rawTopics.add(topic)
      this.bridge.rawSubscribe(topic, Math.min(5, rateOf(opts) || 2))
    }
  }

  unsubscribe(topic: string): void {
    const count = this.topicRefs.get(topic) ?? 0
    if (count > 1) {
      this.topicRefs.set(topic, count - 1)
      return
    }
    this.topicRefs.delete(topic)
    const mapped = TOPIC_STREAM[topic]
    if (mapped) {
      const refs = (this.streamRefs.get(mapped.stream) ?? 0) - 1
      if (refs <= 0) {
        this.streamRefs.delete(mapped.stream)
        this.bridge.setSubscription(mapped.stream, null)
      } else {
        this.streamRefs.set(mapped.stream, refs)
      }
      return
    }
    if (this.rawTopics.delete(topic)) this.bridge.rawUnsubscribe(topic)
  }

  isSubscribed(topic: string): boolean {
    return (this.topicRefs.get(topic) ?? 0) > 0 || this.rawTopics.has(topic)
  }

  resubscribe(topic: string): void {
    // /global_map 的刷新由 app 直接走 bridge.requestCloud，这里不做事。
    void topic
  }

  onTopic(topic: string, handler: TopicHandler): () => void {
    let set = this.topicHandlers.get(topic)
    if (!set) {
      set = new Set()
      this.topicHandlers.set(topic, set)
    }
    set.add(handler)
    return () => {
      set?.delete(handler)
      if (set && set.size === 0) this.topicHandlers.delete(topic)
    }
  }

  private dispatch(topic: string, msg: any): void {
    const set = this.topicHandlers.get(topic)
    if (!set) return
    for (const fn of set) {
      try {
        fn(msg)
      } catch {
        /* ignore */
      }
    }
  }

  // ---------------------------------------------------------------- commands

  publish(topic: string, msg: any): boolean {
    if (topic !== '/initialpose') {
      this.emitLog('warn', `debug_bridge 不支持发布 ${topic}`)
      return false
    }
    const pose = msg?.pose?.pose
    const x = Number(pose?.position?.x) || 0
    const y = Number(pose?.position?.y) || 0
    const q = pose?.orientation
    const yaw = q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : 0
    return this.bridge.setInitialPose(x, y, yaw)
  }

  callService(name: string, args: Record<string, unknown> = {}, timeout = 15000): Promise<any> {
    return this.bridge.callService(name, args, timeout)
  }

  getTopics(): Promise<TopicInfoData[]> {
    return this.bridge.getTopics()
  }

  rawSubscribe(topic: string, rate = 2): void {
    this.rawTopics.add(topic)
    this.bridge.rawSubscribe(topic, rate)
  }
}
