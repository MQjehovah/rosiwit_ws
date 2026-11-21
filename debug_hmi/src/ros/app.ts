/**
 * HMI 全局状态与数据流。
 *
 * 唯一数据通道是 debug_bridge（WebSocket + protobuf，默认 9100）：
 * TF、定位状态、日志、路径、LaserScan、足迹、点云、栅格、服务都在这里。
 * rosbridge 已不再使用。
 */

import { reactive } from 'vue'
import {
  DebugRosClient,
  type RosLogEvent,
  type RosStatus,
  type SubscribeOptions,
} from './debugRos'
import type { CloudData, GridData } from './decode'
import { limitCloud, parseMapFile } from './mapFile'
import { loadMapCache, saveMapCache } from './mapCache'
import { TfTree } from './tf'

const LAN_URL = 'ws://192.168.1.100:9100'
/** 远程隧道地址（来自 .env.local，通常带 token，不写进源码） */
export const TUNNEL_URL = import.meta.env.VITE_TUNNEL_URL || import.meta.env.VITE_BRIDGE_URL || ''

export interface UrlPreset {
  label: string
  url: string
}

export const urlPresets: UrlPreset[] = [
  { label: '局域网 9100', url: LAN_URL },
  ...(TUNNEL_URL ? [{ label: '远程隧道', url: TUNNEL_URL }] : []),
]

export const ros = new DebugRosClient(
  localStorage.getItem('debug_hmi.url') || import.meta.env.VITE_BRIDGE_URL || LAN_URL,
)
export const bridge = ros.bridge
export const tfTree = new TfTree()

export type LayerKey =
  | 'tf'
  | 'status'
  | 'diag'
  | 'logs'
  | 'scan'
  | 'laser'
  | 'globalMap'
  | 'points2d'
  | 'robot'
  | 'paths'
  | 'lidar'
  | 'grid'
  | 'trail'

export interface SlamInfo {
  slam_state: number
  location_status: number
  matching_degree: number
  update_request: number
  map_name: string
  map_area: number
  robot_pose: { x: number; y: number; theta: number }
}

export interface UiTopicStat {
  hz: number
  count: number
  size: number
  age: number
}

export interface LogLine {
  id: number
  time: number
  level: 'info' | 'warn' | 'error'
  text: string
  source?: string
  /** 连续重复消息合并后的条数 */
  count: number
}

export interface Point2 {
  x: number
  y: number
}

export interface MeasureState {
  active: boolean
  a: Point2 | null
  b: Point2 | null
}

export interface PendingPose {
  x: number
  y: number
  theta: number
}

const initialSlam: SlamInfo = {
  slam_state: -1,
  location_status: -1,
  matching_degree: 0,
  update_request: 0,
  map_name: '',
  map_area: 0,
  robot_pose: { x: 0, y: 0, theta: 0 },
}

export const state = reactive({
  url: ros.url,
  status: 'idle' as RosStatus,
  /** 最新一条核心消息距今多久（ms），-1 = 还没收到 */
  dataAge: -1,
  error: '',
  connectedAt: 0,

  slam: { ...initialSlam } as SlamInfo,
  slamTime: 0,
  fusionDiag: '',

  robot: {
    x: 0,
    y: 0,
    z: 0,
    roll: 0,
    pitch: 0,
    yaw: 0,
    source: '-' as string,
    stamp: 0,
  },

  layers: {
    // 只有 TF 默认订阅；其余像 RViz 一样由用户显式勾选
    tf: true,
    status: false,
    diag: false,
    logs: false,
    scan: false,
    laser: false,
    globalMap: false,
    points2d: false,
    robot: false,
    paths: false,
    lidar: false,
    grid: false,
    trail: true,
  } as Record<LayerKey, boolean>,

  stats: {} as Record<string, UiTopicStat>,
  topicNames: [] as string[],
  topicTypes: {} as Record<string, string>,
  topicScanning: false,
  frames: [] as string[],

  logs: [] as LogLine[],
  logFilter: 'all' as 'all' | 'info' | 'warn' | 'error',
  pausedLogs: false,
  /** 被静音的 rosout 节点名（点击日志来源切换） */
  mutedSources: [] as string[],

  estimating: false,
  pendingPose: null as PendingPose | null,

  measure: { active: false, a: null, b: null } as MeasureState,
  measureDistance: 0,

  /** /slam/info 的 matching_degree 采样（约 2.5Hz，最多 120 点） */
  matchHistory: [] as number[],

  servicePending: false,
  lastService: '',

  rawTopic: '',
  rawText: '',
  rawTime: 0,
  rawPaused: false,

  snapshotPending: false,
  snapshotTime: 0,
  snapshotPoints: 0,

  gridPending: false,
  gridStartedAt: 0,
  gridCells: 0,
  gridTime: 0,
  gridBytes: 0,
  gridFromCache: false,

  /** 本地加载的 PCD 地图 */
  loadedMapName: '',
  loadedMapPoints: 0,
  loadedMapTime: 0,
  mapLoading: false,
  mapError: '',

  /** 点云显示尺寸倍率（0.5 ~ 3） */
  pointScale: Number(localStorage.getItem('hmi.pointScale')) || 1,

  view: { fit: 0, top: 0, shot: 0, follow: false },

  fps: 0,
  pointCounts: {} as Record<string, number>,
})

// ------------------------------------------------------------------ logging

let logId = 1
export function pushLog(level: LogLine['level'], text: string, source?: string): void {
  // 连续相同的消息合并计数：rosout 里 “Waiting for laser_scans...” 这种会刷屏
  const last = state.logs[0]
  if (
    last &&
    last.level === level &&
    last.text === text &&
    last.source === source &&
    Date.now() - last.time < 5000
  ) {
    last.count++
    last.time = Date.now()
    return
  }
  state.logs.unshift({ id: logId++, time: Date.now(), level, text, source, count: 1 })
  if (state.logs.length > 400) state.logs.length = 400
}

export function muteSource(source: string): void {
  if (!state.mutedSources.includes(source)) {
    state.mutedSources.push(source)
    saveMutedSources()
  }
}

export function unmuteSource(source: string): void {
  state.mutedSources = state.mutedSources.filter((s) => s !== source)
  saveMutedSources()
}

export function clearMutedSources(): void {
  state.mutedSources = []
  saveMutedSources()
}

function saveMutedSources(): void {
  try {
    localStorage.setItem('hmi.mutedSources', JSON.stringify(state.mutedSources))
  } catch {
    /* ignore */
  }
}

function loadMutedSources(): string[] {
  try {
    const raw = localStorage.getItem('hmi.mutedSources')
    const parsed = raw ? JSON.parse(raw) : []
    return Array.isArray(parsed) ? parsed.filter((s) => typeof s === 'string') : []
  } catch {
    return []
  }
}

const ROSOUT_LEVEL: Record<number, LogLine['level']> = {
  1: 'info',
  2: 'info',
  4: 'warn',
  8: 'error',
  16: 'error',
}

// -------------------------------------------------------------- subscriptions

const LAYER_SUBS: Record<LayerKey, Array<[string, SubscribeOptions]>> = {
  // TF 常驻订阅（只默认订阅它）；layers.tf 只控制坐标轴显示
  tf: [
    ['/tf', { throttle: 100, queue: 100 }],
    ['/tf_static', { queue: 1 }],
  ],
  status: [['/slam/info', { throttle: 150, queue: 1 }]],
  diag: [['/slam/fusion_diag', { throttle: 1000, queue: 1 }]],
  logs: [['/rosout_agg', { throttle: 500, queue: 5 }]],
  scan: [
    // /scan_f_map 是旧版 PointCloud，未开 debug_publish 时也有数据，已在 map 系配准
    ['/scan_f_map', { throttle: 200, queue: 1, compression: 'cbor' }],
    ['/curr_scan_point', { throttle: 120, queue: 1, compression: 'cbor' }],
  ],
  laser: [['/scan', { throttle: 200, queue: 1 }]],
  // /global_map 是 latched（每次进模式只发一次），用 CBOR 拿原始字节
  globalMap: [['/global_map', { throttle: 0, queue: 1, compression: 'cbor' }]],
  points2d: [['/points_2d', { throttle: 500, queue: 1, compression: 'cbor' }]],
  robot: [['/robot_footprint', { throttle: 1000, queue: 1 }]],
  paths: [
    ['/global_path', { throttle: 1000, queue: 1 }],
    ['/local_path', { throttle: 500, queue: 1 }],
  ],
  lidar: [],
  grid: [],
  trail: [],
}

const BRIDGE_STREAM_TOPICS = new Set(['/scan_f_map', '/curr_scan_point', '/global_map', '/points_2d'])

function desiredSubscriptions(): Map<string, SubscribeOptions> {
  const wanted = new Map<string, SubscribeOptions>()
  for (const [topic, opts] of LAYER_SUBS.tf) wanted.set(topic, opts)
  for (const key of Object.keys(state.layers) as LayerKey[]) {
    if (key === 'tf' || !state.layers[key]) continue
    for (const [topic, opts] of LAYER_SUBS[key]) {
      wanted.set(topic, opts)
    }
  }
  return wanted
}

export function syncSubscriptions(): void {
  const wanted = desiredSubscriptions()
  for (const [topic, opts] of wanted) {
    if (!ros.isSubscribed(topic)) ros.subscribe(topic, opts)
  }
  const managed = new Set<string>()
  for (const list of Object.values(LAYER_SUBS)) {
    for (const [topic] of list) managed.add(topic)
  }
  for (const topic of managed) {
    if (!wanted.has(topic) && ros.isSubscribed(topic)) ros.unsubscribe(topic)
  }
}

const LAYERS_KEY = 'hmi.layers.v2'

function loadLayers(): Partial<Record<LayerKey, boolean>> {
  try {
    const raw = localStorage.getItem(LAYERS_KEY)
    if (!raw) return {}
    const parsed = JSON.parse(raw)
    if (!parsed || typeof parsed !== 'object') return {}
    const out: Partial<Record<LayerKey, boolean>> = {}
    for (const key of Object.keys(state.layers) as LayerKey[]) {
      if (typeof parsed[key] === 'boolean') out[key] = parsed[key]
    }
    return out
  } catch {
    return {}
  }
}

function saveLayers(): void {
  try {
    localStorage.setItem(LAYERS_KEY, JSON.stringify(state.layers))
  } catch {
    /* ignore */
  }
}

export function setLayer(key: LayerKey, value: boolean): void {
  state.layers[key] = value
  saveLayers()
  syncSubscriptions()
}

// ----------------------------------------------------------- one-shot snapshot

type CloudHandler = (cloud: CloudData) => void
type GridHandler = (grid: GridData) => void

const lidarHandlers = new Set<CloudHandler>()
const globalMapHandlers = new Set<CloudHandler>()
const gridHandlers = new Set<GridHandler>()

export function onLidarSnapshot(handler: CloudHandler): () => void {
  lidarHandlers.add(handler)
  return () => lidarHandlers.delete(handler)
}

export function onGlobalMapSnapshot(handler: CloudHandler): () => void {
  globalMapHandlers.add(handler)
  return () => globalMapHandlers.delete(handler)
}

export function onGridSnapshot(handler: GridHandler): () => void {
  gridHandlers.add(handler)
  return () => gridHandlers.delete(handler)
}

function emitLidar(cloud: CloudData): void {
  state.snapshotTime = Date.now()
  state.snapshotPoints = cloud.count
  for (const fn of lidarHandlers) fn(cloud)
  pushLog('info', `雷达帧：${cloud.count.toLocaleString()} 点（${cloud.frame || '未知坐标系'}）`)
}

export async function grabLidarSnapshot(): Promise<void> {
  if (state.snapshotPending) return
  state.snapshotPending = true
  try {
    if (ros.status !== 'open') {
      pushLog('warn', 'debug_bridge 未连接')
      return
    }
    pushLog('info', '正在通过 debug_bridge 抓取一帧 /vanjee_points722…')
    emitLidar(await bridge.requestCloud('vanjee_points722', { voxel: 0.05, max: 200000, timeout: 30000 }))
  } catch (e) {
    pushLog('warn', `雷达快照失败：${(e as Error).message}`)
  } finally {
    state.snapshotPending = false
  }
}

// ------------------------------------------------------------------ grid snapshot

export function cancelGridSnapshot(): void {
  bridge.cancelGrid()
}

export async function grabGridSnapshot(): Promise<void> {
  if (state.gridPending) return
  state.gridPending = true
  state.gridStartedAt = Date.now()
  try {
    if (ros.status !== 'open') {
      pushLog('warn', 'debug_bridge 未连接')
      return
    }
    pushLog('info', '正在通过 debug_bridge 获取栅格地图（服务端 zlib）…')
    const grid = await bridge.requestGrid(true)
    if (!grid) throw new Error('没有有效栅格数据')
    state.gridCells = grid.width * grid.height
    state.gridBytes = grid.data.byteLength
    state.gridTime = Date.now()
    state.gridFromCache = false
    pushLog(
      'info',
      `栅格地图：${grid.width}×${grid.height}（${(state.gridCells / 1e6).toFixed(1)}M 格）`,
    )
    for (const fn of gridHandlers) fn(grid)
  } catch (e) {
    pushLog('warn', `栅格地图抓取失败：${(e as Error).message}`)
  } finally {
    state.gridPending = false
  }
}

// ------------------------------------------------------------------ global map

function emitLoadedMap(cloud: CloudData): void {
  state.pointCounts.globalMap = cloud.count
  if (!state.layers.globalMap) setLayer('globalMap', true)
  for (const fn of globalMapHandlers) fn(cloud)
}

/** 浏览器本地加载标准 PCD（不经过机器人） */
export async function loadMapFile(file: File): Promise<void> {
  if (state.mapLoading) return
  state.mapLoading = true
  state.mapError = ''
  try {
    // 先让 UI 渲染“解析中”
    await new Promise<void>((resolve) => requestAnimationFrame(() => resolve()))
    const parsed = await parseMapFile(file)
    const cloud = limitCloud(parsed, 1200000)
    const data: CloudData = {
      positions: cloud.positions,
      colors: null,
      count: cloud.count,
      total: cloud.count,
      frame: cloud.frame || 'map',
      bbox: null,
    }
    state.loadedMapName = cloud.name
    state.loadedMapPoints = cloud.count
    state.loadedMapTime = Date.now()
    emitLoadedMap(data)
    void saveMapCache(cloud).catch(() => {
      /* 缓存失败不影响显示 */
    })
    pushLog('info', `已加载地图文件 ${cloud.name}：${cloud.count.toLocaleString()} 点`)
  } catch (e) {
    state.mapError = (e as Error).message
    pushLog('error', `加载地图失败：${state.mapError}`)
  } finally {
    state.mapLoading = false
  }
}

/** 刷新页面后恢复上次加载的 PCD */
export async function restoreMapCache(): Promise<void> {
  try {
    const cached = await loadMapCache()
    if (!cached) return
    const data: CloudData = {
      positions: cached.positions,
      colors: null,
      count: cached.count,
      total: cached.count,
      frame: cached.frame || 'map',
      bbox: null,
    }
    state.loadedMapName = cached.name
    state.loadedMapPoints = cached.count
    state.loadedMapTime = cached.savedAt
    emitLoadedMap(data)
    pushLog('info', `已恢复地图缓存：${cached.name}（${cached.count.toLocaleString()} 点）`)
  } catch {
    /* ignore */
  }
}

/** debug_publish=true 时从 /global_map 拉一份（文件加载的首选替代） */
export function reloadGlobalMap(): void {
  if (ros.status !== 'open') {
    pushLog('warn', 'debug_bridge 未连接')
    return
  }
  pushLog('info', '正在请求 /global_map（需要 debug_publish=true）…')
  void bridge
    .requestCloud('global_map', { voxel: 0.2, max: 500000, timeout: 40000 })
    .then((cloud) => {
      emitLoadedMap(cloud)
      pushLog('info', `全局地图：${cloud.count.toLocaleString()} 点（${cloud.frame || 'map'}）`)
    })
    .catch((e) => pushLog('warn', `全局地图加载失败：${(e as Error).message}`))
}

// ------------------------------------------------------------------ handlers

const TOPIC_CATALOG: Record<string, string> = {
  '/slam/info': 'xzrobot_ros_dependency/SlamInfo',
  '/slam/fusion_diag': 'std_msgs/String',
  '/slam_warning_code': 'std_msgs/String',
  '/curr_localization_pose': 'geometry_msgs/PoseStamped',
  '/curr_scan_point': 'sensor_msgs/PointCloud2',
  '/curr_submap_point': 'sensor_msgs/PointCloud2',
  '/global_map': 'sensor_msgs/PointCloud2',
  '/global_map2': 'nav_msgs/OccupancyGrid',
  '/global_grid_map': 'nav_msgs/OccupancyGrid',
  '/map': 'nav_msgs/OccupancyGrid',
  '/clean_map': 'nav_msgs/OccupancyGrid',
  '/local_map': 'nav_msgs/OccupancyGrid',
  '/loc_map': '',
  '/points_2d': 'sensor_msgs/PointCloud2',
  '/scan': 'sensor_msgs/LaserScan',
  '/scan_f_map': 'sensor_msgs/PointCloud',
  '/scan_f_laser': 'sensor_msgs/PointCloud',
  '/scan_matched_points2': '',
  '/loop_constraint': 'visualization_msgs/MarkerArray',
  '/mapping_path': 'nav_msgs/Path',
  '/origin_point': 'sensor_msgs/PointCloud2',
  '/lio_odom': 'nav_msgs/Odometry',
  '/odom': 'nav_msgs/Odometry',
  '/odometry/filtered': 'nav_msgs/Odometry',
  '/tf': 'tf2_msgs/TFMessage',
  '/tf_static': 'tf2_msgs/TFMessage',
  '/initialpose': 'geometry_msgs/PoseWithCovarianceStamped',
  '/set_pose': 'geometry_msgs/PoseWithCovarianceStamped',
  '/global_path': 'nav_msgs/Path',
  '/local_path': 'nav_msgs/Path',
  '/robot_footprint': 'geometry_msgs/PolygonStamped',
  '/robot_polygon': 'geometry_msgs/PolygonStamped',
  '/robot_pose': 'geometry_msgs/PoseStamped',
  '/particlecloud': '',
  '/laser_odom': 'nav_msgs/Odometry',
  '/trajectories_rviz': 'visualization_msgs/MarkerArray',
  '/re_point': 'geometry_msgs/PointStamped',
  '/location_info': '',
  '/nav_feedback': 'std_msgs/String',
  '/cmd_vel': 'geometry_msgs/Twist',
  '/movebase_cmd_vel': 'geometry_msgs/Twist',
  '/diagnostics': 'diagnostic_msgs/DiagnosticArray',
  '/rosout_agg': 'rosgraph_msgs/Log',
  '/rosout': 'rosgraph_msgs/Log',
  '/robot_monitor': 'xzrobot_ros_dependency/MonitorInfo',
  '/monitor/info': 'std_msgs/String',
  '/driver/chassis/info': 'xzrobot_ros_dependency/ChassisInfo',
  '/driver/bms': 'sensor_msgs/BatteryState',
  '/imu/data': 'sensor_msgs/Imu',
  '/livox/lidar': 'livox_ros_driver2/CustomMsg',
  '/livox/imu': 'sensor_msgs/Imu',
  '/vanjee_points722': 'sensor_msgs/PointCloud2',
  '/device/base64_map': 'std_msgs/String',
  '/mapBase64': 'std_msgs/String',
  '/encoder_raw_odom': 'geometry_msgs/PointStamped',
}

let handlersReady = false

let lastMatchSample = 0

function registerHandlers(): void {
  if (handlersReady) return
  handlersReady = true

  ros.onTopic('/tf', (msg) => tfTree.ingest(msg, false))
  ros.onTopic('/tf_static', (msg) => tfTree.ingest(msg, true))

  ros.onTopic('/slam/info', (msg) => {
    state.slam = {
      slam_state: Number(msg?.slam_state ?? -1),
      location_status: Number(msg?.location_status ?? -1),
      matching_degree: Number(msg?.matching_degree ?? 0),
      update_request: Number(msg?.update_request ?? 0),
      map_name: String(msg?.map_name ?? ''),
      map_area: Number(msg?.map_area ?? 0),
      robot_pose: {
        x: Number(msg?.robot_pose?.x ?? 0),
        y: Number(msg?.robot_pose?.y ?? 0),
        theta: Number(msg?.robot_pose?.theta ?? 0),
      },
    }
    state.slamTime = Date.now()

    // 匹配度趋势：约 2.5Hz 采样，保留最近 120 点（~48s）
    const now = performance.now()
    if (now - lastMatchSample > 400) {
      lastMatchSample = now
      state.matchHistory.push(state.slam.matching_degree)
      if (state.matchHistory.length > 120) state.matchHistory.shift()
    }
  })

  ros.onTopic('/slam/fusion_diag', (msg) => {
    state.fusionDiag = String(msg?.data ?? '')
  })

  ros.onTopic('/rosout_agg', (msg) => {
    if (state.pausedLogs) return
    const source = String(msg?.name ?? 'ros')
    if (state.mutedSources.includes(source)) return
    const level = ROSOUT_LEVEL[Number(msg?.level)] ?? 'info'
    pushLog(level, String(msg?.msg ?? ''), source)
  })

  // /tf 帧列表低频刷新（面板展示用）
  window.setInterval(() => {
    if (ros.status === 'open') state.frames = tfTree.frames()
  }, 2000)
}

// ------------------------------------------------------------------ stats

const prevCounts = new Map<string, { count: number; time: number }>()

function statsTick(): void {
  const now = Date.now()
  const next: Record<string, UiTopicStat> = {}
  let newest = 0
  for (const [topic, stat] of ros.stats) {
    const prev = prevCounts.get(topic)
    let hz = 0
    if (prev && now > prev.time) {
      hz = ((stat.count - prev.count) * 1000) / (now - prev.time)
    }
    prevCounts.set(topic, { count: stat.count, time: now })
    next[topic] = {
      hz: Math.round(hz * 10) / 10,
      count: stat.count,
      size: stat.lastSize,
      age: stat.lastTime ? now - stat.lastTime : -1,
    }
    if (topic === '/slam/info' || topic === '/tf') {
      if (stat.lastTime > newest) newest = stat.lastTime
    }
  }
  state.stats = next
  state.dataAge = newest ? now - newest : -1
}

// ------------------------------------------------------------------ connect

export function connect(url?: string): void {
  if (url) {
    state.url = url.trim()
    localStorage.setItem('debug_hmi.url', state.url)
  }
  ros.connect(state.url)
}

export function disconnect(): void {
  ros.disconnect()
}

export async function refreshTopics(): Promise<void> {
  if (state.topicScanning) return
  state.topicScanning = true
  try {
    const topics = await ros.getTopics()
    const map: Record<string, string> = { ...TOPIC_CATALOG }
    for (const item of topics) {
      if (item.type) map[item.name] = item.type
    }
    state.topicNames = topics.map((t) => t.name)
    state.topicTypes = map
    pushLog('info', `话题列表已更新：${topics.length} 个`)
  } catch (e) {
    pushLog('warn', `读取话题列表失败：${(e as Error).message}`)
  } finally {
    state.topicScanning = false
  }
}

export function topicType(topic: string): string {
  return state.topicTypes[topic] ?? TOPIC_CATALOG[topic] ?? ''
}

let started = false

export function initHmi(): void {
  if (started) return
  started = true
  registerHandlers()

  state.topicNames = Object.keys(TOPIC_CATALOG).sort()
  state.topicTypes = { ...TOPIC_CATALOG }
  state.mutedSources = loadMutedSources()
  Object.assign(state.layers, loadLayers())

  ros.onStatus((status, detail) => {
    state.status = status
    if (status === 'open') {
      state.connectedAt = Date.now()
      state.error = ''
      syncSubscriptions()
    }
    if (detail && (status === 'error' || status === 'closed')) state.error = detail
  })

  ros.onLog((e: RosLogEvent) => pushLog(e.level, e.text))

  window.setInterval(statsTick, 500)
  window.addEventListener('beforeunload', () => ros.disconnect())
  connect(state.url)
}

// ------------------------------------------------------------------ services

function poseMessage(x: number, y: number, theta: number, frame = 'map'): any {
  const cov = new Array(36).fill(0)
  cov[0] = 0.25
  cov[7] = 0.25
  cov[35] = 0.0685
  return {
    header: { frame_id: frame, stamp: { secs: 0, nsecs: 0 } },
    pose: {
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(theta / 2), w: Math.cos(theta / 2) },
      },
      covariance: cov,
    },
  }
}

export async function callService(
  name: string,
  args: Record<string, unknown> = {},
  timeout = 15000,
): Promise<any> {
  state.servicePending = true
  try {
    const result = await ros.callService(name, args, timeout)
    state.lastService = `${name} ✓ ${JSON.stringify(result)}`
    pushLog('info', `服务 ${name} → ${JSON.stringify(result)}`)
    return result
  } catch (e) {
    const message = (e as Error).message
    state.lastService = `${name} ✗ ${message}`
    pushLog('error', `服务 ${name} 失败：${message}`)
    throw e
  } finally {
    state.servicePending = false
  }
}

export async function switchSlamMode(mode: 0 | 1): Promise<any> {
  return callService('/slam/mode_switch', { mode })
}

export async function dumpRecorder(): Promise<any> {
  return callService('/slam/dump_recorder', {})
}

export async function stopMapOperation(mapName?: string): Promise<any> {
  return callService('/stop_map_operation', {
    map_name: mapName || state.slam.map_name || 'map',
    map_dir: '',
    map_resolution: 0.05,
    include_unfinished_submaps: false,
  })
}

export async function relocate(mapName: string, x: number, y: number, theta: number): Promise<any> {
  return callService('/relocation', {
    map_name: mapName,
    initial_pose: poseMessage(x, y, theta),
  })
}

export function publishInitialPose(x: number, y: number, theta: number): void {
  const ok = ros.publish('/initialpose', poseMessage(x, y, theta))
  pushLog(
    ok ? 'info' : 'error',
    ok
      ? `已发布 /initialpose x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${((theta * 180) / Math.PI).toFixed(1)}°`
      : '发布 /initialpose 失败：连接未就绪',
  )
}

// ------------------------------------------------------------------ misc

export function setPendingPose(pose: PendingPose | null): void {
  if (pose) clearMeasure()
  state.pendingPose = pose
  state.estimating = !!pose
}

// ------------------------------------------------------------------ measure

export function toggleMeasure(): void {
  if (state.measure.active) {
    clearMeasure()
    return
  }
  setPendingPose(null)
  state.measure.active = true
  state.measure.a = null
  state.measure.b = null
  state.measureDistance = 0
  pushLog('info', '测距：在地图上点两个点（Esc 取消）')
}

export function clearMeasure(): void {
  state.measure.active = false
  state.measure.a = null
  state.measure.b = null
  state.measureDistance = 0
}

export function setMeasurePoint(x: number, y: number): void {
  if (!state.measure.active) return
  if (!state.measure.a || state.measure.b) {
    state.measure.a = { x, y }
    state.measure.b = null
    state.measureDistance = 0
    return
  }
  state.measure.b = { x, y }
  state.measureDistance = Math.hypot(x - state.measure.a.x, y - state.measure.a.y)
}

export function setPointScale(scale: number): void {
  const clamped = Math.min(3, Math.max(0.4, scale))
  state.pointScale = clamped
  try {
    localStorage.setItem('hmi.pointScale', String(clamped))
  } catch {
    /* ignore */
  }
}

export function selectRawTopic(topic: string): void {
  state.rawTopic = topic
  state.rawText = ''
  state.rawTime = 0
}

export function viewFit(): void {
  state.view.fit++
}

export function viewTop(): void {
  state.view.top++
}

export function viewShot(): void {
  state.view.shot++
}
