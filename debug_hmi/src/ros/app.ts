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

export type LayerKey = 'tf' | 'status' | 'logs' | 'trail'

export type RendererKind = 'points' | 'grid' | 'laser' | 'path' | 'polygon' | 'json' | 'status' | 'image'

export interface DisplayStats {
  hz: number
  bytes: number
  total: number
  age: number
}

export interface DisplayItem {
  id: string
  topic: string
  type: string
  /** topic = 订阅 ROS 话题；pcd = 本地 PCD 文件 */
  source: 'topic' | 'pcd'
  pcdName: string
  renderer: RendererKind
  enabled: boolean
  rate: number
  voxel: number
  max: number
  pointSize: number
  color: string
  colorMode: 'flat' | 'height'
  stats: DisplayStats
}

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
  /** 前端“定位状态”插件根据 /slam/info 推导出的展示文本 */
  statusText: '',
  /** /device/robot_shadow 解析后的设备状态 */
  robotShadow: null as Record<string, any> | null,
  shadowTime: 0,

  displays: [] as DisplayItem[],
  topics: [] as Array<{ name: string; type: string }>,
  topicQuery: '',
  displayError: '',
  /** 本地 PCD 显示项的点数 / 版本（用于触发重绘） */
  pcdCounts: {} as Record<string, number>,
  pcdVersion: 0,
  /** 图像话题最新帧（Blob URL） */
  images: {} as Record<string, { url: string; width: number; height: number; stamp: number; encoding: string }>,
  previewImage: '',
  /** 全局流量：所有已发送话题的累计字节与当前速率 */
  topicTraffic: {} as Record<string, { rate: number; total: number }>,
  totalBytes: 0,
  totalRate: 0,

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
    // 只有 TF 默认订阅；日志/定位面板按需打开
    tf: true,
    status: true,
    logs: false,
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
  logs: [['/rosout_agg', { throttle: 500, queue: 5 }]],
  trail: [],
}

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
  // 动态显示项：断线重连 / 面板切换后重新对齐订阅
  for (const d of state.displays) {
    if (d.enabled) subscribeDisplay(d)
    else unsubscribeDisplay(d)
  }
}

// -------------------------------------------------------------- dynamic displays

const DISPLAYS_KEY = 'hmi.displays.v3'
const PALETTE = ['#38bdf8', '#fbbf24', '#34d399', '#f472b6', '#a78bfa', '#fb923c', '#22d3ee', '#f87171']
const PCD_CLOUDS = new Map<string, CloudData>()

export function getPcdCloud(id: string): CloudData | undefined {
  return PCD_CLOUDS.get(id)
}

export function rendererForType(type: string): RendererKind {
  if (type === 'sensor_msgs/PointCloud2' || type === 'sensor_msgs/PointCloud') return 'points'
  if (type === 'nav_msgs/OccupancyGrid') return 'grid'
  if (type === 'sensor_msgs/LaserScan') return 'laser'
  if (type === 'nav_msgs/Path') return 'path'
  if (type === 'geometry_msgs/PolygonStamped') return 'polygon'
  if (type === 'sensor_msgs/Image' || type === 'sensor_msgs/CompressedImage') return 'image'
  if (type === 'xzrobot_ros_dependency/SlamInfo') return 'status'
  return 'json'
}

function displayDefaults(topic: string, type: string, index: number): DisplayItem {
  const renderer = rendererForType(type)
  return {
    id: `${topic}#${Date.now().toString(36)}`,
    topic,
    type,
    source: 'topic',
    pcdName: '',
    renderer,
    enabled: true,
    rate: renderer === 'points' ? 2 : renderer === 'laser' ? 5 : renderer === 'grid' ? 1 : renderer === 'image' ? 1 : 2,
    voxel: renderer === 'points' ? 0.05 : 0,
    max: renderer === 'points' ? 300000 : 0,
    pointSize: renderer === 'laser' ? 0.12 : 0.07,
    color: PALETTE[index % PALETTE.length],
    colorMode: renderer === 'laser' || renderer === 'path' || renderer === 'polygon' ? 'flat' : 'height',
    stats: { hz: 0, bytes: 0, total: 0, age: -1 },
  }
}

function subscribeDisplay(d: DisplayItem): void {
  if (d.source !== 'topic') return
  if (!d.topic.startsWith('/') || d.renderer === 'status') return
  bridge.setSubscription(d.topic, {
    rate: d.rate,
    voxel: d.voxel,
    max: d.max,
  })
}

function unsubscribeDisplay(d: DisplayItem): void {
  if (!d.topic.startsWith('/') || d.renderer === 'status') return
  bridge.setSubscription(d.topic, null)
}

export function addDisplay(topic: string, type?: string): void {
  const clean = topic.trim()
  if (!clean) return
  const existing = state.displays.find((d) => d.topic === clean)
  if (existing) {
    state.displayError = `${clean} 已经在显示列表中`
    return
  }
  const known = state.topics.find((t) => t.name === clean)
  let typ = type || known?.type || state.topicTypes[clean] || ''
  if (!typ && /image/i.test(clean)) typ = 'sensor_msgs/Image'
  const item = displayDefaults(clean, typ, state.displays.length)
  state.displays.push(item)
  state.displayError = ''
  saveDisplays()
  if (item.enabled) subscribeDisplay(item)
}

export function removeDisplay(id: string): void {
  const i = state.displays.findIndex((d) => d.id === id)
  if (i < 0) return
  const [item] = state.displays.splice(i, 1)
  unsubscribeDisplay(item)
  PCD_CLOUDS.delete(id)
  delete state.pcdCounts[id]
  state.pcdVersion++
  saveDisplays()
}

/** 新建一个“本地 PCD”显示项；随后在显示项里选择文件。 */
/** 把一个本地 PCD 文件绑定到显示项。 */
export async function loadPcdIntoDisplay(id: string, file: File): Promise<void> {
  const d = state.displays.find((x) => x.id === id)
  if (!d) return
  state.displayError = ''
  try {
    await new Promise<void>((resolve) => requestAnimationFrame(() => resolve()))
    const parsed = await parseMapFile(file)
    const loaded = limitCloud(parsed, 1200000)
    const cloud: CloudData = {
      positions: loaded.positions,
      colors: null,
      count: loaded.count,
      total: loaded.count,
      frame: 'map',
      bbox: null,
    }
    PCD_CLOUDS.set(id, cloud)
    d.pcdName = loaded.name
    d.source = 'pcd'
    d.renderer = 'points'
    d.enabled = true
    state.pcdCounts[id] = loaded.count
    state.pcdVersion++
    unsubscribeDisplay(d)
    saveDisplays()
    pushLog(
      'info',
      `本地 PCD 已载入：${loaded.name}（${loaded.count.toLocaleString()} 点）`,
    )
  } catch (e) {
    state.displayError = (e as Error).message
    pushLog('error', `加载 PCD 失败：${state.displayError}`)
  }
}

/** 切换数据源：话题 / 本地 PCD。 */
export function changeDisplaySource(id: string, source: 'topic' | 'pcd'): void {
  const d = state.displays.find((x) => x.id === id)
  if (!d) return
  d.source = source
  if (source === 'topic') subscribeDisplay(d)
  else unsubscribeDisplay(d)
  saveDisplays()
  state.pcdVersion++
}

export function updateDisplay(id: string, patch: Partial<DisplayItem>): void {
  const d = state.displays.find((x) => x.id === id)
  if (!d) return
  Object.assign(d, patch)
  saveDisplays()
  if (d.enabled && d.source === 'topic') subscribeDisplay(d)
  else unsubscribeDisplay(d)
  state.pcdVersion++
}

/** 点大小拖动只改渲染，不重订阅。 */
export function setDisplayPointSize(id: string, size: number): void {
  const d = state.displays.find((x) => x.id === id)
  if (!d || !Number.isFinite(size)) return
  d.pointSize = Math.max(0.01, Math.min(0.5, size))
  saveDisplays()
  state.pcdVersion++
}

export function toggleDisplay(id: string): void {
  const d = state.displays.find((x) => x.id === id)
  if (!d) return
  d.enabled = !d.enabled
  if (d.enabled && d.source === 'topic') subscribeDisplay(d)
  else unsubscribeDisplay(d)
  saveDisplays()
}

/** 单帧抓取：点云/栅格/激光/path/polygon 都能用。 */
export async function grabTopic(id: string): Promise<void> {
  const d = state.displays.find((x) => x.id === id)
  if (!d) return
  if (d.source === 'pcd') {
    state.displayError = '本地 PCD 显示项请重新选择文件刷新'
    return
  }
  if (ros.status !== 'open') {
    state.displayError = 'debug_bridge 未连接'
    return
  }
  state.displayError = ''
  pushLog('info', `抓取 ${d.topic} 单帧…`)
  try {
    await bridge.requestAny(d.topic, 120000, false)
    pushLog('info', `${d.topic} 单帧已接收`)
  } catch (e) {
    state.displayError = (e as Error).message
    pushLog('warn', `${d.topic} 抓取失败：${(e as Error).message}`)
  }
}

export function openImagePreview(topic: string): void {
  state.previewImage = topic
}

export function closeImagePreview(): void {
  state.previewImage = ''
}

function saveDisplays(): void {
  try {
    const slim = state.displays.map(({ stats, id, ...rest }) => ({ ...rest, id }))
    localStorage.setItem(DISPLAYS_KEY, JSON.stringify(slim))
  } catch {
    /* ignore */
  }
}

function loadDisplays(): void {
  try {
    const raw = localStorage.getItem(DISPLAYS_KEY)
    if (!raw) return
    const parsed = JSON.parse(raw)
    if (!Array.isArray(parsed)) return
    state.displays = parsed
      .filter((d) => d && typeof d.topic === 'string')
      .map((d, i) => ({ ...displayDefaults(d.topic, String(d.type || ''), i), ...d, stats: { hz: 0, bytes: 0, total: 0, age: -1 } }))
  } catch {
    /* ignore */
  }
}

/** 拉取话题列表，供“添加显示项”下拉框使用。 */
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
    state.topics = topics
      .map((t) => ({ name: t.name, type: t.type || map[t.name] || '' }))
      .sort((a, b) => a.name.localeCompare(b.name))
    if (!state.displays.length) {
      const seed = '/curr_scan_point'
      if (state.topics.some((t) => t.name === seed)) addDisplay(seed, map[seed])
    }
    pushLog('info', `话题列表已更新：${topics.length} 个`)
  } catch (e) {
    pushLog('warn', `读取话题列表失败：${(e as Error).message}`)
  } finally {
    state.topicScanning = false
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
    const grid = await bridge.requestGrid('/map', true, 180000)
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

// ------------------------------------------------------------------ topic frames

type TopicCloudHandler = (topic: string, cloud: CloudData) => void
type TopicGridHandler = (topic: string, grid: GridData) => void

const topicCloudHandlers = new Set<TopicCloudHandler>()
const topicGridHandlers = new Set<TopicGridHandler>()

export function onTopicCloud(handler: TopicCloudHandler): () => void {
  topicCloudHandlers.add(handler)
  return () => topicCloudHandlers.delete(handler)
}

export function onTopicGrid(handler: TopicGridHandler): () => void {
  topicGridHandlers.add(handler)
  return () => topicGridHandlers.delete(handler)
}

export function emitTopicCloud(topic: string, cloud: CloudData): void {
  state.pointCounts[topic] = cloud.count
  for (const fn of topicCloudHandlers) fn(topic, cloud)
}

export function emitTopicGrid(topic: string, grid: GridData): void {
  state.pointCounts[topic] = grid.width * grid.height
  for (const fn of topicGridHandlers) fn(topic, grid)
}

// ------------------------------------------------------------------ global map

function emitLoadedMap(cloud: CloudData): void {
  state.pointCounts.globalMap = cloud.count
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

// ------------------------------------------------------------------ handlers

const TOPIC_CATALOG: Record<string, string> = {
  '/slam/info': 'xzrobot_ros_dependency/SlamInfo',
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

/** 定位状态插件：把 /slam/info 的枚举/字符串统一成可读文本。 */
function numeric(value: unknown, fallback = -1): number {
  if (typeof value === 'number') return Number.isFinite(value) ? value : fallback
  if (typeof value === 'string' && value.trim() !== '') {
    const n = Number(value)
    return Number.isFinite(n) ? n : fallback
  }
  return fallback
}

function describeLocationStatus(slamState: unknown, location: unknown): string {
  const locMap: Record<string, string> = {
    '0': '定位正常',
    '1': '定位中 / 恢复中',
    '2': '定位失败',
    SUCCESS: '定位正常',
    RECOVERING: '定位中 / 恢复中',
    FAILED: '定位失败',
  }
  const slamMap: Record<string, string> = {
    '0': '建图模式',
    '1': '定位模式',
    '2': '错误模式',
    MAPPING: '建图模式',
    LOCALIZATION: '定位模式',
    ERROR: '错误模式',
  }
  const loc = typeof location === 'string' ? (locMap[location] ?? location) : locMap[String(location)]
  const mode = typeof slamState === 'string' ? (slamMap[slamState] ?? slamState) : slamMap[String(slamState)]
  return [mode, loc].filter(Boolean).join(' · ') || '状态未知'
}

function imageMime(encoding: string): string {
  if (encoding === 'jpeg' || encoding === 'jpg') return 'image/jpeg'
  if (encoding === 'png') return 'image/png'
  return 'application/octet-stream'
}

function rawImageUrl(frame: {
  width: number
  height: number
  encoding: string
  step: number
  data: Uint8Array
}): string | null {
  const { width, height, encoding, step, data } = frame
  if (!width || !height) return null
  const canvas = document.createElement('canvas')
  canvas.width = width
  canvas.height = height
  const ctx = canvas.getContext('2d')
  if (!ctx) return null
  const out = ctx.createImageData(width, height)
  const dst = out.data
  if (encoding === 'bgr8' || encoding === 'rgb8') {
    const swap = encoding === 'bgr8'
    for (let y = 0; y < height; y++) {
      const row = y * step
      for (let x = 0; x < width; x++) {
        const s = row + x * 3
        const d = (y * width + x) * 4
        dst[d] = data[swap ? s + 2 : s]
        dst[d + 1] = data[s + 1]
        dst[d + 2] = data[swap ? s : s + 2]
        dst[d + 3] = 255
      }
    }
  } else if (encoding === 'mono8' || encoding === '8uc1') {
    for (let y = 0; y < height; y++) {
      const row = y * step
      for (let x = 0; x < width; x++) {
        const v = data[row + x]
        const d = (y * width + x) * 4
        dst[d] = dst[d + 1] = dst[d + 2] = v
        dst[d + 3] = 255
      }
    }
  } else if (encoding === 'mono16' || encoding === '16uc1') {
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength)
    let min = 65535
    let max = 0
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const v = view.getUint16(y * step + x * 2, true)
        if (v < min) min = v
        if (v > max) max = v
      }
    }
    const span = max - min || 1
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const v = ((view.getUint16(y * step + x * 2, true) - min) / span) * 255
        const d = (y * width + x) * 4
        dst[d] = dst[d + 1] = dst[d + 2] = v
        dst[d + 3] = 255
      }
    }
  } else if (encoding === '32fc1') {
    const view = new DataView(data.buffer, data.byteOffset, data.byteLength)
    let min = Infinity
    let max = -Infinity
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const v = view.getFloat32(y * step + x * 4, true)
        if (Number.isFinite(v)) {
          if (v < min) min = v
          if (v > max) max = v
        }
      }
    }
    const span = max - min || 1
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const raw = view.getFloat32(y * step + x * 4, true)
        const v = Number.isFinite(raw) ? ((raw - min) / span) * 255 : 0
        const d = (y * width + x) * 4
        dst[d] = dst[d + 1] = dst[d + 2] = v
        dst[d + 3] = 255
      }
    }
  } else {
    return null
  }
  ctx.putImageData(out, 0, 0)
  return canvas.toDataURL('image/jpeg', 0.85)
}

function emitTopicImage(topic: string, frame: {
  width: number
  height: number
  encoding: string
  step: number
  data: Uint8Array
  stamp: number
}): void {
  let url: string | null = null
  const mime = imageMime(frame.encoding)
  if (mime.startsWith('image/')) {
    url = URL.createObjectURL(new Blob([frame.data], { type: mime }))
  } else {
    url = rawImageUrl(frame)
  }
  if (!url) return
  const prev = state.images[topic]
  if (prev?.url.startsWith('blob:')) URL.revokeObjectURL(prev.url)
  state.images[topic] = {
    url,
    width: frame.width,
    height: frame.height,
    stamp: frame.stamp,
    encoding: frame.encoding,
  }
  state.pointCounts[topic] = 1
}

function registerHandlers(): void {
  if (handlersReady) return
  handlersReady = true

  ros.onTopic('/tf', (msg) => tfTree.ingest(msg, false))
  ros.onTopic('/tf_static', (msg) => tfTree.ingest(msg, true))

  ros.onTopic('/slam/info', (msg) => {
    const slam = msg?.slam ?? msg
    const stateRaw = slam?.slam_state
    const locRaw = slam?.location_status
    state.slam = {
      slam_state: numeric(stateRaw),
      location_status: numeric(locRaw),
      matching_degree: numeric(slam?.matching_degree, 0),
      update_request: numeric(slam?.update_request, 0),
      map_name: String(slam?.map_name ?? ''),
      map_area: numeric(slam?.map_area, 0),
      robot_pose: {
        x: numeric(slam?.robot_pose?.x, 0),
        y: numeric(slam?.robot_pose?.y, 0),
        theta: numeric(slam?.robot_pose?.theta, 0),
      },
    }
    state.slamTime = Date.now()
    state.statusText = describeLocationStatus(stateRaw, locRaw)

    // 匹配度趋势：约 2.5Hz 采样，保留最近 120 点（~48s）
    const now = performance.now()
    if (now - lastMatchSample > 400) {
      lastMatchSample = now
      state.matchHistory.push(state.slam.matching_degree)
      if (state.matchHistory.length > 120) state.matchHistory.shift()
    }
  })

  ros.onTopic('/device/robot_shadow', (msg) => {
    try {
      const raw = typeof msg?.data === 'string' ? msg.data : msg
      const obj = typeof raw === 'string' ? JSON.parse(raw) : raw
      if (obj && typeof obj === 'object') {
        state.robotShadow = obj
        state.shadowTime = Date.now()
      }
    } catch (e) {
      pushLog('warn', `robot_shadow 解析失败：${(e as Error).message}`)
    }
  })

  ros.onTopic('/rosout_agg', (msg) => {
    if (state.pausedLogs) return
    const source = String(msg?.name ?? 'ros')
    if (state.mutedSources.includes(source)) return
    const level = ROSOUT_LEVEL[Number(msg?.level)] ?? 'info'
    pushLog(level, String(msg?.msg ?? ''), source)
  })

  bridge.onStat((stat) => {
    // rate_hz 是帧率；换算成字节速率用于全局流量
    state.topicTraffic[stat.topic] = {
      rate: stat.rate_hz * stat.bytes,
      total: stat.total_bytes,
    }
    let total = 0
    let rate = 0
    for (const item of Object.values(state.topicTraffic)) {
      total += item.total
      rate += item.rate
    }
    state.totalBytes = total
    state.totalRate = rate
    const d = state.displays.find((x) => x.topic === stat.topic)
    if (d) {
      d.stats = {
        hz: stat.rate_hz,
        bytes: stat.bytes,
        total: stat.total_bytes,
        age: stat.age_ms,
      }
    }
  })

  bridge.onImage((frame) => emitTopicImage(frame.stream, frame))

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
  loadDisplays()
  // 设备信息插件：常驻低频订阅，不依赖显示项
  bridge.setSubscription('/device/robot_shadow', { rate: 1, voxel: 0, max: 0 })

  ros.onStatus((status, detail) => {
    state.status = status
    if (status === 'open') {
      state.connectedAt = Date.now()
      state.error = ''
      syncSubscriptions()
      void refreshTopics()
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
