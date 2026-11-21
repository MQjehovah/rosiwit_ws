/**
 * HMI 全局状态与 ROS 数据流。
 *
 * 设计原则：
 * - 高频数据（点云原始 JSON、TF）不放进 Vue 响应式对象，由 three 层通过 onTopic 回调直接消费；
 * - 这里只保留 10~20 Hz 的可显示状态（位姿、状态码、统计、日志），避免 Vue 反复 diff 大对象。
 */

import { reactive } from 'vue'
import { RosBridgeClient, type RosStatus, type RosLogEvent, type SubscribeOptions } from './client'
import { TfTree } from './tf'

export const ros = new RosBridgeClient(
  localStorage.getItem('hmi.url') || 'ws://192.168.1.100:8089',
)
export const tfTree = new TfTree()

export type LayerKey =
  | 'globalMap'
  | 'scan'
  | 'points2d'
  | 'lidar'
  | 'grid'
  | 'paths'
  | 'robot'
  | 'tf'
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

  robot: { x: 0, y: 0, yaw: 0, source: '-' as string, stamp: 0 },

  layers: {
    globalMap: true,
    scan: true,
    points2d: true,
    lidar: true,
    grid: true,
    paths: true,
    robot: true,
    tf: true,
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

  estimating: false,
  pendingPose: null as PendingPose | null,

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
  gridCells: 0,
  gridTime: 0,
  gridBytes: 0,

  view: { fit: 0, top: 0, shot: 0, follow: false },

  fps: 0,
  pointCounts: {} as Record<string, number>,
})

// ------------------------------------------------------------------ logging

let logId = 1
export function pushLog(level: LogLine['level'], text: string): void {
  state.logs.unshift({ id: logId++, time: Date.now(), level, text })
  if (state.logs.length > 400) state.logs.length = 400
}

const ROSOUT_LEVEL: Record<number, LogLine['level']> = {
  1: 'info',
  2: 'info',
  4: 'warn',
  8: 'error',
  16: 'error',
}

// -------------------------------------------------------------- subscriptions

const BASE_SUBS: Array<[string, SubscribeOptions]> = [
  ['/slam/info', { throttle: 150, queue: 1 }],
  ['/tf', { throttle: 100, queue: 100 }],
  ['/tf_static', { queue: 1 }],
  ['/slam/fusion_diag', { throttle: 1000, queue: 1 }],
  ['/rosout_agg', { throttle: 500, queue: 5 }],
]

const LAYER_SUBS: Record<LayerKey, Array<[string, SubscribeOptions]>> = {
  // /global_map 是 latched（每次进模式只发一次），用 CBOR 拿原始字节
  globalMap: [['/global_map', { throttle: 0, queue: 1, compression: 'cbor' }]],
  scan: [
    // /scan_f_map 是旧版 PointCloud，未开 debug_publish 时也有数据，已在 map 系配准
    ['/scan_f_map', { throttle: 200, queue: 1, compression: 'cbor' }],
    ['/curr_scan_point', { throttle: 120, queue: 1, compression: 'cbor' }],
    ['/scan', { throttle: 200, queue: 1 }],
  ],
  points2d: [['/points_2d', { throttle: 500, queue: 1, compression: 'cbor' }]],
  lidar: [],
  grid: [],
  paths: [
    ['/global_path', { throttle: 1000, queue: 1 }],
    ['/local_path', { throttle: 500, queue: 1 }],
  ],
  robot: [['/robot_footprint', { throttle: 1000, queue: 1 }]],
  tf: [],
  trail: [],
}

function desiredSubscriptions(): Map<string, SubscribeOptions> {
  const wanted = new Map<string, SubscribeOptions>()
  for (const [topic, opts] of BASE_SUBS) wanted.set(topic, opts)
  for (const key of Object.keys(state.layers) as LayerKey[]) {
    if (!state.layers[key]) continue
    for (const [topic, opts] of LAYER_SUBS[key]) wanted.set(topic, opts)
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

export function setLayer(key: LayerKey, value: boolean): void {
  state.layers[key] = value
  syncSubscriptions()
}

// ----------------------------------------------------------- one-shot snapshot

type SnapshotHandler = (msg: any) => void

const lidarHandlers = new Set<SnapshotHandler>()

export function onLidarSnapshot(handler: SnapshotHandler): () => void {
  lidarHandlers.add(handler)
  return () => lidarHandlers.delete(handler)
}

/**
 * 只取话题上的下一条消息，取到后立刻退订。
 *
 * 原始雷达 /vanjee_points722 是 1MB/帧、10Hz（≈10MB/s），rosbridge 的 JSON 编码还会再膨胀，
 * 不适合持续订阅，所以做成手动快照。
 */
function takeOne(topic: string, opts: SubscribeOptions, timeout: number): Promise<any> {
  const already = ros.isSubscribed(topic)
  if (!already) ros.subscribe(topic, { ...opts, queue: 1 })
  return new Promise((resolve, reject) => {
    let done = false
    const off = ros.onTopic(topic, (msg) => {
      if (done) return
      done = true
      clearTimeout(timer)
      off()
      if (!already) ros.unsubscribe(topic)
      resolve(msg)
    })
    const timer = window.setTimeout(() => {
      if (done) return
      done = true
      off()
      if (!already) ros.unsubscribe(topic)
      reject(new Error(`等待 ${topic} 超时（${timeout}ms），该话题当前可能没有数据`))
    }, timeout)
  })
}

export async function grabLidarSnapshot(): Promise<void> {
  if (ros.status !== 'open') {
    pushLog('warn', '未连接 rosbridge，无法抓取雷达点云')
    return
  }
  if (state.snapshotPending) return
  state.snapshotPending = true
  pushLog('info', '正在抓取一帧 /vanjee_points722（约 1MB）…')
  try {
    const msg = await takeOne('/vanjee_points722', { throttle: 0 }, 30000)
    state.snapshotTime = Date.now()
    for (const fn of lidarHandlers) fn(msg)
  } catch (e) {
    pushLog('warn', `雷达快照失败：${(e as Error).message}`)
  } finally {
    state.snapshotPending = false
  }
}

// ------------------------------------------------------------------ grid snapshot

const gridHandlers = new Set<SnapshotHandler>()

export function onGridSnapshot(handler: SnapshotHandler): () => void {
  gridHandlers.add(handler)
  return () => gridHandlers.delete(handler)
}

/**
 * /map 单帧 43MB（4300 万格，1Hz）。JSON 完全传不动，这里用 CBOR 只抓一帧就退订。
 * 仍然会占用几秒钟带宽，按钮里会先让用户确认。
 */
export async function grabGridSnapshot(): Promise<void> {
  if (ros.status !== 'open') {
    pushLog('warn', '未连接 rosbridge，无法抓取栅格地图')
    return
  }
  if (state.gridPending) return
  state.gridPending = true
  pushLog('info', '正在抓取 /map（单帧约 43MB，CBOR 二进制，请稍候）…')
  try {
    const msg = await takeOne('/map', { throttle: 0, compression: 'cbor' }, 180000)
    const width = Number(msg?.info?.width) || 0
    const height = Number(msg?.info?.height) || 0
    state.gridCells = width * height
    state.gridBytes = Number(msg?.data?.byteLength ?? 0)
    state.gridTime = Date.now()
    pushLog(
      'info',
      `栅格地图已收到：${width}×${height}（${(state.gridCells / 1e6).toFixed(1)}M 格，` +
        `${(state.gridBytes / 1e6).toFixed(1)}MB）`,
    )
    for (const fn of gridHandlers) fn(msg)
  } catch (e) {
    pushLog('warn', `栅格地图抓取失败：${(e as Error).message}`)
  } finally {
    state.gridPending = false
  }
}

// ------------------------------------------------------------------ global map

/** /global_map 是 latched。退订再订阅一次即可让发布端重发最近一份。 */
export function reloadGlobalMap(): void {
  if (ros.status !== 'open') {
    pushLog('warn', '未连接 rosbridge')
    return
  }
  if (ros.isSubscribed('/global_map')) {
    ros.resubscribe('/global_map')
  } else {
    ros.subscribe('/global_map', { throttle: 0, queue: 1, compression: 'cbor' })
  }
  pushLog('info', '已请求重新发布 /global_map（若 debug_publish=false 则不会有数据）')
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
  })

  ros.onTopic('/slam/fusion_diag', (msg) => {
    state.fusionDiag = String(msg?.data ?? '')
  })

  ros.onTopic('/rosout_agg', (msg) => {
    if (state.pausedLogs) return
    const level = ROSOUT_LEVEL[Number(msg?.level)] ?? 'info'
    pushLog(level, `[${msg?.name ?? 'ros'}] ${msg?.msg ?? ''}`)
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
    localStorage.setItem('hmi.url', state.url)
  }
  ros.connect(state.url)
}

export function disconnect(): void {
  ros.disconnect()
}

export async function refreshTopics(): Promise<void> {
  // rosbridge 是单线程的：/rosapi/topics 在这台设备上要 8s 以上，期间所有话题都会停更。
  // 所以只在用户手动点击时调用，并且给足超时。
  if (state.topicScanning) return
  state.topicScanning = true
  pushLog('info', '正在扫描全部话题（rosbridge 单线程，期间数据会暂停几秒）…')
  try {
    const value: any = await ros.callService('/rosapi/topics', {}, 60000)
    const topics: string[] = value?.topics ?? []
    const types: string[] = value?.types ?? []
    const map: Record<string, string> = { ...TOPIC_CATALOG }
    topics.forEach((t, i) => {
      if (types[i]) map[t] = types[i]
    })
    state.topicNames = topics
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
  const ok = ros.publish(
    '/initialpose',
    poseMessage(x, y, theta),
    'geometry_msgs/PoseWithCovarianceStamped',
  )
  pushLog(
    ok ? 'info' : 'error',
    ok
      ? `已发布 /initialpose x=${x.toFixed(2)} y=${y.toFixed(2)} yaw=${((theta * 180) / Math.PI).toFixed(1)}°`
      : '发布 /initialpose 失败：连接未就绪',
  )
}

// ------------------------------------------------------------------ misc

export function setPendingPose(pose: PendingPose | null): void {
  state.pendingPose = pose
  state.estimating = !!pose
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
