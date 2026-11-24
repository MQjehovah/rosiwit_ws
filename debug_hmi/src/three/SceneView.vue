<template>
  <div ref="host" class="viewport" :class="{ estimating: state.estimating, measuring: state.measure.active }">
    <div class="hud hud-tl">
      <span class="chip">FPS {{ state.fps.toFixed(0) }}</span>
      <span v-if="displayCount" class="chip">{{ displayCount }} 个显示项</span>
      <span v-if="state.totalBytes" class="chip mono">流量 {{ trafficText }}</span>
    </div>
    <div class="hud hud-tr">
      <span class="chip">{{ sourceShort }}</span>
      <span class="chip mono">
        xyz {{ state.robot.x.toFixed(2) }} {{ state.robot.y.toFixed(2) }} {{ state.robot.z.toFixed(2) }}
      </span>
      <span class="chip mono">
        rpy {{ deg(state.robot.roll) }} {{ deg(state.robot.pitch) }} {{ deg(state.robot.yaw) }}°
      </span>
    </div>
    <div class="hud hud-bl">拖动旋转 · 右键平移 · 滚轮缩放</div>
    <div v-if="state.estimating" class="estimate-banner">
      在地面点击放置位姿，移动鼠标调整朝向，然后在左侧确认发布
    </div>
    <div v-if="noSources" class="empty-hint">
      像 RViz 一样在左侧「显示项」里添加话题：点云、栅格、激光、路径都可以单独设置
    </div>
    <div v-if="state.measure.active || state.measure.a" class="hud hud-measure">
      <span v-if="!state.measure.a" class="chip">测距：点第一个点</span>
      <span v-else-if="!state.measure.b" class="chip">测距：点第二个点</span>
      <span v-else class="chip accent">
        测距 {{ state.measureDistance.toFixed(2) }} m
        <template v-if="state.measure.a && state.measure.b">
          · Δx {{ Math.abs(state.measure.b.x - state.measure.a.x).toFixed(2) }} · Δy
          {{ Math.abs(state.measure.b.y - state.measure.a.y).toFixed(2) }}
        </template>
      </span>
      <button class="chip-btn" @click="clearMeasure">清除</button>
    </div>

    <div v-if="previewImageData" class="image-preview" @click.self="closeImagePreview">
      <div class="image-card">
        <div class="image-head">
          <span>{{ state.previewImage }}</span>
          <button class="chip-btn" @click="closeImagePreview">关闭</button>
        </div>
        <img :src="previewImageData.url" :alt="state.previewImage" />
      </div>
    </div>

    <div class="view-toolbar">
      <button title="顶视 (T)" @click="topView">顶视</button>
      <button title="适配内容 (F)" @click="fitView">适配</button>
      <button :class="{ on: state.view.follow }" title="跟随机器人" @click="toggleFollow">跟随</button>
      <button :class="{ on: state.measure.active }" title="测距 (M)" @click="toggleMeasure">测距</button>
      <button title="截图" @click="screenshot">截图</button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import {
  bridge,
  clearMeasure,
  closeImagePreview,
  emitTopicCloud,
  emitTopicGrid,
  getPcdCloud,
  onTopicCloud,
  onTopicGrid,
  pushLog,
  ros,
  setMeasurePoint,
  setPendingPose,
  state,
  tfTree,
  toggleMeasure,
} from '../ros/app'
import { loadGridCache, saveGridCache } from '../ros/gridCache'
import type { CloudData, GridData } from '../ros/decode'
import { quatToRpy } from '../ros/tf'
import {
  applyTransform,
  GridMapLayer,
  PathLayer,
  PointCloudLayer,
  RobotLayer,
  TfLayer,
  TrailLayer,
} from './layers'

const host = ref<HTMLDivElement | null>(null)

let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene | null = null
let camera: THREE.PerspectiveCamera | null = null
let controls: OrbitControls | null = null
let raf = 0
let resizeObserver: ResizeObserver | null = null
let floorGrid: THREE.GridHelper | null = null

/** 本地 PCD 文件不再单独占用固定图层，全部作为显示项渲染 */
const robotLayer = new RobotLayer()
const tfLayer = new TfLayer()
const trailLayer = new TrailLayer(0x38bdf8)

interface DynamicLayer {
  renderer: string
  layer: PointCloudLayer | GridMapLayer | PathLayer
}

const dynamicLayers = new Map<string, DynamicLayer>()
const lastGrid = new Map<string, GridData>()

const pendingGroup = new THREE.Group()
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0)
const raycaster = new THREE.Raycaster()

const measureGroup = new THREE.Group()
const measureLineGeom = new THREE.BufferGeometry()
measureLineGeom.setAttribute(
  'position',
  new THREE.BufferAttribute(new Float32Array([0, 0, 0.05, 0, 0, 0.05]), 3),
)
const measureLine = new THREE.Line(
  measureLineGeom,
  new THREE.LineDashedMaterial({ color: 0xfbbf24, dashSize: 0.35, gapSize: 0.2 }),
)
measureLine.computeLineDistances()
const measureDots = [0, 1].map(() => {
  const dot = new THREE.Mesh(
    new THREE.RingGeometry(0.12, 0.2, 24),
    new THREE.MeshBasicMaterial({ color: 0xfbbf24, side: THREE.DoubleSide }),
  )
  dot.position.z = 0.05
  return dot
})
measureGroup.add(measureLine, ...measureDots)
measureGroup.visible = false

const unsubs: Array<() => void> = []
const warned = new Set<string>()

let userMoved = false
let autoCentered = false
let lastRobotPose = { x: Number.NaN, y: Number.NaN }
let lastRobotPublish = 0
let lastTrail = 0
let frameCounter = 0
let fpsTime = performance.now()
let frameCache: string[] = []
let frameCacheVersion = -1

function currentFrames(): string[] {
  if (frameCacheVersion !== tfTree.version) {
    frameCacheVersion = tfTree.version
    frameCache = tfTree.frames()
  }
  return frameCache
}

const displayCount = computed(() => state.displays.filter((d) => d.enabled).length)

const previewImageData = computed(() => (state.previewImage ? state.images[state.previewImage] : null))

/** 没有任何显示项 */
const noSources = computed(() => state.displays.length === 0)

const trafficText = computed(() => {
  const rate = state.totalRate
  const rateText =
    rate >= 1024 * 1024 ? `${(rate / 1048576).toFixed(2)} MB/s` : `${(rate / 1024).toFixed(2)} KB/s`
  const total = state.totalBytes
  const totalText =
    total >= 1024 * 1024 ? `${(total / 1048576).toFixed(2)} MB` : `${(total / 1024).toFixed(2)} KB`
  return `${rateText} · ${totalText}`
})

const sourceShort = computed(() => state.robot.source.replace('TF map→', 'TF '))

function deg(rad: number): string {
  return ((rad * 180) / Math.PI).toFixed(1)
}

function toggleFollow(): void {
  state.view.follow = !state.view.follow
}

function warnOnce(key: string, text: string): void {
  if (warned.has(key)) return
  warned.add(key)
  pushLog('warn', text)
}

function placeInMap(object: THREE.Object3D, frame: string): void {
  if (!frame || frame === 'map' || frame === 'world') {
    object.position.set(0, 0, 0)
    object.quaternion.identity()
    return
  }
  let tf = tfTree.lookup('map', frame)
  if (!tf && frame === 'vanjee_lidar') {
    // 万集驱动把点云去旋转成"重力对齐"系, 但 frame_id 仍是 vanjee_lidar,
    // 而 TF 树里只有 lidar(物理安装角 39°). 直接查会失败, 点云会停在原点不旋转.
    // 点云原点=雷达位置, 姿态=重力对齐(与 base_link 一致): 用 lidar 的平移 + base_link 的旋转.
    const lidar = tfTree.lookup('map', 'lidar')
    const base = tfTree.lookup('map', 'base_link')
    if (lidar && base) tf = { parent: 'map', child: frame, t: lidar.t, q: base.q }
  }
  if (tf) applyTransform(object, tf)
}

function buildMarker(): void {
  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.38, 0.5, 40),
    new THREE.MeshBasicMaterial({ color: 0xfbbf24, side: THREE.DoubleSide, transparent: true, opacity: 0.9 }),
  )
  const disc = new THREE.Mesh(
    new THREE.CircleGeometry(0.08, 16),
    new THREE.MeshBasicMaterial({ color: 0xfbbf24 }),
  )
  const arrow = new THREE.Mesh(
    new THREE.ConeGeometry(0.16, 0.65, 4),
    new THREE.MeshBasicMaterial({ color: 0xfbbf24 }),
  )
  arrow.rotation.z = -Math.PI / 2
  arrow.position.set(0.75, 0, 0)
  pendingGroup.add(ring, disc, arrow)
  pendingGroup.visible = false
}

// ---------------------------------------------------------------- dynamic layers

function hexColor(value: string): number {
  const c = new THREE.Color(value)
  return c.getHex()
}

function heightColors(positions: Float32Array): Float32Array {
  const n = positions.length / 3
  const out = new Float32Array(n * 3)
  let zmin = Infinity
  let zmax = -Infinity
  for (let i = 2; i < positions.length; i += 3) {
    const z = positions[i]
    if (z < zmin) zmin = z
    if (z > zmax) zmax = z
  }
  const span = zmax - zmin || 1
  for (let i = 0; i < n; i++) {
    const t = (positions[i * 3 + 2] - zmin) / span
    // 蓝(0.66) -> 青 -> 绿 -> 黄 -> 红(0)
    const h = 0.66 * (1 - t)
    const s = 0.85
    const l = 0.52
    const q = l < 0.5 ? l * (1 + s) : l + s - l * s
    const p = 2 * l - q
    const hue = (x: number): number => {
      let v = x
      if (v < 0) v += 1
      if (v > 1) v -= 1
      if (v < 1 / 6) return p + (q - p) * 6 * v
      if (v < 1 / 2) return q
      if (v < 2 / 3) return p + (q - p) * (2 / 3 - v) * 6
      return p
    }
    out[i * 3] = hue(h + 1 / 3)
    out[i * 3 + 1] = hue(h)
    out[i * 3 + 2] = hue(h - 1 / 3)
  }
  return out
}

function disposeDynamic(topic: string): void {
  const entry = dynamicLayers.get(topic)
  if (!entry) return
  if (scene) scene.remove(entry.layer.object)
  if (entry.layer instanceof PointCloudLayer) entry.layer.dispose()
  else if (entry.layer instanceof PathLayer) entry.layer.dispose()
  else entry.layer.clear()
  dynamicLayers.delete(topic)
}

function ensureDynamic(topic: string, renderer: string): DynamicLayer | null {
  const existing = dynamicLayers.get(topic)
  if (existing && existing.renderer === renderer) return existing
  if (existing) disposeDynamic(topic)
  const d = state.displays.find((x) => x.topic === topic)
  let layer: PointCloudLayer | GridMapLayer | PathLayer
  if (renderer === 'grid') layer = new GridMapLayer()
  else if (renderer === 'path' || renderer === 'polygon') layer = new PathLayer(hexColor(d?.color || '#38bdf8'), 0.9)
  else if (renderer === 'points' || renderer === 'laser') {
    layer = new PointCloudLayer({
      size: d?.pointSize || 0.08,
      color: hexColor(d?.color || '#38bdf8'),
      opacity: 0.95,
    })
  } else {
    return null
  }
  const entry = { renderer, layer }
  dynamicLayers.set(topic, entry)
  if (scene) scene.add(layer.object)
  return entry
}

function syncDisplayLayers(): void {
  const alive = new Set(state.displays.map((d) => d.topic))
  for (const topic of [...dynamicLayers.keys()]) {
    if (!alive.has(topic)) disposeDynamic(topic)
  }
  for (const d of state.displays) {
    const entry = ensureDynamic(d.topic, d.renderer)
    if (!entry) continue
    entry.layer.setVisible(d.enabled)
    if (entry.layer instanceof PointCloudLayer) {
      entry.layer.setSize(d.pointSize)
      entry.layer.setColor(hexColor(d.color))
      if (d.source === 'pcd') {
        const cloud = getPcdCloud(d.id)
        if (cloud) {
          const colors =
            d.colorMode === 'intensity'
              ? cloud.colors
              : d.colorMode === 'height' && cloud.count <= 1500000
                ? heightColors(cloud.positions)
                : null
          entry.layer.setPoints(cloud.positions, colors)
          if (!colors) entry.layer.setColor(hexColor(d.color))
          entry.layer.setVisible(d.enabled)
          placeInMap(entry.layer.object, 'map')
        } else {
          entry.layer.clear()
          entry.layer.setVisible(false)
        }
      }
    } else if (entry.layer instanceof PathLayer) {
      entry.layer.setColor(hexColor(d.color))
    } else if (entry.layer instanceof GridMapLayer) {
      const cached = lastGrid.get(d.topic)
      if (cached) entry.layer.update(cached)
    }
  }
}

// ---------------------------------------------------------------- topic frames

function applyTopicCloud(topic: string, cloud: CloudData): void {
  const d = state.displays.find((x) => x.topic === topic)
  if (!d || !d.enabled || d.renderer !== 'points') return
  const entry = ensureDynamic(topic, 'points')
  if (!entry || !(entry.layer instanceof PointCloudLayer)) return
  // intensity: 用服务端带来的 uint8 标量 (overlay 的持久度 w / 雷达强度) 着色,
  // 固定色标不随 min/max 拉伸; height: 本地按 z 上色; flat: 用条目颜色.
  const colors =
    d.colorMode === 'intensity'
      ? cloud.colors
      : d.colorMode === 'height' && cloud.count <= 1500000
        ? heightColors(cloud.positions)
        : null
  entry.layer.setPoints(cloud.positions, colors)
  if (!colors) entry.layer.setColor(hexColor(d.color))
  entry.layer.setSize(d.pointSize)
  entry.layer.setVisible(true)
  placeInMap(entry.layer.object, cloud.frame)
  state.pointCounts[topic] = cloud.count
}

function applyTopicGrid(topic: string, grid: GridData): void {
  const d = state.displays.find((x) => x.topic === topic)
  if (!d || !d.enabled || d.renderer !== 'grid') return
  const entry = ensureDynamic(topic, 'grid')
  if (!entry || !(entry.layer instanceof GridMapLayer)) return
  entry.layer.update(grid)
  entry.layer.setVisible(true)
  lastGrid.set(topic, grid)
  state.pointCounts[topic] = grid.width * grid.height
  if (topic === '/map') {
    state.gridCells = grid.width * grid.height
    state.gridTime = Date.now()
    state.gridFromCache = false
    void saveGridCache(grid).catch(() => {
      /* 缓存失败不影响显示 */
    })
  }
}

function laserPositions(l: {
  ranges: Float32Array
  angle_min: number
  angle_increment: number
  range_min: number
  range_max: number
}): Float32Array {
  const pts: number[] = []
  for (let i = 0; i < l.ranges.length; i++) {
    const r = l.ranges[i]
    if (!Number.isFinite(r) || r < l.range_min || r > l.range_max) continue
    const a = l.angle_min + i * l.angle_increment
    pts.push(r * Math.cos(a), r * Math.sin(a), 0.15)
  }
  return new Float32Array(pts)
}

function applyTopicLaser(l: {
  stream: string
  frame: string
  ranges: Float32Array
  angle_min: number
  angle_increment: number
  range_min: number
  range_max: number
}): void {
  const d = state.displays.find((x) => x.topic === l.stream)
  if (!d || !d.enabled || d.renderer !== 'laser') return
  const entry = ensureDynamic(l.stream, 'laser')
  if (!entry || !(entry.layer instanceof PointCloudLayer)) return
  const positions = laserPositions(l)
  entry.layer.setPoints(positions, null)
  entry.layer.setColor(hexColor(d.color))
  entry.layer.setSize(d.pointSize)
  entry.layer.setVisible(true)
  placeInMap(entry.layer.object, l.frame)
  state.pointCounts[l.stream] = positions.length / 3
}

function applyTopicPath(p: { stream: string; frame: string; poses: Array<{ x: number; y: number; z: number }> }): void {
  const d = state.displays.find((x) => x.topic === p.stream)
  if (!d || !d.enabled || d.renderer !== 'path') return
  const entry = ensureDynamic(p.stream, 'path')
  if (!entry || !(entry.layer instanceof PathLayer)) return
  const positions = new Float32Array(p.poses.length * 3)
  p.poses.forEach((q, i) => {
    positions[i * 3] = q.x
    positions[i * 3 + 1] = q.y
    positions[i * 3 + 2] = q.z
  })
  entry.layer.setPoints(positions)
  entry.layer.setColor(hexColor(d.color))
  entry.layer.setVisible(true)
  placeInMap(entry.layer.object, p.frame)
}

function applyTopicPolygon(p: { stream: string; frame: string; xy: Float32Array }): void {
  const d = state.displays.find((x) => x.topic === p.stream)
  if (!d || !d.enabled || d.renderer !== 'polygon') return
  const n = Math.floor(p.xy.length / 2)
  if (n < 2) return
  const entry = ensureDynamic(p.stream, 'polygon')
  if (!entry || !(entry.layer instanceof PathLayer)) return
  const positions = new Float32Array((n + 1) * 3)
  for (let i = 0; i < n; i++) {
    positions[i * 3] = p.xy[i * 2]
    positions[i * 3 + 1] = p.xy[i * 2 + 1]
    positions[i * 3 + 2] = 0.02
  }
  positions[n * 3] = p.xy[0]
  positions[n * 3 + 1] = p.xy[1]
  positions[n * 3 + 2] = 0.02
  entry.layer.setPoints(positions)
  entry.layer.setColor(hexColor(d.color))
  entry.layer.setVisible(true)
  placeInMap(entry.layer.object, p.frame)
}

function bindTopics(): void {
  unsubs.push(bridge.onCloud((topic, cloud) => applyTopicCloud(topic, cloud)))
  unsubs.push(bridge.onGrid((topic, grid) => applyTopicGrid(topic, grid)))
  unsubs.push(bridge.onLaser((l) => applyTopicLaser(l)))
  unsubs.push(bridge.onPath((p) => applyTopicPath(p)))
  unsubs.push(bridge.onPolygon((p) => applyTopicPolygon(p)))
  unsubs.push(onTopicCloud((topic, cloud) => applyTopicCloud(topic, cloud)))
  unsubs.push(onTopicGrid((topic, grid) => applyTopicGrid(topic, grid)))
}

function applyVisibility(): void {
  // 机器人本体只依赖 TF，常显示
  robotLayer.setVisible(true)
  robotLayer.setFootprintVisible(true)
  trailLayer.setVisible(state.layers.trail)
  tfLayer.object.visible = state.layers.tf
  for (const d of state.displays) {
    const entry = dynamicLayers.get(d.topic)
    if (entry) entry.layer.setVisible(d.enabled)
  }
}

// ---------------------------------------------------------------- interactions

function syncMeasure(): void {
  const { active, a, b } = state.measure
  measureGroup.visible = active && !!a
  measureDots[0].visible = !!a
  measureDots[1].visible = !!b
  if (a) measureDots[0].position.set(a.x, a.y, 0.05)
  if (b) measureDots[1].position.set(b.x, b.y, 0.05)
  if (a && b) {
    const pos = measureLineGeom.getAttribute('position') as THREE.BufferAttribute
    pos.setXYZ(0, a.x, a.y, 0.05)
    pos.setXYZ(1, b.x, b.y, 0.05)
    pos.needsUpdate = true
    measureLineGeom.computeBoundingSphere()
    measureLine.computeLineDistances()
  }
  measureLine.visible = !!(a && b)
}

function setupScene(): void {
  const el = host.value
  if (!el) return
  scene = new THREE.Scene()
  scene.background = new THREE.Color(0x070b10)

  camera = new THREE.PerspectiveCamera(55, 1, 0.05, 6000)
  camera.up.set(0, 0, 1)
  camera.position.set(-14, -16, 14)

  renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.outputColorSpace = THREE.SRGBColorSpace
  renderer.domElement.style.display = 'block'
  el.appendChild(renderer.domElement)

  controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.dampingFactor = 0.08
  controls.maxPolarAngle = Math.PI * 0.495
  controls.minDistance = 0.8
  controls.maxDistance = 900
  controls.target.set(0, 0, 0)
  controls.addEventListener('start', () => {
    userMoved = true
  })
  controls.update()

  const grid = new THREE.GridHelper(400, 80, 0x1e3a4d, 0x132029)
  floorGrid = grid
  grid.rotation.x = Math.PI / 2
  grid.position.z = -0.02
  ;(grid.material as THREE.Material).transparent = true
  ;(grid.material as THREE.Material).opacity = 0.7
  scene.add(grid)

  const origin = new THREE.AxesHelper(2)
  origin.position.z = 0.03
  scene.add(origin)

  scene.add(new THREE.HemisphereLight(0xd8ecff, 0x091018, 1.4))
  const sun = new THREE.DirectionalLight(0xffffff, 1.2)
  sun.position.set(40, -30, 90)
  scene.add(sun)

  buildMarker()
  const all = [
    trailLayer.object,
    robotLayer.object,
    tfLayer.object,
    pendingGroup,
    measureGroup,
  ]
  for (const object of all) scene.add(object)

  applyVisibility()
  syncDisplayLayers()
  syncMeasure()
  resize()
  resizeObserver = new ResizeObserver(resize)
  resizeObserver.observe(el)
  renderer.domElement.addEventListener('pointerdown', onPointerDown)
  renderer.domElement.addEventListener('pointermove', onPointerMove)
  renderer.domElement.addEventListener('pointerup', onPointerUp)
  renderer.domElement.addEventListener('pointercancel', onPointerUp)
  renderer.domElement.addEventListener('dblclick', onDoubleClick)
  window.addEventListener('keydown', onKeyDown)
}

function resize(): void {
  const el = host.value
  if (!el || !renderer || !camera) return
  const width = Math.max(1, el.clientWidth)
  const height = Math.max(1, el.clientHeight)
  renderer.setSize(width, height, false)
  renderer.domElement.style.width = `${width}px`
  renderer.domElement.style.height = `${height}px`
  camera.aspect = width / height
  camera.updateProjectionMatrix()
}

// ---------------------------------------------------------------- interactions

function pointerToGround(event: { clientX: number; clientY: number }): THREE.Vector3 | null {
  if (!camera || !renderer) return null
  const rect = renderer.domElement.getBoundingClientRect()
  const ndc = new THREE.Vector2(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1,
  )
  raycaster.setFromCamera(ndc, camera)
  const point = new THREE.Vector3()
  return raycaster.ray.intersectPlane(groundPlane, point) ? point : null
}

let draggingPose = false

function onPointerDown(event: PointerEvent): void {
  if (event.button !== 0) return

  if (state.measure.active) {
    const point = pointerToGround(event)
    if (point) setMeasurePoint(point.x, point.y)
    return
  }

  if (!state.estimating) return
  const point = pointerToGround(event)
  if (!point) return
  draggingPose = true
  if (controls) controls.enabled = false
  setPendingPose({ x: point.x, y: point.y, theta: state.pendingPose?.theta ?? 0 })
  renderer?.domElement.setPointerCapture(event.pointerId)
}

function onPointerMove(event: PointerEvent): void {
  if (!draggingPose || !state.pendingPose) return
  const point = pointerToGround(event)
  if (!point) return
  const dx = point.x - state.pendingPose.x
  const dy = point.y - state.pendingPose.y
  if (Math.hypot(dx, dy) < 0.25) return
  setPendingPose({ ...state.pendingPose, theta: Math.atan2(dy, dx) })
}

function onPointerUp(event: PointerEvent): void {
  if (!draggingPose) return
  draggingPose = false
  if (controls) controls.enabled = true
  renderer?.domElement.releasePointerCapture(event.pointerId)
}

function onDoubleClick(event: MouseEvent): void {
  if (!state.estimating) return
  const point = pointerToGround(event)
  if (point) setPendingPose({ x: point.x, y: point.y, theta: state.pendingPose?.theta ?? 0 })
}

function onKeyDown(event: KeyboardEvent): void {
  const target = event.target as HTMLElement | null
  if (target && (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT')) {
    return
  }
  if (target?.isContentEditable) return

  if (event.key === 'Escape') {
    if (state.estimating) {
      setPendingPose(null)
      if (controls) controls.enabled = true
    }
    if (state.measure.active) clearMeasure()
    return
  }
  if (event.key === 'm' || event.key === 'M') {
    toggleMeasure()
  } else if (event.key === 'f' || event.key === 'F') {
    fitView()
  } else if (event.key === 't' || event.key === 'T') {
    topView()
  }
}

// ---------------------------------------------------------------- views

function contentBox(): THREE.Box3 {
  const box = new THREE.Box3()
  const include = (object: THREE.Object3D) => {
    if (!object.visible) return
    const b = new THREE.Box3().setFromObject(object)
    if (!b.isEmpty()) box.union(b)
  }
  for (const entry of dynamicLayers.values()) include(entry.layer.object)
  if (state.robot.source !== '-') {
    box.expandByPoint(new THREE.Vector3(state.robot.x, state.robot.y, 0))
  }
  return box
}

function fitView(): void {
  if (!camera || !controls) return
  const box = contentBox()
  if (box.isEmpty()) {
    box.setFromCenterAndSize(new THREE.Vector3(), new THREE.Vector3(40, 40, 10))
  }
  const center = box.getCenter(new THREE.Vector3())
  const size = box.getSize(new THREE.Vector3())
  const span = Math.max(size.x, size.y, size.z, 4)
  camera.up.set(0, 0, 1)
  controls.target.copy(center)
  camera.position.copy(center).add(new THREE.Vector3(0.8, -0.9, 0.8).normalize().multiplyScalar(span * 0.85 + 6))
  controls.update()
}

function topView(): void {
  if (!camera || !controls) return
  const box = contentBox()
  const center = box.isEmpty()
    ? new THREE.Vector3()
    : box.getCenter(new THREE.Vector3())
  if (state.robot.source !== '-') center.set(state.robot.x, state.robot.y, 0)
  const size = box.isEmpty() ? new THREE.Vector3(30, 30, 1) : box.getSize(new THREE.Vector3())
  const span = Math.max(size.x, size.y, 20)
  camera.up.set(0, 1, 0)
  controls.target.copy(center)
  camera.position.set(center.x + 0.01, center.y, center.z + span * 1.1)
  controls.update()
}

function screenshot(): void {
  if (!renderer) return
  renderer.render(scene as THREE.Scene, camera as THREE.PerspectiveCamera)
  const url = renderer.domElement.toDataURL('image/png')
  const a = document.createElement('a')
  a.href = url
  a.download = `hmi-${new Date().toISOString().replace(/[:.]/g, '-')}.png`
  a.click()
}

// ---------------------------------------------------------------- animate

function animate(): void {
  raf = requestAnimationFrame(animate)
  if (!renderer || !scene || !camera || !controls) return

  const now = performance.now()
  frameCounter++
  if (now - fpsTime > 500) {
    state.fps = (frameCounter * 1000) / (now - fpsTime)
    frameCounter = 0
    fpsTime = now
  }

  // 机器人位姿：优先 TF map→base_link，回退 /slam/info
  const tf = tfTree.lookup('map', 'base_footprint') ?? tfTree.lookup('map', 'base_link')
  if (tf) {
    const [roll, pitch, yaw] = quatToRpy(tf.q)
    // base_footprint 才是接地点；用它的 z 把模型放到地面，而不是 base_link 车体中心
    robotLayer.setPose(tf.t[0], tf.t[1], yaw, tf.t[2])
    if (floorGrid) {
      const z = tf.t[2] - 0.005
      if (Math.abs(floorGrid.position.z - z) > 0.002) floorGrid.position.z = z
    }
    if (now - lastRobotPublish > 100) {
      lastRobotPublish = now
      state.robot = {
        x: tf.t[0],
        y: tf.t[1],
        z: tf.t[2],
        roll,
        pitch,
        yaw,
        source: tf.child === 'base_footprint' ? 'TF map→base_footprint' : 'TF map→base_link',
        stamp: Date.now(),
      }
    }
  } else if (state.slamTime > 0) {
    const pose = state.slam.robot_pose
    robotLayer.setPose(pose.x, pose.y, pose.theta)
    if (now - lastRobotPublish > 100) {
      lastRobotPublish = now
      state.robot = {
        x: pose.x,
        y: pose.y,
        z: 0,
        roll: 0,
        pitch: 0,
        yaw: pose.theta,
        source: '/slam/info',
        stamp: Date.now(),
      }
    }
  }

  if (state.layers.tf) {
    tfLayer.update(currentFrames(), (frame) => tfTree.lookup('map', frame), true)
  }

  // 轨迹
  if (state.layers.trail && state.robot.source !== '-' && now - lastTrail > 200) {
    if (
      !Number.isFinite(lastRobotPose.x) ||
      Math.hypot(state.robot.x - lastRobotPose.x, state.robot.y - lastRobotPose.y) > 0.15
    ) {
      lastRobotPose = { x: state.robot.x, y: state.robot.y }
      lastTrail = now
      trailLayer.push(state.robot.x, state.robot.y)
    }
  }

  // 首次拿到位姿时把相机挪过去（用户手动操作过就不打扰）
  if (!autoCentered && state.robot.source !== '-') {
    autoCentered = true
    if (!userMoved) {
      const target = new THREE.Vector3(state.robot.x, state.robot.y, 0)
      controls.target.copy(target)
      camera.position.copy(target).add(new THREE.Vector3(-10, -12, 11))
      controls.update()
    }
  }

  // 跟随
  if (state.view.follow && state.robot.source !== '-') {
    const target = new THREE.Vector3(state.robot.x, state.robot.y, 0)
    const delta = target.clone().sub(controls.target).multiplyScalar(0.1)
    if (delta.lengthSq() > 1e-6) {
      camera.position.add(delta)
      controls.target.add(delta)
    }
  }

  controls.update()
  renderer.render(scene, camera)
}

watch(
  () => state.layers,
  () => applyVisibility(),
  { deep: true },
)

watch(
  () =>
    state.displays.map((d) => [
      d.id,
      d.topic,
      d.source,
      d.pcdName,
      d.renderer,
      d.enabled,
      d.pointSize,
      d.color,
      d.colorMode,
    ]),
  () => syncDisplayLayers(),
  { deep: true },
)

watch(
  () => state.pcdVersion,
  () => syncDisplayLayers(),
)

watch(
  () => state.measure,
  () => syncMeasure(),
  { deep: true },
)

watch(
  () => state.pendingPose,
  (pose) => {
    pendingGroup.visible = !!pose
    if (pose) {
      pendingGroup.position.set(pose.x, pose.y, 0.06)
      pendingGroup.rotation.z = pose.theta
    }
  },
  { deep: true },
)

watch(
  () => state.view.fit,
  () => fitView(),
)
watch(
  () => state.view.top,
  () => topView(),
)
watch(
  () => state.view.shot,
  () => screenshot(),
)

onMounted(() => {
  setupScene()
  bindTopics()
  animate()

  void robotLayer
    .loadModel(`${import.meta.env.BASE_URL}models/robot.glb`)
    .then((size) => {
      if (size) {
        pushLog(
          'info',
          `机器人模型已加载：${size.width.toFixed(2)} × ${size.depth.toFixed(2)} × ${size.height.toFixed(2)} m`,
        )
      }
    })
    .catch((e) => pushLog('warn', `机器人模型加载失败，使用简易模型：${(e as Error).message}`))

  // 恢复上次抓取的栅格（43MB / 1~2 分钟，刷新页面不必重来）
  void loadGridCache()
    .then((cached) => {
      if (!cached) return
      lastGrid.set('/map', cached.grid)
      state.gridCells = cached.grid.width * cached.grid.height
      state.gridTime = cached.savedAt
      state.gridFromCache = true
      if (state.displays.some((d) => d.topic === '/map')) applyTopicGrid('/map', cached.grid)
      pushLog(
        'info',
        `已从缓存恢复栅格地图：${cached.grid.width}×${cached.grid.height}` +
          `（${(state.gridCells / 1e6).toFixed(1)}M 格）`,
      )
    })
    .catch(() => {
      /* 没有缓存/被浏览器禁用都忽略 */
    })
})

onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  resizeObserver?.disconnect()
  for (const unsub of unsubs) unsub()
  window.removeEventListener('keydown', onKeyDown)
  for (const topic of [...dynamicLayers.keys()]) disposeDynamic(topic)
  controls?.dispose()
  renderer?.dispose()
  renderer?.domElement.remove()
  renderer = null
  scene = null
  camera = null
  controls = null
})
</script>

<style scoped>
.viewport {
  position: relative;
  width: 100%;
  height: 100%;
  overflow: hidden;
  background: #070b10;
  cursor: grab;
}
.viewport.estimating {
  cursor: crosshair;
}
.viewport.measuring {
  cursor: crosshair;
}

.hud-measure {
  bottom: 30px;
  left: 50%;
  transform: translateX(-50%);
  align-items: center;
  pointer-events: auto;
  z-index: 5;
}
.hud-measure .chip.accent {
  color: #fcd34d;
  border-color: rgba(251, 191, 36, 0.55);
  background: rgba(251, 191, 36, 0.12);
  font-variant-numeric: tabular-nums;
}
.chip-btn {
  padding: 3px 8px;
  border-radius: 4px;
  border: 1px solid #33475a;
  background: #0d151e;
  color: #9fb4c7;
  font-size: 11px;
  cursor: pointer;
}
.chip-btn:hover {
  border-color: #4b657d;
  color: #d7e3ef;
}
.viewport :deep(canvas) {
  display: block;
}

.hud {
  position: absolute;
  display: flex;
  gap: 6px;
  pointer-events: none;
  z-index: 3;
  font-size: 11px;
}
.hud-tl {
  top: 10px;
  left: 12px;
}
.hud-tr {
  top: 10px;
  right: 12px;
  flex-direction: column;
  align-items: flex-end;
}
.hud-bl {
  bottom: 10px;
  left: 12px;
  color: #6b7f93;
  letter-spacing: 0.02em;
}
.chip {
  padding: 3px 8px;
  border-radius: 4px;
  background: rgba(13, 21, 30, 0.82);
  border: 1px solid #1c2b3a;
  color: #9fb4c7;
  font-variant-numeric: tabular-nums;
}
.chip.mono {
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 10px;
  letter-spacing: 0.01em;
}

.estimate-banner {
  position: absolute;
  top: 52px;
  left: 50%;
  transform: translateX(-50%);
  z-index: 4;
  padding: 7px 14px;
  border-radius: 6px;
  background: rgba(120, 78, 6, 0.92);
  border: 1px solid #b7791f;
  color: #fde68a;
  font-size: 12px;
  pointer-events: none;
}

.map-hint {
  position: absolute;
  left: 50%;
  top: 46px;
  transform: translateX(-50%);
  z-index: 4;
  padding: 3px 10px;
  border-radius: 11px;
  border: 1px solid rgba(251, 191, 36, 0.32);
  background: rgba(38, 29, 9, 0.88);
  color: #fcd34d;
  font-size: 10.5px;
  line-height: 1.5;
  white-space: nowrap;
  pointer-events: none;
}
.map-hint code {
  color: #fde68a;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.empty-hint {
  position: absolute;
  left: 50%;
  bottom: 56px;
  transform: translateX(-50%);
  z-index: 4;
  padding: 5px 11px;
  border: 1px dashed #2b3d4f;
  border-radius: 6px;
  background: rgba(8, 13, 19, 0.82);
  color: #7e91a4;
  font-size: 10.5px;
  white-space: nowrap;
  pointer-events: none;
}

.image-preview {
  position: absolute;
  inset: 0;
  z-index: 9;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(2, 6, 10, 0.72);
  cursor: zoom-out;
}

.image-card {
  max-width: 86%;
  max-height: 82%;
  display: flex;
  flex-direction: column;
  gap: 6px;
  padding: 8px;
  border: 1px solid #24455c;
  border-radius: 6px;
  background: #05090d;
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.55);
}

.image-head {
  display: flex;
  align-items: center;
  gap: 12px;
  color: #9fb4c7;
  font-size: 11px;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.image-head button {
  margin-left: auto;
}

.image-card img {
  max-width: 100%;
  max-height: 70vh;
  object-fit: contain;
  background: #000;
}

.view-toolbar {
  position: absolute;
  right: 12px;
  bottom: 12px;
  z-index: 6;
  display: flex;
  gap: 3px;
  padding: 4px;
  border-radius: 8px;
  border: 1px solid #1c2b3a;
  background: rgba(10, 16, 22, 0.84);
  backdrop-filter: blur(4px);
}
.view-toolbar button {
  padding: 4px 9px;
  border: 1px solid transparent;
  border-radius: 5px;
  background: transparent;
  color: #8aa0b4;
  font-size: 10.5px;
  line-height: 1.2;
  cursor: pointer;
}
.view-toolbar button:hover {
  background: #15202c;
  color: #d7e3ef;
}
.view-toolbar button.on {
  color: #7dd3fc;
  border-color: rgba(56, 189, 248, 0.4);
  background: rgba(56, 189, 248, 0.14);
}
</style>
