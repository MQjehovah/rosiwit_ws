<template>
  <div ref="host" class="viewport" :class="{ estimating: state.estimating, measuring: state.measure.active }">
    <div class="hud hud-tl">
      <span class="chip">FPS {{ state.fps.toFixed(0) }}</span>
      <span v-if="scanInfo" class="chip">{{ scanInfo }}</span>
      <span v-if="globalPoints" class="chip">地图 {{ globalPoints }}</span>
      <span v-if="state.layers.grid && gridCells" class="chip">
        栅格 {{ (gridCells / 1e6).toFixed(1) }}M
      </span>
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
    <div
      v-if="globalMapMissing"
      class="map-hint"
      title="设备 debug_publish: false 时 /global_map 不发布；当前帧来自 /scan_f_map（已在 map 系配准）"
    >
      全局地图未发布 · 可在左侧加载 PCD 文件
    </div>
    <div v-if="noSources" class="empty-hint">
      只订阅了 TF · 在左侧「数据源」勾选要看的点云 / 地图
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
  onGlobalMapSnapshot,
  onGridSnapshot,
  onLidarSnapshot,
  pushLog,
  restoreMapCache,
  ros,
  setMeasurePoint,
  setPendingPose,
  state,
  tfTree,
  toggleMeasure,
} from '../ros/app'
import { loadGridCache, saveGridCache } from '../ros/gridCache'
import {
  decodeLaserScan,
  decodePath,
  decodePolygon,
  type CloudData,
} from '../ros/decode'
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

const globalLayer = new PointCloudLayer({ size: 0.1, opacity: 0.9 })
const scanLayer = new PointCloudLayer({ size: 0.07, color: 0xffc247 })
const laserLayer = new PointCloudLayer({ size: 0.12, color: 0xff8a3d })
const points2dLayer = new PointCloudLayer({ size: 0.1, color: 0x34d399 })
const lidarLayer = new PointCloudLayer({ size: 0.06, opacity: 0.95 })
const gridLayer = new GridMapLayer()
const robotLayer = new RobotLayer()
const tfLayer = new TfLayer()
const globalPath = new PathLayer(0x60a5fa, 0.9)
const localPath = new PathLayer(0xf472b6, 0.9)
const trailLayer = new TrailLayer(0x38bdf8)

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

const gridCells = computed(() => state.gridCells || (state.pointCounts.grid ?? 0))

/** 勾了「全局地图」但一条都没收到（通常是 debug_publish=false） */
const globalMapMissing = computed(
  () =>
    state.layers.globalMap &&
    state.status === 'open' &&
    (state.pointCounts.globalMap ?? 0) === 0,
)

/** 除 TF 外没有勾选任何数据源 */
const noSources = computed(() =>
  (
    ['status', 'diag', 'logs', 'scan', 'laser', 'globalMap', 'points2d', 'robot', 'paths', 'lidar', 'grid'] as const
  ).every((key) => !state.layers[key]),
)

const scanInfo = computed(() => {
  const n = state.pointCounts.scan ?? 0
  return n ? `扫描 ${n.toLocaleString()}` : ''
})

const globalPoints = computed(() => {
  const n = state.pointCounts.globalMap ?? 0
  if (!n) return ''
  return n >= 1e6 ? `${(n / 1e6).toFixed(1)}M` : n.toLocaleString()
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
  const tf = tfTree.lookup('map', frame)
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

function applyGlobalMap(cloud: CloudData): void {
  globalLayer.setPoints(cloud.positions, cloud.colors)
  placeInMap(globalLayer.object, cloud.frame)
  state.pointCounts.globalMap = cloud.count
  pushLog(
    'info',
    `全局地图：${cloud.count.toLocaleString()} 点` +
      (cloud.total > cloud.count ? `（原始 ${cloud.total.toLocaleString()}，已抽稀）` : ''),
  )
}

function bindTopics(): void {
  unsubs.push(
    onGlobalMapSnapshot((cloud) => applyGlobalMap(cloud)),
  )

  unsubs.push(
    bridge.onCloud((stream, cloud) => applyBridgeCloud(stream, cloud)),
  )

  unsubs.push(
    ros.onTopic('/scan', (msg) => {
      const cloud = decodeLaserScan(msg)
      if (!cloud) return
      laserLayer.setPoints(cloud.positions, null)
      placeInMap(laserLayer.object, cloud.frame)
      state.pointCounts.laser = cloud.count
    }),
  )

  unsubs.push(
    onLidarSnapshot((cloud) => {
      lidarLayer.setPoints(cloud.positions, cloud.colors)
      placeInMap(lidarLayer.object, cloud.frame)
      state.pointCounts.lidar = cloud.count
    }),
  )

  unsubs.push(
    onGridSnapshot((grid) => {
      gridLayer.update(grid)
      state.pointCounts.grid = grid.width * grid.height
      pushLog('info', `栅格已渲染：${grid.width}×${grid.height} @ ${grid.resolution}m`)
      void saveGridCache(grid).catch(() => {
        /* 缓存失败不影响显示 */
      })
    }),
  )

  unsubs.push(
    ros.onTopic('/global_path', (msg) => {
      const path = decodePath(msg)
      if (!path) return
      globalPath.setPoints(path.positions)
      placeInMap(globalPath.object, path.frame)
    }),
  )

  unsubs.push(
    ros.onTopic('/local_path', (msg) => {
      const path = decodePath(msg)
      if (!path) return
      localPath.setPoints(path.positions)
      placeInMap(localPath.object, path.frame)
    }),
  )

  unsubs.push(
    ros.onTopic('/robot_footprint', (msg) => {
      const poly = decodePolygon(msg)
      if (!poly) return
      if (poly.frame && poly.frame !== 'base_link' && poly.frame !== 'base_footprint') {
        warnOnce('footprint-frame', `/robot_footprint 在 ${poly.frame} 系，未使用`)
        return
      }
      robotLayer.setFootprint(poly.positions)
    }),
  )
}

function applyVisibility(): void {
  globalLayer.setVisible(state.layers.globalMap)
  scanLayer.setVisible(state.layers.scan)
  laserLayer.setVisible(state.layers.laser)
  points2dLayer.setVisible(state.layers.points2d)
  lidarLayer.setVisible(state.layers.lidar)
  gridLayer.setVisible(state.layers.grid)
  globalPath.setVisible(state.layers.paths)
  localPath.setVisible(state.layers.paths)
  // 机器人本体只依赖 TF，常显示；足迹是额外订阅
  robotLayer.setVisible(true)
  robotLayer.setFootprintVisible(state.layers.robot)
  trailLayer.setVisible(state.layers.trail)
  tfLayer.object.visible = state.layers.tf
}

function applyPointScale(): void {
  const s = state.pointScale
  globalLayer.setSize(0.1 * s)
  scanLayer.setSize(0.07 * s)
  laserLayer.setSize(0.12 * s)
  points2dLayer.setSize(0.1 * s)
  lidarLayer.setSize(0.06 * s)
}

// ---------------------------------------------------------------- bridge push

function applyBridgeCloud(stream: string, cloud: CloudData): void {
  if (stream === 'scan') {
    scanLayer.setPoints(cloud.positions, cloud.colors)
    placeInMap(scanLayer.object, cloud.frame)
    state.pointCounts.scan = cloud.count
  } else if (stream === 'points_2d') {
    points2dLayer.setPoints(cloud.positions, cloud.colors)
    placeInMap(points2dLayer.object, cloud.frame)
    state.pointCounts.points2d = cloud.count
  }
}

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
    gridLayer.object,
    globalLayer.object,
    scanLayer.object,
    laserLayer.object,
    points2dLayer.object,
    globalPath.object,
    localPath.object,
    trailLayer.object,
    robotLayer.object,
    tfLayer.object,
    pendingGroup,
    measureGroup,
  ]
  for (const object of all) scene.add(object)

  applyVisibility()
  applyPointScale()
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
  include(globalLayer.object)
  include(scanLayer.object)
  include(points2dLayer.object)
  include(laserLayer.object)
  include(lidarLayer.object)
  include(gridLayer.object)
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
  const tf = tfTree.lookup('map', 'base_link') ?? tfTree.lookup('map', 'base_footprint')
  if (tf) {
    robotLayer.setTransform(tf)
    if (now - lastRobotPublish > 100) {
      lastRobotPublish = now
      const [roll, pitch, yaw] = quatToRpy(tf.q)
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
  () => state.pointScale,
  () => applyPointScale(),
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

  // 恢复上次抓取的栅格（43MB / 1~2 分钟，刷新页面不必重来）
  void loadGridCache()
    .then((cached) => {
      if (!cached) return
      gridLayer.update(cached.grid)
      state.gridCells = cached.grid.width * cached.grid.height
      state.gridTime = cached.savedAt
      state.gridFromCache = true
      pushLog(
        'info',
        `已从缓存恢复栅格地图：${cached.grid.width}×${cached.grid.height}` +
          `（${(state.gridCells / 1e6).toFixed(1)}M 格）`,
      )
    })
    .catch(() => {
      /* 没有缓存/被浏览器禁用都忽略 */
    })

  void restoreMapCache()
})

onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  resizeObserver?.disconnect()
  for (const unsub of unsubs) unsub()
  window.removeEventListener('keydown', onKeyDown)
  globalLayer.dispose()
  scanLayer.dispose()
  laserLayer.dispose()
  points2dLayer.dispose()
  lidarLayer.dispose()
  gridLayer.clear()
  globalPath.dispose()
  localPath.dispose()
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
