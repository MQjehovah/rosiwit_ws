<template>
  <div ref="host" class="viewport" :class="{ estimating: state.estimating }">
    <div class="hud hud-tl">
      <span class="chip">FPS {{ state.fps.toFixed(0) }}</span>
      <span class="chip">点云 {{ totalPoints.toLocaleString() }}</span>
      <span v-if="scanInfo" class="chip">{{ scanInfo }}</span>
      <span v-if="state.layers.grid && gridCells" class="chip">
        栅格 {{ (gridCells / 1e6).toFixed(1) }}M
      </span>
    </div>
    <div class="hud hud-tr">
      <span class="chip">{{ state.robot.source }}</span>
      <span class="chip">
        x {{ state.robot.x.toFixed(2) }} · y {{ state.robot.y.toFixed(2) }} · θ
        {{ ((state.robot.yaw * 180) / Math.PI).toFixed(1) }}°
      </span>
    </div>
    <div class="hud hud-bl">左键旋转 · 右键平移 · 滚轮缩放 · 双击地面放置初始位姿</div>
    <div v-if="state.estimating" class="estimate-banner">
      在地面点击放置位姿，移动鼠标调整朝向，然后在左侧确认发布
    </div>
    <div v-if="globalMapMissing" class="map-hint">
      全局地图 <code>/global_map</code> 未发布（设备 <code>debug_publish: false</code>）；
      当前帧来自 <code>/scan_f_map</code>，已配准在 map 系
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import {
  onGridSnapshot,
  onLidarSnapshot,
  pushLog,
  ros,
  setPendingPose,
  state,
  tfTree,
} from '../ros/app'
import {
  decodeLaserScan,
  decodeOccupancyGrid,
  decodePath,
  decodePointCloud,
  decodePointCloud2,
  decodePolygon,
} from '../ros/decode'
import { quatToYaw } from '../ros/tf'
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

const totalPoints = computed(() => {
  const c = state.pointCounts
  return (c.globalMap ?? 0) + (c.scan ?? 0) + (c.points2d ?? 0) + (c.laser ?? 0) + (c.lidar ?? 0)
})

const gridCells = computed(() => state.gridCells || (state.pointCounts.grid ?? 0))

/** 全局地图点云完全没收到时给一个明确提示（通常是 debug_publish=false） */
const globalMapMissing = computed(
  () => state.status === 'open' && (state.stats['/global_map']?.count ?? 0) === 0,
)

const scanInfo = computed(() => {
  const n = state.pointCounts.scan ?? 0
  return n ? `扫描 ${n.toLocaleString()}` : ''
})

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

let lastFineScan = 0

function bindTopics(): void {
  unsubs.push(
    ros.onTopic('/global_map', (msg) => {
      const cloud = decodePointCloud2(msg, { maxPoints: 1200000, colorMode: 'height' })
      if (!cloud) return
      globalLayer.setPoints(cloud.positions, cloud.colors)
      placeInMap(globalLayer.object, cloud.frame)
      state.pointCounts.globalMap = cloud.count
      pushLog(
        'info',
        `全局地图：${cloud.count.toLocaleString()} 点` +
          (cloud.total > cloud.count ? `（原始 ${cloud.total.toLocaleString()}，已抽稀）` : ''),
      )
    }),
  )

  unsubs.push(
    ros.onTopic('/curr_scan_point', (msg) => {
      const cloud = decodePointCloud2(msg, { maxPoints: 200000, colorMode: 'flat' })
      if (!cloud) return
      lastFineScan = performance.now()
      scanLayer.setPoints(cloud.positions, cloud.colors)
      placeInMap(scanLayer.object, cloud.frame)
      state.pointCounts.scan = cloud.count
    }),
  )

  // /scan_f_map: 旧版 PointCloud，未开 debug_publish 时唯一可用的「当前帧（已配准到 map）」
  unsubs.push(
    ros.onTopic('/scan_f_map', (msg) => {
      if (performance.now() - lastFineScan < 2000) return // 有更精细的 /curr_scan_point 就不用它
      const cloud = decodePointCloud(msg, { maxPoints: 200000, colorMode: 'height' })
      if (!cloud) return
      scanLayer.setPoints(cloud.positions, cloud.colors)
      placeInMap(scanLayer.object, cloud.frame)
      state.pointCounts.scan = cloud.count
      state.pointCounts.scanTotal = cloud.total
    }),
  )

  unsubs.push(
    ros.onTopic('/points_2d', (msg) => {
      const cloud = decodePointCloud2(msg, { maxPoints: 200000, colorMode: 'flat' })
      if (!cloud) return
      points2dLayer.setPoints(cloud.positions, cloud.colors)
      placeInMap(points2dLayer.object, cloud.frame)
      state.pointCounts.points2d = cloud.count
    }),
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
    onLidarSnapshot((msg) => {
      const cloud = decodePointCloud2(msg, { maxPoints: 120000, colorMode: 'height' })
      if (!cloud) {
        pushLog('warn', '雷达快照解码失败（没有有效点）')
        return
      }
      lidarLayer.setPoints(cloud.positions, cloud.colors)
      placeInMap(lidarLayer.object, cloud.frame)
      state.pointCounts.lidar = cloud.count
      state.snapshotPoints = cloud.count
      pushLog('info', `雷达快照：${cloud.count.toLocaleString()} 点（${cloud.frame || '未知坐标系'}）`)
    }),
  )

  unsubs.push(
    onGridSnapshot((msg) => {
      const grid = decodeOccupancyGrid(msg)
      if (!grid) {
        pushLog('warn', '栅格地图解码失败')
        return
      }
      gridLayer.update(grid)
      state.pointCounts.grid = grid.width * grid.height
      pushLog('info', `栅格已渲染：${grid.width}×${grid.height} @ ${grid.resolution}m`)
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
  laserLayer.setVisible(state.layers.scan)
  points2dLayer.setVisible(state.layers.points2d)
  lidarLayer.setVisible(state.layers.lidar)
  gridLayer.setVisible(state.layers.grid)
  globalPath.setVisible(state.layers.paths)
  localPath.setVisible(state.layers.paths)
  robotLayer.setVisible(state.layers.robot)
  trailLayer.setVisible(state.layers.trail)
  tfLayer.object.visible = state.layers.tf
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
  ]
  for (const object of all) scene.add(object)

  applyVisibility()
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
  if (!state.estimating || event.button !== 0) return
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
  if (event.key === 'Escape' && state.estimating) {
    setPendingPose(null)
    if (controls) controls.enabled = true
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
      state.robot = {
        x: tf.t[0],
        y: tf.t[1],
        yaw: quatToYaw(tf.q),
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
  bottom: 34px;
  transform: translateX(-50%);
  z-index: 4;
  max-width: min(760px, 90%);
  padding: 6px 12px;
  border-radius: 6px;
  border: 1px solid rgba(251, 191, 36, 0.35);
  background: rgba(40, 30, 8, 0.85);
  color: #fcd34d;
  font-size: 11px;
  line-height: 1.6;
  text-align: center;
  pointer-events: none;
}
.map-hint code {
  color: #fde68a;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}
</style>
