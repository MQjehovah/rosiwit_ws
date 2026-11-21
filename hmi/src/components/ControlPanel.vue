<template>
  <div class="control-panel">
    <!-- 连接 -->
    <section class="card">
      <h3>rosbridge</h3>
      <div class="row">
        <input v-model="urlInput" class="input grow" spellcheck="false" placeholder="ws://ip:port" />
        <button v-if="state.status !== 'open'" class="btn primary" @click="doConnect">连接</button>
        <button v-else class="btn" @click="doDisconnect">断开</button>
      </div>
      <div class="conn-line">
        <span class="dot" :class="statusClass" />
        <span>{{ statusText }}</span>
        <span class="sep">·</span>
        <span>数据 {{ state.dataAge >= 0 ? ageText(state.dataAge) : '—' }}</span>
        <span class="sep">·</span>
        <span class="dim">8089 为 rosbridge，9090 是厂商 http_bridge</span>
      </div>
      <div v-if="state.error" class="error-text">{{ state.error }}</div>
    </section>

    <!-- 定位状态 -->
    <section class="card">
      <h3>
        定位状态
        <span class="age" v-if="state.slamTime">{{ ((Date.now() - state.slamTime) / 1000).toFixed(1) }}s 前</span>
      </h3>
      <div class="badges">
        <span class="badge" :class="locClass">{{ locText }}</span>
        <span class="badge ghost">{{ modeText }}</span>
        <span class="badge ghost">匹配 {{ percent(state.slam.matching_degree) }}</span>
      </div>

      <div class="meter">
        <div class="meter-fill" :class="locClass" :style="{ width: Math.max(0, Math.min(1, state.slam.matching_degree)) * 100 + '%' }" />
        <div class="meter-mark" title="credible 0.80" />
      </div>

      <dl class="kv">
        <dt>地图</dt>
        <dd>{{ state.slam.map_name || '—' }}</dd>
        <dt>建图面积</dt>
        <dd>{{ state.slam.map_area ? (state.slam.map_area / 10000).toFixed(2) + ' 万 m²' : '—' }}</dd>
        <dt>地图自更新</dt>
        <dd>{{ state.slam.update_request === 1 ? '请求中' : '—' }}</dd>
        <dt>位姿来源</dt>
        <dd>{{ state.robot.source }}</dd>
        <dt>位置</dt>
        <dd>x {{ state.robot.x.toFixed(2) }} · y {{ state.robot.y.toFixed(2) }}</dd>
        <dt>朝向</dt>
        <dd>{{ ((state.robot.yaw * 180) / Math.PI).toFixed(1) }}°</dd>
        <dt>TF 坐标系</dt>
        <dd>{{ state.frames.length }} 个</dd>
      </dl>

      <details v-if="state.fusionDiag" class="diag">
        <summary>融合诊断 /slam/fusion_diag</summary>
        <pre>{{ state.fusionDiag }}</pre>
      </details>
    </section>

    <!-- 图层 -->
    <section class="card">
      <h3>图层</h3>
      <label v-for="layer in layerList" :key="layer.key" class="layer">
        <input
          type="checkbox"
          :checked="state.layers[layer.key]"
          @change="setLayer(layer.key, ($event.target as HTMLInputElement).checked)"
        />
        <span class="swatch" :style="{ background: layer.color }" />
        <span class="layer-name">{{ layer.name }}</span>
        <span class="layer-topic">{{ layer.topic }}</span>
        <span v-if="layer.key === 'grid' && state.gridCells" class="layer-count">
          {{ (state.gridCells / 1e6).toFixed(1) }}M
        </span>
      </label>
      <p class="note">
        点云类话题只在图层打开时订阅。大消息走 rosbridge 的 <code>cbor</code>
        二进制编码，避免 JSON 膨胀。
      </p>
    </section>

    <!-- 数据 -->
    <section class="card">
      <h3>地图 / 点云</h3>
      <div class="row">
        <button class="btn small" :disabled="state.status !== 'open'" @click="doReloadMap">
          重新加载全局地图
        </button>
        <button class="btn small" :disabled="state.status !== 'open'" @click="doGrabLidar">
          {{ state.snapshotPending ? '抓取中…' : '抓一帧雷达' }}
        </button>
      </div>
      <div class="row">
        <button
          class="btn small"
          :disabled="state.status !== 'open' || state.gridPending"
          @click="doGrabGrid"
        >
          {{ state.gridPending ? '接收中（约 1~2 分钟）…' : '抓取栅格地图' }}
        </button>
        <span class="inline-info">{{ gridInfo }}</span>
      </div>

      <p class="note">
        <code>/global_map</code> 由设备 <code>debug_publish</code> 控制；
        关闭时只有 <code>/scan_f_map</code>（当前帧，已在 map 系配准）。
        <code>/map</code> 单帧 43MB，只按需抓一份快照。
      </p>
      <div class="row">
        <button class="btn tiny" @click="copyDebugCmd">复制开启 debug_publish 的命令</button>
      </div>
    </section>

    <!-- 初始位姿 -->
    <section class="card">
      <h3>初始位姿</h3>
      <div class="row">
        <button class="btn" :class="{ primary: !state.estimating }" :disabled="state.status !== 'open'" @click="toggleEstimate">
          {{ state.estimating ? '取消点选' : '在地图上点选' }}
        </button>
        <button class="btn" :disabled="!draftValid" @click="publishPose">发布 /initialpose</button>
      </div>
      <div class="grid3">
        <label>x<input v-model.number="draft.x" class="input" type="number" step="0.1" /></label>
        <label>y<input v-model.number="draft.y" class="input" type="number" step="0.1" /></label>
        <label>yaw°<input v-model.number="draftTheta" class="input" type="number" step="1" /></label>
      </div>
      <div class="row">
        <button class="btn small" :disabled="!draftValid" @click="applyDraft">应用到地图标记</button>
        <button class="btn small" :disabled="!draftValid || state.servicePending" @click="doRelocate">
          调用 /relocation
        </button>
      </div>
      <p class="note">
        <code>/initialpose</code> 走话题（App 同款路径）；<code>/relocation</code> 走服务，需要地图名。
      </p>
    </section>

    <!-- 控制 -->
    <section class="card">
      <h3>控制</h3>
      <div class="row">
        <button class="btn" :disabled="busy" @click="doMode(1)">切到定位</button>
        <button class="btn" :disabled="busy" @click="doMode(0)">切到建图</button>
      </div>
      <div class="row">
        <button class="btn warn" :disabled="busy" @click="doStopMap">结束建图</button>
        <button class="btn" :disabled="busy" @click="doDump">dump recorder</button>
      </div>
      <p class="note">
        建图模式会重建地图；结束建图会保存当前地图。按钮执行前都会二次确认。
      </p>
      <div v-if="state.lastService" class="service-result">{{ state.lastService }}</div>
    </section>

    <!-- 视图 -->
    <section class="card">
      <h3>视图</h3>
      <div class="row">
        <button class="btn small" @click="viewTop">顶视</button>
        <button class="btn small" @click="viewFit">适配内容</button>
        <button class="btn small" :class="{ primary: state.view.follow }" @click="toggleFollow">
          {{ state.view.follow ? '停止跟随' : '跟随机器人' }}
        </button>
        <button class="btn small" @click="viewShot">截图</button>
      </div>
    </section>
  </div>
</template>

<script setup lang="ts">
import { computed, reactive, ref, watch } from 'vue'
import {
  connect,
  disconnect,
  dumpRecorder,
  grabGridSnapshot,
  grabLidarSnapshot,
  publishInitialPose,
  pushLog,
  relocate,
  reloadGlobalMap,
  setLayer,
  setPendingPose,
  state,
  stopMapOperation,
  switchSlamMode,
  viewFit,
  viewShot,
  viewTop,
  type LayerKey,
} from '../ros/app'

const urlInput = ref(state.url)

const layerList: Array<{ key: LayerKey; name: string; topic: string; color: string }> = [
  { key: 'globalMap', name: '全局地图点云', topic: '/global_map', color: '#38bdf8' },
  { key: 'scan', name: '当前帧（已配准）', topic: '/scan_f_map · /curr_scan_point', color: '#fbbf24' },
  { key: 'points2d', name: '2D 点云', topic: '/points_2d', color: '#34d399' },
  { key: 'lidar', name: '雷达原始帧（快照）', topic: '/vanjee_points722', color: '#f87171' },
  { key: 'grid', name: '栅格地图（快照）', topic: '/map', color: '#a5c9e8' },
  { key: 'paths', name: '路径', topic: '/global_path · /local_path', color: '#818cf8' },
  { key: 'robot', name: '机器人', topic: 'TF · /robot_footprint', color: '#22d3ee' },
  { key: 'tf', name: 'TF 坐标轴', topic: '/tf', color: '#94a3b8' },
  { key: 'trail', name: '行驶轨迹', topic: '本地累积', color: '#60a5fa' },
]

const statusText = computed(() => {
  const map: Record<string, string> = {
    idle: '未连接',
    connecting: '连接中…',
    open: '已连接',
    closed: '已断开',
    error: '连接失败',
  }
  return map[state.status] ?? state.status
})

const statusClass = computed(() => ({
  ok: state.status === 'open',
  warn: state.status === 'connecting' || state.status === 'idle',
  bad: state.status === 'error' || state.status === 'closed',
}))

const locText = computed(
  () => ['定位正常', '恢复中', '定位失败'][state.slam.location_status] ?? '无数据',
)
const locClass = computed(() => ({
  ok: state.slam.location_status === 0,
  warn: state.slam.location_status === 1,
  bad: state.slam.location_status === 2,
}))
const modeText = computed(() => {
  const map: Record<number, string> = { 0: '建图中', 1: '定位模式', 2: '错误' }
  return map[state.slam.slam_state] ?? '模式未知'
})

const busy = computed(() => state.servicePending || state.status !== 'open')

const gridInfo = computed(() => {
  if (!state.gridTime) return ''
  return `${(state.gridCells / 1e6).toFixed(1)}M 格 · ${ageText(Date.now() - state.gridTime)}前`
})

const draft = reactive({ x: 0, y: 0 })
const draftTheta = ref(0)

const draftValid = computed(
  () => Number.isFinite(draft.x) && Number.isFinite(draft.y) && Number.isFinite(draftTheta.value),
)

watch(
  () => state.pendingPose,
  (pose) => {
    if (!pose) return
    draft.x = Math.round(pose.x * 100) / 100
    draft.y = Math.round(pose.y * 100) / 100
    draftTheta.value = Math.round((pose.theta * 180) / Math.PI)
  },
  { deep: true },
)

function percent(value: number): string {
  return `${(Math.max(0, Math.min(1, value || 0)) * 100).toFixed(0)}%`
}

function ageText(age: number): string {
  if (age < 0) return '—'
  if (age < 1000) return `${age}ms`
  return `${(age / 1000).toFixed(1)}s`
}

function doConnect(): void {
  connect(urlInput.value)
}

function doDisconnect(): void {
  disconnect()
}

function toggleEstimate(): void {
  if (state.estimating) {
    setPendingPose(null)
  } else {
    state.estimating = true
    pushLog('info', '在地图上点击放置初始位姿（Esc 取消）')
  }
}

function applyDraft(): void {
  if (!draftValid.value) return
  setPendingPose({ x: draft.x, y: draft.y, theta: (draftTheta.value * Math.PI) / 180 })
}

function publishPose(): void {
  if (!draftValid.value) return
  publishInitialPose(draft.x, draft.y, (draftTheta.value * Math.PI) / 180)
  setPendingPose(null)
}

async function doRelocate(): Promise<void> {
  if (!draftValid.value) return
  const name = state.slam.map_name
  if (!name) {
    pushLog('warn', '没有地图名（/slam/info 未给出 map_name），无法调用 /relocation')
    return
  }
  if (!window.confirm(`调用 /relocation：地图 ${name}\nx=${draft.x} y=${draft.y} yaw=${draftTheta.value}°`)) return
  try {
    await relocate(name, draft.x, draft.y, (draftTheta.value * Math.PI) / 180)
    setPendingPose(null)
  } catch {
    /* 已写入日志 */
  }
}

async function doMode(mode: 0 | 1): Promise<void> {
  const text = mode === 1 ? '切换到定位模式？' : '切换到建图模式？（会开始新的建图）'
  if (!window.confirm(text)) return
  try {
    await switchSlamMode(mode)
  } catch {
    /* 已写入日志 */
  }
}

async function doStopMap(): Promise<void> {
  const name = state.slam.map_name || 'map'
  if (!window.confirm(`结束建图并保存为「${name}」？`)) return
  try {
    await stopMapOperation(name)
  } catch {
    /* 已写入日志 */
  }
}

async function doDump(): Promise<void> {
  try {
    await dumpRecorder()
  } catch {
    /* 已写入日志 */
  }
}

function doReloadMap(): void {
  reloadGlobalMap()
}

async function doGrabLidar(): Promise<void> {
  await grabLidarSnapshot()
}

async function doGrabGrid(): Promise<void> {
  const ok = window.confirm(
    '/map 单帧约 43MB（4300 万格），走 CBOR 二进制接收。\n' +
      '设备端要逐个打包这些格子，实测约 1~2 分钟；期间实时数据会暂停。\n' +
      '确定要抓取吗？',
  )
  if (!ok) return
  await grabGridSnapshot()
}

function toggleFollow(): void {
  state.view.follow = !state.view.follow
}

async function copyDebugCmd(): Promise<void> {
  const cmd =
    "sudo sed -i 's/debug_publish: false/debug_publish: true/' /opt/ros/melodic/share/rosiwit_slam_ros1/config/default.yaml && sudo systemctl restart xzrobot"
  try {
    await navigator.clipboard.writeText(cmd)
    pushLog('info', '已复制：开启 debug_publish 的命令')
  } catch {
    pushLog('warn', cmd)
  }
}
</script>

<style scoped>
.control-panel {
  display: flex;
  flex-direction: column;
  gap: 10px;
  padding: 10px;
}

.card {
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--panel);
  padding: 10px 11px 11px;
}
.card h3 {
  margin: 0 0 8px;
  font-size: 11px;
  font-weight: 600;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  color: var(--dim);
  display: flex;
  justify-content: space-between;
  align-items: baseline;
}
.age {
  font-weight: 400;
  letter-spacing: 0;
  text-transform: none;
  color: var(--dim);
}

.row {
  display: flex;
  gap: 6px;
  align-items: center;
  margin-top: 6px;
}
.row:first-of-type {
  margin-top: 0;
}
.grow {
  flex: 1;
  min-width: 0;
}

.conn-line {
  margin-top: 8px;
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 11px;
  color: var(--dim);
  flex-wrap: wrap;
}
.conn-line .dim {
  font-size: 10px;
  opacity: 0.75;
}
.sep {
  opacity: 0.4;
}

.dot {
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: var(--dim);
  box-shadow: 0 0 0 3px rgba(125, 142, 161, 0.12);
}
.dot.ok {
  background: var(--ok);
  box-shadow: 0 0 0 3px rgba(52, 211, 153, 0.16);
}
.dot.warn {
  background: var(--warn);
  box-shadow: 0 0 0 3px rgba(251, 191, 36, 0.16);
}
.dot.bad {
  background: var(--bad);
  box-shadow: 0 0 0 3px rgba(248, 113, 113, 0.16);
}

.error-text {
  margin-top: 8px;
  font-size: 11px;
  color: var(--bad);
  word-break: break-all;
  line-height: 1.5;
}

.badges {
  display: flex;
  gap: 6px;
  flex-wrap: wrap;
}
.badge {
  font-size: 11px;
  padding: 2px 8px;
  border-radius: 4px;
  border: 1px solid var(--line);
  color: var(--text);
  background: #0e1620;
}
.badge.ok {
  color: #6ee7b7;
  border-color: rgba(52, 211, 153, 0.45);
  background: rgba(52, 211, 153, 0.1);
}
.badge.warn {
  color: #fcd34d;
  border-color: rgba(251, 191, 36, 0.45);
  background: rgba(251, 191, 36, 0.1);
}
.badge.bad {
  color: #fca5a5;
  border-color: rgba(248, 113, 113, 0.5);
  background: rgba(248, 113, 113, 0.12);
}
.badge.ghost {
  color: var(--dim);
}

.meter {
  position: relative;
  height: 6px;
  margin: 9px 0 10px;
  border-radius: 3px;
  background: #0c141d;
  overflow: hidden;
  border: 1px solid #17222e;
}
.meter-fill {
  height: 100%;
  background: var(--accent);
  transition: width 0.2s linear;
}
.meter-fill.ok {
  background: var(--ok);
}
.meter-fill.warn {
  background: var(--warn);
}
.meter-fill.bad {
  background: var(--bad);
}
.meter-mark {
  position: absolute;
  left: 80%;
  top: -1px;
  width: 1px;
  height: 8px;
  background: rgba(255, 255, 255, 0.45);
}

.kv {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 3px 10px;
  margin: 0;
  font-size: 11px;
}
.kv dt {
  color: var(--dim);
}
.kv dd {
  margin: 0;
  color: var(--text);
  font-variant-numeric: tabular-nums;
  text-align: right;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.diag {
  margin-top: 8px;
  border-top: 1px solid var(--line);
  padding-top: 6px;
}
.diag summary {
  cursor: pointer;
  font-size: 11px;
  color: var(--dim);
}
.diag pre {
  margin: 6px 0 0;
  max-height: 140px;
  overflow: auto;
  font-size: 10px;
  line-height: 1.5;
  color: #9fb4c7;
  white-space: pre-wrap;
  word-break: break-all;
}

.layer {
  display: flex;
  align-items: center;
  gap: 7px;
  padding: 3px 0;
  font-size: 12px;
  cursor: pointer;
}
.layer input {
  accent-color: var(--accent);
  margin: 0;
}
.swatch {
  width: 8px;
  height: 8px;
  border-radius: 2px;
  flex: none;
}
.layer-name {
  color: var(--text);
}
.layer-topic {
  margin-left: auto;
  font-size: 10px;
  color: var(--dim);
  max-width: 52%;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.layer-count {
  font-size: 10px;
  color: var(--accent);
}

.inline-info {
  font-size: 10px;
  color: var(--dim);
  font-variant-numeric: tabular-nums;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.note {
  margin: 8px 0 0;
  font-size: 10.5px;
  line-height: 1.55;
  color: var(--dim);
}
.note code {
  color: #7dd3fc;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.grid3 {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  gap: 6px;
  margin-top: 8px;
}
.grid3 label {
  display: flex;
  flex-direction: column;
  gap: 3px;
  font-size: 10px;
  color: var(--dim);
}

.service-result {
  margin-top: 8px;
  font-size: 10.5px;
  color: #9fb4c7;
  background: #0c141d;
  border: 1px solid #17222e;
  border-radius: 4px;
  padding: 6px 7px;
  word-break: break-all;
  line-height: 1.5;
  max-height: 80px;
  overflow: auto;
}
</style>
