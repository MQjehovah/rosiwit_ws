<template>
  <div class="control-panel">
    <!-- 定位状态 -->
    <details class="sec" open>
      <summary>
        <span class="sec-title">定位状态</span>
        <span class="sec-meta">{{ slamAgeText }}</span>
      </summary>
      <div class="sec-body">
        <div v-if="!state.layers.status" class="prompt">
          未订阅 <code>/slam/info</code>
          <button class="mini" @click="setLayer('status', true)">订阅</button>
        </div>
        <template v-else>
          <div class="badges">
            <span class="badge" :class="locClass">{{ state.statusText || locText }}</span>
            <span class="badge ghost">匹配 {{ percent(state.slam.matching_degree) }}</span>
          </div>

          <div class="meter">
            <div
              class="meter-fill"
              :class="locClass"
              :style="{ width: Math.max(0, Math.min(1, state.slam.matching_degree)) * 100 + '%' }"
            />
            <div class="meter-mark" title="credible 0.80" />
          </div>

          <dl class="kv2">
            <dt>地图</dt>
            <dd>{{ state.slam.map_name || '—' }}</dd>
            <dt>面积</dt>
            <dd>{{ state.slam.map_area ? (state.slam.map_area / 10000).toFixed(2) + ' 万 m²' : '—' }}</dd>
            <dt>TF</dt>
            <dd>{{ state.frames.length }} 帧</dd>
            <dt>来源</dt>
            <dd>{{ shortSource }}</dd>
          </dl>

          <div class="pose" :title="`位姿来源：${state.robot.source}`">
            <div>
              <span class="k">xyz</span>
              <span class="v">
                {{ state.robot.x.toFixed(2) }} {{ state.robot.y.toFixed(2) }}
                {{ state.robot.z.toFixed(2) }}
              </span>
              <span class="u">m</span>
            </div>
            <div>
              <span class="k">rpy</span>
              <span class="v">
                {{ deg(state.robot.roll) }} {{ deg(state.robot.pitch) }} {{ deg(state.robot.yaw) }}
              </span>
              <span class="u">°</span>
            </div>
          </div>

          <details class="sub">
            <summary>定位状态插件</summary>
            <p class="note">
              直接消费 <code>/slam/info</code>：同时兼容数字枚举与字符串，前端翻译成可读状态。
              当前：{{ state.statusText || '等待数据' }}
            </p>
            <pre class="diag-pre">{{ slamRaw }}</pre>
          </details>
        </template>
      </div>
    </details>

    <!-- 显示项（RViz 风格：任意话题动态添加） -->
    <details class="sec" open>
      <summary>
        <span class="sec-title">显示项</span>
        <span class="sec-meta">{{ activeDisplays }} / {{ state.displays.length }}</span>
      </summary>
      <div class="sec-body">
        <p class="source-hint">
          像 RViz 一样按需订阅任意话题。后端只负责订阅 / 压缩 / 传输（点云、栅格、激光、路径、JSON），
          渲染参数由这里控制。
        </p>

        <div class="row add-topic">
          <select v-model="newTopic" class="select">
            <option value="">选择话题…</option>
            <option v-for="t in state.topics" :key="t.name" :value="t.name">
              {{ t.name }} · {{ shortType(t.type) }}
            </option>
          </select>
          <button class="btn small" :disabled="!newTopic" @click="doAddDisplay">添加</button>
          <button class="btn tiny" :disabled="state.status !== 'open' || state.topicScanning" @click="doRefreshTopics">
            {{ state.topicScanning ? '…' : '刷新' }}
          </button>
        </div>
        <p v-if="state.displayError" class="error-text">{{ state.displayError }}</p>

        <div v-if="!state.displays.length" class="prompt">
          还没有显示项。先刷新话题列表，再选择要看的点云 / 栅格 / 激光…
        </div>

        <div v-for="d in state.displays" :key="d.id" class="display-item">
          <div class="display-head">
            <label class="layer" :title="d.type">
              <input
                type="checkbox"
                :checked="d.enabled"
                @change="toggleDisplay(d.id)"
              />
              <span class="swatch" :style="{ background: d.color }" />
              <span class="layer-name">{{ d.topic }}</span>
            </label>
            <span class="type-tag">{{ shortType(d.type) }}</span>
            <button class="mini danger" title="移除" @click="removeDisplay(d.id)">×</button>
          </div>

          <div class="display-controls">
            <label>
              渲染
              <select
                class="select mini-select"
                :value="d.renderer"
                @change="updateDisplay(d.id, { renderer: ($event.target as HTMLSelectElement).value as any })"
              >
                <option v-for="r in rendererOptions(d)" :key="r" :value="r">{{ r }}</option>
              </select>
            </label>
            <label v-if="d.renderer === 'points' || d.renderer === 'laser'">
              点大小
              <input
                class="input tiny-input"
                type="number"
                min="0.01"
                max="1"
                step="0.01"
                :value="d.pointSize"
                @change="updateDisplay(d.id, { pointSize: Number(($event.target as HTMLInputElement).value) || 0.07 })"
              />
            </label>
            <label v-if="d.renderer !== 'json'">
              颜色
              <input
                class="color-input"
                type="color"
                :value="d.color"
                @input="updateDisplay(d.id, { color: ($event.target as HTMLInputElement).value })"
              />
            </label>
            <label v-if="d.renderer === 'points'">
              着色
              <select
                class="select mini-select"
                :value="d.colorMode"
                @change="updateDisplay(d.id, { colorMode: ($event.target as HTMLSelectElement).value as any })"
              >
                <option value="height">高度</option>
                <option value="flat">固定</option>
              </select>
            </label>
            <label v-if="d.renderer === 'points'">
              体素
              <input
                class="input tiny-input"
                type="number"
                min="0"
                max="1"
                step="0.01"
                :value="d.voxel"
                @change="updateDisplay(d.id, { voxel: Number(($event.target as HTMLInputElement).value) || 0 })"
              />
            </label>
            <label v-if="d.renderer === 'points'">
              上限
              <input
                class="input tiny-input"
                type="number"
                min="0"
                step="10000"
                :value="d.max"
                @change="updateDisplay(d.id, { max: Number(($event.target as HTMLInputElement).value) || 0 })"
              />
            </label>
            <label>
              频率
              <select
                class="select mini-select"
                :value="d.rate"
                @change="updateDisplay(d.id, { rate: Number(($event.target as HTMLSelectElement).value) || 2 })"
              >
                <option :value="1">1Hz</option>
                <option :value="2">2Hz</option>
                <option :value="5">5Hz</option>
                <option :value="10">10Hz</option>
                <option :value="20">20Hz</option>
              </select>
            </label>
            <button class="mini" :disabled="state.status !== 'open'" @click="grabTopic(d.id)">抓取</button>
          </div>

          <div class="display-stats">
            <span>{{ d.stats.hz.toFixed(1) }} Hz</span>
            <span>{{ formatBytes(d.stats.bytes) }}/帧</span>
            <span>累计 {{ formatBytes(d.stats.total) }}</span>
            <span v-if="d.stats.age >= 0">{{ (d.stats.age / 1000).toFixed(1) }}s 前</span>
            <span v-if="state.pointCounts[d.topic]">
              {{ state.pointCounts[d.topic].toLocaleString() }} 点/格
            </span>
          </div>
        </div>

        <div class="row map-file">
          <label class="btn small file-btn" :class="{ disabled: state.mapLoading }">
            {{ state.mapLoading ? '解析中…' : '加载 PCD 地图' }}
            <input type="file" accept=".pcd" :disabled="state.mapLoading" @change="onMapFile" />
          </label>
          <span v-if="state.loadedMapName" class="inline-info" :title="state.loadedMapName">
            {{ state.loadedMapName }} · {{ state.loadedMapPoints.toLocaleString() }} 点
          </span>
        </div>
        <p v-if="state.mapError" class="error-text">{{ state.mapError }}</p>

        <div class="foot">
          <label class="slider-row">
            <span>点大小倍率</span>
            <input
              type="range"
              min="0.4"
              max="3"
              step="0.1"
              :value="state.pointScale"
              @input="onPointScale(($event.target as HTMLInputElement).valueAsNumber)"
            />
            <span class="mono">{{ state.pointScale.toFixed(1) }}×</span>
          </label>
          <span v-if="gridInfo" class="dim" :title="state.gridFromCache ? '来自本地缓存' : '本会话抓取'">
            {{ gridInfo }}
          </span>
        </div>
      </div>
    </details>

    <!-- 初始位姿 -->
    <details class="sec">
      <summary>
        <span class="sec-title">初始位姿</span>
        <span class="sec-meta">{{ poseSummary }}</span>
      </summary>
      <div class="sec-body">
        <div class="row">
          <button
            class="btn"
            :class="{ primary: state.estimating }"
            :disabled="state.status !== 'open'"
            @click="toggleEstimate"
          >
            {{ state.estimating ? '取消点选' : '地图上点选' }}
          </button>
          <button class="btn" :disabled="!draftValid" @click="publishPose">发布 /initialpose</button>
        </div>
        <div class="grid3">
          <label>x<input v-model.number="draft.x" class="input" type="number" step="0.1" /></label>
          <label>y<input v-model.number="draft.y" class="input" type="number" step="0.1" /></label>
          <label>yaw°<input v-model.number="draftTheta" class="input" type="number" step="1" /></label>
        </div>
        <div class="row">
          <button class="btn small" :disabled="!draftValid" @click="applyDraft">应用到标记</button>
          <button class="btn small" :disabled="!draftValid || state.servicePending" @click="doRelocate">
            调用 /relocation
          </button>
        </div>
        <p class="note">点选后拖鼠标调朝向；<code>/initialpose</code> 走话题，<code>/relocation</code> 需要地图名。</p>
      </div>
    </details>

    <!-- 控制 -->
    <details class="sec">
      <summary>
        <span class="sec-title">控制</span>
        <span class="sec-meta">{{ busy ? '执行中…' : state.statusText }}</span>
      </summary>
      <div class="sec-body">
        <div class="grid2">
          <button class="btn" :disabled="busy" @click="doMode(1)">切到定位</button>
          <button class="btn" :disabled="busy" @click="doMode(0)">切到建图</button>
          <button class="btn warn" :disabled="busy" @click="doStopMap">结束建图</button>
          <button class="btn" :disabled="busy" @click="doDump">dump recorder</button>
        </div>
        <p class="note">建图模式会重建地图；结束建图会保存当前地图。都会二次确认。</p>
        <div v-if="state.lastService" class="service-result" :title="state.lastService">
          {{ state.lastService }}
        </div>
      </div>
    </details>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'
import {
  addDisplay,
  dumpRecorder,
  grabTopic,
  loadMapFile,
  publishInitialPose,
  pushLog,
  refreshTopics,
  relocate,
  removeDisplay,
  setLayer,
  setPendingPose,
  setPointScale,
  state,
  stopMapOperation,
  switchSlamMode,
  toggleDisplay,
  updateDisplay,
  type DisplayItem,
  type RendererKind,
} from '../ros/app'

const newTopic = ref('')

function doAddDisplay(): void {
  if (!newTopic.value) return
  addDisplay(newTopic.value)
  newTopic.value = ''
}

function doRefreshTopics(): void {
  void refreshTopics()
}

function shortType(type: string): string {
  return type ? type.split('/').pop() || type : '?'
}

function rendererOptions(d: DisplayItem): RendererKind[] {
  const base: RendererKind[] = ['json']
  if (d.type.includes('PointCloud')) return ['points', 'json']
  if (d.type.includes('OccupancyGrid')) return ['grid', 'json']
  if (d.type.includes('LaserScan')) return ['laser', 'json']
  if (d.type.endsWith('/Path')) return ['path', 'json']
  if (d.type.includes('Polygon')) return ['polygon', 'json']
  if (d.type.includes('SlamInfo')) return ['status', 'json']
  // 类型未知时允许手动选择，方便接入自定义消息
  return ['points', 'grid', 'laser', 'path', 'polygon', ...base, 'status']
}

function formatBytes(n: number): string {
  if (!n) return '0 B'
  if (n >= 1024 * 1024) return `${(n / 1024 / 1024).toFixed(1)} MB`
  if (n >= 1024) return `${(n / 1024).toFixed(1)} KB`
  return `${n} B`
}

const slamRaw = computed(() =>
  JSON.stringify(
    {
      slam_state: state.slam.slam_state,
      location_status: state.slam.location_status,
      matching_degree: state.slam.matching_degree,
      update_request: state.slam.update_request,
      map_name: state.slam.map_name,
      map_area: state.slam.map_area,
      robot_pose: state.slam.robot_pose,
    },
    null,
    2,
  ),
)

const locText = computed(
  () => ['定位正常', '恢复中', '定位失败'][state.slam.location_status] ?? '无数据',
)
const locClass = computed(() => ({
  ok: state.slam.location_status === 0,
  warn: state.slam.location_status === 1,
  bad: state.slam.location_status === 2,
}))

const busy = computed(() => state.servicePending || state.status !== 'open')
const activeDisplays = computed(() => state.displays.filter((d) => d.enabled).length)
const shortSource = computed(() => state.robot.source.replace('TF map→', 'TF '))

const nowTick = ref(Date.now())
let tickTimer: number | null = null
onMounted(() => {
  tickTimer = window.setInterval(() => {
    nowTick.value = Date.now()
  }, 500)
})
onBeforeUnmount(() => {
  if (tickTimer !== null) clearInterval(tickTimer)
})

const slamAgeText = computed(() => {
  if (!state.layers.status) return '未订阅'
  if (!state.slamTime) return '等待数据…'
  return `${((nowTick.value - state.slamTime) / 1000).toFixed(1)}s 前`
})

const gridInfo = computed(() => {
  if (!state.gridTime) return ''
  return `${(state.gridCells / 1e6).toFixed(1)}M 格 · ${ageText(Date.now() - state.gridTime)}前`
})

const poseSummary = computed(() => {
  const pose = state.pendingPose
  if (!pose) return state.estimating ? '点选中' : '未选点'
  return `x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · ${((pose.theta * 180) / Math.PI).toFixed(0)}°`
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

function deg(rad: number): string {
  return ((rad * 180) / Math.PI).toFixed(1)
}

function ageText(age: number): string {
  if (age < 0) return '—'
  if (age < 1000) return `${age}ms`
  return `${(age / 1000).toFixed(1)}s`
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

async function onMapFile(event: Event): Promise<void> {
  const input = event.target as HTMLInputElement
  const file = input.files?.[0]
  if (!file) return
  try {
    await loadMapFile(file)
  } finally {
    input.value = ''
  }
}

function onPointScale(value: number): void {
  if (Number.isFinite(value)) setPointScale(value)
}
</script>

<style scoped>
.control-panel {
  display: flex;
  flex-direction: column;
  padding: 2px 0 12px;
}

.sec {
  border-bottom: 1px solid var(--line);
}
.sec > summary {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 9px 12px;
  cursor: pointer;
  user-select: none;
  list-style: none;
  color: var(--dim);
}
.sec > summary::-webkit-details-marker {
  display: none;
}
.sec > summary::before {
  content: '▸';
  font-size: 9px;
  color: #4f6375;
  transition: transform 0.12s ease;
}
.sec[open] > summary::before {
  transform: rotate(90deg);
}
.sec > summary:hover {
  background: rgba(56, 189, 248, 0.04);
}
.sec-title {
  font-size: 11px;
  font-weight: 600;
  letter-spacing: 0.06em;
  color: #a9bccd;
}
.sec-meta {
  margin-left: auto;
  font-size: 10px;
  color: var(--dim);
  font-variant-numeric: tabular-nums;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  max-width: 55%;
}
.sec-body {
  padding: 0 12px 11px;
}

.prompt {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 7px 8px;
  border: 1px dashed #2a3d4f;
  border-radius: 5px;
  background: #0b141d;
  font-size: 10.5px;
  color: #7e91a4;
}
.prompt code {
  color: #7dd3fc;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}
.prompt .mini {
  margin-left: auto;
}

.source-hint {
  margin: 0 0 8px;
  color: var(--dim);
  font-size: 11px;
  line-height: 1.5;
}

.add-topic {
  display: flex;
  gap: 6px;
  align-items: center;
}

.select {
  flex: 1;
  min-width: 0;
  background: #0b141d;
  border: 1px solid var(--line);
  color: var(--text);
  border-radius: 4px;
  padding: 4px 6px;
  font-size: 11px;
}

.mini-select {
  flex: none;
  width: 74px;
  padding: 2px 4px;
}

.tiny-input {
  width: 62px;
  padding: 2px 4px;
}

.color-input {
  width: 28px;
  height: 22px;
  padding: 0;
  border: 1px solid var(--line);
  background: transparent;
  border-radius: 4px;
}

.display-item {
  border: 1px solid var(--line);
  border-radius: 6px;
  padding: 6px 8px;
  margin-bottom: 7px;
  background: rgba(11, 20, 29, 0.55);
}

.display-head {
  display: flex;
  align-items: center;
  gap: 6px;
}

.display-head .layer {
  flex: 1;
  min-width: 0;
}

.type-tag {
  color: var(--dim);
  font-size: 10px;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  white-space: nowrap;
}

.display-controls {
  display: flex;
  flex-wrap: wrap;
  gap: 6px 10px;
  margin-top: 6px;
  align-items: center;
  color: var(--dim);
  font-size: 10.5px;
}

.display-controls label {
  display: flex;
  align-items: center;
  gap: 4px;
}

.display-stats {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  margin-top: 5px;
  color: #7f97ab;
  font-size: 10px;
  font-variant-numeric: tabular-nums;
}

.mini.danger {
  color: #fca5a5;
}

.diag-pre {
  max-height: 180px;
  overflow: auto;
  margin: 6px 0 0;
  padding: 6px 8px;
  background: #080f16;
  border: 1px solid var(--line);
  border-radius: 4px;
  color: #9fb4c7;
  font-size: 10.5px;
  line-height: 1.45;
}

.map-file {
  margin-top: 8px;
}
.file-btn {
  position: relative;
  overflow: hidden;
  cursor: pointer;
}
.file-btn.disabled {
  opacity: 0.5;
  pointer-events: none;
}
.file-btn input {
  position: absolute;
  inset: 0;
  opacity: 0;
  cursor: pointer;
}
.error-text {
  margin: 6px 0 0;
  font-size: 10px;
  line-height: 1.5;
  color: var(--bad);
  word-break: break-all;
}

.badges {
  display: flex;
  gap: 5px;
  flex-wrap: wrap;
}
.badge {
  font-size: 10.5px;
  padding: 2px 7px;
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
  height: 5px;
  margin: 9px 0 6px;
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
  height: 7px;
  background: rgba(255, 255, 255, 0.45);
}

.spark-wrap {
  position: relative;
  margin: 0 0 7px;
}
.spark {
  display: block;
  width: 100%;
  height: 26px;
}
.spark polyline {
  fill: none;
  stroke: var(--accent);
  stroke-width: 1.3;
  vector-effect: non-scaling-stroke;
}
.spark-threshold {
  stroke: rgba(251, 191, 36, 0.45);
  stroke-width: 1;
  stroke-dasharray: 3 3;
}
.spark-label {
  position: absolute;
  right: 1px;
  top: -4px;
  font-size: 9px;
  color: #54687a;
}

.kv2 {
  display: grid;
  grid-template-columns: auto minmax(0, 1fr) auto minmax(0, 1fr);
  gap: 3px 8px;
  margin: 2px 0 0;
  font-size: 10.5px;
  align-items: baseline;
}
.kv2 dt {
  color: var(--dim);
  white-space: nowrap;
}
.kv2 dd {
  margin: 0;
  color: var(--text);
  font-variant-numeric: tabular-nums;
  text-align: left;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.kv2 dt:nth-of-type(2n) {
  padding-left: 4px;
  border-left: 1px solid rgba(30, 42, 56, 0.6);
}

.pose {
  display: flex;
  flex-direction: column;
  gap: 2px;
  margin-top: 7px;
  padding: 5px 7px;
  border: 1px solid #17222e;
  border-radius: 5px;
  background: #0b141d;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  font-size: 10.5px;
}
.pose > div {
  display: flex;
  align-items: baseline;
  gap: 7px;
}
.pose .k {
  flex: none;
  width: 20px;
  color: #54687a;
}
.pose .v {
  color: #c7d8e6;
  font-variant-numeric: tabular-nums;
  letter-spacing: 0.01em;
}
.pose .u {
  margin-left: auto;
  font-size: 9px;
  color: #4a5d6f;
}

.sub {
  margin-top: 7px;
  border-top: 1px solid rgba(30, 42, 56, 0.55);
  padding-top: 6px;
}
.sub > summary {
  cursor: pointer;
  font-size: 10px;
  color: var(--dim);
  list-style: none;
}
.sub > summary::-webkit-details-marker {
  display: none;
}
.sub > summary::before {
  content: '+ ';
  color: #4f6375;
}
.sub[open] > summary::before {
  content: '− ';
}
.sub pre {
  margin: 6px 0 0;
  max-height: 130px;
  overflow: auto;
  font-size: 9.5px;
  line-height: 1.5;
  color: #9fb4c7;
  white-space: pre-wrap;
  word-break: break-all;
}

.layer-grid {
  display: flex;
  flex-direction: column;
  gap: 1px;
}
.layer-item {
  display: flex;
  align-items: center;
  gap: 6px;
  min-height: 25px;
}
.layer {
  display: flex;
  align-items: center;
  gap: 7px;
  flex: 1;
  min-width: 0;
  font-size: 11.5px;
  cursor: pointer;
  color: #b9c9d8;
}
.layer input {
  accent-color: var(--accent);
  margin: 0;
}
.swatch {
  width: 7px;
  height: 7px;
  border-radius: 2px;
  flex: none;
  opacity: 0.9;
}
.layer-name {
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
.layer-count {
  margin-left: 2px;
  font-size: 9.5px;
  color: #6b8ba6;
  font-variant-numeric: tabular-nums;
}
.mini {
  flex: none;
  padding: 1px 6px;
  border: 1px solid #223243;
  border-radius: 4px;
  background: #0b141d;
  color: #7e91a4;
  font-size: 9.5px;
  cursor: pointer;
  font-variant-numeric: tabular-nums;
}
.mini:hover:not(:disabled) {
  border-color: #3a536a;
  color: #cfe4f5;
}
.mini:disabled {
  opacity: 0.35;
  cursor: not-allowed;
}
.mini.warn {
  border-color: rgba(251, 191, 36, 0.5);
  color: #fcd34d;
}

.foot {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-top: 7px;
  padding-top: 7px;
  border-top: 1px solid rgba(30, 42, 56, 0.55);
}
.slider-row {
  display: flex;
  align-items: center;
  gap: 7px;
  flex: 1;
  min-width: 0;
  font-size: 9.5px;
  color: var(--dim);
}
.slider-row input[type='range'] {
  flex: 1;
  min-width: 0;
  accent-color: var(--accent);
}
.slider-row .mono {
  font-variant-numeric: tabular-nums;
  color: #9fb4c7;
}
.dim {
  font-size: 9.5px;
  color: #54687a;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.row {
  display: flex;
  gap: 6px;
  align-items: center;
  margin-top: 6px;
}
.row:first-child {
  margin-top: 0;
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
  font-size: 9.5px;
  color: var(--dim);
}

.grid2 {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 6px;
}
.grid2 .btn {
  padding: 6px 8px;
}

.note {
  margin: 8px 0 0;
  font-size: 10px;
  line-height: 1.55;
  color: #6b7f93;
}
.note code {
  color: #7dd3fc;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.service-result {
  margin-top: 7px;
  font-size: 10px;
  color: #9fb4c7;
  background: #0c141d;
  border: 1px solid #17222e;
  border-radius: 4px;
  padding: 5px 7px;
  word-break: break-all;
  line-height: 1.45;
  max-height: 54px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
</style>
