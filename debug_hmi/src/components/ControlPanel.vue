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

          <div v-if="state.matchHistory.length > 1" class="spark-wrap">
            <svg class="spark" viewBox="0 0 120 24" preserveAspectRatio="none">
              <line class="spark-threshold" x1="0" y1="4.8" x2="120" y2="4.8" />
              <polyline :points="sparkPoints" />
            </svg>
            <span class="spark-label">{{ sparkSeconds }}s</span>
          </div>
          <div v-else class="spark-empty">等待匹配度曲线…</div>

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

    <!-- 设备信息 -->
    <details class="sec" open>
      <summary>
        <span class="sec-title">设备信息</span>
        <span class="sec-meta">{{ shadowAgeText }}</span>
      </summary>
      <div class="sec-body">
        <div v-if="!state.robotShadow" class="prompt">
          等待 <code>/device/robot_shadow</code>…
        </div>
        <template v-else>
          <div class="badges">
            <span class="badge" :class="shadowOnline ? 'ok' : 'bad'">
              {{ shadowOnline ? '在线' : '离线' }}
            </span>
            <span class="badge ghost">{{ shadowDevice }}</span>
            <span class="badge ghost">{{ shadowStateText }}</span>
          </div>

          <div class="tank-grid">
            <div class="tank">
              <div class="tank-head"><span>电池</span><b>{{ pctText(shadowProp.bms_soc) }}</b></div>
              <div class="tank-bar">
                <i :class="levelClass(shadowProp.bms_soc)" :style="{ width: pctWidth(shadowProp.bms_soc) }" />
              </div>
            </div>
            <div class="tank">
              <div class="tank-head"><span>清水量</span><b>{{ pctText(shadowProp.clean_height) }}</b></div>
              <div class="tank-bar">
                <i class="water" :style="{ width: pctWidth(shadowProp.clean_height) }" />
              </div>
            </div>
            <div class="tank">
              <div class="tank-head"><span>污水量</span><b>{{ pctText(shadowProp.dirty_height) }}</b></div>
              <div class="tank-bar">
                <i class="dirty" :style="{ width: pctWidth(shadowProp.dirty_height) }" />
              </div>
            </div>
          </div>

          <dl class="kv2">
            <dt>设备</dt>
            <dd>{{ shadow.device_id || '—' }} · {{ shadow.product_id || '—' }}</dd>
            <dt>地图</dt>
            <dd>{{ shadow.map_name || '—' }}</dd>
            <dt>控制</dt>
            <dd>{{ shadow.control_mode || '—' }}</dd>
            <dt>清洁</dt>
            <dd>{{ shadow.clean_mode || '—' }}</dd>
            <dt>网络</dt>
            <dd>4G {{ shadow.state_4G || '—' }} · WiFi {{ shadow.wifi_state || '—' }}</dd>
            <dt>充电</dt>
            <dd>{{ shadowProp.is_charge ? '充电中' : '未充电' }} · {{ shadowProp.dock ? '已回桩' : '未回桩' }}</dd>
            <dt>环境</dt>
            <dd>
              {{ num(shadowProp.Temp) }}°C · {{ num(shadowProp.Humi) }}%RH · PM2.5
              {{ num(shadowProp.PM2_5) }}
            </dd>
          </dl>

          <div v-if="shadowFaults.length" class="fault-box">
            <div class="fault-head">故障 {{ shadowFaults.length }} 条</div>
            <div v-for="(f, i) in shadowFaults.slice(0, 4)" :key="`${f.code || i}-${i}`" class="fault-item">
              <span class="fault-level">{{ f.level || 'FAULT' }}</span>
              <span class="fault-text" :title="f.fault_desc">{{ f.name || f.fault_desc || f.code }}</span>
            </div>
          </div>

          <details class="sub">
            <summary>原始 robot_shadow</summary>
            <pre class="diag-pre">{{ shadowPretty }}</pre>
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
          <button class="btn tiny" @click="doAddPcd">本地 PCD</button>
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
            <label v-if="d.renderer === 'points'">
              数据源
              <select
                class="select mini-select"
                :value="d.source"
                @change="changeDisplaySource(d.id, ($event.target as HTMLSelectElement).value as any)"
              >
                <option value="topic">话题</option>
                <option value="pcd">本地 PCD</option>
              </select>
            </label>
            <label>
              渲染
              <select
                class="select mini-select"
                :value="d.renderer"
                @change="updateDisplay(d.id, { renderer: ($event.target as HTMLSelectElement).value as any })"
              >
                <option v-for="r in rendererOptions(d)" :key="r" :value="r">{{ rendererLabel(r) }}</option>
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
            <label v-if="d.source === 'topic'">
              频率
              <select
                class="select mini-select"
                :value="d.rate"
                @change="updateDisplay(d.id, { rate: Number(($event.target as HTMLSelectElement).value) })"
              >
                <option :value="0">手动</option>
                <option :value="1">1Hz</option>
                <option :value="2">2Hz</option>
                <option :value="5">5Hz</option>
                <option :value="10">10Hz</option>
                <option :value="20">20Hz</option>
              </select>
            </label>
            <button
              v-if="d.source === 'topic'"
              class="mini"
              :disabled="state.status !== 'open'"
              @click="grabTopic(d.id)"
            >
              抓取
            </button>
            <label v-if="d.source === 'pcd'" class="btn tiny file-btn pcd-file">
              {{ d.pcdName || '选择 PCD 文件' }}
              <input type="file" accept=".pcd" @change="onDisplayPcd(d.id, $event)" />
            </label>
          </div>

          <div v-if="d.source === 'pcd'" class="display-stats">
            <span>本地 PCD</span>
            <span v-if="state.pcdCounts[d.id]">{{ state.pcdCounts[d.id].toLocaleString() }} 点</span>
            <span v-else>未选择文件</span>
          </div>
          <div v-else class="display-stats">
            <span v-if="d.rate === 0" class="manual-tag">手动</span>
            <span v-else>{{ d.stats.hz.toFixed(1) }} Hz</span>
            <span>{{ formatBytes(d.stats.bytes) }}/帧</span>
            <span>累计 {{ formatBytes(d.stats.total) }}</span>
            <span v-if="d.stats.age >= 0">{{ (d.stats.age / 1000).toFixed(1) }}s 前</span>
            <span v-if="d.renderer !== 'image' && state.pointCounts[d.topic]">
              {{ state.pointCounts[d.topic].toLocaleString() }} 点/格
            </span>
          </div>
          <img
            v-if="d.renderer === 'image' && state.images[d.topic]"
            class="topic-thumb"
            :src="state.images[d.topic].url"
            :alt="d.topic"
            @click="openImage(d.topic)"
          />
        </div>

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

    <!-- 设备控制 -->
    <details class="sec" open>
      <summary>
        <span class="sec-title">设备控制</span>
        <span class="sec-meta">{{ motionText }}</span>
      </summary>
      <div class="sec-body">
        <div class="dpad">
          <button
            class="dpad-btn up"
            :class="{ active: activeDir === 'forward' }"
            :disabled="state.status !== 'open'"
            @pointerdown.prevent="startMotion('forward')"
            @pointerup="stopMotion"
            @pointerleave="stopMotion"
            @pointercancel="stopMotion"
            @contextmenu.prevent
          >
            ↑
          </button>
          <button
            class="dpad-btn left"
            :class="{ active: activeDir === 'left' }"
            :disabled="state.status !== 'open'"
            @pointerdown.prevent="startMotion('left')"
            @pointerup="stopMotion"
            @pointerleave="stopMotion"
            @pointercancel="stopMotion"
            @contextmenu.prevent
          >
            ←
          </button>
          <button class="dpad-btn stop" :disabled="state.status !== 'open'" @click="stopMotion">停</button>
          <button
            class="dpad-btn right"
            :class="{ active: activeDir === 'right' }"
            :disabled="state.status !== 'open'"
            @pointerdown.prevent="startMotion('right')"
            @pointerup="stopMotion"
            @pointerleave="stopMotion"
            @pointercancel="stopMotion"
            @contextmenu.prevent
          >
            →
          </button>
          <button
            class="dpad-btn down"
            :class="{ active: activeDir === 'back' }"
            :disabled="state.status !== 'open'"
            @pointerdown.prevent="startMotion('back')"
            @pointerup="stopMotion"
            @pointerleave="stopMotion"
            @pointercancel="stopMotion"
            @contextmenu.prevent
          >
            ↓
          </button>
        </div>

        <div class="speed-row">
          <label>
            线速度
            <input v-model.number="linearSpeed" type="range" min="0.05" max="0.5" step="0.05" />
            <span class="mono">{{ linearSpeed.toFixed(2) }} m/s</span>
          </label>
          <label>
            角速度
            <input v-model.number="angularSpeed" type="range" min="0.1" max="1.5" step="0.1" />
            <span class="mono">{{ angularSpeed.toFixed(1) }} rad/s</span>
          </label>
        </div>

        <p class="note">
          按住方向键持续运动，松开立即停；断线、切换页面或 0.5s 没收到新指令时服务端也会自动发零速。
          直接发布 <code>/cmd_vel</code>，使用前请先停止导航任务并确认周围安全。
        </p>
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
  addPcdDisplay,
  bridge,
  changeDisplaySource,
  dumpRecorder,
  grabTopic,
  loadPcdIntoDisplay,
  openImagePreview,
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

function doAddPcd(): void {
  addPcdDisplay()
}

async function onDisplayPcd(id: string, event: Event): Promise<void> {
  const input = event.target as HTMLInputElement
  const file = input.files?.[0]
  if (!file) return
  try {
    await loadPcdIntoDisplay(id, file)
  } finally {
    input.value = ''
  }
}

function doRefreshTopics(): void {
  void refreshTopics()
}

function openImage(topic: string): void {
  openImagePreview(topic)
}

// -------------------------------------------------------------- device control

type MoveDir = '' | 'forward' | 'back' | 'left' | 'right'

const activeDir = ref<MoveDir>('')
const linearSpeed = ref(0.15)
const angularSpeed = ref(0.6)
let motionTimer: number | null = null

const motionText = computed(() => {
  if (!activeDir.value) return '按住运动'
  const label = { forward: '前进', back: '后退', left: '左转', right: '右转' }[activeDir.value]
  return `${label}中…`
})

function publishMotion(): void {
  const dir = activeDir.value
  if (!dir) return
  let linear = 0
  let angular = 0
  if (dir === 'forward') linear = linearSpeed.value
  else if (dir === 'back') linear = -linearSpeed.value
  else if (dir === 'left') angular = angularSpeed.value
  else if (dir === 'right') angular = -angularSpeed.value
  bridge.setTwist(linear, angular)
}

function startMotion(dir: Exclude<MoveDir, ''>): void {
  if (state.status !== 'open') return
  if (motionTimer !== null) {
    window.clearInterval(motionTimer)
    motionTimer = null
  }
  activeDir.value = dir
  publishMotion()
  motionTimer = window.setInterval(publishMotion, 100)
}

function stopMotion(): void {
  if (motionTimer !== null) {
    window.clearInterval(motionTimer)
    motionTimer = null
  }
  if (state.status === 'open') bridge.setTwist(0, 0)
  activeDir.value = ''
}

function onGlobalStop(): void {
  if (activeDir.value) stopMotion()
}

function onVisibilityChange(): void {
  if (document.hidden) onGlobalStop()
}

function shortType(type: string): string {
  return type ? type.split('/').pop() || type : '?'
}

function rendererLabel(r: RendererKind): string {
  const labels: Record<RendererKind, string> = {
    points: '点云',
    grid: '栅格',
    laser: '激光',
    path: '路径',
    polygon: '多边形',
    image: '图像',
    status: '状态',
    json: 'JSON',
  }
  return labels[r] ?? r
}

function rendererOptions(d: DisplayItem): RendererKind[] {
  const base: RendererKind[] = ['json']
  if (d.type.includes('PointCloud')) return ['points', 'image', 'json']
  if (d.type.includes('OccupancyGrid')) return ['grid', 'image', 'json']
  if (d.type.includes('LaserScan')) return ['laser', 'image', 'json']
  if (d.type.endsWith('/Path')) return ['path', 'image', 'json']
  if (d.type.includes('Polygon')) return ['polygon', 'image', 'json']
  if (d.type.includes('Image')) return ['image', 'json']
  if (d.type.includes('SlamInfo')) return ['status', 'json']
  // 类型未知时允许手动选择，方便接入自定义消息
  return ['points', 'grid', 'laser', 'path', 'polygon', 'image', ...base, 'status']
}

function formatBytes(n: number): string {
  if (!n) return '0 B'
  if (n >= 1024 * 1024) return `${(n / 1024 / 1024).toFixed(2)} MB`
  if (n >= 1024) return `${(n / 1024).toFixed(2)} KB`
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

// -------------------------------------------------------------- robot shadow

const shadow = computed<Record<string, any>>(() => state.robotShadow || {})
const shadowProp = computed<Record<string, any>>(() => (shadow.value.property as any) || {})
const shadowFaults = computed<any[]>(() => (Array.isArray(shadow.value.fault) ? shadow.value.fault : []))
const shadowOnline = computed(() => !!shadow.value.online)
const shadowDevice = computed(
  () =>
    [shadow.value.product_id, shadow.value.product_sub_model].filter(Boolean).join(' ') || '—',
)
const shadowStateText = computed(() => {
  const s = String(shadow.value.robot_state || '')
  const map: Record<string, string> = {
    ROBOT_MODE_IDLE: '空闲',
    ROBOT_MODE_CLEAN: '清洁中',
    ROBOT_MODE_CLEANING: '清洁中',
    ROBOT_MODE_MAPPING: '建图中',
    ROBOT_MODE_CHARGING: '充电中',
    ROBOT_MODE_PAUSE: '暂停',
    ROBOT_MODE_ERROR: '故障',
    ROBOT_MODE_MANUAL: '手动',
    ROBOT_MODE_RETURN: '回桩中',
  }
  return map[s] || s || '—'
})
const shadowAgeText = computed(() => {
  if (!state.shadowTime) return '等待数据…'
  return `${((nowTick.value - state.shadowTime) / 1000).toFixed(1)}s 前`
})
const shadowPretty = computed(() => JSON.stringify(shadow.value, null, 2))

function pctWidth(value: unknown): string {
  const n = Number(value)
  return `${Math.max(0, Math.min(100, Number.isFinite(n) ? n : 0))}%`
}

function pctText(value: unknown): string {
  return pctWidth(value)
}

function levelClass(value: unknown): string {
  const n = Number(value)
  if (!Number.isFinite(n)) return ''
  if (n <= 20) return 'bad'
  if (n <= 50) return 'warn'
  return 'ok'
}

function num(value: unknown): number {
  const n = Number(value)
  return Number.isFinite(n) ? n : 0
}

const nowTick = ref(Date.now())
let tickTimer: number | null = null
onMounted(() => {
  tickTimer = window.setInterval(() => {
    nowTick.value = Date.now()
  }, 500)
  window.addEventListener('pointerup', onGlobalStop)
  window.addEventListener('pointercancel', onGlobalStop)
  window.addEventListener('blur', onGlobalStop)
  document.addEventListener('visibilitychange', onVisibilityChange)
})
onBeforeUnmount(() => {
  if (tickTimer !== null) clearInterval(tickTimer)
  stopMotion()
  window.removeEventListener('pointerup', onGlobalStop)
  window.removeEventListener('pointercancel', onGlobalStop)
  window.removeEventListener('blur', onGlobalStop)
  document.removeEventListener('visibilitychange', onVisibilityChange)
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

const sparkPoints = computed(() => {
  const h = state.matchHistory
  if (h.length < 2) return ''
  const n = h.length
  return h
    .map((v, i) => {
      const y = 24 - Math.min(1, Math.max(0, v)) * 24
      return `${((i / (n - 1)) * 120).toFixed(1)},${y.toFixed(1)}`
    })
    .join(' ')
})

const sparkSeconds = computed(() => Math.round(state.matchHistory.length * 0.4))

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

.manual-tag {
  color: #fbbf24;
}

.topic-thumb {
  display: block;
  width: 100%;
  max-height: 160px;
  object-fit: contain;
  margin-top: 6px;
  border: 1px solid var(--line);
  border-radius: 4px;
  background: #05090d;
  cursor: zoom-in;
}

.traffic-total {
  display: flex;
  gap: 16px;
  margin-top: 8px;
  padding-top: 6px;
  border-top: 1px dashed var(--line);
  color: #7f97ab;
  font-size: 10.5px;
  font-variant-numeric: tabular-nums;
}

.traffic-total b {
  color: #a9bccd;
  font-weight: 600;
}

.tank-grid {
  display: grid;
  gap: 6px;
  margin: 6px 0 8px;
}

.tank-head {
  display: flex;
  justify-content: space-between;
  color: var(--dim);
  font-size: 10px;
  margin-bottom: 3px;
}

.tank-head b {
  color: #b9cedf;
  font-weight: 600;
  font-variant-numeric: tabular-nums;
}

.tank-bar {
  height: 6px;
  border-radius: 3px;
  background: #10202c;
  overflow: hidden;
}

.tank-bar i {
  display: block;
  height: 100%;
  border-radius: 3px;
  background: #38bdf8;
  transition: width 0.25s linear;
}

.tank-bar i.ok {
  background: #34d399;
}
.tank-bar i.warn {
  background: #fbbf24;
}
.tank-bar i.bad {
  background: #f87171;
}
.tank-bar i.water {
  background: #38bdf8;
}
.tank-bar i.dirty {
  background: #a78bfa;
}

.fault-box {
  margin-top: 8px;
  padding: 6px 8px;
  border: 1px solid rgba(248, 113, 113, 0.3);
  border-radius: 5px;
  background: rgba(80, 20, 20, 0.18);
}

.fault-head {
  color: #fca5a5;
  font-size: 10.5px;
  font-weight: 600;
  margin-bottom: 4px;
}

.fault-item {
  display: flex;
  gap: 6px;
  align-items: baseline;
  font-size: 10px;
  color: #d6b3b3;
  line-height: 1.5;
}

.fault-level {
  flex: none;
  color: #fca5a5;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.fault-text {
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.dpad {
  display: grid;
  grid-template-columns: repeat(3, 46px);
  grid-template-rows: repeat(3, 38px);
  gap: 4px;
  justify-content: center;
  margin: 4px auto 10px;
  touch-action: none;
  user-select: none;
}

.dpad-btn {
  border: 1px solid var(--line);
  border-radius: 5px;
  background: #0b141d;
  color: #b9cedf;
  font-size: 17px;
  line-height: 1;
  cursor: pointer;
  touch-action: none;
}

.dpad-btn:hover:not(:disabled) {
  border-color: #3b6d8d;
  color: #e2f3ff;
}

.dpad-btn.active {
  background: #1d4f6a;
  border-color: #38bdf8;
  color: #e0f2fe;
}

.dpad-btn:disabled {
  opacity: 0.35;
  cursor: not-allowed;
}

.dpad-btn.up {
  grid-column: 2;
  grid-row: 1;
}
.dpad-btn.left {
  grid-column: 1;
  grid-row: 2;
}
.dpad-btn.stop {
  grid-column: 2;
  grid-row: 2;
  color: #fca5a5;
  border-color: #5b2b2b;
}
.dpad-btn.right {
  grid-column: 3;
  grid-row: 2;
}
.dpad-btn.down {
  grid-column: 2;
  grid-row: 3;
}

.speed-row {
  display: flex;
  flex-direction: column;
  gap: 5px;
  margin-bottom: 6px;
}

.speed-row label {
  display: grid;
  grid-template-columns: 46px 1fr 68px;
  align-items: center;
  gap: 6px;
  color: var(--dim);
  font-size: 10.5px;
}

.speed-row input[type='range'] {
  width: 100%;
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
.spark-empty {
  margin: 0 0 7px;
  padding: 6px 0;
  border-top: 1px dashed var(--line);
  border-bottom: 1px dashed var(--line);
  color: #54687a;
  font-size: 10px;
  text-align: center;
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
