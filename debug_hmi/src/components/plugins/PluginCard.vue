<template>
  <section class="plugin-card" :class="[mode, { off: !plugin.enabled }]">
    <header class="pc-head">
      <span
        v-if="mode === 'sidebar'"
        class="pc-grip"
        draggable="true"
        title="拖到中间画布"
        @dragstart="emit('dragstart', $event)"
        >⠿</span
      >
      <span class="pc-title">{{ plugin.title }}</span>
      <code class="pc-topic" :title="plugin.topic">{{ plugin.topic }}</code>
      <span class="pc-age">{{ ageLabel }}</span>
      <button
        class="pc-btn"
        :class="{ on: plugin.enabled }"
        :title="plugin.enabled ? '停止订阅' : '订阅该话题'"
        @click="emit('toggle', !plugin.enabled)"
      >
        {{ plugin.enabled ? '停订' : '订阅' }}
      </button>
      <button v-if="mode === 'sidebar'" class="pc-btn" title="投到画布" @click="emit('dock')">
        画布
      </button>
      <button v-else class="pc-btn" title="收回左栏" @click="emit('undock')">收回</button>
    </header>

    <div v-if="!plugin.enabled" class="pc-off">
      未订阅 <code>{{ plugin.topic }}</code>
    </div>
    <div v-else-if="plugin.kind === 'status' && !state.slamTime" class="pc-empty">
      等待 {{ plugin.topic }}…
    </div>
    <div v-else-if="plugin.kind === 'device' && !state.robotShadow" class="pc-empty">
      等待 {{ plugin.topic }}…
    </div>

    <div v-else-if="plugin.kind === 'status'" class="pc-body">
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

      <div
        v-if="state.slam.location_status === 2 || (state.slam.location_status === 1 && state.slam.map_area > 0)"
        class="lost-box"
      >
        <span class="badge" :class="state.slam.location_status === 2 ? 'bad' : 'warn'">
          {{ state.slam.location_status === 2 ? '定位丢失' : '初始化/恢复中' }}
        </span>
        <span class="mono">
          x {{ state.slam.robot_pose.x.toFixed(2) }} · y
          {{ state.slam.robot_pose.y.toFixed(2) }} · yaw
          {{ ((state.slam.robot_pose.theta * 180) / Math.PI).toFixed(0) }}°
        </span>
        <span title="核心当前帧的匹配度，非丢失前缓存的数值">
          当前匹配度 {{ percent(state.slam.matching_degree) }}
        </span>
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

      <details class="sub">
        <summary>原始 /slam/info</summary>
        <pre class="diag-pre">{{ slamRaw }}</pre>
      </details>
    </div>

    <div v-else class="pc-body">
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
    </div>
  </section>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
import { state, type PluginItem } from '../../ros/app'

const props = defineProps<{ plugin: PluginItem; mode: 'sidebar' | 'canvas' }>()
const emit = defineEmits<{
  toggle: [boolean]
  dock: []
  undock: []
  dragstart: [DragEvent]
}>()

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

const locText = computed(
  () => ['定位正常', '恢复中', '定位失败'][state.slam.location_status] ?? '无数据',
)
const locClass = computed(() => ({
  ok: state.slam.location_status === 0,
  warn: state.slam.location_status === 1,
  bad: state.slam.location_status === 2,
}))

const ageLabel = computed(() => {
  if (!props.plugin.enabled) return '未订阅'
  const t = props.plugin.kind === 'status' ? state.slamTime : state.shadowTime
  if (!t) return '等待数据…'
  return `${(Math.max(0, nowTick.value - t) / 1000).toFixed(1)}s 前`
})

const shortSource = computed(() => state.robot.source.replace('TF map→', 'TF '))

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

const shadow = computed<Record<string, any>>(() => state.robotShadow || {})
const shadowProp = computed<Record<string, any>>(() => (shadow.value.property as any) || {})
const shadowFaults = computed<any[]>(() => (Array.isArray(shadow.value.fault) ? shadow.value.fault : []))
const shadowOnline = computed(() => !!shadow.value.online)
const shadowDevice = computed(
  () => [shadow.value.product_id, shadow.value.product_sub_model].filter(Boolean).join(' ') || '—',
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
</script>

<style scoped>
.plugin-card {
  display: flex;
  flex-direction: column;
  min-width: 0;
  border: 1px solid var(--line);
  border-radius: 6px;
  background: rgba(11, 20, 29, 0.92);
  overflow: hidden;
}
.plugin-card.off {
  opacity: 0.85;
}
.plugin-card.canvas {
  max-height: 100%;
  box-shadow: 0 10px 30px rgba(0, 0, 0, 0.45);
}

.pc-head {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 5px 7px;
  background: #0d1720;
  border-bottom: 1px solid var(--line);
  min-width: 0;
  flex-wrap: wrap;
}
.pc-grip {
  cursor: grab;
  color: #4f6375;
  font-size: 13px;
  line-height: 1;
  user-select: none;
}
.pc-grip:active {
  cursor: grabbing;
}
.pc-title {
  font-size: 11px;
  font-weight: 600;
  color: #a9bccd;
  white-space: nowrap;
}
.pc-topic {
  flex: 1;
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  color: #5f7488;
  font-size: 9.5px;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}
.pc-age {
  color: #54687a;
  font-size: 9.5px;
  font-variant-numeric: tabular-nums;
  white-space: nowrap;
}
.pc-btn {
  flex: none;
  padding: 2px 6px;
  border: 1px solid #223243;
  border-radius: 4px;
  background: transparent;
  color: #7e91a4;
  font-size: 9.5px;
  cursor: pointer;
  white-space: nowrap;
}
.pc-btn:hover {
  border-color: #33506a;
  color: #b9d2e6;
}
.pc-btn.on {
  color: #7dd3fc;
  border-color: rgba(56, 189, 248, 0.45);
  background: rgba(56, 189, 248, 0.1);
}

.pc-body {
  padding: 8px 9px;
  overflow: auto;
  min-height: 0;
}
.pc-off,
.pc-empty {
  padding: 12px 10px;
  color: #6f8396;
  font-size: 10.5px;
}
.pc-off code,
.pc-empty code {
  color: #7dd3fc;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
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

.spark-wrap {
  position: relative;
  margin: 8px 0 7px;
}
.spark {
  display: block;
  width: 100%;
  height: 54px;
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
  margin: 7px 0;
  padding: 5px 0;
  border-top: 1px dashed var(--line);
  border-bottom: 1px dashed var(--line);
  color: #54687a;
  font-size: 10px;
  text-align: center;
}

.lost-box {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 6px;
  margin: 0 0 8px;
  padding: 5px 7px;
  border: 1px solid rgba(248, 113, 113, 0.3);
  border-radius: 5px;
  background: rgba(80, 20, 20, 0.18);
  color: #d6b3b3;
  font-size: 10px;
  line-height: 1.5;
}
.lost-box .mono {
  color: #fca5a5;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.kv2 {
  display: grid;
  grid-template-columns: auto minmax(0, 1fr);
  gap: 3px 8px;
  margin: 7px 0 0;
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
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.sub {
  margin-top: 8px;
}
.sub summary {
  cursor: pointer;
  color: #5f7488;
  font-size: 10px;
  list-style: none;
}
.sub summary::-webkit-details-marker {
  display: none;
}
.diag-pre {
  max-height: 150px;
  overflow: auto;
  margin: 5px 0 0;
  padding: 5px 7px;
  background: #080f16;
  border: 1px solid var(--line);
  border-radius: 4px;
  color: #9fb4c7;
  font-size: 10px;
  line-height: 1.45;
}

.tank-grid {
  display: grid;
  gap: 6px;
  margin: 8px 0;
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
</style>
