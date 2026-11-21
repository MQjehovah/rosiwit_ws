<template>
  <div class="app">
    <header class="topbar">
      <div class="brand">rosiwit<span>HMI</span></div>
      <div class="conn">
        <span class="dot" :class="statusClass" />
        <strong>{{ statusText }}</strong>
        <span class="url">{{ state.url }}</span>
        <span v-if="state.dataAge >= 0" class="rtt">数据 {{ ageText(state.dataAge) }}</span>
      </div>

      <div class="spacer" />

      <span class="badge" :class="locClass">{{ locText }}</span>
      <span class="badge ghost">{{ modeText }}</span>
      <span class="badge ghost">匹配 {{ ((state.slam.matching_degree || 0) * 100).toFixed(0) }}%</span>
      <span class="badge ghost">{{ state.slam.map_name || '无地图' }}</span>

      <button class="btn tiny" @click="showLeft = !showLeft">{{ showLeft ? '隐藏左栏' : '左栏' }}</button>
      <button class="btn tiny" @click="showRight = !showRight">{{ showRight ? '隐藏右栏' : '右栏' }}</button>
    </header>

    <div
      class="work"
      :style="{
        gridTemplateColumns: `${showLeft ? '324px' : '0px'} minmax(0, 1fr) ${showRight ? '384px' : '0px'}`,
      }"
    >
      <aside v-show="showLeft" class="left"><ControlPanel /></aside>
      <main class="center"><SceneView /></main>
      <aside v-show="showRight" class="right"><InspectorPanel /></aside>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onMounted, ref } from 'vue'
import ControlPanel from './components/ControlPanel.vue'
import InspectorPanel from './components/InspectorPanel.vue'
import SceneView from './three/SceneView.vue'
import { initHmi, state } from './ros/app'

const showLeft = ref(true)
const showRight = ref(true)

const statusText = computed(() => {
  const map: Record<string, string> = {
    idle: '未连接',
    connecting: '连接中',
    open: '已连接',
    closed: '已断开',
    error: '连接错误',
  }
  return map[state.status] ?? state.status
})

const statusClass = computed(() => ({
  ok: state.status === 'open',
  warn: state.status === 'connecting' || state.status === 'idle',
  bad: state.status === 'error' || state.status === 'closed',
}))

const locText = computed(
  () => ['定位正常', '恢复中', '定位失败'][state.slam.location_status] ?? '无定位数据',
)
const locClass = computed(() => ({
  ok: state.slam.location_status === 0,
  warn: state.slam.location_status === 1,
  bad: state.slam.location_status === 2,
}))
const modeText = computed(() => {
  const map: Record<number, string> = { 0: '建图中', 1: '定位模式', 2: 'SLAM 错误' }
  return map[state.slam.slam_state] ?? '模式未知'
})

function ageText(age: number): string {
  if (age < 1000) return `${age} ms`
  return `${(age / 1000).toFixed(1)} s`
}

onMounted(() => initHmi())
</script>

<style scoped>
.app {
  height: 100%;
  display: grid;
  grid-template-rows: 44px minmax(0, 1fr);
  background: var(--bg);
}

.topbar {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 0 12px;
  border-bottom: 1px solid var(--line);
  background: #0b1219;
  min-width: 0;
}

.brand {
  font-size: 14px;
  font-weight: 600;
  letter-spacing: 0.02em;
  color: #cfe4f5;
  white-space: nowrap;
}
.brand span {
  margin-left: 4px;
  color: var(--accent);
  font-weight: 700;
}

.conn {
  display: flex;
  align-items: center;
  gap: 7px;
  font-size: 11px;
  color: var(--dim);
  min-width: 0;
}
.conn strong {
  font-weight: 500;
  color: var(--text);
  white-space: nowrap;
}
.conn .url {
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  max-width: 220px;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  color: #6f869b;
}
.rtt {
  color: #6f869b;
  font-variant-numeric: tabular-nums;
}

.dot {
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: var(--dim);
  flex: none;
}
.dot.ok {
  background: var(--ok);
  box-shadow: 0 0 8px rgba(52, 211, 153, 0.6);
}
.dot.warn {
  background: var(--warn);
}
.dot.bad {
  background: var(--bad);
  box-shadow: 0 0 8px rgba(248, 113, 113, 0.5);
}

.spacer {
  flex: 1;
}

.badge {
  font-size: 10.5px;
  padding: 2px 7px;
  border-radius: 4px;
  border: 1px solid var(--line);
  background: #0e1620;
  color: var(--text);
  white-space: nowrap;
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

.work {
  display: grid;
  min-height: 0;
  min-width: 0;
}

.left {
  min-width: 0;
  overflow-y: auto;
  border-right: 1px solid var(--line);
  background: #0c131b;
}
.center {
  min-width: 0;
  min-height: 0;
  position: relative;
}
.right {
  min-width: 0;
  min-height: 0;
  overflow: hidden;
}

@media (max-width: 1100px) {
  .conn .url,
  .badge.ghost {
    display: none;
  }
}
</style>
