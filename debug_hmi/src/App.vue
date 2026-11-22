<template>
  <div class="app">
    <header class="topbar">
      <div class="brand">rosiwit<span>debug_hmi</span></div>
      <ConnectionPill />

      <div class="state">
        <span class="badge" :class="locClass">{{ locText }}</span>
        <span class="badge ghost">{{ modeText }}</span>
        <span class="badge ghost">匹配 {{ matchPct }}%</span>
        <span v-if="state.slam.map_name" class="map-chip" :title="state.slam.map_name">
          {{ state.slam.map_name }}
        </span>
      </div>

      <div class="spacer" />

      <button class="panel-btn" :class="{ on: showLeft }" @click="showLeft = !showLeft" title="显示/隐藏左栏">
        左栏
      </button>
      <button class="panel-btn" :class="{ on: showRight }" @click="showRight = !showRight" title="显示/隐藏右栏">
        右栏
      </button>
    </header>

    <div
      class="work"
      :style="{
        gridTemplateColumns: `${showLeft ? '300px' : '0px'} minmax(0, 1fr) ${showRight ? '360px' : '0px'}`,
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
import ConnectionPill from './components/ConnectionPill.vue'
import ControlPanel from './components/ControlPanel.vue'
import InspectorPanel from './components/InspectorPanel.vue'
import SceneView from './three/SceneView.vue'
import { initHmi, state } from './ros/app'

const showLeft = ref(true)
const showRight = ref(true)

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
const matchPct = computed(() =>
  state.slamTime ? ((state.slam.matching_degree || 0) * 100).toFixed(0) : '—',
)

onMounted(() => initHmi())
</script>

<style scoped>
.app {
  height: 100%;
  display: grid;
  grid-template-rows: 42px minmax(0, 1fr);
  background: var(--bg);
}

.topbar {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 0 12px;
  border-bottom: 1px solid var(--line);
  background: #0b1219;
  min-width: 0;
}

.brand {
  font-size: 13.5px;
  font-weight: 600;
  letter-spacing: 0.01em;
  color: #cfe4f5;
  white-space: nowrap;
}
.brand span {
  margin-left: 4px;
  color: var(--accent);
  font-weight: 700;
}

.state {
  display: flex;
  align-items: center;
  gap: 6px;
  min-width: 0;
  overflow: hidden;
}

.badge {
  font-size: 10.5px;
  padding: 2px 8px;
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

.map-chip {
  max-width: 130px;
  padding: 2px 7px;
  border-radius: 4px;
  border: 1px dashed #23384a;
  color: #7e91a4;
  font-size: 10.5px;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.spacer {
  flex: 1;
}

.panel-btn {
  padding: 3px 8px;
  border: 1px solid #223243;
  border-radius: 4px;
  background: transparent;
  color: #54687a;
  font-size: 10px;
  cursor: pointer;
}
.panel-btn:hover {
  border-color: #33506a;
  color: #b9d2e6;
}
.panel-btn.on {
  color: #9fb4c7;
  border-color: #2b4054;
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
  .badge.ghost,
  .map-chip {
    display: none;
  }
}
</style>
