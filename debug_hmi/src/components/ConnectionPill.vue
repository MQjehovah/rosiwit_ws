<template>
  <div ref="root" class="conn-pill">
    <button class="pill" :class="statusClass" :title="state.error || state.url" @click="open = !open">
      <span class="dot" />
      <span class="label">{{ statusText }}</span>
      <span v-if="state.status === 'open' && state.dataAge >= 0" class="age">{{ ageText }}</span>
      <span class="caret">▾</span>
    </button>

    <div v-if="open" class="pop">
      <div class="pop-head">
        <span>rosbridge</span>
        <button class="close" @click="open = false">×</button>
      </div>

      <div class="pop-row">
        <input
          v-model="urlInput"
          class="input grow"
          spellcheck="false"
          placeholder="ws://ip:9100 或 wss://…"
          @keyup.enter="apply"
        />
        <button v-if="state.status !== 'open'" class="btn primary" @click="apply">连接</button>
        <button v-else class="btn" @click="doDisconnect">断开</button>
      </div>

      <div v-if="urlPresets.length > 1" class="presets">
        <button
          v-for="preset in urlPresets"
          :key="preset.url"
          class="preset"
          :class="{ active: urlInput === preset.url }"
          @click="usePreset(preset.url)"
        >
          {{ preset.label }}
        </button>
      </div>

      <div class="pop-meta">
        <span>9100 = debug_bridge</span>
        <span class="sep">·</span>
        <span>9090 = 厂商 http_bridge</span>
        <span v-if="state.status === 'open'" class="sep">·</span>
        <span v-if="state.status === 'open'">数据 {{ ageText }}</span>
      </div>
      <div v-if="state.error" class="err">{{ state.error }}</div>
      <div class="cur" :title="state.url">{{ state.url }}</div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { connect, disconnect, state, urlPresets } from '../ros/app'

const root = ref<HTMLDivElement | null>(null)
const open = ref(false)
const urlInput = ref(state.url)

const statusText = computed(() => {
  const map: Record<string, string> = {
    idle: '未连接',
    connecting: '连接中',
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

const ageText = computed(() => {
  const age = state.dataAge
  if (age < 0) return '—'
  if (age < 1000) return `${age}ms`
  return `${(age / 1000).toFixed(1)}s`
})

watch(
  () => state.url,
  (url) => {
    urlInput.value = url
  },
)

function apply(): void {
  connect(urlInput.value)
  open.value = false
}

function usePreset(url: string): void {
  urlInput.value = url
  connect(url)
  open.value = false
}

function doDisconnect(): void {
  disconnect()
  open.value = false
}

function onDocClick(event: MouseEvent): void {
  if (!open.value) return
  if (root.value && !root.value.contains(event.target as Node)) open.value = false
}

function onKey(event: KeyboardEvent): void {
  if (event.key === 'Escape') open.value = false
}

onMounted(() => {
  document.addEventListener('pointerdown', onDocClick)
  window.addEventListener('keydown', onKey)
})
onBeforeUnmount(() => {
  document.removeEventListener('pointerdown', onDocClick)
  window.removeEventListener('keydown', onKey)
})
</script>

<style scoped>
.conn-pill {
  position: relative;
}

.pill {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  padding: 3px 9px;
  border: 1px solid #223243;
  border-radius: 13px;
  background: #0e1620;
  color: #9fb4c7;
  font-size: 11px;
  cursor: pointer;
}
.pill:hover {
  border-color: #33506a;
}
.pill .dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: #5b6b7b;
}
.pill.ok {
  border-color: rgba(52, 211, 153, 0.4);
}
.pill.ok .dot {
  background: var(--ok);
  box-shadow: 0 0 7px rgba(52, 211, 153, 0.7);
}
.pill.warn .dot {
  background: var(--warn);
}
.pill.bad .dot {
  background: var(--bad);
}
.pill .label {
  color: #c3d4e3;
}
.pill .age {
  color: #6b8ba6;
  font-variant-numeric: tabular-nums;
}
.pill .caret {
  font-size: 8px;
  color: #54687a;
}

.pop {
  position: absolute;
  top: calc(100% + 7px);
  left: 0;
  z-index: 40;
  width: 360px;
  padding: 10px;
  border: 1px solid #263a4e;
  border-radius: 8px;
  background: #0d161f;
  box-shadow: 0 14px 34px rgba(0, 0, 0, 0.55);
}
.pop-head {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 8px;
  font-size: 10px;
  letter-spacing: 0.07em;
  text-transform: uppercase;
  color: var(--dim);
}
.close {
  border: 0;
  background: transparent;
  color: #6b7f93;
  font-size: 15px;
  line-height: 1;
  cursor: pointer;
}
.close:hover {
  color: #d7e3ef;
}
.pop-row {
  display: flex;
  gap: 6px;
}
.grow {
  flex: 1;
  min-width: 0;
}
.presets {
  display: flex;
  gap: 5px;
  margin-top: 6px;
}
.preset {
  padding: 2px 8px;
  border: 1px solid #223243;
  border-radius: 10px;
  background: #0c141d;
  color: #7e91a4;
  font-size: 10px;
  cursor: pointer;
}
.preset:hover {
  border-color: #33506a;
  color: #b9d2e6;
}
.preset.active {
  border-color: rgba(56, 189, 248, 0.6);
  background: rgba(56, 189, 248, 0.12);
  color: #7dd3fc;
}
.pop-meta {
  display: flex;
  align-items: center;
  gap: 5px;
  flex-wrap: wrap;
  margin-top: 8px;
  font-size: 9.5px;
  color: #54687a;
}
.sep {
  opacity: 0.5;
}
.err {
  margin-top: 7px;
  font-size: 10px;
  line-height: 1.5;
  color: #fca5a5;
  word-break: break-all;
}
.cur {
  margin-top: 6px;
  font-size: 9.5px;
  color: #3f5265;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}
</style>
