<template>
  <section class="log-panel" :class="{ collapsed }">
    <div class="log-bar">
      <button class="toggle" type="button" @click="collapsed = !collapsed">
        {{ collapsed ? '▲ 日志' : '▼ 日志' }}
      </button>
      <span v-if="errorCount" class="err">错误 {{ errorCount }}</span>
      <span class="spacer" />
      <template v-if="!collapsed">
        <select v-model="moduleFilter" class="filter" title="按模块筛选">
          <option value="all">全部模块</option>
          <option value="__local__">本机</option>
          <option v-for="m in modules" :key="m" :value="m">{{ m }}</option>
        </select>
        <select v-model="state.logFilter" class="filter" title="按等级筛选">
          <option value="all">全部等级</option>
          <option value="info">info</option>
          <option value="warn">warn</option>
          <option value="error">error</option>
        </select>
        <button class="mini" :class="{ on: state.pausedLogs }" @click="state.pausedLogs = !state.pausedLogs">
          {{ state.pausedLogs ? '继续' : '暂停' }}
        </button>
        <button class="mini" @click="state.logs = []">清空</button>
      </template>
    </div>

    <div v-show="!collapsed" class="log-body">
      <div v-if="!state.layers.logs" class="prompt">
        未订阅 <code>/rosout_agg</code>
        <button class="mini" @click="setLayer('logs', true)">订阅</button>
      </div>

      <div class="logs">
        <div v-for="line in filteredLogs" :key="line.id" class="log" :class="line.level">
          <span class="time">{{ clock(line.time) }}</span>
          <span
            v-if="line.source"
            class="src"
            :title="`点击静音 ${line.source}`"
            @click="muteSource(line.source)"
          >
            {{ line.source }}
          </span>
          <span class="text">{{ line.text }}</span>
          <span v-if="line.count > 1" class="count">×{{ line.count }}</span>
        </div>
        <div v-if="!filteredLogs.length" class="empty">暂无日志</div>
      </div>

      <div v-if="state.mutedSources.length" class="muted-row">
        <span class="muted-label">已静音</span>
        <button
          v-for="source in state.mutedSources"
          :key="source"
          class="muted-chip"
          :title="`恢复 ${source}`"
          @click="unmuteSource(source)"
        >
          {{ source }} ×
        </button>
        <button class="muted-chip clear" @click="clearMutedSources">全部恢复</button>
      </div>
    </div>
  </section>
</template>

<script setup lang="ts">
import { computed, ref } from 'vue'
import {
  clearMutedSources,
  muteSource,
  setLayer,
  state,
  unmuteSource,
} from '../ros/app'

const collapsed = ref(false)
const moduleFilter = ref('all')

const modules = computed(() =>
  Array.from(new Set(state.logs.map((l) => l.source).filter((x): x is string => !!x))).sort(),
)

const errorCount = computed(() => state.logs.filter((l) => l.level === 'error').length)

const filteredLogs = computed(() => {
  const muted = new Set(state.mutedSources)
  return state.logs.filter((l) => {
    if (state.logFilter !== 'all' && l.level !== state.logFilter) return false
    if (moduleFilter.value === '__local__') {
      if (l.source) return false
    } else if (moduleFilter.value !== 'all' && l.source !== moduleFilter.value) {
      return false
    }
    if (l.source && muted.has(l.source)) return false
    return true
  })
})

function clock(time: number): string {
  const d = new Date(time)
  const pad = (n: number, w = 2) => String(n).padStart(w, '0')
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`
}
</script>

<style scoped>
.log-panel {
  flex: none;
  display: flex;
  flex-direction: column;
  height: 190px;
  border-top: 1px solid var(--line);
  background: #0a1118;
  min-height: 0;
}

.log-panel.collapsed {
  height: 32px;
}

.log-bar {
  flex: none;
  height: 32px;
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 0 10px;
  border-bottom: 1px solid rgba(30, 42, 56, 0.55);
  background: #0c141d;
  font-size: 10.5px;
  color: var(--dim);
}

.toggle {
  border: 0;
  background: transparent;
  color: #a9bccd;
  font-size: 10.5px;
  font-weight: 600;
  letter-spacing: 0.05em;
  cursor: pointer;
  padding: 0;
}

.toggle:hover {
  color: #e2f3ff;
}

.err {
  padding: 1px 6px;
  border-radius: 8px;
  background: rgba(248, 113, 113, 0.14);
  border: 1px solid rgba(248, 113, 113, 0.35);
  color: #fca5a5;
  font-size: 9.5px;
}

.spacer {
  flex: 1;
}

.filter {
  background: #0b141d;
  border: 1px solid var(--line);
  color: var(--text);
  border-radius: 4px;
  padding: 1px 4px;
  font-size: 10px;
}

.mini {
  border: 1px solid var(--line);
  background: #0b141d;
  color: #9fb4c7;
  border-radius: 4px;
  padding: 1px 7px;
  font-size: 10px;
  cursor: pointer;
}

.mini:hover {
  color: #e2f3ff;
  border-color: #3b6d8d;
}

.mini.on {
  color: #fbbf24;
  border-color: #7a5a1d;
}

.log-body {
  flex: 1;
  min-height: 0;
  display: flex;
  flex-direction: column;
  padding: 2px 10px 4px;
}

.prompt {
  flex: none;
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 5px 0;
  font-size: 10.5px;
  color: #7e91a4;
}

.prompt code {
  color: #7dd3fc;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}

.logs {
  flex: 1;
  overflow: auto;
  font-size: 10.5px;
  line-height: 1.55;
}

.log {
  display: flex;
  gap: 6px;
  padding: 1px 0;
  color: #9fb4c7;
  border-bottom: 1px solid rgba(30, 42, 56, 0.35);
}

.log .time {
  flex: none;
  color: #4f6375;
  font-variant-numeric: tabular-nums;
}

.log .src {
  flex: none;
  color: #7dd3fc;
  cursor: pointer;
}

.log .src:hover {
  text-decoration: underline;
}

.log .count {
  flex: none;
  margin-left: auto;
  color: #fbbf24;
  font-variant-numeric: tabular-nums;
}

.log .text {
  word-break: break-word;
  min-width: 0;
}

.log.warn {
  color: #fcd34d;
}

.log.error {
  color: #fca5a5;
}

.empty {
  color: var(--dim);
  padding: 10px 4px;
  font-size: 11px;
}

.muted-row {
  flex: none;
  display: flex;
  align-items: center;
  gap: 4px;
  flex-wrap: wrap;
  padding-top: 4px;
  border-top: 1px solid rgba(30, 42, 56, 0.5);
}

.muted-label {
  font-size: 9.5px;
  color: var(--dim);
}

.muted-chip {
  padding: 1px 6px;
  border: 1px solid #2a3a4b;
  border-radius: 9px;
  background: #0c141d;
  color: #7e91a4;
  font-size: 9.5px;
  cursor: pointer;
}

.muted-chip:hover {
  color: #cfe4f5;
  border-color: #3a536a;
}

.muted-chip.clear {
  color: #7dd3fc;
}
</style>
