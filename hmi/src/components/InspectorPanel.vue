<template>
  <div class="inspector">
    <div class="tabs">
      <button :class="{ active: tab === 'topics' }" @click="tab = 'topics'">话题</button>
      <button :class="{ active: tab === 'logs' }" @click="tab = 'logs'">
        日志
        <span v-if="errorCount" class="tab-badge">{{ errorCount }}</span>
      </button>
      <button :class="{ active: tab === 'message' }" @click="tab = 'message'">消息</button>
    </div>

    <!-- 话题 -->
    <div v-show="tab === 'topics'" class="tab-body">
      <div class="row">
        <input v-model="search" class="input grow" placeholder="过滤话题…" spellcheck="false" />
        <button class="btn tiny" :disabled="state.topicScanning" @click="refreshTopics">
          {{ state.topicScanning ? '扫描中…' : '扫描全部' }}
        </button>
      </div>
      <table class="topics">
        <thead>
          <tr>
            <th>话题</th>
            <th class="num">Hz</th>
            <th class="num">KB</th>
            <th class="num">年龄</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          <tr
            v-for="topic in visibleTopics"
            :key="topic.name"
            :class="{ selected: state.rawTopic === topic.name, live: topic.subscribed }"
            @click="inspect(topic.name)"
          >
            <td class="name">
              <span class="live-dot" :class="{ on: topic.hz > 0 }" />
              {{ topic.name }}
              <div class="type">{{ topic.type || '—' }}</div>
            </td>
            <td class="num">{{ topic.hz ? topic.hz.toFixed(1) : '—' }}</td>
            <td class="num">{{ topic.size ? (topic.size / 1024).toFixed(1) : '—' }}</td>
            <td class="num">{{ ageText(topic.age) }}</td>
            <td class="act">
              <button v-if="!topic.subscribed" class="btn tiny" @click.stop="subscribe(topic.name)">订阅</button>
            </td>
          </tr>
        </tbody>
      </table>
      <p class="note">
        点击行 = 临时订阅并在“消息”里查看最新一条。点云类话题不建议长期开着。
        “扫描全部”调用 <code>/rosapi/topics</code>，这台设备要 8s 以上，期间所有话题会暂停。
      </p>
    </div>

    <!-- 日志 -->
    <div v-show="tab === 'logs'" class="tab-body">
      <div class="row">
        <select v-model="state.logFilter" class="input">
          <option value="all">全部</option>
          <option value="info">info</option>
          <option value="warn">warn</option>
          <option value="error">error</option>
        </select>
        <button class="btn tiny" :class="{ primary: state.pausedLogs }" @click="state.pausedLogs = !state.pausedLogs">
          {{ state.pausedLogs ? '继续' : '暂停' }}
        </button>
        <button class="btn tiny" @click="state.logs = []">清空</button>
      </div>
      <div class="logs">
        <div v-for="line in filteredLogs" :key="line.id" class="log" :class="line.level">
          <span class="time">{{ clock(line.time) }}</span>
          <span class="text">{{ line.text }}</span>
        </div>
        <div v-if="!filteredLogs.length" class="empty">暂无日志</div>
      </div>
    </div>

    <!-- 消息 -->
    <div v-show="tab === 'message'" class="tab-body">
      <div class="row">
        <input
          v-model="messageTopic"
          class="input grow"
          list="hmi-topics"
          placeholder="/topic"
          spellcheck="false"
        />
        <datalist id="hmi-topics">
          <option v-for="name in state.topicNames" :key="name" :value="name" />
        </datalist>
        <button class="btn tiny" @click="subscribe(messageTopic)">订阅</button>
      </div>
      <div class="row meta">
        <span class="mono">{{ state.rawTopic || '—' }}</span>
        <span>{{ topicType(state.rawTopic) }}</span>
        <label class="pause">
          <input v-model="state.rawPaused" type="checkbox" /> 暂停
        </label>
        <button class="btn tiny" :disabled="!state.rawText" @click="copyRaw">复制</button>
      </div>
      <pre class="raw">{{ state.rawText || '等待消息…' }}</pre>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, ref, watch } from 'vue'
import { state, selectRawTopic, topicType, refreshTopics as refreshRosTopics, pushLog } from '../ros/app'
import { ros } from '../ros/app'

const tab = ref<'topics' | 'logs' | 'message'>('topics')
const search = ref('')
const messageTopic = ref(state.rawTopic || '/slam/info')

const errorCount = computed(
  () => state.logs.filter((l) => l.level === 'error').length,
)

const filteredLogs = computed(() => {
  if (state.logFilter === 'all') return state.logs
  return state.logs.filter((l) => l.level === state.logFilter)
})

interface TopicRow {
  name: string
  type: string
  hz: number
  size: number
  age: number
  subscribed: boolean
}

const visibleTopics = computed<TopicRow[]>(() => {
  const names = new Set<string>([
    ...Object.keys(state.stats),
    ...state.topicNames,
  ])
  const query = search.value.trim().toLowerCase()
  const rows: TopicRow[] = []
  for (const name of names) {
    if (query && !name.toLowerCase().includes(query)) continue
    const stat = state.stats[name]
    rows.push({
      name,
      type: topicType(name),
      hz: stat?.hz ?? 0,
      size: stat?.size ?? 0,
      age: stat?.age ?? -1,
      subscribed: ros.isSubscribed(name),
    })
  }
  // 活跃话题排前面，其余按名字
  rows.sort((a, b) => {
    if ((b.hz > 0 ? 1 : 0) !== (a.hz > 0 ? 1 : 0)) return (b.hz > 0 ? 1 : 0) - (a.hz > 0 ? 1 : 0)
    return a.name.localeCompare(b.name)
  })
  return rows
})

function ageText(age: number): string {
  if (age < 0) return '—'
  if (age < 1000) return `${age}ms`
  return `${(age / 1000).toFixed(1)}s`
}

function clock(time: number): string {
  const d = new Date(time)
  const pad = (n: number, w = 2) => String(n).padStart(w, '0')
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`
}

let tempUnsub: (() => void) | null = null
let lastFormat = 0

function sanitize(value: any, depth = 0): any {
  if (value === null || value === undefined) return value
  if (Array.isArray(value)) {
    if (value.length > 24) {
      return [...value.slice(0, 12).map((v) => sanitize(v, depth + 1)), `… 共 ${value.length} 项`]
    }
    return value.map((v) => sanitize(v, depth + 1))
  }
  if (typeof value === 'string' && value.length > 240) {
    return `${value.slice(0, 240)}…（${value.length} 字符）`
  }
  if (typeof value === 'object') {
    if (depth > 6) return '{…}'
    const out: Record<string, any> = {}
    for (const [k, v] of Object.entries(value)) out[k] = sanitize(v, depth + 1)
    return out
  }
  return value
}

function subscribe(topic: string): void {
  const name = topic.trim()
  if (!name) return
  messageTopic.value = name
  selectRawTopic(name)
  tab.value = 'message'
  if (!ros.isSubscribed(name)) {
    const type = topicType(name)
    ros.subscribe(name, { throttle: 300, queue: 1, ...(type ? { type } : {}) })
  }
  tempUnsub?.()
  tempUnsub = ros.onTopic(name, (msg) => {
    if (state.rawPaused) return
    const now = performance.now()
    if (now - lastFormat < 250) return
    lastFormat = now
    try {
      state.rawText = JSON.stringify(sanitize(msg), null, 1)
      state.rawTime = Date.now()
    } catch {
      state.rawText = '(消息无法序列化)'
    }
  })
}

function inspect(topic: string): void {
  subscribe(topic)
}

async function refreshTopics(): Promise<void> {
  await refreshRosTopics()
}

async function copyRaw(): Promise<void> {
  try {
    await navigator.clipboard.writeText(state.rawText)
    pushLog('info', '已复制当前消息')
  } catch {
    pushLog('warn', '复制失败（浏览器不允许剪贴板）')
  }
}

watch(
  () => state.rawTopic,
  (topic) => {
    if (topic) messageTopic.value = topic
  },
)

onBeforeUnmount(() => tempUnsub?.())
</script>

<style scoped>
.inspector {
  display: flex;
  flex-direction: column;
  height: 100%;
  min-height: 0;
  background: var(--panel);
  border-left: 1px solid var(--line);
}

.tabs {
  display: flex;
  border-bottom: 1px solid var(--line);
  flex: none;
}
.tabs button {
  flex: 1;
  padding: 8px 0;
  font-size: 12px;
  color: var(--dim);
  border: 0;
  border-bottom: 2px solid transparent;
  background: transparent;
}
.tabs button.active {
  color: var(--text);
  border-bottom-color: var(--accent);
}
.tab-badge {
  display: inline-block;
  margin-left: 4px;
  min-width: 15px;
  padding: 0 4px;
  border-radius: 8px;
  background: rgba(248, 113, 113, 0.18);
  color: #fca5a5;
  font-size: 10px;
}

.tab-body {
  flex: 1;
  min-height: 0;
  display: flex;
  flex-direction: column;
  padding: 8px;
  gap: 6px;
  overflow: hidden;
}

.row {
  display: flex;
  gap: 6px;
  align-items: center;
  flex: none;
}
.grow {
  flex: 1;
  min-width: 0;
}

.topics {
  width: 100%;
  border-collapse: collapse;
  font-size: 10.5px;
  display: block;
  overflow: auto;
  flex: 1;
}
.topics thead {
  position: sticky;
  top: 0;
  background: var(--panel);
  z-index: 1;
}
.topics th {
  text-align: left;
  font-weight: 500;
  color: var(--dim);
  padding: 3px 4px;
  border-bottom: 1px solid var(--line);
  white-space: nowrap;
}
.topics th.num,
.topics td.num {
  text-align: right;
  font-variant-numeric: tabular-nums;
  white-space: nowrap;
}
.topics td {
  padding: 3px 4px;
  border-bottom: 1px solid rgba(30, 42, 56, 0.5);
  vertical-align: top;
}
.topics tr {
  cursor: pointer;
}
.topics tbody tr:hover {
  background: rgba(56, 189, 248, 0.06);
}
.topics tr.selected {
  background: rgba(56, 189, 248, 0.1);
}
.topics tr.live .name {
  color: var(--text);
}
.name {
  color: #93a8bc;
  word-break: break-all;
  line-height: 1.35;
}
.type {
  font-size: 9.5px;
  color: #54687a;
  word-break: break-all;
}
.live-dot {
  display: inline-block;
  width: 5px;
  height: 5px;
  margin-right: 4px;
  border-radius: 50%;
  background: #33475a;
  vertical-align: middle;
}
.live-dot.on {
  background: var(--ok);
}
.topics td.act {
  width: 1%;
  padding-left: 0;
}

.note {
  margin: 0;
  font-size: 10px;
  color: var(--dim);
  line-height: 1.5;
  flex: none;
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
  padding: 2px 0;
  color: #9fb4c7;
  border-bottom: 1px solid rgba(30, 42, 56, 0.35);
}
.log .time {
  flex: none;
  color: #4f6375;
  font-variant-numeric: tabular-nums;
}
.log .text {
  word-break: break-word;
}
.log.warn {
  color: #fcd34d;
}
.log.error {
  color: #fca5a5;
}
.empty {
  color: var(--dim);
  padding: 12px 4px;
  font-size: 11px;
}

.meta {
  font-size: 10px;
  color: var(--dim);
  flex-wrap: wrap;
}
.mono {
  color: #7dd3fc;
  font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
}
.pause {
  display: flex;
  align-items: center;
  gap: 3px;
  margin-left: auto;
}
.raw {
  flex: 1;
  min-height: 0;
  overflow: auto;
  margin: 0;
  padding: 8px;
  border-radius: 6px;
  background: #080e15;
  border: 1px solid var(--line);
  font-size: 10.5px;
  line-height: 1.5;
  color: #a8c0d4;
  white-space: pre-wrap;
  word-break: break-all;
}
</style>
