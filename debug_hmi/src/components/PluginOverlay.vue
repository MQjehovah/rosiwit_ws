<template>
  <div ref="layerEl" class="plugin-layer">
    <div
      v-for="p in canvasPlugins"
      :key="p.id"
      class="plugin-float"
      :style="{
        left: `${clampX(p)}px`,
        top: `${clampY(p)}px`,
        width: `${p.w}px`,
        maxHeight: `calc(100% - ${clampY(p) + 8}px)`,
      }"
      @pointerdown.stop="onPointerDown(p, $event)"
    >
      <PluginCard
        :plugin="p"
        mode="canvas"
        @toggle="setPluginEnabled(p.id, $event)"
        @undock="undockPlugin(p.id)"
      />
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, reactive, ref } from 'vue'
import {
  commitPlugins,
  movePlugin,
  setPluginEnabled,
  state,
  undockPlugin,
  type PluginItem,
} from '../ros/app'
import PluginCard from './plugins/PluginCard.vue'

const layerEl = ref<HTMLDivElement | null>(null)
const size = reactive({ w: 0, h: 0 })

const canvasPlugins = computed(() => state.plugins.filter((p) => p.dock === 'canvas'))

let observer: ResizeObserver | null = null
onMounted(() => {
  const el = layerEl.value
  if (!el) return
  const measure = () => {
    size.w = el.clientWidth
    size.h = el.clientHeight
  }
  measure()
  observer = new ResizeObserver(measure)
  observer.observe(el)
})
onBeforeUnmount(() => {
  observer?.disconnect()
  stopDrag()
})

function clampX(p: PluginItem): number {
  const max = Math.max(4, size.w - p.w - 4)
  return Math.max(4, Math.min(max, p.x))
}
function clampY(p: PluginItem): number {
  const max = Math.max(4, size.h - 60)
  return Math.max(4, Math.min(max, p.y))
}

interface DragState {
  id: string
  offsetX: number
  offsetY: number
}
let drag: DragState | null = null

function onPointerDown(p: PluginItem, e: PointerEvent): void {
  const target = e.target as HTMLElement | null
  if (target?.closest('button,input,select,label,summary,a')) return
  // 头部整条都能拖；正文里只在空白处拖，避免影响选中/复制原文
  if (!target?.closest('.pc-head') && target?.closest('pre,code')) return
  const el = layerEl.value
  if (!el) return
  const rect = el.getBoundingClientRect()
  drag = {
    id: p.id,
    offsetX: e.clientX - rect.left - clampX(p),
    offsetY: e.clientY - rect.top - clampY(p),
  }
  e.preventDefault()
  window.addEventListener('pointermove', onPointerMove)
  window.addEventListener('pointerup', stopDrag)
  window.addEventListener('pointercancel', stopDrag)
}

function onPointerMove(e: PointerEvent): void {
  if (!drag) return
  const el = layerEl.value
  if (!el) return
  const p = state.plugins.find((x) => x.id === drag?.id)
  if (!p) return
  const rect = el.getBoundingClientRect()
  const x = e.clientX - rect.left - drag.offsetX
  const y = e.clientY - rect.top - drag.offsetY
  movePlugin(p.id, Math.min(x, Math.max(4, rect.width - p.w - 4)), Math.min(y, Math.max(4, rect.height - 180)))
}

function stopDrag(): void {
  window.removeEventListener('pointermove', onPointerMove)
  window.removeEventListener('pointerup', stopDrag)
  window.removeEventListener('pointercancel', stopDrag)
  if (!drag) return
  drag = null
  commitPlugins()
}
</script>

<style scoped>
.plugin-layer {
  position: absolute;
  inset: 0;
  z-index: 15;
  pointer-events: none;
  overflow: hidden;
}
.plugin-float {
  position: absolute;
  display: flex;
  flex-direction: column;
  max-height: calc(100% - 8px);
  pointer-events: auto;
  touch-action: none;
}
</style>
