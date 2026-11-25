<template>
  <div class="app">
    <header class="topbar">
      <div class="brand">rosiwit<span>debug_hmi</span></div>
      <ConnectionPill />

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
      <main class="center">
        <div
          ref="sceneEl"
          class="scene"
          @dragover.prevent
          @drop.prevent="onDropPlugin"
        >
          <SceneView />
          <PluginOverlay />
        </div>
        <LogPanel />
      </main>
      <aside v-show="showRight" class="right"><InspectorPanel /></aside>
    </div>
  </div>
</template>

<script setup lang="ts">
import { onMounted, ref } from 'vue'
import ConnectionPill from './components/ConnectionPill.vue'
import ControlPanel from './components/ControlPanel.vue'
import InspectorPanel from './components/InspectorPanel.vue'
import LogPanel from './components/LogPanel.vue'
import PluginOverlay from './components/PluginOverlay.vue'
import SceneView from './three/SceneView.vue'
import { dockPluginToCanvas, initHmi } from './ros/app'

const showLeft = ref(true)
const showRight = ref(true)
const sceneEl = ref<HTMLDivElement | null>(null)

function onDropPlugin(e: DragEvent): void {
  const id = e.dataTransfer?.getData('text/plain') || ''
  if (!id || !sceneEl.value) return
  const rect = sceneEl.value.getBoundingClientRect()
  dockPluginToCanvas(id, e.clientX - rect.left - 20, e.clientY - rect.top - 16)
}

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
  display: flex;
  flex-direction: column;
}

.scene {
  flex: 1;
  min-height: 0;
  position: relative;
}
.right {
  min-width: 0;
  min-height: 0;
  overflow: hidden;
}
</style>
