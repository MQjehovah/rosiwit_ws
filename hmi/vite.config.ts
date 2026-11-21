import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// 前端只通过 WebSocket 访问 rosbridge，不需要任何反向代理。
// base: './' 让 dist 可以直接丢到任意静态目录（甚至 file:// 之外的本地 http 服务）。
export default defineConfig({
  base: './',
  plugins: [vue()],
  server: {
    host: '0.0.0.0',
    port: 5173,
  },
  build: {
    target: 'es2020',
    chunkSizeWarningLimit: 1200,
  },
})
