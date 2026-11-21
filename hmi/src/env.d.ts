/// <reference types="vite/client" />

interface ImportMetaEnv {
  /** 默认 rosbridge 地址（可覆盖局域网默认值） */
  readonly VITE_ROSBRIDGE_URL?: string
  /** 远程隧道 rosbridge 地址（放在 .env.local，不要提交 token） */
  readonly VITE_TUNNEL_URL?: string
}

interface ImportMeta {
  readonly env: ImportMetaEnv
}

declare module '*.vue' {
  import type { DefineComponent } from 'vue'
  const component: DefineComponent<{}, {}, any>
  export default component
}
