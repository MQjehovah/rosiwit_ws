#!/usr/bin/env node
/**
 * Electron 开发模式：
 *   1. 启动一个只监听 127.0.0.1 的 Vite（默认 5178，避免和手动 dev 冲突）
 *   2. 等端口就绪后启动 Electron
 *   3. 任一进程退出时一起清理
 */
import { spawn } from 'node:child_process'
import { createRequire } from 'node:module'
import net from 'node:net'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const require = createRequire(import.meta.url)
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const port = Number(process.env.ELECTRON_DEV_PORT || 5178)
const host = '127.0.0.1'
const url = `http://${host}:${port}/`

function waitPort(port, host, timeoutMs = 30000) {
  const started = Date.now()
  return new Promise((resolve, reject) => {
    const attempt = () => {
      const socket = net.connect({ port, host })
      socket.once('connect', () => {
        socket.destroy()
        resolve()
      })
      socket.once('error', () => {
        socket.destroy()
        if (Date.now() - started > timeoutMs) reject(new Error('等待 Vite 超时'))
        else setTimeout(attempt, 200)
      })
    }
    attempt()
  })
}

const vite = spawn(
  process.execPath,
  [path.join(root, 'node_modules', 'vite', 'bin', 'vite.js'), '--host', host, '--port', String(port), '--strictPort'],
  { cwd: root, stdio: 'inherit', env: process.env },
)

let electron = null
let stopping = false

function stop(code = 0) {
  if (stopping) return
  stopping = true
  try {
    if (electron && !electron.killed) electron.kill()
  } catch {
    /* ignore */
  }
  try {
    if (!vite.killed) vite.kill()
  } catch {
    /* ignore */
  }
  setTimeout(() => process.exit(code), 100)
}

vite.on('exit', (code) => {
  if (!stopping) stop(code ?? 0)
})

waitPort(port, host)
  .then(() => {
    const electronPath = require('electron')
    electron = spawn(electronPath, [root], {
      cwd: root,
      stdio: 'inherit',
      env: { ...process.env, VITE_DEV_SERVER_URL: url },
    })
    electron.on('exit', (code) => stop(code ?? 0))
  })
  .catch((e) => {
    console.error(e.message)
    stop(1)
  })

process.on('SIGINT', () => stop(0))
process.on('SIGTERM', () => stop(0))
