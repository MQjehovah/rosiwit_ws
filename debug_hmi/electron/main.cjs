/* Electron 主进程：开发时加载 Vite，生产时用本地 HTTP 服务加载 dist。 */
const { app, BrowserWindow, Menu, session, shell } = require('electron')
const fs = require('node:fs')
const http = require('node:http')
const path = require('node:path')

const isDev = !!process.env.VITE_DEV_SERVER_URL
const ROOT = path.join(__dirname, '..')

let mainWindow = null
let staticServer = null

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.webp': 'image/webp',
  '.ico': 'image/x-icon',
  '.glb': 'model/gltf-binary',
  '.gltf': 'model/gltf+json',
  '.wasm': 'application/wasm',
  '.woff': 'font/woff',
  '.woff2': 'font/woff2',
}

function startStaticServer(root, preferredPort = 5177) {
  return new Promise((resolve, reject) => {
    const server = http.createServer((req, res) => {
      try {
        const pathname = decodeURIComponent(new URL(req.url, 'http://127.0.0.1').pathname)
        const rel = pathname === '/' ? 'index.html' : pathname.replace(/^\/+/, '')
        let file = path.join(root, rel)
        if (!file.startsWith(root)) {
          res.writeHead(403)
          res.end('forbidden')
          return
        }
        if (!fs.existsSync(file) || fs.statSync(file).isDirectory()) {
          file = path.join(root, 'index.html')
        }
        const body = fs.readFileSync(file)
        res.writeHead(200, {
          'Content-Type': MIME[path.extname(file).toLowerCase()] || 'application/octet-stream',
          'Cache-Control': 'no-cache',
        })
        res.end(body)
      } catch (e) {
        res.writeHead(500)
        res.end(String(e))
      }
    })
    server.once('error', (err) => {
      if (err && err.code === 'EADDRINUSE') {
        // 端口被占时退回随机端口；注意这会让 IndexedDB/localStorage 换 origin
        server.listen(0, '127.0.0.1', () => resolve(server))
        return
      }
      reject(err)
    })
    server.listen(preferredPort, '127.0.0.1', () => resolve(server))
  })
}

function stateFile() {
  return path.join(app.getPath('userData'), 'window-state.json')
}

function loadWindowState() {
  try {
    const raw = JSON.parse(fs.readFileSync(stateFile(), 'utf8'))
    if (raw && Number.isFinite(raw.width) && Number.isFinite(raw.height)) return raw
  } catch {
    /* 首次启动 */
  }
  return { width: 1440, height: 900 }
}

function saveWindowState(win) {
  try {
    if (!win || win.isDestroyed()) return
    const bounds = win.getNormalBounds()
    fs.writeFileSync(stateFile(), JSON.stringify(bounds))
  } catch {
    /* ignore */
  }
}

function buildMenu() {
  const template = [
    {
      label: '文件',
      submenu: [{ role: 'quit', label: '退出' }],
    },
    {
      label: '视图',
      submenu: [
        { role: 'reload', label: '刷新' },
        { role: 'forceReload', label: '强制刷新' },
        { role: 'toggleDevTools', label: '开发者工具' },
        { type: 'separator' },
        { role: 'resetZoom', label: '实际大小' },
        { role: 'zoomIn', label: '放大' },
        { role: 'zoomOut', label: '缩小' },
        { type: 'separator' },
        { role: 'togglefullscreen', label: '全屏' },
      ],
    },
  ]
  Menu.setApplicationMenu(Menu.buildFromTemplate(template))
}

async function createWindow() {
  const state = loadWindowState()
  const win = new BrowserWindow({
    width: state.width,
    height: state.height,
    x: state.x,
    y: state.y,
    minWidth: 1100,
    minHeight: 700,
    backgroundColor: '#070b10',
    title: 'rosiwit debug_hmi',
    show: false,
    webPreferences: {
      preload: path.join(__dirname, 'preload.cjs'),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
      webviewTag: false,
    },
  })
  mainWindow = win
  win.once('ready-to-show', () => win.show())
  win.on('close', () => saveWindowState(win))
  win.on('closed', () => {
    if (mainWindow === win) mainWindow = null
  })
  win.setMenuBarVisibility(false)

  win.webContents.setWindowOpenHandler(({ url }) => {
    if (/^https?:/i.test(url)) shell.openExternal(url)
    return { action: 'deny' }
  })
  win.webContents.on('will-navigate', (event, url) => {
    const current = win.webContents.getURL()
    if (current && url !== current) event.preventDefault()
  })

  if (isDev) {
    await win.loadURL(process.env.VITE_DEV_SERVER_URL)
  } else {
    staticServer = await startStaticServer(path.join(ROOT, 'dist'))
    const port = staticServer.address().port
    await win.loadURL(`http://127.0.0.1:${port}/`)
  }
}

if (!app.requestSingleInstanceLock()) {
  app.quit()
} else {
  app.on('second-instance', () => {
    if (!mainWindow) return
    if (mainWindow.isMinimized()) mainWindow.restore()
    mainWindow.focus()
  })

  app.whenReady().then(async () => {
    app.setAppUserModelId('com.rosiwit.debughmi')
    session.defaultSession.setPermissionRequestHandler((_wc, _permission, callback) => callback(false))
    buildMenu()
    await createWindow()
    app.on('activate', () => {
      if (BrowserWindow.getAllWindows().length === 0) void createWindow()
    })
  })

  app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') app.quit()
  })

  app.on('before-quit', () => {
    if (staticServer) staticServer.close()
  })
}
