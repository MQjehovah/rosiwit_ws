import type { GridData } from './decode'

/**
 * 栅格快照缓存（IndexedDB）。
 *
 * /map 一帧 43MB、设备端要打包 1~2 分钟，刷新页面不应该再来一次。
 * 直接把 downsample 前的 Int8Array 存下来（约 43MB），启动时恢复并重新生成纹理。
 */

const DB_NAME = 'debug_hmi'
const DB_VERSION = 1
const STORE = 'grid'
const KEY = 'latest'
const SCHEMA = 1

interface CachedGrid {
  schema: number
  savedAt: number
  width: number
  height: number
  resolution: number
  origin: { x: number; y: number; theta: number }
  frame: string
  data: Int8Array
}

function openDb(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VERSION)
    req.onupgradeneeded = () => {
      const db = req.result
      if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE)
    }
    req.onsuccess = () => resolve(req.result)
    req.onerror = () => reject(req.error ?? new Error('indexedDB open failed'))
  })
}

export async function saveGridCache(grid: GridData): Promise<void> {
  if (typeof indexedDB === 'undefined') return
  const record: CachedGrid = {
    schema: SCHEMA,
    savedAt: Date.now(),
    width: grid.width,
    height: grid.height,
    resolution: grid.resolution,
    origin: grid.origin,
    frame: grid.frame,
    data: grid.data,
  }
  const db = await openDb()
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readwrite')
    tx.objectStore(STORE).put(record, KEY)
    tx.oncomplete = () => resolve()
    tx.onerror = () => reject(tx.error ?? new Error('grid cache write failed'))
  })
  db.close()
}

export async function loadGridCache(): Promise<{ grid: GridData; savedAt: number } | null> {
  if (typeof indexedDB === 'undefined') return null
  const db = await openDb()
  const record = await new Promise<CachedGrid | null>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readonly')
    const req = tx.objectStore(STORE).get(KEY)
    req.onsuccess = () => resolve((req.result as CachedGrid) ?? null)
    req.onerror = () => reject(req.error ?? new Error('grid cache read failed'))
  })
  db.close()
  if (!record || record.schema !== SCHEMA) return null
  if (!record.width || !record.height) return null

  const raw = record.data
  const data =
    raw instanceof Int8Array
      ? raw
      : new Int8Array(raw as unknown as ArrayBuffer, 0, record.width * record.height)

  return {
    savedAt: record.savedAt,
    grid: {
      width: record.width,
      height: record.height,
      resolution: record.resolution,
      origin: record.origin,
      frame: record.frame || 'map',
      data,
    },
  }
}

export async function clearGridCache(): Promise<void> {
  if (typeof indexedDB === 'undefined') return
  const db = await openDb()
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readwrite')
    tx.objectStore(STORE).delete(KEY)
    tx.oncomplete = () => resolve()
    tx.onerror = () => reject(tx.error ?? new Error('grid cache delete failed'))
  })
  db.close()
}
