import type { LoadedMap } from './mapFile'

/**
 * PCD 地图缓存：浏览器里加载一次后，刷新页面不再需要重新选文件。
 * 存的是降采样后的 Float32Array（约几 MB）。
 */

const DB_NAME = 'debug_hmi_map'
const DB_VERSION = 1
const STORE = 'map'
const KEY = 'latest'
const SCHEMA = 1

interface CachedMap {
  schema: number
  savedAt: number
  name: string
  frame: string
  count: number
  positions: Float32Array
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

export async function saveMapCache(map: LoadedMap): Promise<void> {
  if (typeof indexedDB === 'undefined') return
  const record: CachedMap = {
    schema: SCHEMA,
    savedAt: Date.now(),
    name: map.name,
    frame: map.frame,
    count: map.count,
    positions: map.positions,
  }
  const db = await openDb()
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readwrite')
    tx.objectStore(STORE).put(record, KEY)
    tx.oncomplete = () => resolve()
    tx.onerror = () => reject(tx.error ?? new Error('map cache write failed'))
  })
  db.close()
}

export async function loadMapCache(): Promise<(LoadedMap & { savedAt: number }) | null> {
  if (typeof indexedDB === 'undefined') return null
  const db = await openDb()
  const record = await new Promise<CachedMap | null>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readonly')
    const req = tx.objectStore(STORE).get(KEY)
    req.onsuccess = () => resolve((req.result as CachedMap) ?? null)
    req.onerror = () => reject(req.error ?? new Error('map cache read failed'))
  })
  db.close()
  if (!record || record.schema !== SCHEMA || !record.count) return null
  return {
    name: record.name,
    frame: record.frame || 'map',
    count: record.count,
    positions: record.positions,
    savedAt: record.savedAt,
  }
}

export async function clearMapCache(): Promise<void> {
  if (typeof indexedDB === 'undefined') return
  const db = await openDb()
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readwrite')
    tx.objectStore(STORE).delete(KEY)
    tx.oncomplete = () => resolve()
    tx.onerror = () => reject(tx.error ?? new Error('map cache delete failed'))
  })
  db.close()
}
