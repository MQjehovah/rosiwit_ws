import type { LoadedMap } from './mapFile'

/**
 * 本地 PCD 缓存（IndexedDB）。
 *
 * 每个显示项按 id 存一份，刷新页面后自动恢复，不需要重新选择文件。
 * 和栅格缓存一样，存的是前端已经降采样后的 Float32Array。
 */

const DB_NAME = 'debug_hmi_map'
const DB_VERSION = 1
const STORE = 'map'
const SCHEMA = 1

interface CachedMap {
  schema: number
  savedAt: number
  name: string
  frame: string
  count: number
  positions: Float32Array
  /** 旧缓存没有这一项；新版才有，用于「强度/持久度」着色 */
  intensity?: Float32Array | null
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

export async function saveDisplayMapCache(
  id: string,
  map: LoadedMap,
  positions: Float32Array,
): Promise<void> {
  if (typeof indexedDB === 'undefined') return
  const record: CachedMap = {
    schema: SCHEMA,
    savedAt: Date.now(),
    name: map.name,
    frame: map.frame || 'map',
    count: map.count,
    positions,
    intensity: map.intensity ?? null,
  }
  const db = await openDb()
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readwrite')
    tx.objectStore(STORE).put(record, id)
    tx.oncomplete = () => resolve()
    tx.onerror = () => reject(tx.error ?? new Error('map cache write failed'))
  })
  db.close()
}

export async function loadDisplayMapCache(
  id: string,
): Promise<(LoadedMap & { savedAt: number }) | null> {
  if (typeof indexedDB === 'undefined') return null
  const db = await openDb()
  const record = await new Promise<CachedMap | null>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readonly')
    const req = tx.objectStore(STORE).get(id)
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
    intensity:
      record.intensity instanceof Float32Array && record.intensity.length >= record.count
        ? record.intensity
        : null,
    savedAt: record.savedAt,
  }
}

export async function deleteDisplayMapCache(id: string): Promise<void> {
  if (typeof indexedDB === 'undefined') return
  const db = await openDb()
  await new Promise<void>((resolve, reject) => {
    const tx = db.transaction(STORE, 'readwrite')
    tx.objectStore(STORE).delete(id)
    tx.oncomplete = () => resolve()
    tx.onerror = () => reject(tx.error ?? new Error('map cache delete failed'))
  })
  db.close()
}
