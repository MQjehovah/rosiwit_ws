/**
 * 前端本地 PCD 加载（标准 PCD）：
 *   - DATA ascii
 *   - DATA binary
 *   - DATA binary_compressed（PCL 的 LZF；机器人 map.pcd 就是这种）
 *
 * 不传服务器，浏览器本地解析后直接渲染。
 */

export interface LoadedMap {
  positions: Float32Array
  count: number
  frame: string
  name: string
}

export async function parseMapFile(file: File): Promise<LoadedMap> {
  const buffer = await file.arrayBuffer()
  return parsePcdBuffer(new Uint8Array(buffer), file.name)
}

/** 供测试/非 File 场景使用 */
export function parsePcdBuffer(bytes: Uint8Array, name: string): LoadedMap {
  const head = new TextDecoder().decode(bytes.subarray(0, Math.min(bytes.length, 512)))
  if (!head.startsWith('# .PCD') && !head.startsWith('VERSION')) {
    throw new Error('不是 PCD 文件（支持 .pcd）')
  }
  return parsePcd(bytes, name)
}

function headerEnd(bytes: Uint8Array): { text: string; offset: number } {
  const decoder = new TextDecoder('latin1')
  const max = Math.min(bytes.length, 8192)
  const text = decoder.decode(bytes.subarray(0, max))
  const idx = text.indexOf('\nDATA ')
  if (idx < 0) throw new Error('PCD 头部缺少 DATA')
  const lineEnd = text.indexOf('\n', idx + 1)
  if (lineEnd < 0) throw new Error('PCD 头部不完整')
  return { text: text.slice(0, lineEnd + 1), offset: lineEnd + 1 }
}

function lzfDecompress(input: Uint8Array, outLen: number): Uint8Array {
  const out = new Uint8Array(outLen)
  let i = 0
  let o = 0
  while (i < input.length && o < outLen) {
    const ctrl = input[i++]
    if (ctrl < 32) {
      const n = ctrl + 1
      out.set(input.subarray(i, i + n), o)
      i += n
      o += n
    } else {
      let len = ctrl >> 5
      let ref = o - ((ctrl & 0x1f) << 8) - 1
      if (len === 7) len += input[i++]
      ref -= input[i++]
      if (ref < 0) throw new Error('LZF 数据损坏')
      for (let k = 0; k < len + 2 && o < outLen; k++) out[o++] = out[ref++]
    }
  }
  return out
}

function parsePcd(bytes: Uint8Array, name: string): LoadedMap {
  const { text, offset } = headerEnd(bytes)
  const header: Record<string, string[]> = {}
  for (const line of text.split('\n')) {
    const parts = line.trim().split(/\s+/)
    if (parts.length) header[parts[0].toUpperCase()] = parts.slice(1)
  }
  const fields = header.FIELDS || []
  const sizes = (header.SIZE || []).map(Number)
  const types = header.TYPE || []
  const counts = (header.COUNT || fields.map(() => '1')).map(Number)
  const points = Number((header.POINTS || header.WIDTH || ['0'])[0])
  const dataType = (header.DATA || ['ascii'])[0]
  const ix = fields.indexOf('x')
  const iy = fields.indexOf('y')
  const iz = fields.indexOf('z')
  if (ix < 0 || iy < 0 || iz < 0) throw new Error('PCD 缺少 x/y/z 字段')
  if (!points) throw new Error('PCD 点数为 0')

  if (dataType === 'ascii') {
    const lines = new TextDecoder().decode(bytes.subarray(offset)).trim().split('\n')
    const out = new Float32Array(lines.length * 3)
    let n = 0
    for (const line of lines) {
      if (!line.trim()) continue
      const vals = line.trim().split(/\s+/)
      out[n * 3] = Number(vals[ix]) || 0
      out[n * 3 + 1] = Number(vals[iy]) || 0
      out[n * 3 + 2] = Number(vals[iz]) || 0
      n++
    }
    return { positions: out.slice(0, n * 3), count: n, frame: 'map', name }
  }

  if (dataType === 'binary') {
    let pointStep = 0
    const offsets: number[] = []
    for (let i = 0; i < fields.length; i++) {
      offsets.push(pointStep)
      pointStep += (sizes[i] || 4) * (counts[i] || 1)
    }
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
    const out = new Float32Array(points * 3)
    const read = (base: number, idx: number): number => {
      const o = base + offsets[idx]
      if (types[idx] === 'F' && sizes[idx] === 8) return view.getFloat64(o, true)
      return view.getFloat32(o, true)
    }
    let n = 0
    for (let i = 0; i < points; i++) {
      const base = offset + i * pointStep
      if (base + pointStep > bytes.byteLength) break
      out[n * 3] = read(base, ix)
      out[n * 3 + 1] = read(base, iy)
      out[n * 3 + 2] = read(base, iz)
      n++
    }
    return { positions: out.slice(0, n * 3), count: n, frame: 'map', name }
  }

  if (dataType === 'binary_compressed') {
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
    const compressedSize = view.getUint32(offset, true)
    const uncompressedSize = view.getUint32(offset + 4, true)
    const compressed = bytes.subarray(offset + 8, offset + 8 + compressedSize)
    const raw = lzfDecompress(compressed, uncompressedSize)

    // PCL binary_compressed 是 SoA：每个字段连续 N 个元素
    let cursor = 0
    const arrays: Record<string, Float32Array> = {}
    for (let i = 0; i < fields.length; i++) {
      const elemSize = sizes[i] || 4
      const count = counts[i] || 1
      const byteLen = points * elemSize * count
      if (byteLen > raw.byteLength - cursor) break
      if (count === 1 && (elemSize === 4 || elemSize === 8)) {
        const copy = raw.slice(cursor, cursor + byteLen)
        arrays[fields[i]] =
          elemSize === 8
            ? new Float32Array(new Float64Array(copy.buffer))
            : new Float32Array(copy.buffer, 0, points)
      }
      cursor += byteLen
    }
    const x = arrays.x
    const y = arrays.y
    const z = arrays.z
    if (!x || !y || !z) throw new Error('PCD 解压后缺少 x/y/z')
    const out = new Float32Array(points * 3)
    for (let i = 0; i < points; i++) {
      out[i * 3] = x[i]
      out[i * 3 + 1] = y[i]
      out[i * 3 + 2] = z[i]
    }
    return { positions: out, count: points, frame: 'map', name }
  }

  throw new Error(`不支持的 PCD DATA=${dataType}`)
}

/** 太大的文件按步长抽稀，保证渲染流畅 */
export function limitCloud(cloud: LoadedMap, maxPoints = 1200000): LoadedMap {
  if (cloud.count <= maxPoints) return cloud
  const stride = Math.ceil(cloud.count / maxPoints)
  const count = Math.ceil(cloud.count / stride)
  const positions = new Float32Array(count * 3)
  let n = 0
  for (let i = 0; i < cloud.count; i += stride) {
    positions[n * 3] = cloud.positions[i * 3]
    positions[n * 3 + 1] = cloud.positions[i * 3 + 1]
    positions[n * 3 + 2] = cloud.positions[i * 3 + 2]
    n++
  }
  return { ...cloud, positions: positions.slice(0, n * 3), count: n }
}
