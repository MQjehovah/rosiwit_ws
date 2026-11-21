/**
 * 最小 CBOR 解码器（只服务 rosbridge 的 `compression: "cbor"` / `"cbor-raw"`）。
 *
 * rosbridge 会把 ROS 消息编成 CBOR map：{op, topic, msg}；数值数组用 CBOR typed-array
 * tag（69/70/71/72/77/78/79/85/86）pack 成字节串，正好可以零拷贝地映射成 TypedArray。
 * 这里不追求完整 CBOR，只覆盖 rosbridge 会产生的子集。
 */

const typedArrayTags: Record<number, { ctor: any; stride: number }> = {
  69: { ctor: Uint16Array, stride: 2 },
  70: { ctor: Uint32Array, stride: 4 },
  72: { ctor: Int8Array, stride: 1 },
  77: { ctor: Int16Array, stride: 2 },
  78: { ctor: Int32Array, stride: 4 },
  85: { ctor: Float32Array, stride: 4 },
  86: { ctor: Float64Array, stride: 8 },
}

const textDecoder = new TextDecoder('utf-8')

function halfToNumber(bits: number): number {
  const sign = bits & 0x8000 ? -1 : 1
  const exponent = (bits >> 10) & 0x1f
  const fraction = bits & 0x3ff
  if (exponent === 0) return sign * Math.pow(2, -14) * (fraction / 1024)
  if (exponent === 31) return fraction ? NaN : sign * Infinity
  return sign * Math.pow(2, exponent - 15) * (1 + fraction / 1024)
}

function toAlignedCopy(bytes: Uint8Array): Uint8Array {
  // TypedArray 构造要求 4/8 字节对齐；CBOR 里的字节串不一定对齐，复制一份最稳。
  return bytes.slice()
}

class CborReader {
  private view: DataView
  private pos = 0

  constructor(private bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength)
  }

  private u8(): number {
    return this.view.getUint8(this.pos++)
  }

  /** 读取 major type 附加信息里的长度/值 */
  private readLength(additional: number): number {
    if (additional < 24) return additional
    if (additional === 24) return this.u8()
    if (additional === 25) {
      const v = this.view.getUint16(this.pos)
      this.pos += 2
      return v
    }
    if (additional === 26) {
      const v = this.view.getUint32(this.pos)
      this.pos += 4
      return v
    }
    if (additional === 27) {
      const v = this.view.getBigUint64(this.pos)
      this.pos += 8
      return Number(v)
    }
    return -1 // 31 = 不定长
  }

  readByteString(additional: number): Uint8Array {
    if (additional === 31) {
      const chunks: Uint8Array[] = []
      let total = 0
      for (;;) {
        const head = this.u8()
        if (head === 0xff) break
        const len = this.readLength(head & 0x1f)
        chunks.push(this.bytes.subarray(this.pos, this.pos + len))
        this.pos += len
        total += len
      }
      const out = new Uint8Array(total)
      let offset = 0
      for (const chunk of chunks) {
        out.set(chunk, offset)
        offset += chunk.length
      }
      return out
    }
    const len = this.readLength(additional)
    const out = this.bytes.subarray(this.pos, this.pos + len)
    this.pos += len
    return out
  }

  read(): any {
    const head = this.u8()
    const major = head >> 5
    const additional = head & 0x1f

    switch (major) {
      case 0:
        return this.readLength(additional)
      case 1:
        return -1 - this.readLength(additional)
      case 2:
        return this.readByteString(additional)
      case 3:
        return textDecoder.decode(this.readByteString(additional))
      case 4: {
        if (additional === 31) {
          const out: any[] = []
          while (this.bytes[this.pos] !== 0xff) out.push(this.read())
          this.pos++
          return out
        }
        const len = this.readLength(additional)
        const out = new Array(len)
        for (let i = 0; i < len; i++) out[i] = this.read()
        return out
      }
      case 5: {
        const out: Record<string, any> = {}
        if (additional === 31) {
          while (this.bytes[this.pos] !== 0xff) {
            const key = this.read()
            out[typeof key === 'string' ? key : String(key)] = this.read()
          }
          this.pos++
          return out
        }
        const len = this.readLength(additional)
        for (let i = 0; i < len; i++) {
          const key = this.read()
          out[typeof key === 'string' ? key : String(key)] = this.read()
        }
        return out
      }
      case 6: {
        const tag = this.readLength(additional)
        const value = this.read()
        const entry = typedArrayTags[tag]
        if (entry && value instanceof Uint8Array) {
          const copy = toAlignedCopy(value)
          return new entry.ctor(copy.buffer, 0, Math.floor(copy.byteLength / entry.stride))
        }
        return value
      }
      case 7: {
        if (additional === 20) return false
        if (additional === 21) return true
        if (additional === 22) return null
        if (additional === 23) return undefined
        if (additional === 25) {
          const bits = this.view.getUint16(this.pos)
          this.pos += 2
          return halfToNumber(bits)
        }
        if (additional === 26) {
          const v = this.view.getFloat32(this.pos)
          this.pos += 4
          return v
        }
        if (additional === 27) {
          const v = this.view.getFloat64(this.pos)
          this.pos += 8
          return v
        }
        return null
      }
      default:
        throw new Error(`CBOR: 未支持的 major type ${major}`)
    }
  }
}

export function decodeCbor(bytes: Uint8Array): any {
  return new CborReader(bytes).read()
}
