/**
 * 最小 TF 树：只支持单父节点（ROS TF 的常见情况），用于把任意坐标系下的点/位姿换到 map 系。
 * 不依赖 three.js，纯数学，方便单测。
 */

export type Vec3 = [number, number, number]
export type Quat = [number, number, number, number] // x, y, z, w

export interface TfTransform {
  parent: string
  child: string
  t: Vec3
  q: Quat
}

export function qMul(a: Quat, b: Quat): Quat {
  const [ax, ay, az, aw] = a
  const [bx, by, bz, bw] = b
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]
}

export function qInv(q: Quat): Quat {
  return [-q[0], -q[1], -q[2], q[3]]
}

export function qRotate(q: Quat, v: Vec3): Vec3 {
  // v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
  const [x, y, z, w] = q
  const [vx, vy, vz] = v
  const tx = 2 * (y * vz - z * vy)
  const ty = 2 * (z * vx - x * vz)
  const tz = 2 * (x * vy - y * vx)
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ]
}

export function yawToQuat(yaw: number): Quat {
  return [0, 0, Math.sin(yaw / 2), Math.cos(yaw / 2)]
}

export function quatToYaw(q: Quat): number {
  const [x, y, z, w] = q
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
}

/** 组合：先 b 后 a（等价于矩阵 A * B） */
export function compose(a: TfTransform, b: TfTransform): TfTransform {
  const rt = qRotate(a.q, b.t)
  return {
    parent: a.parent,
    child: b.child,
    t: [rt[0] + a.t[0], rt[1] + a.t[1], rt[2] + a.t[2]],
    q: qMul(a.q, b.q),
  }
}

/** 逆变换：把 A<-B 变成 B<-A */
export function invertTransform(a: TfTransform): TfTransform {
  const qi = qInv(a.q)
  const ti = qRotate(qi, [-a.t[0], -a.t[1], -a.t[2]])
  return { parent: a.child, child: a.parent, t: ti, q: qi }
}

interface AdjEntry {
  next: string
  t: Vec3
  q: Quat
}

export class TfTree {
  /** 每次写入自增，渲染层据此判断是否需要刷新 */
  version = 0

  private dynamic = new Map<string, TfTransform>()
  private fixed = new Map<string, TfTransform>()
  private adjacency: Map<string, AdjEntry[]> | null = null
  private lastVersion = -1
  private memo = new Map<string, TfTransform | null>()

  ingest(msg: any, isStatic = false): void {
    const list = msg?.transforms
    if (!Array.isArray(list)) return
    const target = isStatic ? this.fixed : this.dynamic
    for (const item of list) {
      const parent = item?.header?.frame_id
      const child = item?.child_frame_id
      const tr = item?.transform
      if (!parent || !child || !tr?.translation || !tr?.rotation) continue
      target.set(child, {
        parent,
        child,
        t: [tr.translation.x, tr.translation.y, tr.translation.z],
        q: [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w],
      })
    }
    this.version++
    this.adjacency = null
    this.memo.clear()
  }

  clear(): void {
    this.dynamic.clear()
    this.fixed.clear()
    this.adjacency = null
    this.memo.clear()
    this.version++
  }

  frames(): string[] {
    this.rebuild()
    return [...(this.adjacency?.keys() ?? [])]
  }

  edges(): TfTransform[] {
    return [...this.fixed.values(), ...this.dynamic.values()]
  }

  parentOf(child: string): string | null {
    return (this.dynamic.get(child) ?? this.fixed.get(child))?.parent ?? null
  }

  /** 取 target <- source 的变换；找不到返回 null */
  lookup(target: string, source: string): TfTransform | null {
    if (target === source) {
      return { parent: target, child: source, t: [0, 0, 0], q: [0, 0, 0, 1] }
    }
    this.rebuild()
    const key = `${target}<-${source}`
    const cached = this.memo.get(key)
    if (cached !== undefined) return cached

    const adj = this.adjacency
    if (!adj) return null

    // BFS: 从 source 出发，边是 (next, 把 next 系坐标变到当前系 的变换)
    const prev = new Map<string, { from: string; step: TfTransform }>()
    const queue: string[] = [source]
    const seen = new Set<string>([source])
    let found = false

    while (queue.length) {
      const cur = queue.shift() as string
      if (cur === target) {
        found = true
        break
      }
      for (const edge of adj.get(cur) ?? []) {
        if (seen.has(edge.next)) continue
        seen.add(edge.next)
        prev.set(edge.next, {
          from: cur,
          step: {
            parent: cur,
            child: edge.next,
            t: edge.t,
            q: edge.q,
          },
        })
        queue.push(edge.next)
      }
    }

    if (!found) {
      this.memo.set(key, null)
      return null
    }

    // 回溯得到 target→source 的链，反转后按 source→target 顺序组合
    let node = target
    const chain: TfTransform[] = []
    while (node !== source) {
      const p = prev.get(node)
      if (!p) return null
      chain.push(p.step)
      node = p.from
    }
    chain.reverse()

    let acc: TfTransform = {
      parent: source,
      child: source,
      t: [0, 0, 0],
      q: [0, 0, 0, 1],
    }
    for (const step of chain) acc = compose(acc, step)

    const result = invertTransform(acc)
    this.memo.set(key, result)
    return result
  }

  private rebuild(): void {
    if (this.adjacency && this.lastVersion === this.version) return
    const merged = new Map<string, TfTransform>()
    for (const [k, v] of this.fixed) merged.set(k, v)
    for (const [k, v] of this.dynamic) merged.set(k, v)

    const adj = new Map<string, AdjEntry[]>()
    const add = (from: string, entry: AdjEntry) => {
      let list = adj.get(from)
      if (!list) {
        list = []
        adj.set(from, list)
      }
      list.push(entry)
    }

    for (const tr of merged.values()) {
      // p_parent = R(q) p_child + t
      // 邻接表存的是“把 next 系坐标变到 from 系”的变换：
      //   child -> parent 用逆变换；parent -> child 用正向变换
      const qi = qInv(tr.q)
      const ti = qRotate(qi, [-tr.t[0], -tr.t[1], -tr.t[2]])
      add(tr.child, { next: tr.parent, t: ti, q: qi })
      add(tr.parent, { next: tr.child, t: tr.t, q: tr.q })
    }

    this.adjacency = adj
    this.lastVersion = this.version
    this.memo.clear()
  }
}
