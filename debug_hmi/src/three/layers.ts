import * as THREE from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import type { GridData } from '../ros/decode'
import type { TfTransform } from '../ros/tf'
import { gridToImage } from './gridImage'

export function applyTransform(object: THREE.Object3D, tf: TfTransform): void {
  object.position.set(tf.t[0], tf.t[1], tf.t[2])
  object.quaternion.set(tf.q[0], tf.q[1], tf.q[2], tf.q[3])
}

// ------------------------------------------------------------------ point cloud

export class PointCloudLayer {
  readonly object: THREE.Points
  private geometry = new THREE.BufferGeometry()
  private material: THREE.PointsMaterial
  private colored = false

  constructor(opts: { size: number; color?: number; opacity?: number; depthWrite?: boolean }) {
    this.material = new THREE.PointsMaterial({
      size: opts.size,
      sizeAttenuation: true,
      color: opts.color ?? 0xffffff,
      transparent: true,
      opacity: opts.opacity ?? 1,
      depthWrite: opts.depthWrite ?? true,
      vertexColors: false,
    })
    this.object = new THREE.Points(this.geometry, this.material)
    this.object.frustumCulled = false // 点云经常部分在视锥外，关掉避免整体被裁
  }

  setPoints(positions: Float32Array, colors: Float32Array | null): void {
    const posAttr = new THREE.BufferAttribute(positions, 3)
    this.geometry.setAttribute('position', posAttr)
    this.geometry.computeBoundingSphere()

    if (colors && colors.length) {
      this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
      if (!this.colored) {
        this.colored = true
        this.material.vertexColors = true
        this.material.needsUpdate = true
      }
    } else if (this.colored) {
      this.geometry.deleteAttribute('color')
      this.colored = false
      this.material.vertexColors = false
      this.material.needsUpdate = true
    }
    this.geometry.setDrawRange(0, positions.length / 3)
  }

  setVisible(visible: boolean): void {
    this.object.visible = visible
  }

  setSize(size: number): void {
    this.material.size = size
  }

  setColor(color: number): void {
    this.material.color.setHex(color)
    this.material.needsUpdate = true
  }

  clear(): void {
    this.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3))
    this.geometry.setDrawRange(0, 0)
  }

  get count(): number {
    return this.geometry.getAttribute('position')?.count ?? 0
  }

  dispose(): void {
    this.geometry.dispose()
    this.material.dispose()
  }
}

// ------------------------------------------------------------------ occupancy grid

export class GridMapLayer {
  readonly object = new THREE.Group()
  private mesh: THREE.Mesh | null = null
  private texture: THREE.DataTexture | null = null
  private width = 0
  private height = 0
  private step = 1

  update(grid: GridData): void {
    if (!grid.width || !grid.height) return
    const image = gridToImage(grid)

    const texture = new THREE.DataTexture(image.pixels, image.width, image.height, THREE.RGBAFormat)
    texture.magFilter = THREE.LinearFilter
    texture.minFilter = THREE.LinearFilter
    texture.generateMipmaps = false
    texture.needsUpdate = true
    this.texture?.dispose()
    this.texture = texture

    if (!this.mesh) {
      const material = new THREE.MeshBasicMaterial({
        map: texture,
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
      })
      this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), material)
      this.mesh.renderOrder = -1
      this.object.add(this.mesh)
    } else {
      const material = this.mesh.material as THREE.MeshBasicMaterial
      material.map = texture
      material.needsUpdate = true
    }

    this.width = image.width
    this.height = image.height
    this.step = image.step

    const wm = image.width * image.step * grid.resolution
    const hm = image.height * image.step * grid.resolution
    const cos = Math.cos(grid.origin.theta)
    const sin = Math.sin(grid.origin.theta)
    this.mesh.scale.set(wm, hm, 1)
    this.mesh.position.set(
      grid.origin.x + (wm / 2) * cos - (hm / 2) * sin,
      grid.origin.y + (wm / 2) * sin + (hm / 2) * cos,
      0.01,
    )
    this.mesh.rotation.z = grid.origin.theta
  }

  setVisible(visible: boolean): void {
    this.object.visible = visible
  }

  get cellCount(): number {
    return this.width * this.height
  }

  clear(): void {
    if (this.mesh) {
      this.object.remove(this.mesh)
      this.mesh.geometry.dispose()
      ;(this.mesh.material as THREE.Material).dispose()
      this.mesh = null
    }
    this.texture?.dispose()
    this.texture = null
    this.width = this.height = 0
  }
}

// ------------------------------------------------------------------ robot

export class RobotLayer {
  readonly object = new THREE.Group()
  private footprint: THREE.LineLoop
  private footprintGeom = new THREE.BufferGeometry()
  private placeholder = new THREE.Group()
  private modelRoot: THREE.Object3D | null = null

  constructor() {
    const bodyMat = new THREE.MeshLambertMaterial({
      color: 0x0e7490,
      transparent: true,
      opacity: 0.85,
    })
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.62, 0.32), bodyMat)
    body.position.z = 0.16
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(body.geometry),
      new THREE.LineBasicMaterial({ color: 0x67e8f9 }),
    )
    edges.position.copy(body.position)
    this.placeholder.add(body, edges)

    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.14, 0.34, 4),
      new THREE.MeshLambertMaterial({ color: 0x22d3ee }),
    )
    nose.rotation.z = -Math.PI / 2
    nose.position.set(0.62, 0, 0.16)
    this.placeholder.add(nose)

    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.05, 0.05, 0.55, 8),
      new THREE.MeshLambertMaterial({ color: 0x94a3b8 }),
    )
    mast.position.set(0.05, 0, 0.45)
    this.placeholder.add(mast)

    this.object.add(this.placeholder)

    this.footprint = new THREE.LineLoop(
      this.footprintGeom,
      new THREE.LineBasicMaterial({ color: 0x22d3ee, transparent: true, opacity: 0.9 }),
    )
    this.object.add(this.footprint)
  }

  /** 加载 GLB 机器人模型；加载失败时保留简易占位盒。 */
  async loadModel(url: string): Promise<{ width: number; height: number; depth: number } | null> {
    const gltf = await new Promise<any>((resolve, reject) => {
      new GLTFLoader().load(url, resolve, undefined, reject)
    })
    const root: THREE.Object3D = gltf.scene
    root.updateMatrixWorld(true)

    let box = new THREE.Box3().setFromObject(root)
    let size = box.getSize(new THREE.Vector3())
    // KeyShot 导出的是毫米；大于 20m 的一律按毫米处理
    const unitScale = Math.max(size.x, size.y, size.z) > 20 ? 0.001 : 1
    root.scale.multiplyScalar(unitScale)
    root.updateMatrixWorld(true)

    box = new THREE.Box3().setFromObject(root)
    const center = box.getCenter(new THREE.Vector3())
    size = box.getSize(new THREE.Vector3())

    // 先把模型中心移到原点，再做姿态变换
    root.position.set(-center.x, -center.y, -center.z)
    const wrapper = new THREE.Group()
    const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2)
    const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI)
    // 先绕 X 转 90°，再绕 Z 转 180°（gt.glb 的实际姿态）
    wrapper.quaternion.copy(qz).multiply(qx)
    wrapper.add(root)
    wrapper.updateMatrixWorld(true)

    // 旋转后再把脚底对齐 z=0、水平居中
    const wbox = new THREE.Box3().setFromObject(wrapper)
    const wcenter = wbox.getCenter(new THREE.Vector3())
    wrapper.position.set(-wcenter.x, -wcenter.y, -wbox.min.z)
    wrapper.updateMatrixWorld(true)

    wrapper.traverse((obj) => {
      if ((obj as THREE.Mesh).isMesh) obj.frustumCulled = false
    })

    if (this.modelRoot) this.object.remove(this.modelRoot)
    this.modelRoot = wrapper
    this.placeholder.visible = false
    this.object.add(wrapper)
    const wsize = wbox.getSize(new THREE.Vector3())
    return { width: wsize.x, height: wsize.z, depth: wsize.y }
  }

  setPose(x: number, y: number, yaw: number): void {
    this.object.position.set(x, y, 0)
    this.object.rotation.z = yaw
  }

  setTransform(tf: TfTransform): void {
    applyTransform(this.object, tf)
  }

  setFootprint(points: Float32Array): void {
    if (points.length < 9) return
    const closed = new Float32Array(points.length + 3)
    closed.set(points, 0)
    closed[points.length] = points[0]
    closed[points.length + 1] = points[1]
    closed[points.length + 2] = 0.02
    this.footprintGeom.setAttribute('position', new THREE.BufferAttribute(closed, 3))
    this.footprintGeom.computeBoundingSphere()
  }

  setVisible(visible: boolean): void {
    this.object.visible = visible
  }

  setFootprintVisible(visible: boolean): void {
    this.footprint.visible = visible
  }
}

// ------------------------------------------------------------------ TF frames

const AXIS_SIZE = 0.45

export class TfLayer {
  readonly object = new THREE.Group()
  private axes = new Map<string, THREE.AxesHelper>()
  private updatedAt = 0

  update(
    frameIds: string[],
    resolve: (frame: string) => TfTransform | null,
    visible: boolean,
  ): void {
    this.object.visible = visible
    if (!visible) return
    if (performance.now() - this.updatedAt < 200) return
    this.updatedAt = performance.now()

    const wanted = new Set<string>()
    for (const frame of frameIds) {
      if (frame === 'map' || frame === 'world') continue
      if (/camera|^ar_marker|^tag_|detected/i.test(frame)) continue
      wanted.add(frame)
      if (wanted.size >= 24) break
    }

    for (const [frame, helper] of this.axes) {
      if (!wanted.has(frame)) {
        this.object.remove(helper)
        helper.geometry.dispose()
        ;(helper.material as THREE.Material).dispose()
        this.axes.delete(frame)
      }
    }

    for (const frame of wanted) {
      let helper = this.axes.get(frame)
      if (!helper) {
        helper = new THREE.AxesHelper(AXIS_SIZE)
        this.axes.set(frame, helper)
        this.object.add(helper)
      }
      const tf = resolve(frame)
      if (!tf) {
        helper.visible = false
        continue
      }
      helper.visible = true
      helper.position.set(tf.t[0], tf.t[1], tf.t[2])
      helper.quaternion.set(tf.q[0], tf.q[1], tf.q[2], tf.q[3])
    }
  }

  clear(): void {
    for (const [, helper] of this.axes) {
      this.object.remove(helper)
      helper.geometry.dispose()
      ;(helper.material as THREE.Material).dispose()
    }
    this.axes.clear()
  }
}

// ------------------------------------------------------------------ path / trail

export class PathLayer {
  readonly object: THREE.Line
  private geometry = new THREE.BufferGeometry()

  constructor(color: number, opacity = 1) {
    this.object = new THREE.Line(
      this.geometry,
      new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }),
    )
    this.object.frustumCulled = false
  }

  setPoints(positions: Float32Array): void {
    this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    this.geometry.computeBoundingSphere()
    this.geometry.setDrawRange(0, positions.length / 3)
  }

  setVisible(visible: boolean): void {
    this.object.visible = visible
  }

  setColor(color: number): void {
    ;(this.object.material as THREE.LineBasicMaterial).color.setHex(color)
  }

  dispose(): void {
    this.geometry.dispose()
    ;(this.object.material as THREE.Material).dispose()
  }
}

export class TrailLayer {
  readonly object: THREE.Line
  private geometry = new THREE.BufferGeometry()
  private capacity = 8000
  private positions = new Float32Array(this.capacity * 3)
  private count = 0

  constructor(color = 0x38bdf8) {
    this.geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3))
    this.geometry.setDrawRange(0, 0)
    this.object = new THREE.Line(
      this.geometry,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.7 }),
    )
    this.object.frustumCulled = false
  }

  push(x: number, y: number, z = 0.03): void {
    if (this.count >= this.capacity) {
      this.positions.copyWithin(0, 3 * 1000)
      this.count -= 1000
    }
    const o = this.count * 3
    this.positions[o] = x
    this.positions[o + 1] = y
    this.positions[o + 2] = z
    this.count++
    const attr = this.geometry.getAttribute('position') as THREE.BufferAttribute
    attr.needsUpdate = true
    this.geometry.setDrawRange(0, this.count)
  }

  clear(): void {
    this.count = 0
    this.geometry.setDrawRange(0, 0)
  }

  setVisible(visible: boolean): void {
    this.object.visible = visible
  }
}
