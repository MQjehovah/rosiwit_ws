import type { GridData } from '../ros/decode'

/** 单帧纹理最多 800 万像素，超过就按 step×step 聚合 */
export const MAX_GRID_CELLS = 8_000_000

export interface GridImage {
  width: number
  height: number
  step: number
  pixels: Uint8Array
}

/**
 * 把 OccupancyGrid 转成 RGBA 纹理数据。
 *
 * 降采样按 step×step 取**最大占据值**：-1 未知、0 空闲、100 占据。
 * 抽点采样会把 5cm 的墙打散成断续的点，取最大能保持墙线连续。
 */
export function gridToImage(grid: GridData, maxCells = MAX_GRID_CELLS): GridImage {
  const total = grid.width * grid.height
  const step = Math.max(1, Math.ceil(Math.sqrt(total / maxCells)))
  const width = Math.ceil(grid.width / step)
  const height = Math.ceil(grid.height / step)
  const pixels = new Uint8Array(width * height * 4)

  for (let y = 0; y < height; y++) {
    const y0 = y * step
    const y1 = Math.min(grid.height, y0 + step)
    for (let x = 0; x < width; x++) {
      const x0 = x * step
      const x1 = Math.min(grid.width, x0 + step)
      let best = -1
      for (let sy = y0; sy < y1; sy++) {
        const rowBase = sy * grid.width
        for (let sx = x0; sx < x1; sx++) {
          const v = grid.data[rowBase + sx]
          if (v > best) best = v
        }
      }

      const o = (y * width + x) * 4
      if (best < 0) {
        pixels[o + 3] = 0
      } else if (best >= 60) {
        pixels[o] = 188
        pixels[o + 1] = 222
        pixels[o + 2] = 255
        pixels[o + 3] = 235
      } else if (best >= 50) {
        pixels[o] = 110
        pixels[o + 1] = 160
        pixels[o + 2] = 190
        pixels[o + 3] = 140
      } else {
        pixels[o] = 26
        pixels[o + 1] = 54
        pixels[o + 2] = 66
        pixels[o + 3] = 110
      }
    }
  }

  return { width, height, step, pixels }
}
