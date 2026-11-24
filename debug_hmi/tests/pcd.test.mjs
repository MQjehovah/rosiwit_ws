import fs from 'node:fs'
import { parsePcdBuffer } from './mapFile.mjs'

function assert(cond, msg) {
  if (!cond) {
    console.error('FAIL:', msg)
    process.exitCode = 1
  } else {
    console.log('ok  :', msg)
  }
}

const binary = new Uint8Array(fs.readFileSync(new URL('./fixtures/test.pcd', import.meta.url)))
const cloud = parsePcdBuffer(binary, 'test.pcd')
assert(cloud.count === 383, `binary pcd count (${cloud.count})`)
assert(cloud.positions.length === 383 * 3, 'binary pcd positions length')
assert(Number.isFinite(cloud.positions[0]), 'binary pcd finite')

const ascii = `# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 2
HEIGHT 1
POINTS 2
DATA ascii
1 2 3
4 5 6
`
const a = parsePcdBuffer(new Uint8Array(Buffer.from(ascii, 'ascii')), 'a.pcd')
assert(a.count === 2 && a.positions[0] === 1 && a.positions[5] === 6, 'ascii pcd')
assert(a.intensity === null, 'ascii pcd without intensity -> null')

// ascii + intensity
const asciiI = `# .PCD v0.7
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 2
HEIGHT 1
POINTS 2
DATA ascii
1 2 3 10
4 5 6 200
`
const ai = parsePcdBuffer(new Uint8Array(Buffer.from(asciiI, 'ascii')), 'ai.pcd')
assert(ai.intensity instanceof Float32Array && ai.count === 2, 'ascii pcd intensity present')
assert(ai.intensity && ai.intensity[0] === 10 && ai.intensity[1] === 200, 'ascii pcd intensity values')

// binary x y z intensity
const head = Buffer.from(
  `# .PCD v0.7
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 2
HEIGHT 1
POINTS 2
DATA binary
`,
  'ascii',
)
const body = Buffer.alloc(2 * 16)
for (const [i, p] of [
  [1, 2, 3, 0],
  [4, 5, 6, 255],
].entries()) {
  body.writeFloatLE(p[0], i * 16)
  body.writeFloatLE(p[1], i * 16 + 4)
  body.writeFloatLE(p[2], i * 16 + 8)
  body.writeFloatLE(p[3], i * 16 + 12)
}
const bi = parsePcdBuffer(new Uint8Array(Buffer.concat([head, body])), 'bi.pcd')
assert(bi.intensity && bi.intensity[0] === 0 && bi.intensity[1] === 255, 'binary pcd intensity values')

if (!process.exitCode) console.log('\nall pcd tests passed')
