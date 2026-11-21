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

if (!process.exitCode) console.log('\nall pcd tests passed')
