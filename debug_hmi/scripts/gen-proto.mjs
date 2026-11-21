#!/usr/bin/env node
/**
 * 从 debug_bridge/proto/debug_bridge.proto 生成 src/ros/proto/debug_bridge.json。
 *
 * 浏览器端用 protobufjs/light + 这个 JSON 描述符做编解码，这样运行时不需要
 * 解析 .proto，也不需要把 protobufjs 的 parser 打进包里。
 *
 *   node scripts/gen-proto.mjs
 */
import fs from 'node:fs'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import protobuf from 'protobufjs'

const here = path.dirname(fileURLToPath(import.meta.url))
const protoPath = path.join(here, '..', '..', 'debug_bridge', 'proto', 'debug_bridge.proto')
const outPath = path.join(here, '..', 'src', 'ros', 'proto', 'debug_bridge.json')

const source = fs.readFileSync(protoPath, 'utf8')
const root = protobuf.parse(source, { keepCase: true }).root
const json = root.toJSON()

fs.mkdirSync(path.dirname(outPath), { recursive: true })
fs.writeFileSync(outPath, `${JSON.stringify(json, null, 2)}\n`)
console.log(`wrote ${path.relative(process.cwd(), outPath)}`)
