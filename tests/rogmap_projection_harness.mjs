// Pure protocol and classification checks for web/rogmap-projection.js.
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const SOURCE = fs.readFileSync(path.join(HERE, "..", "web", "rogmap-projection.js"), "utf8");
const window = {};
const document = { getElementById: () => ({}) };
const sandbox = { window, document, console, ArrayBuffer, DataView, Int8Array, Uint8Array, Uint16Array, Uint32Array, Float32Array, Number, String, Math, Error };
vm.createContext(sandbox);
vm.runInContext(SOURCE, sandbox, { filename: "rogmap-projection.js" });

const api = window.RogMapProjectionDebug;
const failures = [];
function check(label, condition) {
  if (condition) console.log(`  PASS  ${label}`);
  else { failures.push(label); console.log(`  FAIL  ${label}`); }
}

const buffer = new ArrayBuffer(api.HEADER_SIZE + api.RECORD_SIZE);
const view = new DataView(buffer);
for (const [index, value] of [..."ROG1"].entries()) view.setUint8(index, value.charCodeAt(0));
view.setUint16(4, 1, true); view.setUint16(6, api.RECORD_SIZE, true);
view.setUint32(8, 1, true); view.setUint32(12, 1, true);
view.setFloat64(16, 0.05, true); view.setFloat64(24, -1.0, true); view.setFloat64(32, -2.0, true);
view.setFloat64(40, 10.0, true); view.setFloat64(48, 10.01, true); view.setFloat64(56, 10.02, true);
const offset = api.HEADER_SIZE;
view.setInt8(offset, 66); view.setUint8(offset + 1, 4); view.setUint8(offset + 2, 3); view.setUint16(offset + 3, 2, true);
view.setFloat32(offset + 5, 0.7, true); view.setFloat32(offset + 9, 0.3, true); view.setFloat32(offset + 13, 1 / 6, true);

const frame = api.decodeFrame(buffer);
check("解码 ROG1 几何", frame.width === 1 && frame.height === 1 && frame.resolution === 0.05);
check("解码选中格原始量", frame.types[0] === 66 && frame.counts[0] === 2 && Math.abs(frame.ratios[0] - 1 / 6) < 1e-6);

const config = {
  surface_height_delta_max: 0.2, wall_height_delta_min: 0.8, wall_occupancy_ratio_min: 0.9,
  tunnel_height_delta_min: 0.25, tunnel_height_delta_max: 0.4, tunnel_occupancy_ratio_max: 0.45,
};
check("tunnel 分支与 C++ 判定顺序一致", api.classifyCell(66, 0.3, 1 / 6, 3, config) === 4);
check("ratio 缺失不猜 tunnel", api.classifyCell(66, 0.3, Number.NaN, 1, config) === 6);
check("拦截非法参数关系", api.validateConfig({ ...config, tunnel_height_delta_min: 0.2 }).includes("surface"));

if (failures.length) {
  console.error(`${failures.length} ROGMap harness checks failed`);
  process.exitCode = 1;
} else {
  console.log("ROGMap projection harness passed");
}
