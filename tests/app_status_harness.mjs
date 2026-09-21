// Headless status-rendering check for web/app.js.
//
// The operator console must stay testable without a browser or a build tool, so
// this harness stubs the DOM surface the console uses and drives the real
// module through both status paths (HTTP polling and a WebSocket push).  It
// guards the map-name field: a finished session used to rewrite the input on
// every render, which made the name impossible to change for the next session.
// Run it with: node tests/app_status_harness.mjs
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const APP_JS = process.env.APP_JS || path.join(HERE, "..", "web", "app.js");
const SOURCE = fs.readFileSync(APP_JS, "utf8");

const noop = () => {};

function makeClassList() {
  const set = new Set();
  return {
    add: (...names) => names.forEach((name) => set.add(name)),
    remove: (...names) => names.forEach((name) => set.delete(name)),
    contains: (name) => set.has(name),
    toggle: (name, force) => {
      const on = force === undefined ? !set.has(name) : Boolean(force);
      if (on) set.add(name); else set.delete(name);
      return on;
    },
  };
}

const listeners = new Map();
const elements = new Map();

function makeElement(tag, id = "") {
  return {
    tagName: tag.toUpperCase(), id, dataset: {}, children: [], hidden: false,
    disabled: false, value: "", textContent: "", innerHTML: "", title: "", className: "",
    classList: makeClassList(),
    style: { setProperty: noop, removeProperty: noop, getPropertyValue: () => "" },
    addEventListener(type, handler) {
      const key = `${id || tag}:${type}`;
      if (!listeners.has(key)) listeners.set(key, []);
      listeners.get(key).push(handler);
    },
    removeEventListener: noop,
    dispatchEvent() { return true; },
    appendChild(child) { this.children.push(child); return child; },
    replaceChildren(...kids) { this.children = kids; },
    setAttribute: noop, getAttribute: () => null, removeAttribute: noop,
    focus: noop, remove: noop, closest: () => null,
    querySelectorAll: () => [],
  };
}

const byId = (id) => {
  if (!elements.has(id)) elements.set(id, makeElement("div", id));
  return elements.get(id);
};
for (const match of SOURCE.matchAll(/\$\("([^"]+)"\)/g)) byId(match[1]);

const document = {
  getElementById: byId,
  createElement: (tag) => makeElement(tag),
  querySelectorAll: () => [],
  addEventListener: noop,
};

// The console only ever decodes the 8-byte empty cloud frame in this harness.
function emptyCloudFrame() {
  const buffer = new ArrayBuffer(8);
  const view = new DataView(buffer);
  view.setUint8(0, 0x4d); view.setUint8(1, 0x41); view.setUint8(2, 0x50); view.setUint8(3, 0x31);
  view.setUint32(4, 0, true);
  return buffer;
}

let currentStatus = null;
const requests = [];
const posted = [];
async function fetchStub(url, options) {
  const target = String(url);
  requests.push(target.split("?")[0]);
  if (target.startsWith("/api/status")) return { ok: true, json: async () => currentStatus };
  if (target.startsWith("/api/cloud")) return { ok: true, arrayBuffer: async () => emptyCloudFrame() };
  if (target.startsWith("/api/pcd-files")) {
    return { ok: true, json: async () => ({ pcd_files: [{ name: "venue_map", filename: "venue_map.pcd", size_bytes: 2048 }] }) };
  }
  if (target.startsWith("/api/pcd/map-preview")) {
    posted.push({ url: target, body: options?.body ? JSON.parse(options.body) : null });
    return { ok: true, headers: previewHeaders(), arrayBuffer: async () => previewPgm().buffer };
  }
  if (target.startsWith("/api/pcd/preview")) {
    requests.push(target);
    return { ok: true, headers: new Map([["X-Preview-Points", "12"], ["X-Preview-Total", "12"]]), arrayBuffer: async () => emptyCloudFrame() };
  }
  if (target.startsWith("/api/pcd/convert")) {
    posted.push({ url: target, body: options?.body ? JSON.parse(options.body) : null });
    return { ok: true, json: async () => ({ ok: true, message: "点云已转换", result: {} }) };
  }
  return { ok: true, json: async () => ({ ok: true, message: "stub" }) };
}

// A 4×3 P5 preview: three occupied cells (0) on a 254 free background.
const PREVIEW_WIDTH = 4;
const PREVIEW_HEIGHT = 3;
function previewPgm() {
  const header = new TextEncoder().encode(`P5\n# preview of venue_map.pcd\n${PREVIEW_WIDTH} ${PREVIEW_HEIGHT}\n255\n`);
  const bytes = new Uint8Array(header.length + PREVIEW_WIDTH * PREVIEW_HEIGHT);
  bytes.set(header, 0);
  bytes.fill(254, header.length);
  for (const index of [0, 5, 11]) bytes[header.length + index] = 0;
  return bytes;
}
const previewHeaders = () => new Map([
  ["X-Map-Width", String(PREVIEW_WIDTH)], ["X-Map-Height", String(PREVIEW_HEIGHT)],
  ["X-Map-Resolution", "0.05"], ["X-Map-Origin-X", "-1"], ["X-Map-Origin-Y", "-2"],
  ["X-Map-Height-Mode", "ground"],
  ["X-Map-Ground-Tilt", "2.17"],
  ["X-Map-Filter-Mode", "voxel"], ["X-Map-Filter-Removed", "37"],
  ["X-Map-Slice-Points", "900"], ["X-Map-Occupied", "3"], ["X-Map-Preview-Stride", "1"],
]);

const viewCalls = [];
class PointCloudViewerStub {
  constructor(canvas) { this.canvas = canvas; viewCalls.push("construct"); }
  setPoints(points) { this.points = points; }
  fit() { viewCalls.push("fit"); }
  top() { viewCalls.push("top"); }
  resize() { viewCalls.push("resize"); }
}

let lastSocket = null;
class WebSocketStub {
  static OPEN = 1;
  static CLOSED = 3;
  constructor(url) { this.url = url; this.readyState = 0; this.sent = []; lastSocket = this; }
  send(payload) { this.sent.push(payload); }
  close() { this.readyState = WebSocketStub.CLOSED; }
}

const intervals = new Map();
const window = {
  devicePixelRatio: 1, addEventListener: noop, dispatchEvent: noop,
  PointCloudViewer: PointCloudViewerStub,
};

const sandbox = {
  document, window, fetch: fetchStub, console, Date, Math, JSON, Object, Array, Number, String,
  Boolean, Error, Promise, ArrayBuffer, DataView, Float32Array, Uint8Array, Intl, isNaN,
  parseFloat, parseInt, performance: { now: () => Date.now() },
  location: { protocol: "http:", host: "127.0.0.1:8765" },
  WebSocket: WebSocketStub,
  URLSearchParams,
  setInterval: (handler, ms) => { intervals.set(ms, handler); return ms; },
  clearInterval: noop,
  setTimeout: (handler) => { void handler; return 0; },
  clearTimeout: noop,
  Event: class { constructor(type) { this.type = type; } },
  Intl: Intl,
};
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(SOURCE, sandbox, { filename: "app.js" });

const settle = async (rounds = 8) => {
  for (let index = 0; index < rounds; index += 1) await new Promise((resolve) => setImmediate(resolve));
};

function status(overrides = {}) {
  return {
    state: "IDLE", session_name: "", message: "", cloud_online: true, odom_online: true,
    storage_writable: true, point_count: 0, elapsed_seconds: 0, travel_distance: 0,
    cloud_rate_hz: 0, frame_id: "odom", voxel_size: 0.05, save_progress: 0, save_stage: "",
    bounds: { min_x: 0, min_y: 0, min_z: 0, max_x: 1, max_y: 1, max_z: 1 },
    map_export: { z_min: 0.05, z_max: 1.5 }, dynamic_removal: {}, keyframe_history: {},
    stack: { state: "STOPPED", managed_count: 0, message: "", nodes: {} }, output: {},
    ...overrides,
  };
}

// Both transports must behave identically, so every render is replayed twice.
async function renderVia(next) {
  currentStatus = next;
  intervals.get(1000)();
  await settle();
}

async function pushVia(next) {
  currentStatus = next;
  lastSocket.onmessage({ data: JSON.stringify({ type: "status", payload: next }) });
  await settle();
}

const failures = [];
function check(label, condition, detail = "") {
  if (condition) console.log(`  PASS  ${label}`);
  else { failures.push(label); console.log(`  FAIL  ${label}${detail ? ` — ${detail}` : ""}`); }
}

const mapName = byId("map-name");
const field = () => mapName.value;
const type = (name) => { mapName.value = name; };

await settle();
console.log("1. 待机状态不写入名称");
await renderVia(status({ state: "IDLE" }));
check("IDLE 不填名称", field() === "", `value=${field()}`);

console.log("2. 建图中显示会话名并锁住输入框");
await renderVia(status({ state: "MAPPING", session_name: "venue_map" }));
check("MAPPING 显示会话名", field() === "venue_map", `value=${field()}`);
check("MAPPING 输入框禁用", mapName.disabled === true);
await renderVia(status({ state: "MAPPING", session_name: "venue_map" }));
check("MAPPING 重复渲染不改变取值", field() === "venue_map", `value=${field()}`);

console.log("3. 保存完成后同步一次实际落盘名称");
await renderVia(status({ state: "SAVED", session_name: "venue_map_20260921_184409" }));
check("SAVED 显示带时间戳的保存名", field() === "venue_map_20260921_184409", `value=${field()}`);
check("SAVED 输入框可编辑", mapName.disabled === false);

console.log("4. 保存后仍能改成新名字（回归点）");
type("venue_map2");
await renderVia(status({ state: "SAVED", session_name: "venue_map_20260921_184409" }));
check("HTTP 轮询不会覆盖新名字", field() === "venue_map2", `value=${field()}`);
await pushVia(status({ state: "SAVED", session_name: "venue_map_20260921_184409" }));
check("WebSocket 推送不会覆盖新名字", field() === "venue_map2", `value=${field()}`);
for (let round = 0; round < 5; round += 1) {
  await renderVia(status({ state: "SAVED", session_name: "venue_map_20260921_184409" }));
}
check("连续 5 秒轮询后名字仍在", field() === "venue_map2", `value=${field()}`);

console.log("5. 用新名字开第二个会话并再次保存");
await renderVia(status({ state: "MAPPING", session_name: "venue_map2" }));
check("第二次会话沿用输入的名字", field() === "venue_map2", `value=${field()}`);
await renderVia(status({ state: "SAVED", session_name: "venue_map2_20260921_191500" }));
check("第二次保存同步新落盘名称", field() === "venue_map2_20260921_191500", `value=${field()}`);
type("lab_map");
await renderVia(status({ state: "SAVED", session_name: "venue_map2_20260921_191500" }));
check("第三次改名同样不被覆盖", field() === "lab_map", `value=${field()}`);

console.log("6. 重置后输入框保持可编辑");
await renderVia(status({ state: "IDLE", session_name: "" }));
check("重置不清空操作者已输入的名字", field() === "lab_map", `value=${field()}`);
check("重置后输入框可用", mapName.disabled === false);

const click = (id) => {
  const handlers = listeners.get(`${id}:click`) || [];
  if (!handlers.length) throw new Error(`没有 ${id} 的点击处理器`);
  handlers[handlers.length - 1]();
};
const fire = (id, type) => {
  const handlers = listeners.get(`${id}:${type}`) || [];
  if (!handlers.length) throw new Error(`没有 ${id} 的 ${type} 处理器`);
  handlers[handlers.length - 1]();
};

const sliceDefaults = { resolution: 0.05, z_min: 0.05, z_max: 1.5, radius: 0.5, min_neighbors: 10, padding: 0.25, height_mode: "ground", filter_mode: "voxel", filter_voxel_size: 0.1 };

console.log("7. 二维切片参数以后端默认值为准，且不覆盖操作者的改动");
await renderVia(status({ state: "IDLE", map_export: sliceDefaults }));
check("默认 Z 上限来自 status", byId("pcd-z-max").value === "1.5", `value=${byId("pcd-z-max").value}`);
check("默认使用自动地面基准", byId("pcd-height-mode").value === "ground", `value=${byId("pcd-height-mode").value}`);
check("默认使用结构体素滤波", byId("pcd-filter-mode").value === "voxel", `value=${byId("pcd-filter-mode").value}`);
check("默认结构体素尺寸来自 status", byId("pcd-filter-voxel-size").value === "0.1", `value=${byId("pcd-filter-voxel-size").value}`);
check("默认分辨率来自 status", byId("pcd-resolution").value === "0.05", `value=${byId("pcd-resolution").value}`);
check("默认最小邻点来自 status", byId("pcd-min-neighbors").value === "10", `value=${byId("pcd-min-neighbors").value}`);
check("体素模式禁用半径参数", byId("pcd-radius").disabled === true && byId("pcd-min-neighbors").disabled === true);
byId("pcd-filter-mode").value = "radius";
fire("pcd-filter-mode", "change");
check("半径模式启用半径参数", byId("pcd-radius").disabled === false && byId("pcd-min-neighbors").disabled === false && byId("pcd-filter-voxel-size").disabled === true);
byId("pcd-filter-mode").value = "voxel";
fire("pcd-filter-mode", "change");
byId("pcd-z-max").value = "2.5";
fire("pcd-z-max", "input");
await renderVia(status({ state: "IDLE", map_export: sliceDefaults }));
await pushVia(status({ state: "IDLE", map_export: sliceDefaults }));
check("连续渲染不覆盖操作者改过的 Z 上限", byId("pcd-z-max").value === "2.5", `value=${byId("pcd-z-max").value}`);
click("pcd-defaults-button");
check("「恢复默认值」拉回后端默认", byId("pcd-z-max").value === "1.5", `value=${byId("pcd-z-max").value}`);

console.log("8. 自助二维切片预览解码 PGM 并给出尺寸与占据统计");
byId("pcd-file-select").value = "venue_map";
fire("pcd-file-select", "change");
click("pcd-map-preview-button");
await settle();
const previewBody = posted.at(-1)?.body || {};
check("预览请求使用当前参数", previewBody.map_name === "venue_map" && previewBody.height_mode === "ground" && previewBody.filter_mode === "voxel" && previewBody.filter_voxel_size === 0.1 && previewBody.z_min === 0.05 && previewBody.resolution === 0.05, JSON.stringify(previewBody));
const stats = byId("pcd-map-preview-stats").textContent;
check("统计行给出地面校正、体素去除、栅格与占据格数", stats.includes("地面倾斜 2.17° 已校正") && stats.includes("体素去除 37 点") && stats.includes("4 × 3 px") && stats.includes("占据 3 格"), stats);
check("预览面板展开", byId("pcd-map-preview-wrap").hidden === false);
check("状态行提示切片完成", byId("pcd-convert-status").textContent.includes("切片完成"), byId("pcd-convert-status").textContent);

console.log("9. 生成 PGM/YAML 时提交参数与输出名");
byId("pcd-output-name").value = "venue_slice";
click("pcd-convert-button");
await settle();
const convertBody = posted.at(-1)?.body || {};
check("转换请求带切片参数", convertBody.height_mode === "ground" && convertBody.filter_mode === "voxel" && convertBody.filter_voxel_size === 0.1 && convertBody.resolution === 0.05 && convertBody.min_neighbors === 10 && convertBody.z_max === 1.5, JSON.stringify(convertBody));
check("转换请求带输出名", convertBody.output_name === "venue_slice", JSON.stringify(convertBody));
check("转换结果写回状态行", byId("pcd-convert-status").textContent === "点云已转换", byId("pcd-convert-status").textContent);

console.log("10. 三维预览按切片高度过滤；保存指令不再携带 z_max");
click("pcd-look-button");
await settle();
const previewRequest = requests.filter((entry) => entry.includes("/api/pcd/preview")).at(-1) || "";
check("三维预览请求带高度基准与上下限", previewRequest.includes("height_mode=ground") && previewRequest.includes("z_min=") && previewRequest.includes("z_max="), previewRequest);
check("预览面板被打开", byId("preview-panel").hidden === false);
await renderVia(status({ state: "MAPPING", session_name: "venue_map" }));
lastSocket.readyState = WebSocketStub.OPEN;
click("stop-button");
const command = lastSocket?.sent?.at(-1) || "";
check("保存指令只含 action 与地图名", command.includes('"action":"stop"') && !command.includes("z_max"), command);

console.log(failures.length ? `\n${failures.length} 项检查未通过` : "\n全部状态渲染检查通过");
process.exit(failures.length ? 1 : 0);
