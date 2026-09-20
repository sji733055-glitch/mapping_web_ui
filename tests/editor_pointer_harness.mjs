// Headless pointer-interaction check for web/map-editor.js.
//
// The mapping console must stay testable without a browser or a build tool, so
// this harness stubs the small DOM surface the editor uses and drives the real
// module: it loads a map, activates the `map` frame picker, and replays the
// three pointer gestures (place, move the origin handle, turn the +X handle).
// Run it with: node tests/editor_pointer_harness.mjs
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const EDITOR_JS = process.env.EDITOR_JS || path.join(HERE, "..", "web", "map-editor.js");
const SOURCE = fs.readFileSync(EDITOR_JS, "utf8");

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

// Canvas 2D stub that records the arcs the editor draws for its grab handles.
const arcs = [];
function context2d() {
  const target = {
    canvas: { width: 800, height: 600 },
    createImageData: (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    getImageData: (x, y, w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    putImageData: noop,
    measureText: () => ({ width: 10 }),
    arc: (x, y, radius) => arcs.push({ x, y, radius }),
  };
  return new Proxy(target, {
    get: (object, key) => (key in object ? object[key] : noop),
    set: (object, key, value) => { object[key] = value; return true; },
  });
}

const listeners = new Map();
const toolButtons = [];

function makeElement(tag, id = "") {
  return {
    tagName: tag.toUpperCase(), id, dataset: {}, children: [], hidden: false,
    disabled: false, checked: false, value: "", textContent: "", className: "",
    width: 800, height: 600, classList: makeClassList(),
    style: { setProperty: noop, removeProperty: noop, getPropertyValue: () => "" },
    addEventListener(type, handler) {
      const key = `${id || tag}:${type}`;
      if (!listeners.has(key)) listeners.set(key, []);
      listeners.get(key).push(handler);
    },
    removeEventListener: noop,
    dispatchEvent(event) {
      for (const handler of listeners.get(`${id || tag}:${event.type}`) || []) handler(event);
      return true;
    },
    getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600, right: 800, bottom: 600 }),
    appendChild(child) { this.children.push(child); return child; },
    replaceChildren(...kids) { this.children = kids; },
    setPointerCapture: noop, releasePointerCapture: noop, hasPointerCapture: () => false,
    getContext: () => context2d(),
    querySelectorAll: () => [],
    setAttribute: noop, getAttribute: () => null, removeAttribute: noop, focus: noop,
  };
}

const elements = new Map();
const byId = (id) => {
  if (!elements.has(id)) elements.set(id, makeElement("div", id));
  return elements.get(id);
};

for (const match of SOURCE.matchAll(/\$\("([^"]+)"\)/g)) byId(match[1]);
for (const tool of ["brush", "rect", "line", "pan", "frame"]) {
  const button = makeElement("button", tool === "frame" ? "editor-frame-pick" : `editor-tool-${tool}`);
  button.dataset.editorTool = tool;
  toolButtons.push(button);
}
elements.set("editor-frame-pick", toolButtons.at(-1));

const document = {
  getElementById: byId,
  createElement: (tag) => makeElement(tag),
  querySelectorAll: (selector) => (selector === "[data-editor-tool]" ? toolButtons : []),
  addEventListener: (type, handler) => {
    const key = `document:${type}`;
    if (!listeners.has(key)) listeners.set(key, []);
    listeners.get(key).push(handler);
  },
};

const MAP = {
  name: "venue_map", has_occupancy: true, has_terrain: false, width: 189, height: 170,
  resolution: 0.05, error: "", has_pcd: true, has_frame_metadata: false, frame_id: "map",
};

// The browser editor protocol: MPE2 header followed by one byte per cell.
function editorPayload(originX, originY) {
  const buffer = new ArrayBuffer(48 + MAP.width * MAP.height);
  const view = new DataView(buffer);
  view.setUint8(0, 0x4d); view.setUint8(1, 0x50); view.setUint8(2, 0x45); view.setUint8(3, 0x32);
  view.setUint8(4, 0);
  view.setUint32(8, MAP.width, true);
  view.setUint32(12, MAP.height, true);
  view.setFloat64(16, MAP.resolution, true);
  view.setFloat64(24, originX, true);
  view.setFloat64(32, originY, true);
  view.setFloat64(40, 0, true);
  new Uint8Array(buffer, 48).fill(254);
  return buffer;
}

const requests = [];
async function fetchStub(url, options = {}) {
  requests.push(`${options.method || "GET"} ${url}`);
  const target = String(url);
  if (target.startsWith("/api/maps")) return { ok: true, json: async () => ({ maps: [MAP] }) };
  if (target.startsWith("/api/editor/occupancy")) {
    return { ok: true, arrayBuffer: async () => editorPayload(-6.8096313, -5.0894675) };
  }
  return { ok: true, json: async () => ({ ok: true, message: "stub" }) };
}

const frames = [];
const window = {
  devicePixelRatio: 1,
  confirm: () => true,
  dispatchEvent: noop,
  addEventListener: (type, handler) => {
    const key = `window:${type}`;
    if (!listeners.has(key)) listeners.set(key, []);
    listeners.get(key).push(handler);
  },
};

const sandbox = {
  document, window, fetch: fetchStub, console, Date, Math, JSON, Object, Array, Number, String,
  Boolean, Error, Promise, Uint8Array, Uint8ClampedArray, DataView, ArrayBuffer, Map, Set,
  URLSearchParams, TextDecoder, isNaN, parseFloat, parseInt, setTimeout, clearTimeout,
  requestAnimationFrame: (handler) => { frames.push(handler); return frames.length; },
  cancelAnimationFrame: noop,
  ResizeObserver: undefined,
  HTMLInputElement: class {}, HTMLSelectElement: class {},
  Event: class { constructor(type) { this.type = type; } },
};
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(SOURCE, sandbox, { filename: "map-editor.js" });

const settle = async (rounds = 6) => {
  for (let index = 0; index < rounds; index += 1) await new Promise((resolve) => setImmediate(resolve));
};
const paint = () => { for (const handler of frames.splice(0)) handler(); };

const canvas = byId("map-editor-canvas");
const pick = byId("editor-frame-pick");
const readFrame = () => ({
  x: Number(byId("editor-frame-x").value),
  y: Number(byId("editor-frame-y").value),
  yaw: Number(byId("editor-frame-yaw").value),
});
// Handles are located from what the editor actually drew, not from assumptions.
const drawnHandles = () => {
  arcs.length = 0;
  paint();
  const origin = arcs.find((entry) => entry.radius === 5);
  const tip = arcs.find((entry) => entry.radius === 4.5);
  if (!origin || !tip) throw new Error("frame handles were not drawn");
  return { origin, tip };
};

const pointer = (type, x, y) => canvas.dispatchEvent({
  type, clientX: x, clientY: y, button: 0, buttons: type === "pointerup" ? 0 : 1,
  pointerId: 7, preventDefault: noop,
});
const drag = (from, to) => {
  pointer("pointerdown", from.x, from.y);
  pointer("pointermove", (from.x + to.x) / 2, (from.y + to.y) / 2);
  pointer("pointermove", to.x, to.y);
  pointer("pointerup", to.x, to.y);
};

const failures = [];
const check = (label, ok, detail = "") => {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? ` — ${detail}` : ""}`);
  if (!ok) failures.push(label);
};

byId("editor-refresh-maps").dispatchEvent({ type: "click" });
await settle();
byId("editor-map-select").value = MAP.name;
byId("editor-map-select").dispatchEvent({ type: "change" });
byId("editor-load-occupancy").dispatchEvent({ type: "click" });
await settle();
check("帧工具在地图与 PCD 就绪后可用", pick.disabled === false, byId("editor-frame-note").textContent);

pick.dispatchEvent({ type: "click" });
check("按下按钮后进入取帧模式", canvas.classList.contains("is-frame-picking"));

// 1. Press on empty map space: the press point becomes the origin, the drag sets +X.
drag({ x: 200, y: 300 }, { x: 320, y: 300 });
const placed = readFrame();
check("空白处按下即设定原点", placed.x !== 0 || placed.y !== 0, `origin=(${placed.x}, ${placed.y})`);
check("拖动方向设定 +X 朝向", Math.abs(placed.yaw) < 0.6, `yaw=${placed.yaw}`);

// 2. Grab the origin marker: the frame moves, the heading must not change.
const grabbed = readFrame();
const originHandle = drawnHandles().origin;
drag({ x: originHandle.x, y: originHandle.y }, { x: originHandle.x + 40, y: originHandle.y + 50 });
const moved = readFrame();
check("拖动原点手柄可平移坐标系",
  moved.x !== grabbed.x || moved.y !== grabbed.y,
  `(${grabbed.x}, ${grabbed.y}) -> (${moved.x}, ${moved.y})`);
check("平移时朝向保持不变", moved.yaw === grabbed.yaw, `yaw=${moved.yaw}`);

// 3. Grab the +X arrow head: the heading turns, the origin must not move.
const beforeTurn = readFrame();
const tip = drawnHandles().tip;
pointer("pointerdown", tip.x, tip.y);
pointer("pointerup", tip.x, tip.y);
const clickedTip = readFrame();
check("单击箭头不会误改朝向", clickedTip.yaw === beforeTurn.yaw, `yaw=${clickedTip.yaw}`);
drag({ x: tip.x, y: tip.y }, { x: tip.x, y: tip.y + 70 });
const turned = readFrame();
check("拖动 +X 箭头可转向", turned.yaw !== beforeTurn.yaw, `yaw ${beforeTurn.yaw} -> ${turned.yaw}`);
check("转向时原点位置保持不变",
  turned.x === beforeTurn.x && turned.y === beforeTurn.y,
  `origin=(${turned.x}, ${turned.y})`);

// 4. A press away from both handles still re-places the frame from scratch.
const handle = drawnHandles().origin;
drag({ x: handle.x - 120, y: handle.y - 90 }, { x: handle.x - 60, y: handle.y - 90 });
const replaced = readFrame();
check("远离手柄处按下可重新定位",
  replaced.x !== turned.x || replaced.y !== turned.y,
  `origin=(${replaced.x}, ${replaced.y})`);
check("取帧过程不写入后端", !requests.some((entry) => entry.includes("/api/editor/frame")));

console.log(failures.length ? `\n${failures.length} 项检查未通过` : "\n全部取帧交互检查通过");
process.exit(failures.length ? 1 : 0);
