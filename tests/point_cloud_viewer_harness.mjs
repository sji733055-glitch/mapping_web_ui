// Headless interaction/performance regression check for the real WebGL viewer.
// Run with: node tests/point_cloud_viewer_harness.mjs
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const SOURCE = fs.readFileSync(path.join(HERE, "..", "web", "point-cloud-viewer.js"), "utf8");
const listeners = new Map();
const frames = [];
const uploads = [];
const matrices = [];
const draws = [];
const attributePointers = [];
const noop = () => {};

function on(target, type, handler) {
  const key = `${target}:${type}`;
  if (!listeners.has(key)) listeners.set(key, []);
  listeners.get(key).push(handler);
}

function emit(target, type, event = {}) {
  for (const handler of listeners.get(`${target}:${type}`) || []) handler({
    type, pointerId: 1, button: 0, shiftKey: false, clientX: 0, clientY: 0,
    deltaY: 0, deltaMode: 0, preventDefault: noop, ...event,
  });
}

const glTarget = {
  VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, COMPILE_STATUS: 3, LINK_STATUS: 4,
  ARRAY_BUFFER: 5, STATIC_DRAW: 6, DYNAMIC_DRAW: 7, DEPTH_TEST: 8, BLEND: 9,
  SRC_ALPHA: 10, ONE_MINUS_SRC_ALPHA: 11, FLOAT: 12, LINES: 13, POINTS: 14,
  COLOR_BUFFER_BIT: 0x4000, DEPTH_BUFFER_BIT: 0x0100,
  createShader: () => ({}), createProgram: () => ({}), createBuffer: () => ({}),
  getShaderParameter: () => true, getProgramParameter: () => true,
  getShaderInfoLog: () => "", getProgramInfoLog: () => "",
  getAttribLocation: () => 0, getUniformLocation: (_program, name) => name,
  bufferData: (_target, data, usage) => uploads.push({ data, usage }),
  uniformMatrix4fv: (_location, _transpose, matrix) => matrices.push(Array.from(matrix)),
  vertexAttribPointer: (_location, size, _type, _normalized, stride, offset) => attributePointers.push({ size, stride, offset }),
  drawArrays: (mode, first, count) => draws.push({ mode, first, count }),
};
const gl = new Proxy(glTarget, {
  get: (target, key) => key in target ? target[key] : noop,
});

const canvas = {
  clientWidth: 900, clientHeight: 540, width: 0, height: 0,
  getContext: (kind) => kind === "webgl" ? gl : null,
  addEventListener: (type, handler) => on("canvas", type, handler),
  setPointerCapture: noop,
};

const window = {
  devicePixelRatio: 1,
  addEventListener: (type, handler) => on("window", type, handler),
};
const sandbox = {
  window, console, Math, Float32Array, Array, Error,
  requestAnimationFrame: (handler) => { frames.push(handler); return frames.length; },
};
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(SOURCE, sandbox, { filename: "point-cloud-viewer.js" });

const viewer = new window.PointCloudViewer(canvas);
const failures = [];
function check(label, ok, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? ` — ${detail}` : ""}`);
  if (!ok) failures.push(label);
}
function frame() {
  const handler = frames.shift();
  if (!handler) throw new Error("viewer did not schedule an animation frame");
  handler();
}

const first = new Float32Array(120000 * 3);
for (let index = 0; index < first.length; index += 3) {
  first[index] = index / 3 % 400;
  first[index + 1] = Math.floor(index / 3 / 400);
  first[index + 2] = index / 3 % 7;
}
viewer.setPoints(first);
const uploadsAfterFirstCloud = uploads.length;

emit("canvas", "pointerdown", { clientX: 100, clientY: 100 });
const second = new Float32Array([2, 2, 2]);
const newest = new Float32Array([3, 3, 3, 4, 4, 4]);
viewer.setPoints(second);
viewer.setPoints(newest);
check("拖动时不上传新的实时点云", uploads.length === uploadsAfterFirstCloud);

const yawBefore = viewer.yaw;
const pitchBefore = viewer.pitch;
emit("canvas", "pointermove", {
  clientX: 150, clientY: 118,
  getCoalescedEvents: () => [{ clientX: 120, clientY: 108 }, { clientX: 150, clientY: 118 }],
});
frame();
check("拖动在下一帧直接更新相机", viewer.yaw !== yawBefore && matrices.length > 0, `yaw=${viewer.yaw.toFixed(3)}`);
check("旋转方向与 RViz Orbit 一致", viewer.yaw < yawBefore && viewer.pitch > pitchBefore,
  `yaw ${yawBefore.toFixed(3)} -> ${viewer.yaw.toFixed(3)}, pitch ${pitchBefore.toFixed(3)} -> ${viewer.pitch.toFixed(3)}`);
check("拖动帧仍不会被 GPU 点云上传阻塞", uploads.length === uploadsAfterFirstCloud);
const interactionDraw = draws.filter((entry) => entry.mode === gl.POINTS).at(-1);
const interactionPointer = attributePointers.at(-1);
check("拖动时等间隔降低点密度", interactionDraw?.count === 60000 && interactionPointer?.stride === 24,
  `draw=${interactionDraw?.count}, stride=${interactionPointer?.stride}`);

emit("canvas", "pointerup", { clientX: 150, clientY: 118 });
frame();
check("松手后只上传最新快照", uploads.length === uploadsAfterFirstCloud + 1 && uploads.at(-1).data === newest);

viewer.top();
check("俯视按钮把相机置于地图上方", viewer.pitch > 1.5, `pitch=${viewer.pitch.toFixed(3)}`);

const targetBefore = viewer.target.slice();
emit("canvas", "pointerdown", { clientX: 200, clientY: 180, button: 1 });
emit("canvas", "pointermove", { clientX: 240, clientY: 210, button: 1 });
emit("canvas", "pointerup", { clientX: 240, clientY: 210, button: 1 });
check("中键与 RViz 一样在相机平面内平移", viewer.target.some((value, index) => value !== targetBefore[index]), viewer.target.map((v) => v.toFixed(2)).join(", "));

const rightDragDistance = viewer.distance;
emit("canvas", "pointerdown", { clientX: 300, clientY: 220, button: 2 });
emit("canvas", "pointermove", { clientX: 300, clientY: 190, button: 2 });
emit("canvas", "pointerup", { clientX: 300, clientY: 190, button: 2 });
check("RViz 右键向上拖动放大", viewer.distance < rightDragDistance, `${rightDragDistance.toFixed(2)} -> ${viewer.distance.toFixed(2)}`);

const distanceBefore = viewer.distance;
emit("canvas", "wheel", { deltaY: 3, deltaMode: 1 });
check("滚轮缩放对行模式差异做了归一化", viewer.distance > distanceBefore, `${distanceBefore.toFixed(2)} -> ${viewer.distance.toFixed(2)}`);

console.log(failures.length ? `\n${failures.length} 项检查未通过` : "\n全部 WebGL 交互检查通过");
process.exit(failures.length ? 1 : 0);
