// Protocol, coordinate and request checks for the real-planner trajectory UI.
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import vm from "node:vm";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const SOURCE = fs.readFileSync(path.join(HERE, "..", "web", "trajectory-lab.js"), "utf8");
const window = {};
const document = { getElementById: () => ({}) };
const sandbox = { window, document, console, ArrayBuffer, DataView, Uint8Array, Number, String, Math, Error, Object, Set };
vm.createContext(sandbox);
vm.runInContext(SOURCE, sandbox, { filename: "trajectory-lab.js" });

const api = window.TrajectoryLabCore;
const failures = [];
function check(label, condition) {
  if (condition) console.log(`  PASS  ${label}`);
  else { failures.push(label); console.log(`  FAIL  ${label}`); }
}

function terrainPayload(width, height, yaw = 0) {
  const count = width * height, buffer = new ArrayBuffer(api.HEADER_SIZE + count), view = new DataView(buffer);
  for (const [index, value] of [..."MPE2"].entries()) view.setUint8(index, value.charCodeAt(0));
  view.setUint8(4, 1); view.setUint32(8, width, true); view.setUint32(12, height, true);
  view.setFloat64(16, 0.1, true); view.setFloat64(24, -1.0, true); view.setFloat64(32, -2.0, true); view.setFloat64(40, yaw, true);
  return buffer;
}

const frame = api.decodeTerrainPayload(terrainPayload(7, 5, Math.PI / 2));
check("解码 MPE2 terrain 标签通道", frame.width === 7 && frame.height === 5 && frame.labels.length === 35);
const world = api.cellToWorld(frame, { x: 2.5, y: 3.5 });
const roundTrip = api.worldToCell(frame, world);
check("旋转地图的栅格/世界坐标可逆", Math.abs(roundTrip.x - 2.5) < 1e-9 && Math.abs(roundTrip.y - 3.5) < 1e-9);

const obstacles = new Set([1 * frame.width + 4]);
const request = api.buildSceneRequest("arena", frame, { x: 1, y: 1 }, { x: 5, y: 3 }, obstacles, api.PARAMETER_DEFAULTS);
check("真实规划请求包含地图、起终点和世界坐标障碍", request.map_name === "arena" && request.start.length === 3 && request.goal.length === 3 && request.obstacles.length === 1);
check("参数覆盖随请求发送且不包含本地文件写入", request.parameters.safe_dist === 0.33 && !Object.hasOwn(request, "params_file"));

const pathPoints = [[0, 0], [2, 0], [2, 2]];
check("真实路径长度按世界米计算", api.pathLength(pathPoints) === 4);
const halfway = api.pointAlongPath(pathPoints, 3);
check("移动质点沿真实 MINCO 路径按弧长回放", halfway[0] === 2 && halfway[1] === 1);

// Every brush stroke re-sends the whole obstacle set, so the browser must stop
// at the same bound the backend enforces instead of building a rejected body.
const backendSource = fs.readFileSync(path.join(HERE, "..", "backend", "trajectory_lab_core.py"), "utf8");
const backendLimit = Number(/^MAX_LAB_OBSTACLES\s*=\s*([\d_]+)/m.exec(backendSource)[1].replaceAll("_", ""));
check("临时障碍上限与后端 MAX_LAB_OBSTACLES 一致", api.OBSTACLE_LIMIT === backendLimit);

const bounded = new Set();
check("未达上限时不报告已满", api.addObstacles(bounded, [1, 2, 3], 3) === false && bounded.size === 3);
check("达到上限后停止新增并报告已满", api.addObstacles(bounded, [3, 4, 5], 3) === true && bounded.size === 3 && !bounded.has(4));
const capped = new Set();
const overflowFull = api.addObstacles(capped, Array.from({ length: backendLimit + 500 }, (_, index) => index));
check("超过上限的批量新增绝不越界", overflowFull === true && capped.size === backendLimit);
check("擦除后可以继续放置", api.addObstacles(bounded, [9], 4) === false && bounded.has(9));
check("前端不再包含本地 A* 或伪 MINCO 求解器", !SOURCE.includes("planGridPath") && !SOURCE.includes("buildSmoothPath"));
check("前端只从真实轨迹 API 读取路径", SOURCE.includes("/api/trajectory/start") && SOURCE.includes("status.minco_path") && SOURCE.includes("status.global_path"));
check("轨迹轮询只在工作区可见时发出请求", /async function pollPlanner\(\)[\s\S]{0,400}?dom\.workspace\.hidden/.test(SOURCE));
check("切回轨迹工作区时立即刷新一次", /trajectory-workspace-activated"[\s\S]{0,160}?pollPlanner\(\)/.test(SOURCE));

// The map picker reads fields off each `/api/maps` entry. A field the backend
// never sends is silently `undefined`, which once filtered out every map and
// left the workspace claiming no YAML + terrain map existed on disk.
function responseKeys(file, functionName) {
  const source = fs.readFileSync(path.join(HERE, "..", "backend", file), "utf8");
  const start = source.indexOf(`def ${functionName}(`);
  if (start < 0) throw new Error(`missing ${functionName} in ${file}`);
  const rest = source.slice(start + 1);
  const nextDef = rest.indexOf("\ndef ");
  const body = nextDef < 0 ? rest : rest.slice(0, nextDef);
  return [...body.matchAll(/"([a-z_][a-z0-9_]*)":/g)].map((match) => match[1]);
}
const servedKeys = new Set([
  ...responseKeys("terrain_core.py", "list_editable_maps"),
  ...responseKeys("map_frame_core.py", "map_frame_status"),
]);
const readKeys = [...new Set([...SOURCE.matchAll(/entry\??\.([A-Za-z_][A-Za-z0-9_]*)/g)].map((match) => match[1]))];
const missingKeys = readKeys.filter((key) => !servedKeys.has(key));
check("地图选择器读取的字段后端都提供", readKeys.length >= 4 && missingKeys.length === 0);
if (missingKeys.length) console.log(`        后端未提供：${missingKeys.join(", ")}`);

if (failures.length) {
  console.error(`${failures.length} trajectory lab harness checks failed`);
  process.exitCode = 1;
} else console.log("Trajectory lab harness passed");
