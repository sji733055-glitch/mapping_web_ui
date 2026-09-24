(function () {
  "use strict";

  const MAGIC = "MPE2";
  const HEADER_SIZE = 48;
  const LABEL_NAMES = ["平地", "障碍物", null, null, null, "上坡", "隧道", "起伏路段"];
  const LABEL_COLORS = [
    [27, 55, 48], [112, 42, 42], null, null, null,
    [104, 71, 29], [82, 43, 91], [24, 79, 86],
  ];
  const PARAMETER_DEFAULTS = Object.freeze({
    safe_dist: 0.33,
    collision_dist: 0.28,
    max_velocity: 3.0,
    max_acceleration: 4.0,
    penalty_weight_time: 100.0,
    esdf_weight: 1.0,
  });
  // Mirrors backend MAX_LAB_OBSTACLES. Every brush stroke re-sends the whole
  // set, so the cap also keeps the request body well inside the server limit.
  const OBSTACLE_LIMIT = 20000;

  function addObstacles(obstacles, indices, limit = OBSTACLE_LIMIT) {
    let full = false;
    for (const index of indices) {
      if (obstacles.has(index)) continue;
      if (obstacles.size >= limit) { full = true; continue; }
      obstacles.add(index);
    }
    return full;
  }

  function decodeTerrainPayload(buffer) {
    if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < HEADER_SIZE) throw new Error("terrain 数据头不完整");
    const view = new DataView(buffer);
    const magic = String.fromCharCode(view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
    if (magic !== MAGIC) throw new Error("terrain 数据格式不兼容");
    const layer = view.getUint8(4);
    const width = view.getUint32(8, true), height = view.getUint32(12, true), count = width * height;
    if (layer !== 1 || !width || !height || !Number.isSafeInteger(count) || count > 16000000) throw new Error("terrain 图层或尺寸无效");
    if (buffer.byteLength !== HEADER_SIZE + count) throw new Error("terrain label 通道长度无效");
    const frame = {
      width, height,
      resolution: view.getFloat64(16, true),
      originX: view.getFloat64(24, true),
      originY: view.getFloat64(32, true),
      originYaw: view.getFloat64(40, true),
      labels: new Uint8Array(buffer.slice(HEADER_SIZE, HEADER_SIZE + count)),
    };
    if (![frame.resolution, frame.originX, frame.originY, frame.originYaw].every(Number.isFinite) || frame.resolution <= 0) throw new Error("terrain 地图几何信息无效");
    for (const value of frame.labels) if (![0, 1, 5, 6, 7].includes(value)) throw new Error(`terrain 包含非法 label ${value}`);
    return frame;
  }

  function cellToWorld(frame, point) {
    const localX = point.x * frame.resolution, localY = point.y * frame.resolution;
    const cosine = Math.cos(frame.originYaw), sine = Math.sin(frame.originYaw);
    return {
      x: frame.originX + cosine * localX - sine * localY,
      y: frame.originY + sine * localX + cosine * localY,
    };
  }

  function worldToCell(frame, point) {
    const dx = point.x - frame.originX, dy = point.y - frame.originY;
    const cosine = Math.cos(frame.originYaw), sine = Math.sin(frame.originYaw);
    return {
      x: (cosine * dx + sine * dy) / frame.resolution,
      y: (-sine * dx + cosine * dy) / frame.resolution,
    };
  }

  function pathLength(path) {
    let total = 0;
    for (let index = 1; index < path.length; index += 1) total += Math.hypot(path[index][0] - path[index - 1][0], path[index][1] - path[index - 1][1]);
    return total;
  }

  function pointAlongPath(path, distance) {
    if (!path.length) return null;
    let remaining = Math.max(0, distance);
    for (let index = 1; index < path.length; index += 1) {
      const a = path[index - 1], b = path[index], segment = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (remaining <= segment || index === path.length - 1) {
        const ratio = segment > 1e-9 ? Math.min(1, remaining / segment) : 0;
        return [a[0] + (b[0] - a[0]) * ratio, a[1] + (b[1] - a[1]) * ratio];
      }
      remaining -= segment;
    }
    return path[path.length - 1].slice();
  }

  function buildSceneRequest(mapName, frame, start, goal, obstacles, parameters) {
    if (!mapName || !frame || !start || !goal) throw new Error("请先导入 terrain 并设置车体和目标位置");
    const pose = (cell) => {
      const world = cellToWorld(frame, { x: cell.x + 0.5, y: cell.y + 0.5 });
      return [world.x, world.y, 0];
    };
    return {
      map_name: mapName,
      start: pose(start),
      goal: pose(goal),
      obstacles: Array.from(obstacles, (index) => {
        const world = cellToWorld(frame, { x: index % frame.width + 0.5, y: Math.floor(index / frame.width) + 0.5 });
        return [world.x, world.y];
      }),
      parameters: { ...parameters },
    };
  }

  window.TrajectoryLabCore = {
    HEADER_SIZE, PARAMETER_DEFAULTS, OBSTACLE_LIMIT, addObstacles, decodeTerrainPayload, cellToWorld, worldToCell,
    pathLength, pointAlongPath, buildSceneRequest,
  };

  const $ = (id) => document.getElementById(id);
  const dom = {
    workspace: $("trajectory-workspace"), canvas: $("trajectory-canvas"), wrap: $("trajectory-canvas-wrap"), empty: $("trajectory-empty"),
    title: $("trajectory-title"), badge: $("trajectory-state-badge"), fit: $("trajectory-fit"), zoomIn: $("trajectory-zoom-in"), zoomOut: $("trajectory-zoom-out"),
    refresh: $("trajectory-refresh-maps"), select: $("trajectory-map-select"), load: $("trajectory-load"), run: $("trajectory-run"), stop: $("trajectory-stop"), sourceNote: $("trajectory-source-note"),
    toolLabel: $("trajectory-tool-label"), brush: $("trajectory-brush-radius"), brushOutput: $("trajectory-brush-output"), clearObstacles: $("trajectory-clear-obstacles"),
    play: $("trajectory-play"), resetParticle: $("trajectory-reset-particle"), playbackSpeed: $("trajectory-playback-speed"), speedOutput: $("trajectory-speed-output"), particleStatus: $("trajectory-particle-status"),
    safeDist: $("trajectory-safe-dist"), safeDistOutput: $("trajectory-safe-dist-output"), collisionDist: $("trajectory-collision-dist"), collisionDistOutput: $("trajectory-collision-dist-output"),
    maxVelocity: $("trajectory-max-velocity"), maxVelocityOutput: $("trajectory-max-velocity-output"), maxAcceleration: $("trajectory-max-acceleration"), maxAccelerationOutput: $("trajectory-max-acceleration-output"),
    penaltyTime: $("trajectory-penalty-time"), penaltyTimeOutput: $("trajectory-penalty-time-output"), esdfWeight: $("trajectory-esdf-weight"), esdfWeightOutput: $("trajectory-esdf-weight-output"), resetParams: $("trajectory-reset-params"),
    coordinate: $("trajectory-coordinate"), worldCoordinate: $("trajectory-world-coordinate"), cellValue: $("trajectory-cell-value"), replanState: $("trajectory-replan-state"),
    planTime: $("trajectory-plan-time"), globalLength: $("trajectory-global-length"), mincoLength: $("trajectory-minco-length"), pathPoints: $("trajectory-path-points"),
    blockedCells: $("trajectory-blocked-cells"), obstacleCount: $("trajectory-obstacle-count"), cmdCount: $("trajectory-cmd-count"), impactNote: $("trajectory-impact-note"),
  };
  if (!dom.canvas || typeof dom.canvas.getContext !== "function") return;
  const context = dom.canvas.getContext("2d", { alpha: false });
  const state = {
    maps: [], mapName: "", frame: null, mapCanvas: document.createElement("canvas"), imageDirty: true,
    tool: "start", brushRadius: 4, obstacles: new Set(), start: null, goal: null,
    obstacleFullWarned: false,
    globalPath: [], mincoPath: [], plannerState: "STOPPED", generation: 0,
    scale: 1, offsetX: 0, offsetY: 0, drag: null, renderPending: false,
    playing: false, particleDistance: 0, particle: null, lastAnimationAt: 0, animationFrame: 0,
    obstacleTimer: 0, pollTimer: 0, requestBusy: false,
  };

  function toast(message, kind) {
    if (typeof window.showToast === "function") window.showToast(message, kind);
    else if (kind === "error") console.error(message);
  }
  async function responseError(response) {
    try { const body = await response.json(); return new Error(body.message || `HTTP ${response.status}`); }
    catch (_error) { return new Error(`HTTP ${response.status}`); }
  }
  async function postJson(url, payload) {
    const response = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload), cache: "no-store" });
    if (!response.ok) throw await responseError(response);
    return response.json();
  }
  function selectedMap() { return state.maps.find((entry) => entry.name === dom.select.value && entry.has_terrain); }
  async function refreshMaps() {
    dom.refresh.disabled = true;
    try {
      const response = await fetch("/api/maps", { cache: "no-store" });
      if (!response.ok) throw await responseError(response);
      const previous = dom.select.value, body = await response.json();
      state.maps = (body.maps || []).filter((entry) => entry.has_terrain && entry.has_yaml);
      dom.select.replaceChildren();
      if (!state.maps.length) dom.select.append(new Option("没有同时含 YAML 与 terrain 的地图", ""));
      else for (const entry of state.maps) dom.select.append(new Option(entry.name, entry.name));
      if (state.maps.some((entry) => entry.name === previous)) dom.select.value = previous;
      updateSourceUi();
    } catch (error) {
      dom.select.replaceChildren(new Option("terrain 列表读取失败", "")); dom.sourceNote.textContent = error.message; toast(error.message, "error");
    } finally { dom.refresh.disabled = false; }
  }
  function updateSourceUi() {
    const entry = selectedMap(); dom.load.disabled = !entry;
    if (!entry) dom.sourceNote.textContent = "data/map 中没有可供真实规划器使用的 YAML + terrain 地图。";
    else dom.sourceNote.textContent = `${entry.width || "?"}×${entry.height || "?"} · ${Number(entry.resolution || 0).toFixed(3)} m/px · terrain 只读`;
  }
  function nearestFree(targetX, targetY) {
    const frame = state.frame;
    if (!frame) return null;
    const tx = Math.max(0, Math.min(frame.width - 1, Math.round(targetX))), ty = Math.max(0, Math.min(frame.height - 1, Math.round(targetY)));
    for (let radius = 0; radius <= Math.max(frame.width, frame.height); radius += 1) {
      for (let y = ty - radius; y <= ty + radius; y += 1) for (let x = tx - radius; x <= tx + radius; x += 1) {
        if (x >= 0 && y >= 0 && x < frame.width && y < frame.height && frame.labels[y * frame.width + x] !== 1) return { x, y };
      }
    }
    return null;
  }
  async function loadTerrain() {
    const entry = selectedMap(); if (!entry) return;
    dom.load.disabled = true; dom.sourceNote.textContent = "正在读取 terrain label…";
    try {
      const response = await fetch(`/api/editor/terrain?${new URLSearchParams({ map_name: entry.name })}`, { cache: "no-store" });
      if (!response.ok) throw await responseError(response);
      state.frame = decodeTerrainPayload(await response.arrayBuffer()); state.mapName = entry.name; state.imageDirty = true;
      state.obstacles.clear(); state.obstacleFullWarned = false; state.globalPath = []; state.mincoPath = []; state.start = nearestFree(state.frame.width * 0.18, state.frame.height * 0.5); state.goal = nearestFree(state.frame.width * 0.82, state.frame.height * 0.5);
      resetParticle(); dom.empty.hidden = true; dom.title.textContent = `${entry.name} · 真实轨迹验证`; dom.badge.textContent = "terrain 已加载"; dom.badge.className = "layer-badge terrain";
      fitMap(); updateControls(); updateMetrics(null); toast(`已只读导入 ${entry.name}`, "success");
    } catch (error) { dom.sourceNote.textContent = error.message; toast(error.message, "error"); }
    finally { updateSourceUi(); }
  }

  function currentParameters() {
    return {
      safe_dist: Number(dom.safeDist.value), collision_dist: Number(dom.collisionDist.value),
      max_velocity: Number(dom.maxVelocity.value), max_acceleration: Number(dom.maxAcceleration.value),
      penalty_weight_time: Number(dom.penaltyTime.value), esdf_weight: Number(dom.esdfWeight.value),
    };
  }
  function updateParameterOutputs() {
    dom.safeDistOutput.textContent = Number(dom.safeDist.value).toFixed(2); dom.collisionDistOutput.textContent = Number(dom.collisionDist.value).toFixed(2);
    dom.maxVelocityOutput.textContent = Number(dom.maxVelocity.value).toFixed(1); dom.maxAccelerationOutput.textContent = Number(dom.maxAcceleration.value).toFixed(1);
    dom.penaltyTimeOutput.textContent = Number(dom.penaltyTime.value).toFixed(0); dom.esdfWeightOutput.textContent = Number(dom.esdfWeight.value).toFixed(1);
    dom.run.textContent = state.plannerState === "STOPPED" ? "启动真实规划" : "重新启动并规划";
  }
  function resetParameters() {
    dom.safeDist.value = PARAMETER_DEFAULTS.safe_dist; dom.collisionDist.value = PARAMETER_DEFAULTS.collision_dist;
    dom.maxVelocity.value = PARAMETER_DEFAULTS.max_velocity; dom.maxAcceleration.value = PARAMETER_DEFAULTS.max_acceleration;
    dom.penaltyTime.value = PARAMETER_DEFAULTS.penalty_weight_time; dom.esdfWeight.value = PARAMETER_DEFAULTS.esdf_weight; updateParameterOutputs();
  }
  async function runPlanner() {
    if (state.requestBusy) return;
    state.requestBusy = true; dom.run.disabled = true; dom.replanState.textContent = "正在启动真实规划器…";
    try {
      const payload = buildSceneRequest(state.mapName, state.frame, state.start, state.goal, state.obstacles, currentParameters());
      const body = await postJson("/api/trajectory/start", payload); applyPlannerStatus(body.trajectory); toast(body.message, "success");
    } catch (error) { toast(error.message, "error"); dom.replanState.textContent = error.message; }
    finally { state.requestBusy = false; updateControls(); }
  }
  async function stopPlanner() {
    if (state.requestBusy) return;
    state.requestBusy = true;
    try { const body = await postJson("/api/trajectory/stop", {}); applyPlannerStatus(body.trajectory); toast(body.message, "success"); }
    catch (error) { toast(error.message, "error"); }
    finally { state.requestBusy = false; updateControls(); }
  }
  function scheduleObstacleUpdate() {
    clearTimeout(state.obstacleTimer);
    if (!["STARTING", "PLANNING", "READY"].includes(state.plannerState)) return;
    state.obstacleTimer = setTimeout(async () => {
      try {
        const payload = buildSceneRequest(state.mapName, state.frame, state.start, state.goal, state.obstacles, currentParameters());
        const body = await postJson("/api/trajectory/obstacles", { obstacles: payload.obstacles }); applyPlannerStatus(body.trajectory);
      } catch (error) { toast(error.message, "error"); }
    }, 180);
  }
  async function pollPlanner() {
    // A running laboratory can publish thousands of path points, so only poll
    // while the operator is actually looking at this workspace; activation
    // polls once immediately, matching the ROGMap workspace.
    if (!dom.workspace.hidden) {
      try {
        const response = await fetch("/api/trajectory", { cache: "no-store" });
        if (response.ok) applyPlannerStatus(await response.json());
      } catch (_error) { /* app-level transport state already reports backend failures */ }
    }
    state.pollTimer = window.setTimeout(pollPlanner, 500);
  }
  function applyPlannerStatus(status) {
    if (!status || typeof status !== "object") return;
    const changed = status.generation !== state.generation;
    state.generation = Number(status.generation || 0); state.plannerState = status.state || "STOPPED";
    state.globalPath = Array.isArray(status.global_path) ? status.global_path : [];
    state.mincoPath = Array.isArray(status.minco_path) ? status.minco_path : [];
    if (changed) resetParticle();
    dom.badge.textContent = ({ STOPPED: "已停止", STARTING: "启动中", PLANNING: "真实规划中", READY: "真实轨迹就绪", ERROR: "规划错误", STOPPING: "停止中" })[state.plannerState] || state.plannerState;
    dom.badge.className = `layer-badge ${state.plannerState === "READY" ? "terrain" : ""}`;
    dom.replanState.textContent = status.message || "等待规划器";
    updateMetrics(status); updateControls(); scheduleRender();
  }

  function fitCanvasDimensions() {
    const ratio = Math.min(Number(window.devicePixelRatio) || 1, 2), rect = dom.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width * ratio)), height = Math.max(1, Math.round(rect.height * ratio));
    if (dom.canvas.width !== width || dom.canvas.height !== height) { dom.canvas.width = width; dom.canvas.height = height; }
    return { ratio, width: width / ratio, height: height / ratio };
  }
  function fitMap() {
    if (!state.frame) return;
    const box = fitCanvasDimensions(); state.scale = Math.max(0.05, Math.min(80, Math.min((box.width - 30) / state.frame.width, (box.height - 30) / state.frame.height)));
    state.offsetX = (box.width - state.frame.width * state.scale) / 2; state.offsetY = (box.height - state.frame.height * state.scale) / 2; scheduleRender();
  }
  function rebuildMapImage() {
    if (!state.frame || !state.imageDirty) return;
    const frame = state.frame; state.mapCanvas.width = frame.width; state.mapCanvas.height = frame.height;
    const imageContext = state.mapCanvas.getContext("2d", { alpha: false }), image = imageContext.createImageData(frame.width, frame.height);
    for (let y = 0; y < frame.height; y += 1) for (let x = 0; x < frame.width; x += 1) {
      const color = LABEL_COLORS[frame.labels[y * frame.width + x]] || [70, 70, 70], target = ((frame.height - 1 - y) * frame.width + x) * 4;
      image.data[target] = color[0]; image.data[target + 1] = color[1]; image.data[target + 2] = color[2]; image.data[target + 3] = 255;
    }
    imageContext.putImageData(image, 0, 0); state.imageDirty = false;
  }
  function screenPoint(cell) { return { x: state.offsetX + cell.x * state.scale, y: state.offsetY + (state.frame.height - cell.y) * state.scale }; }
  function drawWorldPath(path, color, width) {
    if (!state.frame || path.length < 2) return;
    context.save(); context.strokeStyle = color; context.lineWidth = width; context.lineJoin = "round"; context.lineCap = "round"; context.beginPath();
    path.forEach((point, index) => { const screen = screenPoint(worldToCell(state.frame, { x: point[0], y: point[1] })); if (!index) context.moveTo(screen.x, screen.y); else context.lineTo(screen.x, screen.y); });
    context.stroke(); context.restore();
  }
  function drawMarker(cell, color, label) {
    if (!cell) return; const point = screenPoint({ x: cell.x + 0.5, y: cell.y + 0.5 });
    context.save(); context.fillStyle = color; context.strokeStyle = "#031010"; context.lineWidth = 2; context.beginPath(); context.arc(point.x, point.y, 7, 0, Math.PI * 2); context.fill(); context.stroke();
    context.fillStyle = "#041111"; context.font = "700 10px ui-monospace, monospace"; context.textAlign = "center"; context.textBaseline = "middle"; context.fillText(label, point.x, point.y + 0.5); context.restore();
  }
  function render() {
    const box = fitCanvasDimensions(); context.setTransform(box.ratio, 0, 0, box.ratio, 0, 0); context.fillStyle = "#030708"; context.fillRect(0, 0, box.width, box.height);
    if (!state.frame) return;
    rebuildMapImage(); context.imageSmoothingEnabled = false; context.drawImage(state.mapCanvas, state.offsetX, state.offsetY, state.frame.width * state.scale, state.frame.height * state.scale);
    context.save(); context.fillStyle = "rgba(255, 65, 61, .88)";
    for (const index of state.obstacles) { const x = index % state.frame.width, y = Math.floor(index / state.frame.width), topLeft = screenPoint({ x, y: y + 1 }); context.fillRect(topLeft.x, topLeft.y, Math.max(1, state.scale), Math.max(1, state.scale)); }
    context.restore(); drawWorldPath(state.globalPath, "#49e8d4", 2.5); drawWorldPath(state.mincoPath, "#f0b85c", 3.2); drawMarker(state.start, "#53d99f", "V"); drawMarker(state.goal, "#ff716a", "G");
    if (state.particle) { const cell = worldToCell(state.frame, { x: state.particle[0], y: state.particle[1] }), point = screenPoint(cell); context.save(); context.shadowColor = "#fff"; context.shadowBlur = 14; context.fillStyle = "#fff"; context.beginPath(); context.arc(point.x, point.y, 5.5, 0, Math.PI * 2); context.fill(); context.restore(); }
  }
  function scheduleRender() { if (state.renderPending) return; state.renderPending = true; requestAnimationFrame(() => { state.renderPending = false; render(); }); }
  function updateMetrics(status) {
    dom.globalLength.textContent = state.globalPath.length ? `${pathLength(state.globalPath).toFixed(2)} m` : "—";
    dom.mincoLength.textContent = state.mincoPath.length ? `${pathLength(state.mincoPath).toFixed(2)} m` : "—";
    dom.pathPoints.textContent = `${state.globalPath.length} / ${state.mincoPath.length}`; dom.obstacleCount.textContent = state.obstacles.size.toLocaleString("zh-CN");
    const constraints = status?.constraints || {}; dom.blockedCells.textContent = Number(constraints.blocked || 0).toLocaleString("zh-CN");
    dom.cmdCount.textContent = Number(status?.isolated_cmd_vel?.received || 0).toLocaleString("zh-CN"); dom.planTime.textContent = status?.plan_elapsed_seconds ? `${Number(status.plan_elapsed_seconds).toFixed(1)} s` : "—";
    if (state.mincoPath.length) dom.impactNote.textContent = `当前显示来自 mas2027_nav_executor 的 ${state.globalPath.length} 点全局搜索折线与 ${state.mincoPath.length} 点 MINCO 轨迹，背景为地图地形标签。`;
    else if (state.plannerState === "ERROR") dom.impactNote.textContent = status?.message || "真实规划器发生错误，请查看隔离日志。";
  }
  function updateControls() {
    const active = ["STARTING", "PLANNING", "READY"].includes(state.plannerState), hasPath = state.mincoPath.length > 1;
    dom.run.disabled = !state.frame || !state.start || !state.goal || state.requestBusy; dom.stop.disabled = !active || state.requestBusy; dom.clearObstacles.disabled = !state.obstacles.size;
    dom.play.disabled = !hasPath; dom.resetParticle.disabled = !hasPath; updateParameterOutputs();
    const arrived = hasPath && state.particleDistance >= pathLength(state.mincoPath) - 1e-6;
    dom.play.textContent = state.playing ? "❚❚ 暂停" : arrived ? "↻ 重新回放" : "▶ 开始";
    dom.particleStatus.textContent = !hasPath ? "等待真实 MINCO" : state.playing ? "回放中" : arrived ? "已到达" : state.particleDistance > 0 ? "已暂停" : "待命";
  }
  function setTool(tool) {
    state.tool = tool; const names = { start: "设置车体", goal: "设置目标", obstacle: "放障碍", erase: "擦除", pan: "平移" }; dom.toolLabel.textContent = names[tool] || tool;
    for (const button of document.querySelectorAll("[data-trajectory-tool]")) button.classList.toggle("is-active", button.dataset.trajectoryTool === tool);
    dom.canvas.classList.toggle("is-obstacle", tool === "obstacle"); dom.canvas.classList.toggle("is-erasing", tool === "erase"); dom.canvas.classList.toggle("is-panning", tool === "pan");
  }
  function eventPoint(event) { const rect = dom.canvas.getBoundingClientRect(); return { x: event.clientX - rect.left, y: event.clientY - rect.top }; }
  function screenToCell(point) {
    if (!state.frame) return null; const x = Math.floor((point.x - state.offsetX) / state.scale), displayY = Math.floor((point.y - state.offsetY) / state.scale), y = state.frame.height - 1 - displayY;
    return x >= 0 && y >= 0 && x < state.frame.width && y < state.frame.height ? { x, y } : null;
  }
  function updateHover(cell) {
    if (!cell || !state.frame) return; const index = cell.y * state.frame.width + cell.x, world = cellToWorld(state.frame, { x: cell.x + 0.5, y: cell.y + 0.5 });
    dom.coordinate.textContent = `栅格 X ${cell.x} · Y ${cell.y}`; dom.worldCoordinate.textContent = `世界 ${world.x.toFixed(2)}, ${world.y.toFixed(2)} m`;
    const label = state.frame.labels[index]; dom.cellValue.textContent = `${label} · ${LABEL_NAMES[label]}`;
  }
  function paintObstacle(cell, erase) {
    if (!cell || !state.frame) return;
    const additions = [];
    for (let dy = -state.brushRadius + 1; dy < state.brushRadius; dy += 1) for (let dx = -state.brushRadius + 1; dx < state.brushRadius; dx += 1) {
      if (dx * dx + dy * dy > (state.brushRadius - 0.25) ** 2) continue; const x = cell.x + dx, y = cell.y + dy;
      if (x < 0 || y < 0 || x >= state.frame.width || y >= state.frame.height) continue; const index = y * state.frame.width + x;
      if (erase) state.obstacles.delete(index); else additions.push(index);
    }
    const full = addObstacles(state.obstacles, additions);
    if (full && !state.obstacleFullWarned) { state.obstacleFullWarned = true; toast(`临时障碍最多 ${OBSTACLE_LIMIT.toLocaleString("en-US")} 个格，已达到上限`, "error"); }
    if (!full) state.obstacleFullWarned = false;
    updateControls(); scheduleRender(); scheduleObstacleUpdate();
  }
  dom.canvas.addEventListener("pointerdown", (event) => {
    if (!state.frame) return; const point = eventPoint(event), cell = screenToCell(point); state.drag = { pointerId: event.pointerId, lastX: point.x, lastY: point.y }; dom.canvas.setPointerCapture(event.pointerId); event.preventDefault();
    if (state.tool === "start" && cell && state.frame.labels[cell.y * state.frame.width + cell.x] !== 1) { state.start = cell; updateControls(); scheduleRender(); }
    else if (state.tool === "goal" && cell && state.frame.labels[cell.y * state.frame.width + cell.x] !== 1) { state.goal = cell; updateControls(); scheduleRender(); }
    else if (state.tool === "obstacle" || state.tool === "erase") paintObstacle(cell, state.tool === "erase");
  });
  dom.canvas.addEventListener("pointermove", (event) => {
    const point = eventPoint(event), cell = screenToCell(point); updateHover(cell); if (!state.drag || state.drag.pointerId !== event.pointerId) return;
    if (state.tool === "pan") { state.offsetX += point.x - state.drag.lastX; state.offsetY += point.y - state.drag.lastY; scheduleRender(); }
    else if (state.tool === "obstacle" || state.tool === "erase") paintObstacle(cell, state.tool === "erase"); state.drag.lastX = point.x; state.drag.lastY = point.y;
  });
  const releasePointer = (event) => { if (!state.drag || state.drag.pointerId !== event.pointerId) return; state.drag = null; if (dom.canvas.hasPointerCapture(event.pointerId)) dom.canvas.releasePointerCapture(event.pointerId); scheduleObstacleUpdate(); };
  dom.canvas.addEventListener("pointerup", releasePointer); dom.canvas.addEventListener("pointercancel", releasePointer);
  dom.canvas.addEventListener("wheel", (event) => { if (!state.frame) return; event.preventDefault(); const point = eventPoint(event), next = Math.max(0.05, Math.min(100, state.scale * (event.deltaY < 0 ? 1.15 : 0.87))), factor = next / state.scale; state.offsetX = point.x - (point.x - state.offsetX) * factor; state.offsetY = point.y - (point.y - state.offsetY) * factor; state.scale = next; scheduleRender(); }, { passive: false });
  dom.canvas.addEventListener("contextmenu", (event) => event.preventDefault());

  function resetParticle() { state.playing = false; state.particleDistance = 0; state.particle = state.mincoPath[0]?.slice() || null; updateControls(); scheduleRender(); }
  function animationTick(now) {
    if (!state.playing || !state.mincoPath.length) return; const delta = Math.min(0.1, Math.max(0, (now - state.lastAnimationAt) / 1000)); state.lastAnimationAt = now; state.particleDistance += delta * Number(dom.playbackSpeed.value);
    const total = pathLength(state.mincoPath); if (state.particleDistance >= total) { state.particleDistance = total; state.playing = false; } state.particle = pointAlongPath(state.mincoPath, state.particleDistance); updateControls(); scheduleRender(); if (state.playing) state.animationFrame = requestAnimationFrame(animationTick);
  }
  function togglePlayback() {
    if (!state.mincoPath.length) return; state.playing = !state.playing;
    if (state.playing) { if (state.particleDistance >= pathLength(state.mincoPath)) state.particleDistance = 0; state.lastAnimationAt = performance.now(); state.animationFrame = requestAnimationFrame(animationTick); }
    else cancelAnimationFrame(state.animationFrame); updateControls();
  }

  dom.refresh.addEventListener("click", refreshMaps); dom.select.addEventListener("change", updateSourceUi); dom.load.addEventListener("click", loadTerrain); dom.run.addEventListener("click", runPlanner); dom.stop.addEventListener("click", stopPlanner);
  for (const button of document.querySelectorAll("[data-trajectory-tool]")) button.addEventListener("click", () => setTool(button.dataset.trajectoryTool));
  dom.brush.addEventListener("input", () => { state.brushRadius = Number(dom.brush.value); dom.brushOutput.textContent = `${state.brushRadius} px`; });
  dom.clearObstacles.addEventListener("click", () => { state.obstacles.clear(); state.obstacleFullWarned = false; updateControls(); scheduleRender(); scheduleObstacleUpdate(); });
  dom.play.addEventListener("click", togglePlayback); dom.resetParticle.addEventListener("click", resetParticle); dom.playbackSpeed.addEventListener("input", () => { dom.speedOutput.textContent = `${Number(dom.playbackSpeed.value).toFixed(1)} m/s`; });
  for (const input of [dom.safeDist, dom.collisionDist, dom.maxVelocity, dom.maxAcceleration, dom.penaltyTime, dom.esdfWeight]) input.addEventListener("input", updateParameterOutputs);
  dom.resetParams.addEventListener("click", resetParameters); dom.fit.addEventListener("click", fitMap); dom.zoomIn.addEventListener("click", () => { state.scale = Math.min(100, state.scale * 1.25); scheduleRender(); }); dom.zoomOut.addEventListener("click", () => { state.scale = Math.max(0.05, state.scale * 0.8); scheduleRender(); });
  window.addEventListener("resize", scheduleRender); window.addEventListener("trajectory-workspace-activated", () => { scheduleRender(); if (!state.maps.length) void refreshMaps(); void pollPlanner(); }); if (window.ResizeObserver) new ResizeObserver(scheduleRender).observe(dom.wrap);
  resetParameters(); setTool("start"); updateControls(); void pollPlanner();
})();
