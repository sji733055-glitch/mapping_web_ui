(function () {
  "use strict";

  const MAGIC = "ROG1";
  const VERSION = 1;
  const HEADER_SIZE = 64;
  const RECORD_SIZE = 17;
  const HEIGHT_VALID = 1;
  const RATIO_VALID = 2;
  const REASONS = ["观测不足", "空列", "薄面", "实心墙", "中空 tunnel", "模糊障碍", "ratio 不可用"];
  const ACTUAL_TYPES = { "-1": "UNKNOWN", 33: "FREE", 66: "PASSABLE", 100: "OCCUPIED" };
  const PARAM_KEYS = ["surface_height_delta_max", "wall_height_delta_min", "wall_occupancy_ratio_min", "tunnel_height_delta_min", "tunnel_height_delta_max", "tunnel_occupancy_ratio_max"];
  const DEFAULT_CONFIG = {
    surface_height_delta_max: 0.20,
    wall_height_delta_min: 0.80,
    wall_occupancy_ratio_min: 0.90,
    tunnel_height_delta_min: 0.25,
    tunnel_height_delta_max: 0.40,
    tunnel_occupancy_ratio_max: 0.45,
  };
  const COLORS = {
    unknown: [45, 55, 55, 255], free: [17, 48, 49, 255], thin: [42, 194, 171, 255],
    tunnel: [166, 124, 255, 255], wall: [230, 79, 76, 255], ambiguous: [224, 155, 63, 255],
    unavailable: [80, 96, 97, 255], actualPassable: [51, 176, 158, 255],
  };

  function validateConfig(config) {
    for (const key of PARAM_KEYS) {
      if (!Number.isFinite(config[key])) return `${key} 必须是数值`;
    }
    if (config.surface_height_delta_max < 0 || config.wall_height_delta_min < 0 || config.tunnel_height_delta_min < 0 || config.tunnel_height_delta_max < 0) return "delta 不能为负数";
    if (config.wall_occupancy_ratio_min < 0 || config.wall_occupancy_ratio_min > 1 || config.tunnel_occupancy_ratio_max < 0 || config.tunnel_occupancy_ratio_max > 1) return "ratio 必须在 0–1 之间";
    if (config.surface_height_delta_max >= config.tunnel_height_delta_min) return "surface delta max 必须小于 tunnel delta min";
    if (config.surface_height_delta_max >= config.wall_height_delta_min) return "surface delta max 必须小于 wall delta min";
    if (config.tunnel_height_delta_min > config.tunnel_height_delta_max) return "tunnel delta min 不能大于 tunnel delta max";
    if (config.tunnel_occupancy_ratio_max >= config.wall_occupancy_ratio_min) return "tunnel ratio max 必须小于 wall ratio min";
    return "";
  }

  function classifyCell(type, span, ratio, flags, config) {
    if (!(flags & HEIGHT_VALID) || !Number.isFinite(span)) return type === 33 ? 1 : type === -1 ? 0 : 6;
    if (span <= config.surface_height_delta_max) return 2;
    if (!(flags & RATIO_VALID) || !Number.isFinite(ratio)) return 6;
    if (span >= config.wall_height_delta_min && ratio >= config.wall_occupancy_ratio_min) return 3;
    if (span >= config.tunnel_height_delta_min && span <= config.tunnel_height_delta_max && ratio <= config.tunnel_occupancy_ratio_max) return 4;
    return 5;
  }

  function decodeFrame(buffer) {
    if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < HEADER_SIZE) throw new Error("ROGMap 数据头不完整");
    const view = new DataView(buffer);
    const magic = String.fromCharCode(view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
    if (magic !== MAGIC) throw new Error("ROGMap 数据 magic 不兼容");
    const version = view.getUint16(4, true), stride = view.getUint16(6, true);
    if (version !== VERSION || stride !== RECORD_SIZE) throw new Error(`ROGMap 数据版本不兼容（${version}/${stride}）`);
    const width = view.getUint32(8, true), height = view.getUint32(12, true), count = width * height;
    if (!width || !height || count > 16000000 || buffer.byteLength !== HEADER_SIZE + count * stride) throw new Error("ROGMap 数据尺寸不正确");
    const frame = {
      width, height, resolution: view.getFloat64(16, true), originX: view.getFloat64(24, true), originY: view.getFloat64(32, true),
      typeStamp: view.getFloat64(40, true), heightStamp: view.getFloat64(48, true), occupiedStamp: view.getFloat64(56, true),
      types: new Int8Array(count), reasons: new Uint8Array(count), flags: new Uint8Array(count), counts: new Uint16Array(count),
      zMax: new Float32Array(count), spans: new Float32Array(count), ratios: new Float32Array(count),
    };
    let offset = HEADER_SIZE;
    for (let index = 0; index < count; index += 1, offset += stride) {
      frame.types[index] = view.getInt8(offset);
      frame.reasons[index] = view.getUint8(offset + 1);
      frame.flags[index] = view.getUint8(offset + 2);
      frame.counts[index] = view.getUint16(offset + 3, true);
      frame.zMax[index] = view.getFloat32(offset + 5, true);
      frame.spans[index] = view.getFloat32(offset + 9, true);
      frame.ratios[index] = view.getFloat32(offset + 13, true);
    }
    return frame;
  }

  window.RogMapProjectionDebug = { decodeFrame, classifyCell, validateConfig, HEADER_SIZE, RECORD_SIZE };

  const $ = (id) => document.getElementById(id);
  const dom = {
    workspace: $("rogmap-workspace"), canvas: $("rogmap-canvas"), wrap: $("rogmap-canvas-wrap"), empty: $("rogmap-empty"),
    badge: $("rogmap-live-badge"), mode: $("rogmap-view-mode"), fit: $("rogmap-fit"), zoomIn: $("rogmap-zoom-in"), zoomOut: $("rogmap-zoom-out"),
    cellCoordinate: $("rogmap-cell-coordinate"), worldCoordinate: $("rogmap-world-coordinate"), frameId: $("rogmap-frame-id"), updateAge: $("rogmap-update-age"),
    cellClass: $("rogmap-cell-class"), actualType: $("rogmap-actual-type"), predictedReason: $("rogmap-predicted-reason"), heightDelta: $("rogmap-height-delta"),
    heightRange: $("rogmap-height-range"), occupiedCount: $("rogmap-occupied-count"), ratio: $("rogmap-ratio"), ruleResult: $("rogmap-rule-result"),
    configState: $("rogmap-config-state"), reset: $("rogmap-reset-params"), validation: $("rogmap-validation"), gridSize: $("rogmap-grid-size"), counts: $("rogmap-counts"), sourceNote: $("rogmap-source-note"),
    inputs: {
      surface_height_delta_max: $("rogmap-surface-max"), wall_height_delta_min: $("rogmap-wall-min"), wall_occupancy_ratio_min: $("rogmap-wall-ratio"),
      tunnel_height_delta_min: $("rogmap-tunnel-min"), tunnel_height_delta_max: $("rogmap-tunnel-max"), tunnel_occupancy_ratio_max: $("rogmap-tunnel-ratio"),
    },
  };
  if (!dom.canvas || typeof dom.canvas.getContext !== "function") return;
  const context = dom.canvas.getContext("2d", { alpha: false });
  const state = {
    frame: null, status: null, baseline: { ...DEFAULT_CONFIG }, config: { ...DEFAULT_CONFIG }, paramsTouched: false,
    imageCanvas: document.createElement("canvas"), imageDirty: true, scale: 1, offsetX: 0, offsetY: 0,
    selected: null, selectedWorld: null, drag: null, pollBusy: false, renderPending: false, lastFrameAt: 0,
  };

  function fillInputs(config) {
    for (const key of PARAM_KEYS) dom.inputs[key].value = String(config[key]);
  }

  function readConfig() {
    const config = {};
    for (const key of PARAM_KEYS) config[key] = Number(dom.inputs[key].value);
    return config;
  }

  function applyConfig() {
    const config = readConfig(), error = validateConfig(config);
    dom.validation.textContent = error || "参数关系合法；图上已按 what-if 重算。";
    dom.validation.classList.toggle("is-error", Boolean(error));
    dom.configState.textContent = state.paramsTouched ? "本地已调整" : "后端基线";
    if (!error) { state.config = config; state.imageDirty = true; updateSelection(); scheduleRender(); }
    return !error;
  }

  function fitCanvasDimensions() {
    const ratio = Math.min(Number(window.devicePixelRatio) || 1, 2), rect = dom.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width * ratio)), height = Math.max(1, Math.round(rect.height * ratio));
    if (dom.canvas.width !== width || dom.canvas.height !== height) { dom.canvas.width = width; dom.canvas.height = height; }
    return { ratio, width: width / ratio, height: height / ratio };
  }

  function fitMap() {
    if (!state.frame) return;
    const box = fitCanvasDimensions();
    state.scale = Math.max(0.05, Math.min(80, Math.min((box.width - 30) / state.frame.width, (box.height - 30) / state.frame.height)));
    state.offsetX = (box.width - state.frame.width * state.scale) / 2;
    state.offsetY = (box.height - state.frame.height * state.scale) / 2;
    scheduleRender();
  }

  function reasonFor(index) {
    const frame = state.frame;
    return classifyCell(frame.types[index], frame.spans[index], frame.ratios[index], frame.flags[index], state.config);
  }

  function colorFor(index, reason) {
    const mode = dom.mode.value, type = state.frame.types[index];
    if (mode === "actual") {
      if (type === -1) return COLORS.unknown;
      if (type === 33) return COLORS.free;
      if (type === 66) return COLORS.actualPassable;
      return COLORS.wall;
    }
    let color = reason === 0 ? COLORS.unknown : reason === 1 ? COLORS.free : reason === 2 ? COLORS.thin : reason === 3 ? COLORS.wall : reason === 4 ? COLORS.tunnel : reason === 5 ? COLORS.ambiguous : COLORS.unavailable;
    if (mode === "tunnel" && reason !== 4) color = [Math.round((color[0] + color[1] + color[2]) / 5), Math.round((color[0] + color[1] + color[2]) / 5), Math.round((color[0] + color[1] + color[2]) / 5), 255];
    return color;
  }

  function rebuildImage() {
    const frame = state.frame;
    if (!frame || !state.imageDirty) return;
    state.imageCanvas.width = frame.width; state.imageCanvas.height = frame.height;
    const imageContext = state.imageCanvas.getContext("2d", { alpha: false });
    const image = imageContext.createImageData(frame.width, frame.height);
    const counts = new Uint32Array(REASONS.length);
    for (let y = 0; y < frame.height; y += 1) {
      for (let x = 0; x < frame.width; x += 1) {
        const index = y * frame.width + x, reason = reasonFor(index), color = colorFor(index, reason);
        counts[reason] += 1;
        const target = ((frame.height - 1 - y) * frame.width + x) * 4;
        image.data[target] = color[0]; image.data[target + 1] = color[1]; image.data[target + 2] = color[2]; image.data[target + 3] = 255;
      }
    }
    imageContext.putImageData(image, 0, 0); state.imageDirty = false;
    dom.counts.replaceChildren();
    for (let reason = 0; reason < counts.length; reason += 1) {
      const item = document.createElement("span"), value = document.createElement("b");
      item.append(REASONS[reason]); value.textContent = counts[reason].toLocaleString("zh-CN"); item.append(value); dom.counts.append(item);
    }
  }

  function scheduleRender() {
    if (state.renderPending) return;
    state.renderPending = true;
    requestAnimationFrame(() => { state.renderPending = false; render(); });
  }

  function render() {
    const box = fitCanvasDimensions();
    context.setTransform(box.ratio, 0, 0, box.ratio, 0, 0); context.fillStyle = "#030708"; context.fillRect(0, 0, box.width, box.height);
    if (!state.frame) return;
    rebuildImage(); context.imageSmoothingEnabled = false;
    context.drawImage(state.imageCanvas, state.offsetX, state.offsetY, state.frame.width * state.scale, state.frame.height * state.scale);
    if (state.selected) {
      const x = state.offsetX + state.selected.x * state.scale, y = state.offsetY + (state.frame.height - 1 - state.selected.y) * state.scale;
      context.strokeStyle = "#ffffff"; context.lineWidth = Math.max(1.5, Math.min(3, state.scale * 0.13)); context.strokeRect(x, y, Math.max(2, state.scale), Math.max(2, state.scale));
    }
  }

  function cellWorld(x, y) {
    return { x: state.frame.originX + (x + 0.5) * state.frame.resolution, y: state.frame.originY + (y + 0.5) * state.frame.resolution };
  }

  function screenToCell(point) {
    if (!state.frame) return null;
    const x = Math.floor((point.x - state.offsetX) / state.scale), displayY = Math.floor((point.y - state.offsetY) / state.scale), y = state.frame.height - 1 - displayY;
    return x >= 0 && y >= 0 && x < state.frame.width && y < state.frame.height ? { x, y } : null;
  }

  function formatMeters(value) { return Number.isFinite(value) ? `${value.toFixed(3)} m` : "—"; }

  function updateSelection() {
    const frame = state.frame, cell = state.selected;
    if (!frame || !cell) return;
    const index = cell.y * frame.width + cell.x, type = frame.types[index], flags = frame.flags[index], span = frame.spans[index], ratio = frame.ratios[index], reason = reasonFor(index), zMax = frame.zMax[index], world = cellWorld(cell.x, cell.y);
    dom.cellCoordinate.textContent = `栅格 X ${cell.x} · Y ${cell.y}`; dom.worldCoordinate.textContent = `世界 ${world.x.toFixed(2)}, ${world.y.toFixed(2)} m`;
    dom.cellClass.textContent = REASONS[reason]; dom.actualType.textContent = ACTUAL_TYPES[String(type)] || `值 ${type}`; dom.predictedReason.textContent = REASONS[reason];
    dom.predictedReason.className = reason === 4 ? "is-tunnel" : reason === 3 ? "is-wall" : reason >= 5 ? "is-warning" : "";
    dom.heightDelta.textContent = flags & HEIGHT_VALID ? formatMeters(span) : "无占据高度数据";
    dom.heightRange.textContent = flags & HEIGHT_VALID ? `${(zMax - span).toFixed(3)} → ${zMax.toFixed(3)} m` : "—";
    dom.occupiedCount.textContent = flags & RATIO_VALID ? `${frame.counts[index]} 层` : "不可完整重建";
    dom.ratio.textContent = flags & RATIO_VALID ? `${ratio.toFixed(3)} (${(ratio * 100).toFixed(1)}%)` : "不可用";
    dom.ratio.className = reason === 4 ? "is-tunnel" : !(flags & RATIO_VALID) ? "is-warning" : "";

    let explanation;
    if (!(flags & HEIGHT_VALID)) explanation = "该格没有可用的占据高度柱，不能做 tunnel 判定。";
    else if (span <= state.config.surface_height_delta_max) explanation = `height_delta ${span.toFixed(3)} ≤ surface max ${state.config.surface_height_delta_max.toFixed(3)}，先命中薄面分支，不会走 tunnel。`;
    else if (!(flags & RATIO_VALID)) explanation = `height_delta 已取到，但 /rog_map/occupied 未完整覆盖该柱上下端或三路快照不同步，ratio 不做猜测。`;
    else {
      const heightOk = span >= state.config.tunnel_height_delta_min && span <= state.config.tunnel_height_delta_max;
      const ratioOk = ratio <= state.config.tunnel_occupancy_ratio_max;
      explanation = `tunnel 高度条件：${span.toFixed(3)} ∈ [${state.config.tunnel_height_delta_min.toFixed(3)}, ${state.config.tunnel_height_delta_max.toFixed(3)}] ${heightOk ? "✓" : "✗"}；ratio 条件：${ratio.toFixed(3)} ≤ ${state.config.tunnel_occupancy_ratio_max.toFixed(3)} ${ratioOk ? "✓" : "✗"}。${reason === 3 ? "实心墙分支优先命中。" : reason === 4 ? "该格在当前 what-if 参数下是 HOLLOW_TUNNEL。" : "未同时满足，落入 AMBIGUOUS_OCCUPIED。"}`;
    }
    dom.ruleResult.textContent = explanation; dom.ruleResult.className = `rogmap-rule-result ${reason === 4 ? "is-tunnel" : reason >= 5 ? "is-warning" : ""}`;
  }

  function updateHover(cell) {
    if (!cell || !state.frame) return;
    const world = cellWorld(cell.x, cell.y); dom.cellCoordinate.textContent = `栅格 X ${cell.x} · Y ${cell.y}`; dom.worldCoordinate.textContent = `世界 ${world.x.toFixed(2)}, ${world.y.toFixed(2)} m`;
  }

  function preserveSelection() {
    if (!state.frame || !state.selectedWorld) return;
    const x = Math.floor((state.selectedWorld.x - state.frame.originX) / state.frame.resolution), y = Math.floor((state.selectedWorld.y - state.frame.originY) / state.frame.resolution);
    state.selected = x >= 0 && y >= 0 && x < state.frame.width && y < state.frame.height ? { x, y } : null;
    if (state.selected) updateSelection();
  }

  async function pollStatus() {
    if (dom.workspace.hidden) return;
    try {
      const response = await fetch(`/api/status?t=${Date.now()}`, { cache: "no-store" });
      if (!response.ok) return;
      const payload = await response.json(), status = payload.rogmap_projection || {};
      state.status = status; dom.frameId.textContent = `坐标系 ${status.frame_id || "—"}`;
      const config = status.config || {};
      if (!state.paramsTouched && PARAM_KEYS.every((key) => Number.isFinite(Number(config[key])))) {
        state.baseline = Object.fromEntries(PARAM_KEYS.map((key) => [key, Number(config[key])])); state.config = { ...state.baseline }; fillInputs(state.config); applyConfig();
      }
      const online = status.layer_type?.online, height = status.height_delta?.online, occupied = status.occupied?.online;
      dom.badge.textContent = online ? "实时" : "无 layer_type"; dom.badge.className = `layer-badge ${online ? "terrain" : ""}`;
      dom.sourceNote.textContent = status.last_error || (!online ? `未收到 ${status.topics?.layer_type || "/rog_map/layer_type"}；确认 nav_executor 和 visualization.enable。` : !height ? `layer_type 已在线，但 ${status.topics?.height_delta || "/rog_map/layer_height_delta"} 无数据。` : !occupied ? `height_delta 已在线；${status.topics?.occupied || "/rog_map/occupied"} 无数据，ratio 将显示不可用。` : "三路 ROGMap 诊断数据已在线；ratio 仅在柱上下端都被占据云覆盖时显示。");
    } catch (_) { /* Main console owns the global connection indicator. */ }
  }

  async function pollFrame() {
    if (dom.workspace.hidden || state.pollBusy) return;
    state.pollBusy = true;
    try {
      const response = await fetch(`/api/rogmap/projection?t=${Date.now()}`, { cache: "no-store" });
      if (!response.ok) { const payload = await response.json().catch(() => ({})); throw new Error(payload.message || `HTTP ${response.status}`); }
      const previous = state.frame, frame = decodeFrame(await response.arrayBuffer()); state.frame = frame; state.lastFrameAt = performance.now(); state.imageDirty = true;
      dom.empty.hidden = true; dom.gridSize.textContent = `${frame.width}×${frame.height} · ${frame.resolution.toFixed(3)} m`;
      if (!previous || previous.width !== frame.width || previous.height !== frame.height) fitMap(); else preserveSelection();
      updateSelection(); scheduleRender(); dom.updateAge.textContent = "刚刚更新";
    } catch (error) {
      if (!state.frame) { dom.empty.hidden = false; dom.empty.querySelector("span").textContent = error.message; }
      dom.updateAge.textContent = error.message;
    } finally { state.pollBusy = false; }
  }

  function eventPoint(event) { const rect = dom.canvas.getBoundingClientRect(); return { x: event.clientX - rect.left, y: event.clientY - rect.top }; }
  dom.canvas.addEventListener("pointerdown", (event) => {
    const point = eventPoint(event); state.drag = { pointerId: event.pointerId, startX: point.x, startY: point.y, lastX: point.x, lastY: point.y, moved: false };
    dom.canvas.setPointerCapture(event.pointerId); event.preventDefault();
  });
  dom.canvas.addEventListener("pointermove", (event) => {
    const point = eventPoint(event);
    if (!state.drag || state.drag.pointerId !== event.pointerId) { updateHover(screenToCell(point)); return; }
    const dx = point.x - state.drag.lastX, dy = point.y - state.drag.lastY;
    if (Math.hypot(point.x - state.drag.startX, point.y - state.drag.startY) > 3) state.drag.moved = true;
    if (state.drag.moved) { state.offsetX += dx; state.offsetY += dy; dom.canvas.classList.add("is-panning"); scheduleRender(); }
    state.drag.lastX = point.x; state.drag.lastY = point.y;
  });
  const releasePointer = (event) => {
    if (!state.drag || state.drag.pointerId !== event.pointerId) return;
    const moved = state.drag.moved; state.drag = null; dom.canvas.classList.remove("is-panning");
    if (!moved) { const cell = screenToCell(eventPoint(event)); if (cell) { state.selected = cell; state.selectedWorld = cellWorld(cell.x, cell.y); updateSelection(); scheduleRender(); } }
    if (dom.canvas.hasPointerCapture(event.pointerId)) dom.canvas.releasePointerCapture(event.pointerId);
  };
  dom.canvas.addEventListener("pointerup", releasePointer); dom.canvas.addEventListener("pointercancel", releasePointer);
  dom.canvas.addEventListener("wheel", (event) => {
    if (!state.frame) return; event.preventDefault(); const point = eventPoint(event), next = Math.max(0.05, Math.min(100, state.scale * (event.deltaY < 0 ? 1.15 : 0.87))), factor = next / state.scale;
    state.offsetX = point.x - (point.x - state.offsetX) * factor; state.offsetY = point.y - (point.y - state.offsetY) * factor; state.scale = next; scheduleRender();
  }, { passive: false });
  dom.canvas.addEventListener("contextmenu", (event) => event.preventDefault());
  dom.fit.addEventListener("click", fitMap); dom.zoomIn.addEventListener("click", () => { state.scale = Math.min(100, state.scale * 1.25); scheduleRender(); }); dom.zoomOut.addEventListener("click", () => { state.scale = Math.max(0.05, state.scale * 0.8); scheduleRender(); });
  dom.mode.addEventListener("change", () => { state.imageDirty = true; scheduleRender(); });
  for (const key of PARAM_KEYS) dom.inputs[key].addEventListener("input", () => { state.paramsTouched = true; applyConfig(); });
  dom.reset.addEventListener("click", () => { state.paramsTouched = false; state.config = { ...state.baseline }; fillInputs(state.config); applyConfig(); });
  window.addEventListener("resize", scheduleRender);
  window.addEventListener("rogmap-workspace-activated", () => { scheduleRender(); if (!state.frame) fitCanvasDimensions(); void pollStatus(); void pollFrame(); });
  if (window.ResizeObserver) new ResizeObserver(scheduleRender).observe(dom.wrap);
  fillInputs(state.config); applyConfig();
  setInterval(pollFrame, 750); setInterval(pollStatus, 2000); setInterval(() => { if (!dom.workspace.hidden && state.lastFrameAt) dom.updateAge.textContent = `${((performance.now() - state.lastFrameAt) / 1000).toFixed(1)} s 前更新`; }, 500);
})();
