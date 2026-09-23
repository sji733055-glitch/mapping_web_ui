(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const dom = {
    mappingTab: $("mapping-tab"), editorTab: $("editor-tab"), rogmapTab: $("rogmap-tab"), trajectoryTab: $("trajectory-tab"),
    mappingWorkspace: $("mapping-workspace"), editorWorkspace: $("editor-workspace"), rogmapWorkspace: $("rogmap-workspace"), trajectoryWorkspace: $("trajectory-workspace"),
    canvas: $("map-editor-canvas"), canvasWrap: $("map-canvas-wrap"), empty: $("editor-empty"),
    title: $("editor-title"), layerBadge: $("editor-layer-badge"), zoomLabel: $("editor-zoom-label"),
    zoomIn: $("editor-zoom-in"), zoomOut: $("editor-zoom-out"), fit: $("editor-fit"),
    cloudToggle: $("editor-cloud-toggle"), cloudStatus: $("editor-cloud-status"),
    select: $("editor-map-select"), refresh: $("editor-refresh-maps"),
    loadOccupancy: $("editor-load-occupancy"), openTerrain: $("editor-open-terrain"), convert: $("editor-convert"), sourceNote: $("editor-source-note"),
    brushSize: $("editor-brush-size"), brushOutput: $("editor-brush-output"),
    undo: $("editor-undo"), historyLabel: $("editor-history-label"), palette: $("editor-label-palette"),
    frameX: $("editor-frame-x"), frameY: $("editor-frame-y"), frameYaw: $("editor-frame-yaw"),
    framePick: $("editor-frame-pick"), frameApply: $("editor-frame-apply"), frameNote: $("editor-frame-note"), frameRevision: $("editor-frame-revision"),
    directionCard: $("editor-direction-card"), direction: $("editor-direction"), directionValue: $("editor-direction-value"), directionArrow: $("editor-direction-arrow"), showArrows: $("editor-show-arrows"),
    save: $("editor-save"), saveStatus: $("editor-save-status"), coordinate: $("editor-coordinate"), worldCoordinate: $("editor-world-coordinate"), cellValue: $("editor-cell-value"), dirty: $("editor-dirty")
  };

  const MAGIC = "MPE2";
  const HEADER_SIZE = 48;
  const FRAME_AXIS_PX = 58;
  const FRAME_HANDLE_PX = 13;
  const LAYER_OCCUPANCY = 0;
  const LAYER_TERRAIN = 1;
  const MAX_HISTORY_BYTES = 64 * 1024 * 1024;
  const MAX_HISTORY_STEPS = 12;

  const occupancyLabels = [
    { value: 0, name: "障碍物", color: "#111718" },
    { value: 254, name: "可通行", color: "#e8f0ef" },
    { value: 205, name: "未知区域", color: "#74817f" }
  ];
  const terrainLabels = [
    { value: 0, name: "平地", color: "#4caf50" },
    { value: 1, name: "障碍物", color: "#f44336" },
    { value: 2, name: "斜坡", color: "#ff9800" },
    { value: 3, name: "一级台阶", color: "#ffeb3b" },
    { value: 4, name: "二级台阶", color: "#9c27b0" },
    { value: 5, name: "飞坡", color: "#00bcd4" },
    { value: 6, name: "高台阶", color: "#00f2ff" }
  ];
  const terrainRgb = [
    [76, 175, 80], [244, 67, 54], [255, 152, 0], [255, 235, 59],
    [156, 39, 176], [0, 188, 212], [0, 242, 255]
  ];

  const state = {
    maps: [], mapName: "", layer: null, width: 0, height: 0, resolution: 0.05,
    originX: 0, originY: 0, originYaw: 0, values: null, direction: null,
    dirty: false, busy: false, tool: "brush", label: 0, brushSize: 3, directionValue: 64,
    zoom: 1, panX: 0, panY: 0, hover: null, drag: null, preview: null,
    history: [], historyBytes: 0, renderPending: false, mapImageDirty: true,
    mapCanvas: document.createElement("canvas"), mapsLoaded: false, frameDraft: null,
    cloudCanvas: document.createElement("canvas"), cloudPoints: null, cloudMapName: "",
    cloudVisible: false, cloudLoading: false, cloudShown: 0, cloudTotal: 0,
    cloudInBounds: 0, cloudSliceLabel: "", cloudRequestSerial: 0
  };
  const ctx = dom.canvas.getContext("2d", { alpha: false });
  const mapCtx = state.mapCanvas.getContext("2d", { alpha: false });

  function toast(message, kind = "info") {
    if (window.mappingConsoleToast) window.mappingConsoleToast(message, kind);
  }

  async function responseError(response) {
    try {
      const payload = await response.json();
      return new Error(payload.message || `请求失败 (${response.status})`);
    } catch (_) {
      return new Error(`请求失败 (${response.status})`);
    }
  }

  function selectedMap() {
    return state.maps.find((entry) => entry.name === dom.select.value) || null;
  }

  function activateSurface(editorActive) {
    dom.mappingWorkspace.hidden = editorActive;
    dom.editorWorkspace.hidden = !editorActive;
    dom.rogmapWorkspace.hidden = true;
    dom.trajectoryWorkspace.hidden = true;
    dom.mappingTab.classList.toggle("is-active", !editorActive);
    dom.editorTab.classList.toggle("is-active", editorActive);
    dom.rogmapTab.classList.remove("is-active");
    dom.trajectoryTab.classList.remove("is-active");
    dom.mappingTab.setAttribute("aria-selected", String(!editorActive));
    dom.editorTab.setAttribute("aria-selected", String(editorActive));
    dom.rogmapTab.setAttribute("aria-selected", "false");
    dom.trajectoryTab.setAttribute("aria-selected", "false");
    if (editorActive) {
      resizeCanvas();
      scheduleRender();
      if (!state.mapsLoaded) void refreshMaps();
    } else {
      window.dispatchEvent(new Event("resize"));
    }
  }

  function updateSourceControls() {
    const entry = selectedMap();
    dom.loadOccupancy.disabled = state.busy || !entry?.has_occupancy;
    dom.openTerrain.disabled = state.busy || !entry?.has_terrain;
    dom.convert.disabled = state.busy || !entry?.has_occupancy;
    const canSetFrame = Boolean(entry?.has_occupancy && entry?.has_pcd && state.values && state.mapName === entry.name);
    dom.framePick.disabled = state.busy || !canSetFrame;
    dom.frameApply.disabled = state.busy || !canSetFrame;
    dom.frameX.disabled = state.busy || !canSetFrame;
    dom.frameY.disabled = state.busy || !canSetFrame;
    dom.frameYaw.disabled = state.busy || !canSetFrame;
    dom.frameRevision.textContent = entry?.has_frame_metadata ? `rev ${entry.frame_revision || 0}` : "未定义";
    if (!entry?.has_occupancy) {
      dom.frameNote.textContent = "需要先加载 PGM/YAML。";
    } else if (!entry?.has_pcd) {
      dom.frameNote.textContent = "缺少 data/pcd 下的同名 PCD，不能保证成组变换。";
    } else if (entry.has_frame_metadata) {
      const transform = entry.source_to_map || {};
      const yaw = Number(transform.yaw || 0) * 180 / Math.PI;
      dom.frameNote.textContent = `${entry.source_frame || "odom"} → map：X ${Number(transform.x || 0).toFixed(3)} · Y ${Number(transform.y || 0).toFixed(3)} · ${yaw.toFixed(1)}°`;
    } else {
      dom.frameNote.textContent = "当前按 odom 与 map 数值重合处理；可在图上重新定义。";
    }
    if (!entry) {
      dom.sourceNote.textContent = state.maps.length ? "请选择地图文件。" : "data/map 中还没有可编辑的地图。";
    } else if (entry.error) {
      dom.sourceNote.textContent = `地图文件异常：${entry.error}`;
    } else {
      const size = entry.width && entry.height ? `${entry.width}×${entry.height}` : "尺寸待读取";
      const resolution = entry.resolution ? ` · ${Number(entry.resolution).toFixed(3)} m/px` : "";
      dom.sourceNote.textContent = `${size}${resolution} · ${entry.has_terrain ? "terrain 已生成" : "尚未生成 terrain"}`;
    }
    updateCloudUi();
  }

  function updateCloudUi() {
    const loadedEntry = state.maps.find((entry) => entry.name === state.mapName);
    const available = Boolean(state.values && loadedEntry?.has_pcd);
    dom.cloudToggle.disabled = state.busy || state.cloudLoading || !available;
    dom.cloudToggle.classList.toggle("is-active", state.cloudVisible);
    dom.cloudToggle.setAttribute("aria-pressed", String(state.cloudVisible));
    dom.cloudToggle.textContent = state.cloudLoading ? "点云…" : state.cloudVisible ? "隐藏点云 P" : "点云 P";
    dom.cloudStatus.hidden = !state.cloudVisible;
    if (state.cloudVisible) {
      const count = state.cloudShown.toLocaleString("zh-CN");
      const total = state.cloudTotal.toLocaleString("zh-CN");
      const outside = Math.max(0, state.cloudShown - state.cloudInBounds);
      const bounds = outside ? ` · 图内 ${state.cloudInBounds.toLocaleString("zh-CN")}` : "";
      dom.cloudStatus.textContent = `俯视点云 ${count}/${total}${bounds} · ${state.cloudSliceLabel} · P 隐藏`;
    }
  }

  function clearCloudOverlay() {
    state.cloudRequestSerial += 1;
    state.cloudPoints = null; state.cloudMapName = ""; state.cloudVisible = false;
    state.cloudShown = 0; state.cloudTotal = 0; state.cloudInBounds = 0; state.cloudSliceLabel = "";
    state.cloudCanvas.width = 1; state.cloudCanvas.height = 1;
    updateCloudUi(); scheduleRender();
  }

  function decodeCloudFrame(buffer) {
    if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 8) throw new Error("点云数据头不完整");
    const view = new DataView(buffer);
    const magic = String.fromCharCode(view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
    if (magic !== "MAP1") throw new Error("点云数据格式不兼容");
    const count = view.getUint32(4, true);
    if (buffer.byteLength !== 8 + count * 12) throw new Error("点云数据长度不匹配");
    return { count, values: new Float32Array(buffer, 8, count * 3) };
  }

  function rebuildCloudOverlay() {
    const points = state.cloudPoints;
    if (!points || !state.values || state.cloudMapName !== state.mapName) return;
    state.cloudCanvas.width = state.width; state.cloudCanvas.height = state.height;
    const cloudContext = state.cloudCanvas.getContext("2d", { alpha: true });
    if (!cloudContext) return;
    const image = cloudContext.createImageData(state.width, state.height);
    const cosine = Math.cos(state.originYaw), sine = Math.sin(state.originYaw);
    let inBounds = 0;
    for (let index = 0; index < points.length; index += 3) {
      const worldX = points[index], worldY = points[index + 1];
      if (!Number.isFinite(worldX) || !Number.isFinite(worldY)) continue;
      const dx = worldX - state.originX, dy = worldY - state.originY;
      const localX = cosine * dx + sine * dy;
      const localY = -sine * dx + cosine * dy;
      const x = Math.floor(localX / state.resolution), y = Math.floor(localY / state.resolution);
      if (x < 0 || y < 0 || x >= state.width || y >= state.height) continue;
      const target = ((state.height - 1 - y) * state.width + x) * 4;
      inBounds += 1;
      image.data[target] = 18; image.data[target + 1] = 244; image.data[target + 2] = 218;
      image.data[target + 3] = Math.min(255, image.data[target + 3] + 112);
    }
    cloudContext.putImageData(image, 0, 0);
    state.cloudInBounds = inBounds;
  }

  async function cloudSliceQuery(mapName) {
    const query = new URLSearchParams({ name: mapName });
    let label = "完整 PCD 的 XY 投影";
    try {
      const response = await fetch(`/api/status?t=${Date.now()}`, { cache: "no-store" });
      if (!response.ok) return { query, label };
      const payload = await response.json();
      const config = payload?.map_export || {};
      const zMin = Number(config.z_min), zMax = Number(config.z_max);
      if (Number.isFinite(zMin) && Number.isFinite(zMax) && zMin < zMax) {
        query.set("z_min", String(zMin)); query.set("z_max", String(zMax));
        if (["ground", "absolute"].includes(config.height_mode)) query.set("height_mode", config.height_mode);
        label = config.height_mode === "ground"
          ? `相对地面 ${zMin}–${zMax} m 障碍切片`
          : `全局 Z ${zMin}–${zMax} m 障碍切片`;
      }
    } catch (_) {
      // The overlay can still fall back to the complete PCD if status polling fails.
    }
    return { query, label };
  }

  async function showCloudOverlay() {
    const loadedEntry = state.maps.find((entry) => entry.name === state.mapName);
    if (!state.values || !loadedEntry?.has_pcd || state.cloudLoading) return;
    if (state.cloudPoints && state.cloudMapName === state.mapName) {
      state.cloudVisible = true; updateCloudUi(); scheduleRender(); return;
    }
    const mapName = state.mapName;
    const requestSerial = ++state.cloudRequestSerial;
    state.cloudLoading = true; updateCloudUi();
    try {
      const { query, label } = await cloudSliceQuery(mapName);
      const response = await fetch(`/api/pcd/preview?${query}`, { cache: "no-store" });
      if (!response.ok) throw await responseError(response);
      const decoded = decodeCloudFrame(await response.arrayBuffer());
      if (requestSerial !== state.cloudRequestSerial || state.mapName !== mapName) return;
      state.cloudPoints = decoded.values; state.cloudMapName = mapName;
      state.cloudShown = Number(response.headers?.get?.("X-Preview-Points")) || decoded.count;
      state.cloudTotal = Number(response.headers?.get?.("X-Preview-Total")) || state.cloudShown;
      state.cloudSliceLabel = label; state.cloudVisible = true;
      rebuildCloudOverlay(); updateCloudUi(); scheduleRender();
    } catch (error) {
      if (requestSerial === state.cloudRequestSerial) {
        clearCloudOverlay();
        toast(`点云叠加失败：${error.message}`, "error");
      }
    } finally {
      state.cloudLoading = false; updateCloudUi();
    }
  }

  function toggleCloudOverlay() {
    if (state.cloudVisible) {
      state.cloudVisible = false; updateCloudUi(); scheduleRender();
    } else {
      void showCloudOverlay();
    }
  }

  async function refreshMaps() {
    state.busy = true;
    updateSourceControls();
    try {
      const response = await fetch(`/api/maps?t=${Date.now()}`, { cache: "no-store" });
      if (!response.ok) throw await responseError(response);
      const payload = await response.json();
      state.maps = Array.isArray(payload.maps) ? payload.maps : [];
      state.mapsLoaded = true;
      const previous = dom.select.value || state.mapName;
      dom.select.replaceChildren();
      if (!state.maps.length) {
        const option = document.createElement("option");
        option.value = "";
        option.textContent = "没有可编辑的地图";
        dom.select.appendChild(option);
      } else {
        for (const entry of state.maps) {
          const option = document.createElement("option");
          option.value = entry.name;
          option.textContent = `${entry.name}${entry.has_terrain ? " · terrain" : ""}`;
          dom.select.appendChild(option);
        }
        dom.select.value = state.maps.some((entry) => entry.name === previous) ? previous : state.maps[0].name;
      }
    } catch (error) {
      dom.select.replaceChildren(new Option("地图列表读取失败", ""));
      dom.sourceNote.textContent = error.message;
      toast(error.message, "error");
    } finally {
      state.busy = false;
      updateSourceControls();
    }
  }

  function hasUnsavedChanges() {
    return state.dirty && !window.confirm("当前图层有未保存修改，继续会丢失这些修改。是否继续？");
  }

  function decodeEditorPayload(buffer) {
    if (buffer.byteLength < HEADER_SIZE) throw new Error("地图数据头不完整");
    const view = new DataView(buffer);
    const magic = String.fromCharCode(view.getUint8(0), view.getUint8(1), view.getUint8(2), view.getUint8(3));
    if (magic !== MAGIC) throw new Error("地图数据格式不兼容");
    const layer = view.getUint8(4);
    const width = view.getUint32(8, true), height = view.getUint32(12, true);
    const count = width * height;
    if (!Number.isSafeInteger(count) || count <= 0) throw new Error("地图尺寸无效");
    const channels = layer === LAYER_OCCUPANCY ? 1 : layer === LAYER_TERRAIN ? 2 : 0;
    if (!channels || buffer.byteLength !== HEADER_SIZE + count * channels) throw new Error("地图通道或数据长度无效");
    return {
      layer, width, height,
      resolution: view.getFloat64(16, true), originX: view.getFloat64(24, true), originY: view.getFloat64(32, true), originYaw: view.getFloat64(40, true),
      values: new Uint8Array(buffer.slice(HEADER_SIZE, HEADER_SIZE + count)),
      direction: layer === LAYER_TERRAIN ? new Uint8Array(buffer.slice(HEADER_SIZE + count)) : null
    };
  }

  function encodeEditorPayload() {
    const count = state.width * state.height;
    const channels = state.layer === LAYER_TERRAIN ? 2 : 1;
    const buffer = new ArrayBuffer(HEADER_SIZE + count * channels);
    const view = new DataView(buffer);
    for (let index = 0; index < MAGIC.length; index += 1) view.setUint8(index, MAGIC.charCodeAt(index));
    view.setUint8(4, state.layer);
    view.setUint32(8, state.width, true); view.setUint32(12, state.height, true);
    view.setFloat64(16, state.resolution, true); view.setFloat64(24, state.originX, true); view.setFloat64(32, state.originY, true); view.setFloat64(40, state.originYaw, true);
    new Uint8Array(buffer, HEADER_SIZE, count).set(state.values);
    if (state.direction) new Uint8Array(buffer, HEADER_SIZE + count, count).set(state.direction);
    return buffer;
  }

  async function loadLayer(layer, { skipDirtyCheck = false } = {}) {
    const entry = selectedMap();
    if (!entry || (!skipDirtyCheck && hasUnsavedChanges())) return;
    const mapChanged = state.mapName !== entry.name;
    const endpoint = layer === LAYER_OCCUPANCY ? "occupancy" : "terrain";
    state.busy = true; updateSourceControls(); dom.saveStatus.textContent = "正在读取地图…";
    try {
      const query = new URLSearchParams({ map_name: entry.name });
      const response = await fetch(`/api/editor/${endpoint}?${query}`, { cache: "no-store" });
      if (!response.ok) throw await responseError(response);
      const decoded = decodeEditorPayload(await response.arrayBuffer());
      if (mapChanged) clearCloudOverlay();
      state.mapName = entry.name; state.layer = decoded.layer; state.width = decoded.width; state.height = decoded.height;
      state.resolution = decoded.resolution; state.originX = decoded.originX; state.originY = decoded.originY; state.originYaw = decoded.originYaw;
      state.values = decoded.values; state.direction = decoded.direction; state.dirty = false;
      state.history = []; state.historyBytes = 0; state.hover = null; state.drag = null; state.preview = null;
      state.frameDraft = null; dom.frameX.value = "0"; dom.frameY.value = "0"; dom.frameYaw.value = "0";
      state.label = decoded.layer === LAYER_OCCUPANCY ? 0 : 1;
      state.mapImageDirty = true;
      if (!mapChanged && state.cloudPoints && state.cloudMapName === entry.name) rebuildCloudOverlay();
      buildPalette(); updateEditorUi(); fitMap();
      dom.saveStatus.textContent = decoded.layer === LAYER_OCCUPANCY ? "二维 PGM 已加载" : "terrain msgpack 已加载";
    } catch (error) {
      dom.saveStatus.textContent = error.message;
      toast(error.message, "error");
    } finally {
      state.busy = false; updateSourceControls(); updateEditorUi();
    }
  }

  async function saveCurrent({ quiet = false } = {}) {
    if (!state.values || !state.mapName || state.busy) return false;
    state.busy = true; updateSourceControls(); updateEditorUi(); dom.saveStatus.textContent = "正在原子写入地图文件…";
    try {
      const query = new URLSearchParams({ map_name: state.mapName });
      const response = await fetch(`/api/editor/save?${query}`, {
        method: "POST", headers: { "Content-Type": "application/x-mapping-editor" }, body: encodeEditorPayload()
      });
      if (!response.ok) throw await responseError(response);
      const result = await response.json();
      state.dirty = false;
      dom.saveStatus.textContent = result.message;
      if (!quiet) toast(result.message, "success");
      return true;
    } catch (error) {
      dom.saveStatus.textContent = error.message;
      toast(error.message, "error");
      return false;
    } finally {
      state.busy = false; updateSourceControls(); updateEditorUi();
    }
  }

  async function convertToTerrain() {
    const entry = selectedMap();
    if (!entry || !entry.has_occupancy || state.busy) return;
    if (state.dirty) {
      if (state.mapName === entry.name && state.layer === LAYER_OCCUPANCY) {
        if (!(await saveCurrent({ quiet: true }))) return;
      } else if (hasUnsavedChanges()) {
        return;
      }
    }
    let overwrite = false;
    if (entry.has_terrain) {
      overwrite = window.confirm("重新生成会覆盖已有 terrain 语义和方向标注。确认从当前二维 PGM 重建吗？");
      if (!overwrite) return;
    }
    state.busy = true; updateSourceControls(); dom.saveStatus.textContent = "正在把二维占据图转换为 terrain msgpack…";
    try {
      const response = await fetch("/api/editor/convert", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ map_name: entry.name, overwrite })
      });
      if (!response.ok) throw await responseError(response);
      const result = await response.json();
      toast(result.message, "success");
      await refreshMaps();
      dom.select.value = entry.name;
      await loadLayer(LAYER_TERRAIN, { skipDirtyCheck: true });
    } catch (error) {
      dom.saveStatus.textContent = error.message;
      toast(error.message, "error");
    } finally {
      state.busy = false; updateSourceControls(); updateEditorUi();
    }
  }

  function writeFrameDraft(draft) {
    state.frameDraft = draft;
    dom.frameX.value = draft.originX.toFixed(3);
    dom.frameY.value = draft.originY.toFixed(3);
    let degrees = (draft.heading * 180 / Math.PI) % 360;
    if (degrees < 0) degrees += 360;
    if (Number(degrees.toFixed(1)) >= 360) degrees = 0;
    dom.frameYaw.value = degrees.toFixed(1);
    scheduleRender();
  }

  function frameDraft() {
    return state.frameDraft || { originX: 0, originY: 0, heading: 0 };
  }

  // Screen-space grab handles so the frame can be moved and turned after it is placed.
  function frameHandles() {
    const draft = frameDraft();
    const origin = worldToScreen(draft.originX, draft.originY);
    if (!origin) return null;
    const localHeading = draft.heading - state.originYaw;
    return {
      draft, origin,
      tip: {
        x: origin.x + Math.cos(localHeading) * FRAME_AXIS_PX,
        y: origin.y - Math.sin(localHeading) * FRAME_AXIS_PX
      }
    };
  }

  function frameHandleAt(screen) {
    const handles = frameHandles();
    if (!handles) return null;
    const near = (point) => Math.hypot(screen.x - point.x, screen.y - point.y) <= FRAME_HANDLE_PX;
    if (near(handles.origin)) return "origin";
    if (near(handles.tip)) return "tip";
    return null;
  }

  function setFrameDraft(start, end) {
    const origin = mapCellToWorld(start.x, start.y);
    const target = mapCellToWorld(end.x, end.y);
    const heading = Math.atan2(target.y - origin.y, target.x - origin.x);
    writeFrameDraft({ originX: origin.x, originY: origin.y, heading });
  }

  function updateFrameDrag(drag, point) {
    const target = mapCellToWorld(point.x, point.y);
    const draft = drag.draft || frameDraft();
    if (drag.kind === "frame-move") {
      writeFrameDraft({
        originX: target.x + drag.offsetX,
        originY: target.y + drag.offsetY,
        heading: drag.heading
      });
      return;
    }
    if (drag.kind === "frame-rotate") {
      // A click on the arrow head without dragging must not nudge the heading.
      if (point.x === drag.start.x && point.y === drag.start.y) return;
      const dx = target.x - draft.originX, dy = target.y - draft.originY;
      const heading = Math.hypot(dx, dy) < 1e-9 ? draft.heading : Math.atan2(dy, dx);
      writeFrameDraft({ originX: draft.originX, originY: draft.originY, heading });
      return;
    }
    setFrameDraft(drag.start, point);
  }

  // Grabbing the origin marker moves the frame; grabbing the +X tip turns it.
  function beginFrameDrag(screen, point, pointerId) {
    const grab = frameHandleAt(screen);
    if (grab === "origin") {
      const draft = frameDraft();
      const world = mapCellToWorld(point.x, point.y);
      state.drag = {
        kind: "frame-move", pointerId, start: point, last: point, draft,
        heading: draft.heading, offsetX: draft.originX - world.x, offsetY: draft.originY - world.y
      };
      return;
    }
    if (grab === "tip") {
      state.drag = { kind: "frame-rotate", pointerId, start: point, last: point, draft: frameDraft() };
      return;
    }
    state.drag = { kind: "frame", pointerId, start: point, last: point };
    setFrameDraft(point, point);
  }

  function updateFrameCursor(screen) {
    if (state.tool !== "frame") {
      dom.canvas.classList.remove("is-frame-grabbing", "is-frame-rotating");
      return;
    }
    const grab = state.drag ? state.drag.kind : frameHandleAt(screen);
    dom.canvas.classList.toggle("is-frame-grabbing", grab === "frame-move" || grab === "origin");
    dom.canvas.classList.toggle("is-frame-rotating", grab === "frame-rotate" || grab === "tip");
  }

  function updateFrameDraftFromInputs() {
    const originX = Number(dom.frameX.value), originY = Number(dom.frameY.value);
    const heading = Number(dom.frameYaw.value) * Math.PI / 180;
    if ([originX, originY, heading].every(Number.isFinite)) {
      state.frameDraft = { originX, originY, heading };
      scheduleRender();
    }
  }

  async function applyMapFrame() {
    const entry = selectedMap();
    if (!entry || !state.values || state.mapName !== entry.name || state.busy) return;
    const originX = Number(dom.frameX.value), originY = Number(dom.frameY.value);
    const heading = Number(dom.frameYaw.value) * Math.PI / 180;
    if (![originX, originY, heading].every(Number.isFinite)) {
      toast("map 原点和朝向必须是有效数字", "error");
      return;
    }
    if (state.dirty && !(await saveCurrent({ quiet: true }))) return;
    const terrainText = entry.has_terrain ? "、terrain 语义与方向" : "";
    if (!window.confirm(`将以当前坐标 (${originX.toFixed(3)}, ${originY.toFixed(3)}) 为新 map 原点，并把 +X 设为 ${(heading * 180 / Math.PI).toFixed(1)}°。\n\n完整 PCD、PGM/YAML${terrainText}会成组变换；二维栅格旋转会进行最近邻重采样，操作不能在网页中撤销。确认继续吗？`)) return;
    const previousLayer = state.layer;
    const restoreCloudOverlay = state.cloudVisible;
    state.busy = true; updateSourceControls(); updateEditorUi();
    dom.saveStatus.textContent = "正在统一 PCD、二维图与 terrain 的 map 坐标系…";
    try {
      const response = await fetch("/api/editor/frame", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ map_name: entry.name, origin_x: originX, origin_y: originY, heading_yaw: heading })
      });
      if (!response.ok) throw await responseError(response);
      const result = await response.json();
      toast(result.message, "success");
      clearCloudOverlay();
      state.busy = false;
      await refreshMaps();
      dom.select.value = entry.name;
      await loadLayer(previousLayer, { skipDirtyCheck: true });
      if (restoreCloudOverlay) await showCloudOverlay();
      dom.saveStatus.textContent = result.message;
    } catch (error) {
      dom.saveStatus.textContent = error.message;
      toast(error.message, "error");
    } finally {
      state.busy = false; updateSourceControls(); updateEditorUi();
    }
  }

  function currentLabels() { return state.layer === LAYER_TERRAIN ? terrainLabels : occupancyLabels; }
  function labelName(value) { return currentLabels().find((entry) => entry.value === value)?.name || String(value); }

  function buildPalette() {
    dom.palette.replaceChildren();
    for (const label of currentLabels()) {
      const button = document.createElement("button");
      button.type = "button"; button.className = `label-chip${state.label === label.value ? " is-active" : ""}`;
      button.style.setProperty("--label-color", label.color); button.dataset.value = String(label.value);
      button.setAttribute("role", "radio"); button.setAttribute("aria-checked", String(state.label === label.value));
      button.innerHTML = `<i aria-hidden="true"></i><span>${label.value <= 6 ? `${label.value} · ` : ""}${label.name}</span>`;
      button.addEventListener("click", () => selectLabel(label.value));
      dom.palette.appendChild(button);
    }
    updateDirectionUi();
  }

  function selectLabel(value) {
    if (!currentLabels().some((entry) => entry.value === value)) return;
    state.label = value;
    for (const button of dom.palette.querySelectorAll("button")) {
      const active = Number(button.dataset.value) === value;
      button.classList.toggle("is-active", active); button.setAttribute("aria-checked", String(active));
    }
    updateDirectionUi(); scheduleRender();
  }

  function updateDirectionUi() {
    const directional = state.layer === LAYER_TERRAIN && state.label >= 2;
    dom.direction.disabled = !directional;
    dom.directionCard.classList.toggle("is-disabled", !directional);
    const degrees = state.directionValue / 255 * 360;
    dom.directionValue.textContent = `${degrees.toFixed(0)}° · ${state.directionValue}`;
    dom.directionArrow.style.transform = `rotate(${-degrees}deg)`;
  }

  function updateEditorUi() {
    const loaded = Boolean(state.values);
    dom.empty.hidden = loaded;
    dom.title.textContent = loaded ? state.mapName : "二维地图与 terrain 标注";
    dom.layerBadge.textContent = !loaded ? "未加载" : state.layer === LAYER_OCCUPANCY ? "二维 PGM" : "terrain MSG";
    dom.layerBadge.className = `layer-badge ${!loaded ? "" : state.layer === LAYER_OCCUPANCY ? "occupancy" : "terrain"}`;
    dom.save.disabled = !loaded || state.busy || !state.dirty;
    dom.undo.disabled = !loaded || state.busy || !state.history.length;
    dom.historyLabel.textContent = `${state.history.length} 步`;
    dom.dirty.textContent = state.dirty ? "有未保存修改" : "未修改";
    dom.dirty.classList.toggle("is-dirty", state.dirty);
    dom.zoomLabel.textContent = `${Math.round(state.zoom * 100)}%`;
    dom.canvas.classList.toggle("is-panning", state.tool === "pan");
    dom.canvas.classList.toggle("is-frame-picking", state.tool === "frame");
    updateCloudUi();
    updateDirectionUi();
  }

  function snapshotForUndo() {
    if (!state.values) return;
    const snapshot = {
      values: state.values.slice(), direction: state.direction ? state.direction.slice() : null,
      bytes: state.values.byteLength + (state.direction?.byteLength || 0)
    };
    state.history.push(snapshot); state.historyBytes += snapshot.bytes;
    while (state.history.length > 1 && (state.history.length > MAX_HISTORY_STEPS || state.historyBytes > MAX_HISTORY_BYTES)) {
      const removed = state.history.shift(); state.historyBytes -= removed.bytes;
    }
    updateEditorUi();
  }

  function discardLatestSnapshot() {
    const snapshot = state.history.pop();
    if (snapshot) state.historyBytes -= snapshot.bytes;
    updateEditorUi();
  }

  function undo() {
    const snapshot = state.history.pop();
    if (!snapshot) return;
    state.historyBytes -= snapshot.bytes; state.values = snapshot.values; state.direction = snapshot.direction;
    state.dirty = true; state.mapImageDirty = true; state.preview = null;
    updateEditorUi(); scheduleRender();
  }

  function markChanged() {
    state.dirty = true; state.mapImageDirty = true;
    if (state.drag) state.drag.changed = true;
    updateEditorUi(); scheduleRender();
  }

  function resizeCanvas() {
    const rect = dom.canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.round(rect.width * ratio));
    const height = Math.max(1, Math.round(rect.height * ratio));
    if (dom.canvas.width !== width || dom.canvas.height !== height) {
      dom.canvas.width = width; dom.canvas.height = height;
      scheduleRender();
    }
  }

  function rebuildMapImage() {
    if (!state.values || !state.mapImageDirty) return;
    state.mapCanvas.width = state.width; state.mapCanvas.height = state.height;
    const image = mapCtx.createImageData(state.width, state.height);
    for (let y = 0; y < state.height; y += 1) {
      const displayY = state.height - 1 - y;
      for (let x = 0; x < state.width; x += 1) {
        const source = y * state.width + x;
        const target = (displayY * state.width + x) * 4;
        if (state.layer === LAYER_OCCUPANCY) {
          const gray = state.values[source];
          image.data[target] = gray; image.data[target + 1] = gray; image.data[target + 2] = gray;
        } else {
          const color = terrainRgb[state.values[source]] || [200, 200, 200];
          image.data[target] = color[0]; image.data[target + 1] = color[1]; image.data[target + 2] = color[2];
        }
        image.data[target + 3] = 255;
      }
    }
    mapCtx.putImageData(image, 0, 0);
    state.mapImageDirty = false;
  }

  function scheduleRender() {
    if (state.renderPending) return;
    state.renderPending = true;
    requestAnimationFrame(() => { state.renderPending = false; render(); });
  }

  function render() {
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = dom.canvas.width / ratio, height = dom.canvas.height / ratio;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.fillStyle = "#030708"; ctx.fillRect(0, 0, width, height);
    if (!state.values) return;
    rebuildMapImage();
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(state.mapCanvas, state.panX, state.panY, state.width * state.zoom, state.height * state.zoom);
    if (state.cloudVisible && state.cloudPoints && state.cloudMapName === state.mapName) {
      ctx.save();
      ctx.globalAlpha = 0.82;
      ctx.drawImage(state.cloudCanvas, state.panX, state.panY, state.width * state.zoom, state.height * state.zoom);
      ctx.restore();
    }
    drawDirections(ctx);
    drawPreview(ctx);
    drawCoordinateFrame(ctx);
  }

  function drawDirections(context) {
    if (state.layer !== LAYER_TERRAIN || !state.direction || !dom.showArrows.checked) return;
    const cellStep = Math.max(1, Math.ceil(26 / state.zoom));
    const arrowLength = Math.max(8, Math.min(22, cellStep * state.zoom * 0.72));
    let drawn = 0;
    context.lineWidth = 1.4; context.strokeStyle = "rgba(238,255,253,.86)"; context.fillStyle = "rgba(238,255,253,.86)";
    for (let y = 0; y < state.height && drawn < 5000; y += cellStep) {
      for (let x = 0; x < state.width && drawn < 5000; x += cellStep) {
        const index = y * state.width + x;
        if (state.values[index] < 2) continue;
        const angle = state.direction[index] / 255 * Math.PI * 2 - state.originYaw;
        const sx = state.panX + (x + 0.5) * state.zoom;
        const sy = state.panY + (state.height - y - 0.5) * state.zoom;
        const ex = sx + Math.cos(angle) * arrowLength, ey = sy - Math.sin(angle) * arrowLength;
        context.beginPath(); context.moveTo(sx, sy); context.lineTo(ex, ey); context.stroke();
        const head = Math.max(3, arrowLength * 0.28);
        context.beginPath(); context.moveTo(ex, ey);
        context.lineTo(ex - Math.cos(angle - 0.55) * head, ey + Math.sin(angle - 0.55) * head);
        context.lineTo(ex - Math.cos(angle + 0.55) * head, ey + Math.sin(angle + 0.55) * head);
        context.closePath(); context.fill(); drawn += 1;
      }
    }
  }

  function drawPreview(context) {
    const color = currentLabels().find((entry) => entry.value === state.label)?.color || "#49e8d4";
    context.strokeStyle = color; context.lineWidth = 2; context.setLineDash([6, 4]);
    if (state.preview) {
      const a = mapToScreen(state.preview.start.x, state.preview.start.y);
      const b = mapToScreen(state.preview.end.x, state.preview.end.y);
      if (state.tool === "rect") {
        const left = Math.min(a.left, b.left), top = Math.min(a.top, b.top);
        context.strokeRect(left, top, Math.abs(a.left - b.left) + state.zoom, Math.abs(a.top - b.top) + state.zoom);
      } else if (state.tool === "line") {
        context.lineWidth = Math.max(2, state.brushSize * state.zoom); context.lineCap = "square";
        context.beginPath(); context.moveTo(a.cx, a.cy); context.lineTo(b.cx, b.cy); context.stroke(); context.lineCap = "butt";
      }
    } else if (state.hover && state.tool === "brush") {
      const offset = -Math.floor(state.brushSize / 2);
      const x = state.panX + (state.hover.x + offset) * state.zoom;
      const y = state.panY + (state.height - state.hover.y - offset - state.brushSize) * state.zoom;
      context.strokeRect(x, y, state.brushSize * state.zoom, state.brushSize * state.zoom);
    }
    context.setLineDash([]);
  }

  function drawCoordinateFrame(context) {
    const definition = frameDraft();
    const start = worldToScreen(definition.originX, definition.originY);
    if (!start) return;
    const length = FRAME_AXIS_PX;
    const drawAxis = (heading, color, label) => {
      const localHeading = heading - state.originYaw;
      const endX = start.x + Math.cos(localHeading) * length;
      const endY = start.y - Math.sin(localHeading) * length;
      context.strokeStyle = color; context.fillStyle = color; context.lineWidth = 2.2;
      context.beginPath(); context.moveTo(start.x, start.y); context.lineTo(endX, endY); context.stroke();
      const head = 8;
      context.beginPath(); context.moveTo(endX, endY);
      context.lineTo(endX - Math.cos(localHeading - 0.55) * head, endY + Math.sin(localHeading - 0.55) * head);
      context.lineTo(endX - Math.cos(localHeading + 0.55) * head, endY + Math.sin(localHeading + 0.55) * head);
      context.closePath(); context.fill();
      context.font = "700 12px ui-monospace, monospace"; context.fillText(label, endX + 5, endY - 5);
    };
    context.save();
    context.shadowBlur = 6; context.shadowColor = "rgba(0,0,0,.8)";
    drawAxis(definition.heading, "#ff6b62", "+X");
    drawAxis(definition.heading + Math.PI / 2, "#53d99f", "+Y");
    // Grab handles: the origin moves the frame, the +X tip turns it.
    const localHeading = definition.heading - state.originYaw;
    const tipX = start.x + Math.cos(localHeading) * length;
    const tipY = start.y - Math.sin(localHeading) * length;
    const moving = state.drag?.kind === "frame-move";
    const turning = state.drag?.kind === "frame-rotate";
    context.fillStyle = moving ? "#f0b85c" : "#eefefd";
    context.beginPath(); context.arc(start.x, start.y, 5, 0, Math.PI * 2); context.fill();
    context.strokeStyle = moving ? "#f0b85c" : "rgba(238,254,253,.5)"; context.lineWidth = 1.6;
    context.beginPath(); context.arc(start.x, start.y, FRAME_HANDLE_PX - 4, 0, Math.PI * 2); context.stroke();
    context.fillStyle = turning ? "#f0b85c" : "#ff6b62";
    context.beginPath(); context.arc(tipX, tipY, 4.5, 0, Math.PI * 2); context.fill();
    context.restore();
  }

  function fitMap() {
    if (!state.values) return;
    const rect = dom.canvas.getBoundingClientRect();
    state.zoom = Math.max(0.05, Math.min(50, Math.min((rect.width - 32) / state.width, (rect.height - 32) / state.height)));
    state.panX = (rect.width - state.width * state.zoom) / 2;
    state.panY = (rect.height - state.height * state.zoom) / 2;
    updateEditorUi(); scheduleRender();
  }

  function zoomAt(factor, clientX, clientY) {
    if (!state.values) return;
    const rect = dom.canvas.getBoundingClientRect();
    const x = clientX == null ? rect.width / 2 : clientX - rect.left;
    const y = clientY == null ? rect.height / 2 : clientY - rect.top;
    const old = state.zoom, next = Math.max(0.05, Math.min(50, old * factor));
    state.panX = x - (x - state.panX) * (next / old);
    state.panY = y - (y - state.panY) * (next / old);
    state.zoom = next; updateEditorUi(); scheduleRender();
  }

  function eventPoint(event) {
    const rect = dom.canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function screenToMap(point, clamp = false) {
    if (!state.values) return null;
    let x = Math.floor((point.x - state.panX) / state.zoom);
    const displayY = Math.floor((point.y - state.panY) / state.zoom);
    let y = state.height - 1 - displayY;
    if (clamp) {
      x = Math.min(state.width - 1, Math.max(0, x)); y = Math.min(state.height - 1, Math.max(0, y));
    }
    return x >= 0 && y >= 0 && x < state.width && y < state.height ? { x, y } : null;
  }

  function mapToScreen(x, y) {
    const left = state.panX + x * state.zoom, top = state.panY + (state.height - 1 - y) * state.zoom;
    return { left, top, cx: left + state.zoom / 2, cy: top + state.zoom / 2 };
  }

  function mapCellToWorld(x, y) {
    const localX = (x + 0.5) * state.resolution;
    const localY = (y + 0.5) * state.resolution;
    const cosine = Math.cos(state.originYaw), sine = Math.sin(state.originYaw);
    return {
      x: state.originX + cosine * localX - sine * localY,
      y: state.originY + sine * localX + cosine * localY
    };
  }

  function worldToScreen(worldX, worldY) {
    if (!state.values) return null;
    const dx = worldX - state.originX, dy = worldY - state.originY;
    const cosine = Math.cos(state.originYaw), sine = Math.sin(state.originYaw);
    const localX = cosine * dx + sine * dy;
    const localY = -sine * dx + cosine * dy;
    return {
      x: state.panX + localX / state.resolution * state.zoom,
      y: state.panY + (state.height - localY / state.resolution) * state.zoom
    };
  }

  function updateHover(point) {
    state.hover = screenToMap(point);
    if (!state.hover) {
      dom.coordinate.textContent = "栅格 —"; dom.worldCoordinate.textContent = "世界坐标 —"; dom.cellValue.textContent = "标签 —";
    } else {
      const { x, y } = state.hover, index = y * state.width + x;
      const world = mapCellToWorld(x, y), worldX = world.x, worldY = world.y;
      dom.coordinate.textContent = `栅格 X ${x} · Y ${y}`;
      dom.worldCoordinate.textContent = `世界 ${worldX.toFixed(2)}, ${worldY.toFixed(2)} m`;
      const direction = state.layer === LAYER_TERRAIN && state.values[index] >= 2 ? ` · ${(state.direction[index] / 255 * 360).toFixed(0)}°` : "";
      dom.cellValue.textContent = `${labelName(state.values[index])}${direction}`;
    }
    scheduleRender();
  }

  function rasterLine(x0, y0, x1, y1, callback) {
    let dx = Math.abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    let dy = -Math.abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    let error = dx + dy;
    while (true) {
      callback(x0, y0);
      if (x0 === x1 && y0 === y1) break;
      const doubled = error * 2;
      if (doubled >= dy) { error += dy; x0 += sx; }
      if (doubled <= dx) { error += dx; y0 += sy; }
    }
  }

  function paintCell(x, y, directionValue = state.directionValue) {
    if (x < 0 || y < 0 || x >= state.width || y >= state.height) return false;
    const index = y * state.width + x;
    const nextDirection = state.layer === LAYER_TERRAIN && state.label >= 2 ? directionValue : 0;
    if (state.values[index] === state.label && (!state.direction || state.direction[index] === nextDirection)) return false;
    state.values[index] = state.label;
    if (state.direction) state.direction[index] = nextDirection;
    return true;
  }

  function paintBrush(x, y, directionValue = state.directionValue) {
    const offset = -Math.floor(state.brushSize / 2);
    let changed = false;
    for (let dy = offset; dy < offset + state.brushSize; dy += 1) {
      for (let dx = offset; dx < offset + state.brushSize; dx += 1) changed = paintCell(x + dx, y + dy, directionValue) || changed;
    }
    return changed;
  }

  function paintStroke(from, to) {
    let changed = false;
    rasterLine(from.x, from.y, to.x, to.y, (x, y) => { changed = paintBrush(x, y) || changed; });
    if (changed) markChanged();
  }

  function applyRectangle(from, to) {
    let changed = false;
    const minX = Math.min(from.x, to.x), maxX = Math.max(from.x, to.x);
    const minY = Math.min(from.y, to.y), maxY = Math.max(from.y, to.y);
    for (let y = minY; y <= maxY; y += 1) for (let x = minX; x <= maxX; x += 1) changed = paintCell(x, y) || changed;
    if (changed) markChanged();
  }

  function directionFromLine(from, to) {
    const dx = to.x - from.x, dy = to.y - from.y, length = Math.hypot(dx, dy);
    if (length < 1e-6) return 0;
    let angle = Math.atan2(dx / length, -dy / length) + state.originYaw;
    angle %= Math.PI * 2;
    if (angle < 0) angle += Math.PI * 2;
    return Math.max(0, Math.min(255, Math.round(angle / (Math.PI * 2) * 255)));
  }

  function applyLine(from, to) {
    const directionValue = directionFromLine(from, to);
    let changed = false;
    rasterLine(from.x, from.y, to.x, to.y, (x, y) => { changed = paintBrush(x, y, directionValue) || changed; });
    if (changed) markChanged();
  }

  function pointerDown(event) {
    if (!state.values) return;
    const screen = eventPoint(event);
    if (state.tool === "pan" || event.button === 1 || event.button === 2) {
      state.drag = { kind: "pan", screen, panX: state.panX, panY: state.panY, pointerId: event.pointerId };
      dom.canvas.setPointerCapture(event.pointerId); event.preventDefault(); return;
    }
    if (event.button !== 0) return;
    const point = screenToMap(screen);
    if (!point) return;
    if (state.tool === "frame") {
      beginFrameDrag(screen, point, event.pointerId);
      dom.canvas.setPointerCapture(event.pointerId); event.preventDefault(); return;
    }
    snapshotForUndo();
    state.drag = { kind: state.tool, start: point, last: point, changed: false, pointerId: event.pointerId };
    dom.canvas.setPointerCapture(event.pointerId);
    if (state.tool === "brush" && paintBrush(point.x, point.y)) markChanged();
    else state.preview = { start: point, end: point };
    event.preventDefault();
  }

  function pointerMove(event) {
    const screen = eventPoint(event);
    updateHover(screen);
    updateFrameCursor(screen);
    if (!state.drag || state.drag.pointerId !== event.pointerId) return;
    if (state.drag.kind === "pan") {
      state.panX = state.drag.panX + screen.x - state.drag.screen.x;
      state.panY = state.drag.panY + screen.y - state.drag.screen.y;
      scheduleRender(); return;
    }
    const point = screenToMap(screen, true);
    if (!point) return;
    if (state.drag.kind.startsWith("frame")) updateFrameDrag(state.drag, point);
    else if (state.drag.kind === "brush") paintStroke(state.drag.last, point);
    else state.preview = { start: state.drag.start, end: point };
    state.drag.last = point; scheduleRender();
  }

  function pointerUp(event) {
    if (!state.drag || state.drag.pointerId !== event.pointerId) return;
    const drag = state.drag;
    if (drag.kind.startsWith("frame")) updateFrameDrag(drag, drag.last);
    else if (drag.kind === "rect") applyRectangle(drag.start, drag.last);
    else if (drag.kind === "line") applyLine(drag.start, drag.last);
    state.preview = null; state.drag = null;
    if (drag.kind !== "pan" && !drag.kind.startsWith("frame") && !drag.changed) discardLatestSnapshot();
    if (dom.canvas.hasPointerCapture(event.pointerId)) dom.canvas.releasePointerCapture(event.pointerId);
    scheduleRender();
  }

  function setTool(tool) {
    state.tool = tool;
    for (const button of document.querySelectorAll("[data-editor-tool]")) button.classList.toggle("is-active", button.dataset.editorTool === tool);
    dom.canvas.classList.remove("is-frame-grabbing", "is-frame-rotating");
    updateEditorUi(); scheduleRender();
  }

  dom.mappingTab.addEventListener("click", () => activateSurface(false));
  dom.editorTab.addEventListener("click", () => activateSurface(true));
  dom.rogmapTab.addEventListener("click", () => {
    dom.mappingWorkspace.hidden = true;
    dom.editorWorkspace.hidden = true;
    dom.rogmapWorkspace.hidden = false;
    dom.trajectoryWorkspace.hidden = true;
    dom.mappingTab.classList.remove("is-active");
    dom.editorTab.classList.remove("is-active");
    dom.rogmapTab.classList.add("is-active");
    dom.trajectoryTab.classList.remove("is-active");
    dom.mappingTab.setAttribute("aria-selected", "false");
    dom.editorTab.setAttribute("aria-selected", "false");
    dom.rogmapTab.setAttribute("aria-selected", "true");
    dom.trajectoryTab.setAttribute("aria-selected", "false");
    window.dispatchEvent(new Event("rogmap-workspace-activated"));
  });
  dom.trajectoryTab.addEventListener("click", () => {
    dom.mappingWorkspace.hidden = true;
    dom.editorWorkspace.hidden = true;
    dom.rogmapWorkspace.hidden = true;
    dom.trajectoryWorkspace.hidden = false;
    dom.mappingTab.classList.remove("is-active");
    dom.editorTab.classList.remove("is-active");
    dom.rogmapTab.classList.remove("is-active");
    dom.trajectoryTab.classList.add("is-active");
    dom.mappingTab.setAttribute("aria-selected", "false");
    dom.editorTab.setAttribute("aria-selected", "false");
    dom.rogmapTab.setAttribute("aria-selected", "false");
    dom.trajectoryTab.setAttribute("aria-selected", "true");
    window.dispatchEvent(new Event("trajectory-workspace-activated"));
  });
  dom.refresh.addEventListener("click", refreshMaps);
  dom.select.addEventListener("change", updateSourceControls);
  dom.loadOccupancy.addEventListener("click", () => loadLayer(LAYER_OCCUPANCY));
  dom.openTerrain.addEventListener("click", () => loadLayer(LAYER_TERRAIN));
  dom.convert.addEventListener("click", convertToTerrain);
  dom.frameApply.addEventListener("click", applyMapFrame);
  dom.cloudToggle.addEventListener("click", toggleCloudOverlay);
  for (const input of [dom.frameX, dom.frameY, dom.frameYaw]) input.addEventListener("input", updateFrameDraftFromInputs);
  dom.save.addEventListener("click", () => saveCurrent());
  dom.undo.addEventListener("click", undo);
  dom.fit.addEventListener("click", fitMap);
  dom.zoomIn.addEventListener("click", () => zoomAt(1.25));
  dom.zoomOut.addEventListener("click", () => zoomAt(0.8));
  dom.brushSize.addEventListener("input", () => { state.brushSize = Number(dom.brushSize.value); dom.brushOutput.textContent = `${state.brushSize} px`; scheduleRender(); });
  dom.direction.addEventListener("input", () => { state.directionValue = Number(dom.direction.value); updateDirectionUi(); });
  dom.showArrows.addEventListener("change", scheduleRender);
  for (const button of document.querySelectorAll("[data-editor-tool]")) button.addEventListener("click", () => setTool(button.dataset.editorTool));

  dom.canvas.addEventListener("pointerdown", pointerDown);
  dom.canvas.addEventListener("pointermove", pointerMove);
  dom.canvas.addEventListener("pointerup", pointerUp);
  dom.canvas.addEventListener("pointercancel", pointerUp);
  dom.canvas.addEventListener("pointerleave", () => { if (!state.drag) { state.hover = null; updateHover({ x: -1, y: -1 }); } dom.canvas.classList.remove("is-frame-grabbing", "is-frame-rotating"); });
  dom.canvas.addEventListener("contextmenu", (event) => event.preventDefault());
  dom.canvas.addEventListener("wheel", (event) => { if (!state.values) return; event.preventDefault(); zoomAt(event.deltaY < 0 ? 1.12 : 0.89, event.clientX, event.clientY); }, { passive: false });

  document.addEventListener("keydown", (event) => {
    if (dom.editorWorkspace.hidden) return;
    const target = event.target;
    if (target instanceof HTMLInputElement || target instanceof HTMLSelectElement) return;
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") { event.preventDefault(); undo(); return; }
    if (!event.ctrlKey && !event.metaKey && !event.altKey && event.key.toLowerCase() === "p" && !dom.cloudToggle.disabled) {
      event.preventDefault(); toggleCloudOverlay(); return;
    }
    if (/^[0-6]$/.test(event.key)) selectLabel(Number(event.key));
  });

  window.addEventListener("resize", resizeCanvas);
  window.addEventListener("mapping-map-list-changed", () => {
    if (!dom.editorWorkspace.hidden || state.mapsLoaded) void refreshMaps();
  });
  if (window.ResizeObserver) new ResizeObserver(resizeCanvas).observe(dom.canvasWrap);
  buildPalette(); updateEditorUi(); updateSourceControls(); resizeCanvas();
})();
