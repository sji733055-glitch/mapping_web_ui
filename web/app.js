(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const dom = {
    connectionPill: $("connection-pill"), connectionLabel: $("connection-label"), clock: $("clock"),
    stackBadge: $("stack-badge"), startStack: $("start-stack-button"), stopStack: $("stop-stack-button"), stackMessage: $("stack-message"),
    driverNodeDot: $("driver-node-dot"), driverNodeStatus: $("driver-node-status"), lioNodeDot: $("lio-node-dot"), lioNodeStatus: $("lio-node-status"), tfNodeDot: $("tf-node-dot"), tfNodeStatus: $("tf-node-status"),
    stateBadge: $("state-badge"), mapName: $("map-name"), start: $("start-button"), stop: $("stop-button"), reset: $("reset-button"),
    pointCount: $("point-count"), mapArea: $("map-area"), travel: $("travel-distance"), mapSize: $("map-size"), sessionTime: $("session-time"),
    lidarDot: $("lidar-dot"), lidarStatus: $("lidar-status"), odomDot: $("odom-dot"), odomStatus: $("odom-status"), storageDot: $("storage-dot"), storageStatus: $("storage-status"), dynamicRemovalDot: $("dynamic-removal-dot"), dynamicRemovalStatus: $("dynamic-removal-status"),
    outputCard: $("output-card"), outputPath: $("output-path"), saveProgress: $("save-progress"), saveProgressBar: $("save-progress-bar"), saveStage: $("save-stage"),
    pcdSelect: $("pcd-file-select"), pcdRefresh: $("pcd-refresh-button"), pcdConvert: $("pcd-convert-button"), pcdStatus: $("pcd-convert-status"), pcdLook: $("pcd-look-button"),
    pcdHeightMode: $("pcd-height-mode"), pcdZMin: $("pcd-z-min"), pcdZMax: $("pcd-z-max"), pcdResolution: $("pcd-resolution"), pcdRadius: $("pcd-radius"),
    pcdMinNeighbors: $("pcd-min-neighbors"), pcdPadding: $("pcd-padding"), pcdOutputName: $("pcd-output-name"), pcdDefaults: $("pcd-defaults-button"),
    pcdMapPreview: $("pcd-map-preview-button"), pcdMapPreviewWrap: $("pcd-map-preview-wrap"), pcdMapPreviewCanvas: $("pcd-map-preview-canvas"), pcdMapPreviewStats: $("pcd-map-preview-stats"),
    previewToggle: $("preview-toggle-button"), previewPanel: $("preview-panel"), previewCanvas: $("preview-canvas"), previewSelect: $("preview-file-select"),
    previewRefresh: $("preview-refresh-button"), previewLoad: $("preview-load-button"), previewClear: $("preview-clear-button"), previewClose: $("preview-close-button"),
    previewStatus: $("preview-status"), previewPoints: $("preview-points"),
    viewerEmpty: $("viewer-empty"), emptyTitle: $("empty-title"), emptyCopy: $("empty-copy"), visiblePoints: $("visible-points"), frameId: $("frame-id"), heightRange: $("height-range"), cloudRate: $("cloud-rate"),
    backendAddress: $("backend-address"), statusMessage: $("status-message"), voxelSize: $("voxel-size"), toastRegion: $("toast-region")
  };

  let viewer;
  try { viewer = new window.PointCloudViewer($("cloud-canvas")); }
  catch (error) { dom.emptyTitle.textContent="无法启动三维视图"; dom.emptyCopy.textContent=error.message; }

  const state = { socket:null, connected:false, reconnectTimer:null, previousStatus:null, cloudFrames:[], httpOnline:false, cloudPollBusy:false, pcdBusy:false, pcdFilesLoaded:false, previewViewer:null, previewOpen:false, previewLoaded:false, previewBusy:false, sessionName:"", exportDefaults:null, exportTouched:false, mapPreview:null, mapPreviewCanvas:null, mapPreviewName:"", mapView:null, mapDrag:null, mapPreviewBusy:false };
  const exportFields = [
    ["z_min", "pcdZMin"], ["z_max", "pcdZMax"], ["resolution", "pcdResolution"],
    ["radius", "pcdRadius"], ["min_neighbors", "pcdMinNeighbors"], ["padding", "pcdPadding"]
  ];
  const stateLabels = { IDLE:"待机", MAPPING:"建图中", SAVING:"保存中", SAVED:"已保存", ERROR:"异常" };

  function formatNumber(value) { return new Intl.NumberFormat("zh-CN").format(Number(value || 0)); }
  function formatDuration(seconds) { const total=Math.max(0,Math.floor(seconds||0)); const h=String(Math.floor(total/3600)).padStart(2,"0"),m=String(Math.floor(total%3600/60)).padStart(2,"0"),s=String(total%60).padStart(2,"0"); return `${h}:${m}:${s}`; }
  function setHealth(dot, text, ok, label) { dot.className=`health-dot ${ok?"ok":label?"warn":""}`; text.textContent=label || (ok?"正常":"未连接"); }
  function formatFileSize(bytes) { const value=Number(bytes||0); if(value>=1024**3)return `${(value/1024**3).toFixed(1)} GiB`; if(value>=1024**2)return `${(value/1024**2).toFixed(1)} MiB`; if(value>=1024)return `${(value/1024).toFixed(1)} KiB`; return `${value} B`; }

  function updatePcdControls(mode=state.previousStatus?.state||"IDLE") {
    const blocked=mode==="MAPPING"||mode==="SAVING", busy=state.pcdBusy||state.mapPreviewBusy;
    dom.pcdSelect.disabled=busy||blocked;
    dom.pcdRefresh.disabled=busy;
    dom.pcdLook.disabled=!state.connected||busy||blocked||!dom.pcdSelect.value;
    dom.pcdMapPreview.disabled=!state.connected||busy||blocked||!dom.pcdSelect.value;
    dom.pcdConvert.disabled=!state.connected||busy||blocked||!dom.pcdSelect.value;
    dom.pcdDefaults.disabled=busy||blocked;
    dom.pcdOutputName.disabled=busy||blocked;
    dom.pcdHeightMode.disabled=busy||blocked;
    for (const [, field] of exportFields) dom[field].disabled=busy||blocked;
  }

  function fillPcdSelect(select, files, previous) {
    select.replaceChildren();
    if (!files.length) { const option=document.createElement("option"); option.value=""; option.textContent="暂无 .pcd 文件"; select.appendChild(option); return; }
    for (const file of files) { const option=document.createElement("option"); option.value=file.name; option.textContent=`${file.filename} · ${formatFileSize(file.size_bytes)}`; select.appendChild(option); }
    if (files.some(file=>file.name===previous)) select.value=previous;
  }

  function applyExportDefaults(config, force=false) {
    if (!config) return;
    state.exportDefaults=config;
    if (state.exportTouched && !force) return;
    for (const [key, field] of exportFields) {
      const value=Number(config[key]);
      if (Number.isFinite(value)) dom[field].value=String(value);
    }
    if (["ground","absolute"].includes(config.height_mode)) dom.pcdHeightMode.value=config.height_mode;
  }

  function collectExportParams() {
    const params={height_mode:dom.pcdHeightMode.value};
    for (const [key, field] of exportFields) {
      const raw=String(dom[field].value??"").trim();
      if (raw==="") continue;
      const value=Number(raw);
      if (!Number.isFinite(value)) throw new Error(`切片参数「${key}」必须是数值`);
      params[key]=value;
    }
    if (params.min_neighbors!==undefined && !Number.isInteger(params.min_neighbors)) throw new Error("最小邻点数必须是整数");
    return params;
  }

  function collectSliceFilter() {
    const filter={height_mode:dom.pcdHeightMode.value};
    const zMin=Number(dom.pcdZMin.value), zMax=Number(dom.pcdZMax.value);
    if (Number.isFinite(zMin)) filter.z_min=zMin;
    if (Number.isFinite(zMax)) filter.z_max=zMax;
    return filter;
  }

  function readPreviewHeader(headers, name) {
    const value=headers&&typeof headers.get==="function"?headers.get(name):null;
    const number=Number(value);
    return value===null||value===undefined||value===""||!Number.isFinite(number)?null:number;
  }

  function decodePgm(buffer) {
    if (!(buffer instanceof ArrayBuffer)||buffer.byteLength<8) return null;
    const bytes=new Uint8Array(buffer), tokens=[];
    let offset=0;
    while (tokens.length<4 && offset<bytes.length) {
      const byte=bytes[offset];
      if (byte===0x23) { while (offset<bytes.length && bytes[offset]!==0x0a) offset+=1; continue; }
      if (byte===0x20||byte===0x09||byte===0x0a||byte===0x0d) { offset+=1; continue; }
      let token="";
      while (offset<bytes.length && ![0x20,0x09,0x0a,0x0d].includes(bytes[offset])) { token+=String.fromCharCode(bytes[offset]); offset+=1; }
      tokens.push(token);
    }
    if (tokens.length<4||tokens[0]!=="P5") return null;
    const width=Number(tokens[1]), height=Number(tokens[2]), maximum=Number(tokens[3]);
    if (!Number.isInteger(width)||!Number.isInteger(height)||width<=0||height<=0||!(maximum>0&&maximum<256)) return null;
    if (offset<bytes.length) offset+=1;
    if (bytes.length-offset!==width*height) return null;
    return { width, height, pixels:bytes.subarray(offset,offset+width*height) };
  }

  function describePreview(meta, grid) {
    const parts=[`${formatNumber(meta.width||grid.width)} × ${formatNumber(meta.height||grid.height)} px`];
    if (meta.heightMode==="ground") parts.push(Number.isFinite(meta.groundTilt)?`地面倾斜 ${meta.groundTilt.toFixed(2)}° 已校正`:"已校正倾斜地面");
    if (Number.isFinite(meta.resolution)) parts.push(`分辨率 ${meta.resolution} m/px`);
    if (Number.isFinite(meta.slicePoints)) parts.push(`切片 ${formatNumber(meta.slicePoints)} 点`);
    if (Number.isFinite(meta.occupied)) parts.push(`占据 ${formatNumber(meta.occupied)} 格`);
    if (Number.isFinite(meta.stride)&&meta.stride>1) parts.push(`显示降采样 ${meta.stride}×`);
    return parts.join(" · ");
  }

  function fitPreviewCanvas(canvas) {
    const width=canvas.clientWidth||0, height=canvas.clientHeight||0;
    if (!width||!height) return null;
    const ratio=Number(window.devicePixelRatio)||1;
    const pixelWidth=Math.max(1,Math.round(width*ratio)), pixelHeight=Math.max(1,Math.round(height*ratio));
    if (canvas.width!==pixelWidth) canvas.width=pixelWidth;
    if (canvas.height!==pixelHeight) canvas.height=pixelHeight;
    return { width:pixelWidth, height:pixelHeight };
  }

  function buildMapPreviewCanvas(grid) {
    if (typeof document.createElement!=="function") return null;
    const canvas=document.createElement("canvas");
    if (!canvas||typeof canvas.getContext!=="function") return null;
    canvas.width=grid.width; canvas.height=grid.height;
    const context=canvas.getContext("2d");
    if (!context||typeof context.createImageData!=="function") return null;
    const image=context.createImageData(grid.width,grid.height), data=image.data;
    for (let index=0; index<grid.pixels.length; index+=1) {
      const value=grid.pixels[index], target=index*4;
      // PGM 约定：0 = 占据，254 = 空闲，其余值（例如 ROS 的 205 未知）保持中性灰。
      const occupied=value<=127, free=value>=250;
      data[target]=occupied?13:free?226:138;
      data[target+1]=occupied?32:free?241:165;
      data[target+2]=occupied?36:free?238:162;
      data[target+3]=255;
    }
    context.putImageData(image,0,0);
    return canvas;
  }

  function renderMapPreview(fit=false) {
    const grid=state.mapPreview, canvas=dom.pcdMapPreviewCanvas;
    if (!grid||!canvas||typeof canvas.getContext!=="function") return;
    const context=canvas.getContext("2d");
    if (!context||typeof context.drawImage!=="function") return;
    const box=fitPreviewCanvas(canvas);
    if (!box) return;
    const source=state.mapPreviewCanvas||buildMapPreviewCanvas(grid);
    state.mapPreviewCanvas=source;
    if (fit||!state.mapView) {
      const scale=Math.min(box.width/grid.width,box.height/grid.height);
      state.mapView={ scale, offsetX:(box.width-grid.width*scale)/2, offsetY:(box.height-grid.height*scale)/2 };
    }
    const view=state.mapView;
    context.setTransform(1,0,0,1,0,0);
    context.fillStyle="#050a0b";
    context.fillRect(0,0,box.width,box.height);
    if (!source) return;
    context.imageSmoothingEnabled=false;
    context.drawImage(source,0,0,grid.width,grid.height,view.offsetX,view.offsetY,grid.width*view.scale,grid.height*view.scale);
  }

  function bindMapPreviewInteractions(canvas) {
    if (!canvas||typeof canvas.addEventListener!=="function") return;
    const zoom=(event)=>{
      if (!state.mapPreview||!state.mapView) return;
      if (typeof event.preventDefault==="function") event.preventDefault();
      const box=fitPreviewCanvas(canvas);
      if (!box) return;
      const ratio=Number(window.devicePixelRatio)||1;
      const pointerX=(event.offsetX||0)*ratio, pointerY=(event.offsetY||0)*ratio;
      const scale=Math.max(0.05,Math.min(60,state.mapView.scale*Math.exp(-(event.deltaY||0)*0.0015)));
      const applied=scale/state.mapView.scale;
      state.mapView={ scale, offsetX:pointerX-(pointerX-state.mapView.offsetX)*applied, offsetY:pointerY-(pointerY-state.mapView.offsetY)*applied };
      renderMapPreview();
    };
    canvas.addEventListener("wheel",zoom,{passive:false});
    canvas.addEventListener("pointerdown",(event)=>{
      if (!state.mapView) return;
      state.mapDrag={ x:event.clientX||0, y:event.clientY||0 };
      if (typeof canvas.setPointerCapture==="function"&&event.pointerId!==undefined) canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointermove",(event)=>{
      if (!state.mapDrag||!state.mapView) return;
      const ratio=Number(window.devicePixelRatio)||1;
      state.mapView.offsetX+=((event.clientX||0)-state.mapDrag.x)*ratio;
      state.mapView.offsetY+=((event.clientY||0)-state.mapDrag.y)*ratio;
      state.mapDrag={ x:event.clientX||0, y:event.clientY||0 };
      renderMapPreview();
    });
    const release=()=>{ state.mapDrag=null; };
    canvas.addEventListener("pointerup",release);
    canvas.addEventListener("pointercancel",release);
    canvas.addEventListener("dblclick",()=>renderMapPreview(true));
  }

  async function refreshPcdFiles() {
    const selected=dom.pcdSelect.value, previewSelected=dom.previewSelect.value;
    state.pcdBusy=true; updatePcdControls(); dom.pcdStatus.textContent="正在读取 data/pcd…";
    try {
      const response=await fetch(`/api/pcd-files?t=${Date.now()}`,{cache:"no-store"});
      const payload=await response.json();
      if(!response.ok)throw new Error(payload.message||"无法读取 PCD 列表");
      const files=Array.isArray(payload.pcd_files)?payload.pcd_files:[];
      fillPcdSelect(dom.pcdSelect,files,selected);
      fillPcdSelect(dom.previewSelect,files,previewSelected);
      state.pcdFilesLoaded=true;
      dom.pcdStatus.textContent=files.length?`已找到 ${files.length} 个 PCD；选一个后先预览二维切片，再生成 PGM/YAML。`:`data/pcd 中没有 .pcd 文件。`;
    } catch(error) {
      dom.pcdStatus.textContent=error.message;
      if(state.previewOpen) dom.previewStatus.textContent=error.message;
    } finally {
      state.pcdBusy=false; updatePcdControls(); updatePreviewControls();
    }
  }

  async function previewPcdMap() {
    const mapName=dom.pcdSelect.value;
    if(!mapName) return;
    let params;
    try { params=collectExportParams(); } catch(error) { dom.pcdStatus.textContent=error.message; toast(error.message,"error"); return; }
    state.mapPreviewBusy=true; updatePcdControls(); dom.pcdStatus.textContent=`正在按当前参数切片 ${mapName}.pcd…`;
    try {
      const response=await fetch("/api/pcd/map-preview",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({map_name:mapName,...params})});
      if(!response.ok){const payload=await response.json().catch(()=>({}));throw new Error(payload.message||`二维预览失败（HTTP ${response.status}）`);}
      const grid=decodePgm(await response.arrayBuffer());
      if(!grid) throw new Error("二维预览数据不是有效的 PGM (P5)");
      const headers=response.headers;
      const meta={
        width:readPreviewHeader(headers,"X-Map-Width"), height:readPreviewHeader(headers,"X-Map-Height"),
        heightMode:headers.get("X-Map-Height-Mode"),
        groundTilt:readPreviewHeader(headers,"X-Map-Ground-Tilt"),
        resolution:readPreviewHeader(headers,"X-Map-Resolution"), slicePoints:readPreviewHeader(headers,"X-Map-Slice-Points"),
        occupied:readPreviewHeader(headers,"X-Map-Occupied"), stride:readPreviewHeader(headers,"X-Map-Preview-Stride")
      };
      if(meta.width&&meta.width!==grid.width) throw new Error("二维预览尺寸与后端元数据不一致");
      state.mapPreview=grid; state.mapPreviewName=mapName; state.mapView=null; state.mapPreviewCanvas=null;
      dom.pcdMapPreviewWrap.hidden=false;
      renderMapPreview(true);
      dom.pcdMapPreviewStats.textContent=describePreview(meta,grid);
      dom.pcdStatus.textContent=meta.stride>1
        ?`预览按 ${meta.stride}×${meta.stride} 栅格取最暗值降采样；生成时仍写入完整栅格，源 PCD 未被修改。`
        :`切片完成；确认后点「生成 PGM · YAML」写入 data/map，源 PCD 不会被改写。`;
    } catch(error) {
      dom.pcdStatus.textContent=error.message; toast(error.message,"error");
    } finally {
      state.mapPreviewBusy=false; updatePcdControls();
    }
  }

  async function convertSelectedPcd() {
    const mapName=dom.pcdSelect.value;
    if(!mapName) return;
    let params;
    try { params=collectExportParams(); } catch(error) { dom.pcdStatus.textContent=error.message; toast(error.message,"error"); return; }
    const outputName=dom.pcdOutputName.value.trim();
    if(outputName&&!/^[A-Za-z0-9._-]{1,48}$/.test(outputName)){ const message="输出地图名只能包含字母、数字、点、短横线和下划线"; dom.pcdStatus.textContent=message; toast(message,"error"); return; }
    state.pcdBusy=true; updatePcdControls(); dom.pcdStatus.textContent=`正在解析 ${mapName}.pcd 并生成二维地图…`;
    try {
      const response=await fetch("/api/pcd/convert",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({map_name:mapName,output_name:outputName,...params})});
      const result=await response.json();
      if(!response.ok||!result.ok)throw new Error(result.message||"PCD 转换失败");
      dom.pcdStatus.textContent=result.message;
      toast(result.message,"success");
      window.dispatchEvent(new Event("mapping-map-list-changed"));
    } catch(error) {
      dom.pcdStatus.textContent=error.message; toast(error.message,"error");
    } finally {
      state.pcdBusy=false; updatePcdControls();
    }
  }

  function updatePreviewControls() {
    dom.previewSelect.disabled=state.previewBusy;
    dom.previewRefresh.disabled=state.previewBusy;
    dom.previewLoad.disabled=!state.connected||state.previewBusy||!dom.previewSelect.value;
    dom.previewClear.disabled=state.previewBusy||!state.previewLoaded;
  }

  function ensurePreviewViewer() {
    if(state.previewViewer) return state.previewViewer;
    try { state.previewViewer=new window.PointCloudViewer(dom.previewCanvas); }
    catch(error) { dom.previewStatus.textContent=`无法启动预览视图：${error.message}`; }
    return state.previewViewer;
  }

  function setPreviewOpen(open) {
    state.previewOpen=open;
    dom.previewPanel.hidden=!open;
    dom.previewPanel.closest(".viewer-card")?.classList.toggle("is-preview-open",open);
    dom.previewToggle.setAttribute("aria-expanded",String(open));
    dom.previewToggle.classList.toggle("is-active",open);
    if(open) {
      ensurePreviewViewer();
      state.previewViewer?.resize();
      if(!state.pcdFilesLoaded) refreshPcdFiles();
    }
    if(viewer) viewer.resize();
    updatePreviewControls();
  }

  async function loadPreviewCloud(sliceFilter=null) {
    const name=dom.previewSelect.value;
    if(!name) return;
    const filter=sliceFilter||{}, query=new URLSearchParams({ name, t:String(Date.now()) });
    if(Number.isFinite(filter.z_min)) query.set("z_min",String(filter.z_min));
    if(Number.isFinite(filter.z_max)) query.set("z_max",String(filter.z_max));
    if(["ground","absolute"].includes(filter.height_mode)) query.set("height_mode",filter.height_mode);
    const sliced=query.has("z_min")||query.has("z_max");
    state.previewBusy=true; updatePreviewControls();
    dom.previewStatus.textContent=sliced?`正在读取 data/pcd/${name}.pcd 的 Z 切片…`:`正在读取 data/pcd/${name}.pcd…`;
    try {
      const response=await fetch(`/api/pcd/preview?${query.toString()}`,{cache:"no-store"});
      if(!response.ok){const payload=await response.json().catch(()=>({}));throw new Error(payload.message||`预览失败（HTTP ${response.status}）`);}
      const frame=parseCloudFrame(await response.arrayBuffer());
      if(!frame) throw new Error("点云数据格式不正确");
      const target=ensurePreviewViewer();
      if(!target) throw new Error("浏览器不支持 WebGL");
      target.setPoints(frame.points); target.fit(); target.resize();
      state.previewLoaded=true;
      const shown=Number(response.headers.get("X-Preview-Points"))||frame.count;
      const total=Number(response.headers.get("X-Preview-Total"))||frame.count;
      const scope=sliced?`Z=${Number(query.get("z_min")??"-∞").toFixed(2)}–${Number(query.get("z_max")??"∞").toFixed(2)} m 切片`:`全部 ${formatNumber(total)} 点`;
      dom.previewPoints.textContent=`${formatNumber(shown)} 点`;
      dom.previewStatus.textContent=total>shown
        ?`${name}.pcd：${scope}，按网页上限抽稀显示 ${formatNumber(shown)} 点；磁盘文件未被修改。`
        :`${name}.pcd：${scope}，已全部显示。`;
    } catch(error) {
      dom.previewStatus.textContent=error.message; toast(error.message,"error");
    } finally {
      state.previewBusy=false; updatePreviewControls();
    }
  }

  function clearPreviewCloud() {
    if(state.previewViewer) state.previewViewer.setPoints(new Float32Array(0));
    state.previewLoaded=false;
    dom.previewPoints.textContent="0 点";
    dom.previewStatus.textContent="已清除预览；实时点云不受影响。";
    updatePreviewControls();
  }

  function toast(message, kind="info") { const el=document.createElement("div"); el.className=`toast ${kind}`; el.textContent=message; dom.toastRegion.appendChild(el); setTimeout(()=>el.remove(),4200); }
  window.mappingConsoleToast=toast;

  function renderStatus(s) {
    const mode=s.state||"IDLE", mapping=mode==="MAPPING", saving=mode==="SAVING", available=state.connected&&!saving;
    updatePcdControls(mode);
    const stack=s.stack||{}, stackState=stack.state||"STOPPED", nodes=stack.nodes||{};
    const stackLabels={RUNNING:"运行中",STOPPED:"未启动",PARTIAL:"部分运行",STARTING:"启动中",STOPPING:"停止中"};
    dom.stackBadge.textContent=stackLabels[stackState]||stackState; dom.stackBadge.className=`state-badge ${stackState==="RUNNING"?"mapping":stackState==="STARTING"||stackState==="STOPPING"?"saving":stackState==="PARTIAL"?"error":"idle"}`;
    const nodeLabel=(node)=>node?.online?(node.managed?(node.implementation==="project_local"?"项目内启动":"网页启动"):"外部运行"):"未运行";
    setHealth(dom.driverNodeDot,dom.driverNodeStatus,!!nodes.mid360_driver?.online,nodeLabel(nodes.mid360_driver));
    setHealth(dom.lioNodeDot,dom.lioNodeStatus,!!nodes.small_point_lio?.online,nodeLabel(nodes.small_point_lio));
    setHealth(dom.tfNodeDot,dom.tfNodeStatus,!!nodes.robot_state_publisher?.online,nodeLabel(nodes.robot_state_publisher));
    dom.stackMessage.textContent=stack.message||"";
    const localLio=stack.local_lio||{}, localLioBlocked=!nodes.small_point_lio?.online&&(localLio.ready===false||localLio.stale===true);
    dom.startStack.disabled=!state.connected||localLioBlocked||stackState==="RUNNING"||stackState==="STARTING"||stackState==="STOPPING";
    dom.stopStack.disabled=!state.connected||!stack.managed_count||stackState==="STOPPING"||mapping||saving;
    dom.stateBadge.textContent=stateLabels[mode]||mode; dom.stateBadge.className=`state-badge ${mode.toLowerCase()}`;
    dom.mapName.disabled=mapping||saving; dom.start.disabled=!available||mapping||!s.cloud_online; dom.stop.disabled=!state.connected||!mapping; dom.reset.disabled=!available||mode==="IDLE"||mapping;
    dom.pointCount.textContent=formatNumber(s.point_count); dom.sessionTime.textContent=formatDuration(s.elapsed_seconds);
    const b=s.bounds||{}; const width=Math.max(0,(b.max_x||0)-(b.min_x||0)),height=Math.max(0,(b.max_y||0)-(b.min_y||0));
    applyExportDefaults(s.map_export);
    dom.mapArea.innerHTML=`${(width*height).toFixed(1)} <small>m²</small>`; dom.mapSize.innerHTML=`${width.toFixed(1)} × ${height.toFixed(1)} <small>m</small>`; dom.travel.innerHTML=`${Number(s.travel_distance||0).toFixed(1)} <small>m</small>`;
    setHealth(dom.lidarDot,dom.lidarStatus,!!s.cloud_online,s.cloud_online?`${Number(s.cloud_rate_hz||0).toFixed(1)} Hz`:"无数据");
    setHealth(dom.odomDot,dom.odomStatus,!!s.odom_online,s.odom_online?"正常":"无数据"); setHealth(dom.storageDot,dom.storageStatus,!!s.storage_writable,s.storage_writable?"可写":"不可写");
    const dynamic=s.dynamic_removal||{}, history=s.keyframe_history||{}, dynamicError=String(dynamic.last_error||history.last_error||"");
    const cleaningEnabled=!!dynamic.enabled||!!history.enabled;
    const historyLabel=history.enabled?` · 关键帧 ${formatNumber(history.written_frames)}`:"";
    setHealth(dom.dynamicRemovalDot,dom.dynamicRemovalStatus,cleaningEnabled&&!dynamicError,cleaningEnabled?(dynamicError?"处理异常":`在线 ${formatNumber(dynamic.removed_points)} 点${historyLabel}`):"已禁用");
    const offline=history.offline_filter||{}, rayOffline=offline.ray||{}, polarOffline=offline.polar||{};
    const offlineBreakdown=(Number(rayOffline.removed_points)||0)||(Number(polarOffline.removed_points)||0)
      ?`（射线 ${formatNumber(rayOffline.removed_points||0)} / 区域投票 ${formatNumber(polarOffline.removed_points||0)}）`:"";
    const offlineText=offline.applied?`；保存时离线删除 ${formatNumber(offline.removed_points)} 点${offlineBreakdown}`:offline.reason?`；${offline.reason}`:"";
    dom.dynamicRemovalStatus.title=dynamicError||`在线处理 ${formatNumber(dynamic.processed_frames)} 帧，关键帧 ${formatNumber(history.written_frames)} 帧 / ${formatFileSize(history.bytes_written)}${history.dropped_frames?`，丢弃 ${formatNumber(history.dropped_frames)} 帧`:""}${offlineText}`;
    dom.frameId.textContent=`坐标系 ${s.frame_id||"—"}`; dom.voxelSize.textContent=s.voxel_size?`${Number(s.voxel_size).toFixed(2)} m`:"—";
    dom.statusMessage.textContent=s.message||({MAPPING:"正在累计点云",SAVING:"正在生成地图文件",SAVED:"地图保存完成",IDLE:"可以开始新的建图会话"}[mode]||"");
    dom.saveProgress.hidden=!saving; dom.saveProgressBar.style.width=`${Math.round((s.save_progress||0)*100)}%`; dom.saveStage.textContent=s.save_stage||"正在保存…";
    if (s.output && s.output.pcd) { dom.outputCard.hidden=false; dom.outputPath.textContent=s.output.pcd; dom.outputPath.title=s.output.pcd; }
    const zmin=b.min_z,zmax=b.max_z; dom.heightRange.textContent=Number.isFinite(zmin)&&Number.isFinite(zmax)?`${Number(zmin).toFixed(1)} → ${Number(zmax).toFixed(1)} m`:"—";
    const sessionName=String(s.session_name||"");
    if(sessionName&&sessionName!==state.sessionName){ state.sessionName=sessionName; if(mapping||saving||mode==="SAVED") dom.mapName.value=sessionName; }
    if (state.previousStatus && state.previousStatus.state!==mode) {
      if (mode==="MAPPING") toast("建图会话已开始","success");
      if (mode==="SAVED") toast("地图已保存：PCD、PGM、YAML","success");
      if (mode==="ERROR") toast(s.message||"建图服务发生错误","error");
    }
    state.previousStatus=s;
  }

  function setConnected(connected, mode="") {
    state.connected=connected; dom.connectionPill.className=`connection-pill ${connected?"is-online":"is-offline"}`; dom.connectionLabel.textContent=connected?(mode==="http"?"HTTP 兼容模式":"实时连接"):"连接已断开";
    updatePcdControls();
    updatePreviewControls();
    if (!connected) { dom.start.disabled=true; dom.stop.disabled=true; dom.reset.disabled=true; dom.startStack.disabled=true; dom.stopStack.disabled=true; dom.statusMessage.textContent="等待建图服务连接…"; }
  }

  function parseCloudFrame(buffer) {
    if(!(buffer instanceof ArrayBuffer)||buffer.byteLength<8) return null;
    const view=new DataView(buffer); const magic=String.fromCharCode(view.getUint8(0),view.getUint8(1),view.getUint8(2),view.getUint8(3)); if(magic!=="MAP1") return null;
    const count=view.getUint32(4,true); if(buffer.byteLength!==8+count*12) return null;
    return { count, points:new Float32Array(buffer,8,count*3) };
  }

  function handleBinary(buffer) {
    const frame=parseCloudFrame(buffer); if(!frame) return;
    if(viewer) viewer.setPoints(frame.points);
    dom.visiblePoints.textContent=formatNumber(frame.count); dom.viewerEmpty.classList.toggle("is-hidden",frame.count>0);
    const now=performance.now(); state.cloudFrames.push(now); state.cloudFrames=state.cloudFrames.filter(t=>now-t<4000); dom.cloudRate.textContent=`${(state.cloudFrames.length/4).toFixed(1)} Hz`;
  }

  function connect() {
    clearTimeout(state.reconnectTimer); const scheme=location.protocol==="https:"?"wss":"ws",url=`${scheme}://${location.host}/ws`; dom.backendAddress.textContent=`后端 ${url}`;
    const socket=new WebSocket(url); socket.binaryType="arraybuffer"; state.socket=socket;
    socket.onopen=()=>setConnected(true,"websocket");
    socket.onmessage=(event)=>{ if(event.data instanceof ArrayBuffer) handleBinary(event.data); else { try { const data=JSON.parse(event.data); if(data.type==="status")renderStatus(data.payload); else if(data.type==="result")toast(data.message,data.ok?"success":"error"); } catch(_){} } };
    socket.onclose=()=>{ if(state.socket!==socket)return; if(state.httpOnline)setConnected(true,"http");else setConnected(false); state.reconnectTimer=setTimeout(connect,2500); };
    socket.onerror=()=>socket.close();
  }

  function send(action) {
    const name=dom.mapName.value.trim(); if(action==="start"&&!/^[A-Za-z0-9._-]+$/.test(name)){toast("地图名称只能包含字母、数字、点、短横线和下划线","error");dom.mapName.focus();return;}
    if(state.socket&&state.socket.readyState===WebSocket.OPEN){state.socket.send(JSON.stringify({type:"command",action,map_name:name}));return;}
    if(!state.httpOnline){toast("建图服务尚未连接","error");return;}
    apiCommand(action,name).then(result=>toast(result.message,"success")).catch(error=>toast(error.message,"error"));
  }

  async function apiCommand(action, mapName="") {
    const payload={map_name:mapName};
    const response=await fetch(`/api/${action}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(payload)});
    const result=await response.json();
    if(!response.ok||!result.ok) throw new Error(result.message||"操作失败");
    return result;
  }

  function registerAgentTools() {
    const context=document.modelContext;
    if(!context?.registerTool) return;
    const register=(tool)=>{try{void Promise.resolve(context.registerTool(tool)).catch(()=>{});}catch(_){}};
    register({name:"get_mapping_status",title:"读取建图状态",description:"读取当前建图会话、累计点数、输入链路和保存进度，不改变建图状态。",inputSchema:{type:"object",properties:{},additionalProperties:false},annotations:{readOnlyHint:true,untrustedContentHint:false},async execute(){const response=await fetch("/api/status");if(!response.ok)throw new Error("无法读取建图状态");return response.json();}});
    register({name:"start_ros_mapping_stack",title:"启动建图节点",description:"启动缺失的机器人 TF、MID360 驱动和 Small Point-LIO 节点。不会重复启动已经存在的节点。",inputSchema:{type:"object",properties:{},additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},async execute(){return apiCommand("start_stack");}});
    register({name:"stop_managed_ros_mapping_stack",title:"停止网页启动的建图节点",description:"停止由本网页后端启动的机器人 TF、MID360 驱动和 Small Point-LIO；不会停止外部终端启动的节点。",inputSchema:{type:"object",properties:{},additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},async execute(){return apiCommand("stop_stack");}});
    register({name:"start_mapping_session",title:"开始建图",description:"清空之前的内存累计点云，并以给定地图名称开始一个新的建图会话。",inputSchema:{type:"object",properties:{map_name:{type:"string",pattern:"^[A-Za-z0-9._-]{1,48}$",description:"保存地图时使用的文件名，不含扩展名。"}},required:["map_name"],additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},async execute(input){return apiCommand("start",String(input.map_name||""));}});
    register({name:"stop_mapping_and_save",title:"结束并保存点云",description:"结束当前建图会话，把完整累计点云写入 data/pcd/<名称>.pcd。二维 PGM/YAML 不随保存生成，需另行调用 convert_existing_pcd_to_pgm。",inputSchema:{type:"object",properties:{},additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},async execute(){return apiCommand("stop","");}});
    register({name:"convert_existing_pcd_to_pgm",title:"将已有 PCD 转为 PGM",description:"把项目 data/pcd 下的 PCD 按给定切片参数生成 Nav2 PGM/YAML；不会覆盖已有地图，同名冲突时自动加时间戳。",inputSchema:{type:"object",properties:{map_name:{type:"string",pattern:"^[A-Za-z0-9._-]{1,48}$",description:"data/pcd 中的文件名，不含 .pcd。"},output_name:{type:"string",pattern:"^[A-Za-z0-9._-]{1,48}$",description:"输出地图名，省略时沿用点云名称。"},height_mode:{type:"string",enum:["ground","absolute"],description:"高度基准：ground 自动拟合倾斜地面（推荐），absolute 使用全局 Z。"},z_min:{type:"number",description:"障碍高度下限（米）。"},z_max:{type:"number",description:"障碍高度上限（米）。"},resolution:{type:"number",description:"栅格分辨率（米/格）。"},radius:{type:"number",description:"离群点滤波半径（米），0 表示关闭。"},min_neighbors:{type:"integer",description:"滤波半径内的最小邻点数，0 表示关闭。"},padding:{type:"number",description:"地图边缘留白（米）。"}},required:["map_name"],additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},async execute(input){return convertPcd(String(input.map_name||""),input);}});
    register({name:"preview_pcd_occupancy_map",title:"预览 PCD 二维切片",description:"按给定切片参数在内存中生成二维栅格并返回尺寸、占据格数与元数据；不写入任何文件，用于生成前确认切片是否合适。",inputSchema:{type:"object",properties:{map_name:{type:"string",pattern:"^[A-Za-z0-9._-]{1,48}$",description:"data/pcd 中的文件名，不含 .pcd。"},height_mode:{type:"string",enum:["ground","absolute"],description:"高度基准：ground 自动拟合倾斜地面（推荐），absolute 使用全局 Z。"},z_min:{type:"number",description:"障碍高度下限（米）。"},z_max:{type:"number",description:"障碍高度上限（米）。"},resolution:{type:"number",description:"栅格分辨率（米/格）。"},radius:{type:"number",description:"离群点滤波半径（米）。"},min_neighbors:{type:"integer",description:"滤波半径内的最小邻点数。"},padding:{type:"number",description:"地图边缘留白（米）。"}},required:["map_name"],additionalProperties:false},annotations:{readOnlyHint:true,untrustedContentHint:false},async execute(input){return previewPcd(String(input.map_name||""),input);}});
  }

  function sliceParams(input={}) {
    const payload={};
    if(["ground","absolute"].includes(input?.height_mode)) payload.height_mode=input.height_mode;
    for(const key of ["z_min","z_max","resolution","radius","min_neighbors","padding"]){ const value=Number(input?.[key]); if(Number.isFinite(value)) payload[key]=value; }
    return payload;
  }

  async function convertPcd(mapName, input={}) {
    if(!mapName) throw new Error("必须提供 data/pcd 中的点云名称");
    const payload={ map_name:mapName, ...sliceParams(input) };
    const outputName=String(input.output_name||"").trim();
    if(outputName) payload.output_name=outputName;
    const response=await fetch("/api/pcd/convert",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(payload)});
    const result=await response.json();
    if(!response.ok||!result.ok)throw new Error(result.message||"PCD 转换失败");
    return result;
  }

  async function previewPcd(mapName, input={}) {
    if(!mapName) throw new Error("必须提供 data/pcd 中的点云名称");
    const response=await fetch("/api/pcd/map-preview",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({ map_name:mapName, ...sliceParams(input) })});
    if(!response.ok){const error=await response.json().catch(()=>({}));throw new Error(error.message||"二维切片预览失败");}
    const grid=decodePgm(await response.arrayBuffer());
    if(!grid) throw new Error("二维预览数据不是有效的 PGM (P5)");
    const headers=response.headers;
    return {
      map_name:mapName, height_mode:headers.get("X-Map-Height-Mode"), ground_tilt_deg:readPreviewHeader(headers,"X-Map-Ground-Tilt"), width:readPreviewHeader(headers,"X-Map-Width"), height:readPreviewHeader(headers,"X-Map-Height"),
      resolution:readPreviewHeader(headers,"X-Map-Resolution"), origin_x:readPreviewHeader(headers,"X-Map-Origin-X"),
      origin_y:readPreviewHeader(headers,"X-Map-Origin-Y"), z_min:readPreviewHeader(headers,"X-Map-Z-Min"),
      z_max:readPreviewHeader(headers,"X-Map-Z-Max"), slice_points:readPreviewHeader(headers,"X-Map-Slice-Points"),
      filtered_points:readPreviewHeader(headers,"X-Map-Filtered-Points"), occupied_cells:readPreviewHeader(headers,"X-Map-Occupied"),
      preview_stride:readPreviewHeader(headers,"X-Map-Preview-Stride"), decoded_pixels:grid.width*grid.height
    };
  }

  async function pollCloudOverHttp() {
    if(state.cloudPollBusy||state.socket?.readyState===WebSocket.OPEN) return;
    state.cloudPollBusy=true;
    try {
      const response=await fetch(`/api/cloud?t=${Date.now()}`,{cache:"no-store"});
      if(!response.ok) throw new Error("cloud request failed");
      handleBinary(await response.arrayBuffer());
    } catch(_) {} finally { state.cloudPollBusy=false; }
  }

  async function pollStatusOverHttp() {
    try {
      const response=await fetch(`/api/status?t=${Date.now()}`,{cache:"no-store"});
      if(!response.ok) throw new Error("status request failed");
      const status=await response.json(); state.httpOnline=true;
      setConnected(true,state.socket?.readyState===WebSocket.OPEN?"websocket":"http"); renderStatus(status);
      await pollCloudOverHttp();
    } catch(_) {
      state.httpOnline=false;
      if(!state.socket||state.socket.readyState!==WebSocket.OPEN)setConnected(false);
    }
  }

  dom.start.addEventListener("click",()=>send("start")); dom.stop.addEventListener("click",()=>send("stop")); dom.reset.addEventListener("click",()=>send("reset"));
  dom.pcdRefresh.addEventListener("click",refreshPcdFiles);
  dom.pcdSelect.addEventListener("change",()=>{ state.mapPreview=null; state.mapView=null; dom.pcdMapPreviewWrap.hidden=true; updatePcdControls(); });
  dom.pcdConvert.addEventListener("click",convertSelectedPcd);
  dom.pcdMapPreview.addEventListener("click",previewPcdMap);
  dom.pcdDefaults.addEventListener("click",()=>{ state.exportTouched=false; applyExportDefaults(state.exportDefaults,true); dom.pcdStatus.textContent="已恢复后端切片默认值；可重新预览。"; });
  dom.pcdLook.addEventListener("click",()=>{ const name=dom.pcdSelect.value; if(!name)return; dom.previewSelect.value=name; setPreviewOpen(true); updatePreviewControls(); loadPreviewCloud(collectSliceFilter()); });
  for (const [, field] of exportFields) dom[field].addEventListener("input",()=>{ state.exportTouched=true; });
  dom.pcdHeightMode.addEventListener("change",()=>{ state.exportTouched=true; });
  dom.startStack.addEventListener("click",()=>send("start_stack")); dom.stopStack.addEventListener("click",()=>send("stop_stack"));
  dom.previewToggle.addEventListener("click",()=>setPreviewOpen(!state.previewOpen));
  dom.previewClose.addEventListener("click",()=>setPreviewOpen(false));
  dom.previewRefresh.addEventListener("click",refreshPcdFiles);
  dom.previewLoad.addEventListener("click",()=>loadPreviewCloud());
  dom.previewClear.addEventListener("click",clearPreviewCloud);
  dom.previewSelect.addEventListener("change",updatePreviewControls);
  bindMapPreviewInteractions(dom.pcdMapPreviewCanvas);
  window.addEventListener("resize",()=>{ if(state.mapPreview) renderMapPreview(true); });
  $("fit-view-button").addEventListener("click",()=>viewer&&viewer.fit()); $("top-view-button").addEventListener("click",()=>viewer&&viewer.top());
  setInterval(()=>{dom.clock.textContent=new Date().toLocaleTimeString("zh-CN",{hour12:false});},500);
  setInterval(pollStatusOverHttp,1000); updatePreviewControls(); registerAgentTools(); connect(); pollStatusOverHttp(); refreshPcdFiles();
})();
