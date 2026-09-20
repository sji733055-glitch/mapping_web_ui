(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const dom = {
    connectionPill: $("connection-pill"), connectionLabel: $("connection-label"), clock: $("clock"),
    stackBadge: $("stack-badge"), startStack: $("start-stack-button"), stopStack: $("stop-stack-button"), stackMessage: $("stack-message"),
    driverNodeDot: $("driver-node-dot"), driverNodeStatus: $("driver-node-status"), lioNodeDot: $("lio-node-dot"), lioNodeStatus: $("lio-node-status"), tfNodeDot: $("tf-node-dot"), tfNodeStatus: $("tf-node-status"),
    stateBadge: $("state-badge"), mapName: $("map-name"), start: $("start-button"), stop: $("stop-button"), reset: $("reset-button"),
    pointCount: $("point-count"), mapArea: $("map-area"), travel: $("travel-distance"), mapSize: $("map-size"), sessionTime: $("session-time"),
    lidarDot: $("lidar-dot"), lidarStatus: $("lidar-status"), odomDot: $("odom-dot"), odomStatus: $("odom-status"), storageDot: $("storage-dot"), storageStatus: $("storage-status"),
    outputCard: $("output-card"), outputPath: $("output-path"), saveProgress: $("save-progress"), saveProgressBar: $("save-progress-bar"), saveStage: $("save-stage"),
    viewerEmpty: $("viewer-empty"), emptyTitle: $("empty-title"), emptyCopy: $("empty-copy"), visiblePoints: $("visible-points"), frameId: $("frame-id"), heightRange: $("height-range"), cloudRate: $("cloud-rate"),
    backendAddress: $("backend-address"), statusMessage: $("status-message"), voxelSize: $("voxel-size"), toastRegion: $("toast-region")
  };

  let viewer;
  try { viewer = new window.PointCloudViewer($("cloud-canvas")); }
  catch (error) { dom.emptyTitle.textContent="无法启动三维视图"; dom.emptyCopy.textContent=error.message; }

  const state = { socket:null, connected:false, reconnectTimer:null, previousStatus:null, cloudFrames:[], httpOnline:false, cloudPollBusy:false };
  const stateLabels = { IDLE:"待机", MAPPING:"建图中", SAVING:"保存中", SAVED:"已保存", ERROR:"异常" };

  function formatNumber(value) { return new Intl.NumberFormat("zh-CN").format(Number(value || 0)); }
  function formatDuration(seconds) { const total=Math.max(0,Math.floor(seconds||0)); const h=String(Math.floor(total/3600)).padStart(2,"0"),m=String(Math.floor(total%3600/60)).padStart(2,"0"),s=String(total%60).padStart(2,"0"); return `${h}:${m}:${s}`; }
  function setHealth(dot, text, ok, label) { dot.className=`health-dot ${ok?"ok":label?"warn":""}`; text.textContent=label || (ok?"正常":"未连接"); }

  function toast(message, kind="info") { const el=document.createElement("div"); el.className=`toast ${kind}`; el.textContent=message; dom.toastRegion.appendChild(el); setTimeout(()=>el.remove(),4200); }
  window.mappingConsoleToast=toast;

  function renderStatus(s) {
    const mode=s.state||"IDLE", mapping=mode==="MAPPING", saving=mode==="SAVING", available=state.connected&&!saving;
    const stack=s.stack||{}, stackState=stack.state||"STOPPED", nodes=stack.nodes||{};
    const stackLabels={RUNNING:"运行中",STOPPED:"未启动",PARTIAL:"部分运行",STARTING:"启动中",STOPPING:"停止中"};
    dom.stackBadge.textContent=stackLabels[stackState]||stackState; dom.stackBadge.className=`state-badge ${stackState==="RUNNING"?"mapping":stackState==="STARTING"||stackState==="STOPPING"?"saving":stackState==="PARTIAL"?"error":"idle"}`;
    const nodeLabel=(node)=>node?.online?(node.managed?"网页启动":"外部运行"):"未运行";
    setHealth(dom.driverNodeDot,dom.driverNodeStatus,!!nodes.mid360_driver?.online,nodeLabel(nodes.mid360_driver));
    setHealth(dom.lioNodeDot,dom.lioNodeStatus,!!nodes.small_point_lio?.online,nodeLabel(nodes.small_point_lio));
    setHealth(dom.tfNodeDot,dom.tfNodeStatus,!!nodes.robot_state_publisher?.online,nodeLabel(nodes.robot_state_publisher));
    dom.stackMessage.textContent=stack.message||"";
    dom.startStack.disabled=!state.connected||stackState==="RUNNING"||stackState==="STARTING"||stackState==="STOPPING";
    dom.stopStack.disabled=!state.connected||!stack.managed_count||stackState==="STOPPING"||mapping||saving;
    dom.stateBadge.textContent=stateLabels[mode]||mode; dom.stateBadge.className=`state-badge ${mode.toLowerCase()}`;
    dom.mapName.disabled=mapping||saving; dom.start.disabled=!available||mapping||!s.cloud_online; dom.stop.disabled=!state.connected||!mapping; dom.reset.disabled=!available||mode==="IDLE"||mapping;
    dom.pointCount.textContent=formatNumber(s.point_count); dom.sessionTime.textContent=formatDuration(s.elapsed_seconds);
    const b=s.bounds||{}; const width=Math.max(0,(b.max_x||0)-(b.min_x||0)),height=Math.max(0,(b.max_y||0)-(b.min_y||0));
    dom.mapArea.innerHTML=`${(width*height).toFixed(1)} <small>m²</small>`; dom.mapSize.innerHTML=`${width.toFixed(1)} × ${height.toFixed(1)} <small>m</small>`; dom.travel.innerHTML=`${Number(s.travel_distance||0).toFixed(1)} <small>m</small>`;
    setHealth(dom.lidarDot,dom.lidarStatus,!!s.cloud_online,s.cloud_online?`${Number(s.cloud_rate_hz||0).toFixed(1)} Hz`:"无数据");
    setHealth(dom.odomDot,dom.odomStatus,!!s.odom_online,s.odom_online?"正常":"无数据"); setHealth(dom.storageDot,dom.storageStatus,!!s.storage_writable,s.storage_writable?"可写":"不可写");
    dom.frameId.textContent=`坐标系 ${s.frame_id||"—"}`; dom.voxelSize.textContent=s.voxel_size?`${Number(s.voxel_size).toFixed(2)} m`:"—";
    dom.statusMessage.textContent=s.message||({MAPPING:"正在累计点云",SAVING:"正在生成地图文件",SAVED:"地图保存完成",IDLE:"可以开始新的建图会话"}[mode]||"");
    dom.saveProgress.hidden=!saving; dom.saveProgressBar.style.width=`${Math.round((s.save_progress||0)*100)}%`; dom.saveStage.textContent=s.save_stage||"正在保存…";
    if (s.output && s.output.pcd) { dom.outputCard.hidden=false; dom.outputPath.textContent=s.output.pcd; dom.outputPath.title=s.output.pcd; }
    const zmin=b.min_z,zmax=b.max_z; dom.heightRange.textContent=Number.isFinite(zmin)&&Number.isFinite(zmax)?`${Number(zmin).toFixed(1)} → ${Number(zmax).toFixed(1)} m`:"—";
    if (s.session_name && (mapping||saving||mode==="SAVED")) dom.mapName.value=s.session_name;
    if (state.previousStatus && state.previousStatus.state!==mode) {
      if (mode==="MAPPING") toast("建图会话已开始","success");
      if (mode==="SAVED") toast("地图已保存：PCD、PGM、YAML","success");
      if (mode==="ERROR") toast(s.message||"建图服务发生错误","error");
    }
    state.previousStatus=s;
  }

  function setConnected(connected, mode="") {
    state.connected=connected; dom.connectionPill.className=`connection-pill ${connected?"is-online":"is-offline"}`; dom.connectionLabel.textContent=connected?(mode==="http"?"HTTP 兼容模式":"实时连接"):"连接已断开";
    if (!connected) { dom.start.disabled=true; dom.stop.disabled=true; dom.reset.disabled=true; dom.startStack.disabled=true; dom.stopStack.disabled=true; dom.statusMessage.textContent="等待建图服务连接…"; }
  }

  function handleBinary(buffer) {
    if (buffer.byteLength<8) return; const view=new DataView(buffer); const magic=String.fromCharCode(view.getUint8(0),view.getUint8(1),view.getUint8(2),view.getUint8(3)); if(magic!=="MAP1") return;
    const count=view.getUint32(4,true), expected=8+count*12; if(buffer.byteLength!==expected) return;
    const points=new Float32Array(buffer,8,count*3); if(viewer) viewer.setPoints(points);
    dom.visiblePoints.textContent=formatNumber(count); dom.viewerEmpty.classList.toggle("is-hidden",count>0);
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
    const response=await fetch(`/api/${action}`,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({map_name:mapName})});
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
    register({name:"stop_mapping_and_save",title:"结束并保存地图",description:"结束当前建图会话，并开始生成 PCD、PGM 和 YAML 文件。",inputSchema:{type:"object",properties:{},additionalProperties:false},annotations:{readOnlyHint:false,untrustedContentHint:false},async execute(){return apiCommand("stop");}});
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
  dom.startStack.addEventListener("click",()=>send("start_stack")); dom.stopStack.addEventListener("click",()=>send("stop_stack"));
  $("fit-view-button").addEventListener("click",()=>viewer&&viewer.fit()); $("top-view-button").addEventListener("click",()=>viewer&&viewer.top());
  setInterval(()=>{dom.clock.textContent=new Date().toLocaleTimeString("zh-CN",{hour12:false});},500);
  setInterval(pollStatusOverHttp,1000); registerAgentTools(); connect(); pollStatusOverHttp();
})();
