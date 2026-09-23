# Change history

## 2026-09-23 — 轨迹实验室首次跑通：修复参数类型与海康 MVS 库遮蔽

- 症状与根因（参数类型）：隔离 `nav_executor` 启动后立刻以 code 250 中止，日志为 `parameter 'planner.smac_2d.esdf_weight' has invalid type: ... is of type {double}, setting it to {integer} is not allowed`。`backend/trajectory_lab_core.py` 的 `_parameter_arguments()` 用 `f"{value:g}"` 拼 `-p` 覆盖，而 6 个可调参数里有 4 个默认值恰好是整数（`max_velocity=3.0`、`max_acceleration=4.0`、`penalty_weight_time=100.0`、`esdf_weight=1.0`），`:g` 把它们渲染成 `3`/`4`/`100`/`1`，ROS 2 便把覆盖当成 integer，与导航端声明为 double 的参数类型冲突并直接中止执行器——也就是说**任何操作者用默认滑块点“启动真实规划”都必然失败**。新增 `_format_parameter()`：整数值补上显式小数点（沿用 `terrain_core._format_origin_value` 的既有写法），六个参数连同滑块上下限现在都以 double 形式下发。已核对导航端 `planner_params.yaml` 里这 6 项确实都写成 `1.0`/`0.33`/`0.28`/`3.0`/`4.0`/`100.0`，故“一律按 double 下发”对全部六项都正确。
- 症状与根因（动态库遮蔽）：首次真实启动隔离子进程时，`map_server` 正常加载 terrain，`nav_executor` 却以 code 127 退出：`/lib/x86_64-linux-gnu/libpcl_io.so.1.14: undefined symbol: libusb_set_option`。`/etc/profile:38` 与 `~/.bashrc:128` 把海康 MVS 的 `/opt/MVS/lib/64:/opt/MVS/lib/32` 前置进 `LD_LIBRARY_PATH`，其中自带的 `libusb-1.0.so.0` 比系统版旧、不含 `libusb_set_option`，于是遮蔽系统 libusb，使任何加载系统 libpcl_io 的进程（受管建图链与隔离轨迹实验室）都无法启动。按操作者要求**只改 `run.sh`、不动系统配置**：脚本在 source 完 ROS 工作区后，若检测到 MVS 的那份 libusb 存在且系统目录尚不在首位，就把 `/usr/lib/x86_64-linux-gnu` 置为 `LD_LIBRARY_PATH` 首项并打印一行说明。该处理幂等、在未装 MVS 的机器上完全空转，且只影响本脚本启动的后端及其受管子进程——单独运行的 MVS 客户端环境不受影响。
- 为什么给海康换系统 libusb 是安全的：两者 soname 相同（`libusb-1.0.so.0`），MVS 那份导出 83 个 API、系统那份 97 个，系统版是其**严格超集**，多出的 14 个全是 libusb 1.0.23+ 的标准 API（`libusb_set_option`、`libusb_init_context`、`libusb_set_log_cb` 等），MVS 那份唯一“独有”的只是 `_init`/`_fini`/`_edata` 一类 ELF 样板、无任何厂商私有符号。且只有 `libMvUsb3vTL.so`（USB3 传输层）链接 libusb，GigE 那套 `libMVGigEVisionSDK.so` 完全不碰它。实测：换用系统 libusb 后 MVS SDK 仍能正常加载与枚举设备（`MV_CC_EnumDevices` 返回 0），`ldd`/`LD_DEBUG=libs` 确认 MVS 的 USB3 传输层与 `nav_executor` 都解析到系统 libusb，且 `nav_executor` 带真实参数存活满 15 秒。**注意**：本机没有海康相机可接（`lsusb` 无设备），因此“枚举正常”已实测，**USB3 实际取流未验证**——若取流异常，把 `run.sh` 里那段判断去掉即可回到原状。
- 首次端到端跑通（本次为该项目首次）：从带 MVS 前置 `LD_LIBRARY_PATH` 的 shell 执行 `./run.sh --no-browser`，再经 `POST /api/trajectory/start` 启动隔离规划器，2 秒内即到达 `READY`：`managed_processes=['map_server','nav_executor']`、全局折线 728 点、MINCO 轨迹 56 点、`planning_constraints` 硬障碍 23774 格、隔离 `cmd_vel` 收到 29 条指令，执行器日志出现真实的 `Queued goal (26.44, -0.15) in map` 与 `[MincoPlanner] Minco optimization time: 0.00120568 seconds, cost: 293.582`。`POST /api/trajectory/stop` 后两个子进程均干净退出，无任何残留进程。
- 回归覆盖：`tests/test_trajectory_lab_core.py` 新增 `test_whole_number_overrides_stay_doubles`，从真实 `build_lab_commands()` 输出里取回每个 `-p` 覆盖文本，断言 6 个参数的默认值以及各自滑块上下限都不会被写成裸整数，并钉住 `esdf_weight:=1.0`、`max_velocity:=3.0`、`penalty_weight_time:=100.0`、`safe_dist:=0.33`；探测上下限时同步调整 `collision_dist`/`safe_dist`，以免触发两者的大小关系校验。已实测：把格式化改回 `:g` 后该测试立刻失败并指出 `planner.minco_optimizer.max_velocity 写成了整数`。验证：`python3 -m compileall -q backend tests`；`python3 -m unittest discover -s tests -v`（78 项）；5 个前端 `node --check`；5 个 Node harness；`bash -n run.sh`；`git diff --check` 全通过。**未进行真实浏览器视觉验收**；MVS USB3 取流、`esdf_weight` 等参数在浏览器滑块上的端到端生效仍未验证。

## 2026-09-23 — 修复轨迹验证地图列表永远为空（`has_yaml` 字段缺失）

- 症状：`data/map` 里明明存在同时具备 `<名称>.yaml` 与 `<名称>_terrain.msgpack` 的地图，但“轨迹验证”页始终显示“没有同时含 YAML 与 terrain 的地图”，整个工作区因此完全无法进入。
- 根因：`web/trajectory-lab.js` 的 `refreshMaps()` 用 `entry.has_terrain && entry.has_yaml` 过滤 `GET /api/maps`，而 `backend/terrain_core.py` 的 `list_editable_maps()` 只返回 `has_occupancy` 与 `has_terrain`——`has_yaml` 恒为 `undefined`，过滤结果永远为空。上一版新增的 Node harness 只覆盖 `TrajectoryLabCore` 的纯函数：`trajectory-lab.js` 在缺少 `canvas.getContext` 时会提前 `return`，所以这行筛选代码从未被执行过，测试因此全绿。
- 修正：`list_editable_maps()` 新增 `has_yaml`（= YAML 存在），前端筛选逻辑保持不变。没有改指 `has_occupancy`，因为轨迹实验室的后端守卫只要求 YAML + terrain，并不需要 PGM；`has_occupancy`（YAML + PGM）会反过来把一个可用的 terrain 地图藏起来。已核对全部前端消费方：`map-editor.js` 读的 `has_occupancy`/`has_terrain`/`has_pcd`/`has_frame_metadata`/`frame_revision`/`source_to_map`/`source_frame` 与 `app.js` 均无同类缺失，`has_yaml` 是唯一一处。
- 回归覆盖（防的是“前端读了后端没发的字段”这一类问题）：`tests/test_terrain_core.py` 新增 `test_map_listing_separates_has_yaml_from_has_occupancy`，构造“只有 YAML + terrain、没有 PGM”的地图，断言 `has_yaml`/`has_terrain` 为真而 `has_occupancy` 为假，补上 PGM 后再变真；`tests/trajectory_lab_harness.mjs` 新增“地图选择器读取的字段后端都提供”，从 `terrain_core.py`/`map_frame_core.py` 解析后端真实返回的键集合，与前端 `entry.*` 读取逐一比对，缺键时直接打印缺失字段名。已实测：删掉后端的 `has_yaml` 后该检查立刻失败并报出 `后端未提供：has_yaml`。
- 验证：`python3 -m compileall -q backend tests`；`python3 -m unittest discover -s tests -v`（77 项）；5 个前端 `node --check`；5 个 Node harness（`trajectory_lab_harness` 16 项）；`bash -n run.sh`；`git diff --check` 全通过。另用真实 `data/` 做端到端核对：`/api/maps` 过滤后**只剩** `lab_map_20260921_211523`（此前为 0 张），该地图成功按 `772×308 @ 0.05 m/px` 解码 terrain，并用浏览器 `cellToWorld` 的同一套数学把默认起终点栅格换算成世界坐标后，后端场景守卫返回接受。
- 顺带实测真实启动（本轮首次真正拉起隔离子进程）：隔离 `map_server` 成功启动并加载 terrain——日志 `loaded 772x308 static terrain map at 0.050 m/px, origin=(-5.237015, -7.874179)`，节点名 `mapping.trajectory_lab.trajectory_lab_map_server`，停止时按 `SIGINT` 干净退出，前后均无残留进程；隔离 `nav_executor` 以 code 127 退出，报 `/lib/x86_64-linux-gnu/libpcl_io.so.1.14: undefined symbol: libusb_set_option`。该错误与本仓库无关：`/etc/profile:38` 与 `~/.bashrc:128` 把海康 MVS 的 `/opt/MVS/lib/64` 前置进 `LD_LIBRARY_PATH`，其 `libusb-1.0.so.0` 不含 `libusb_set_option`，遮蔽了系统 libusb；直接 `ros2 run mas2027_nav_executor mas2027_nav_executor_node` 同样 code 127，即**实车导航链在当前终端环境下同样起不来**。把 `/usr/lib/x86_64-linux-gnu` 前置后 `ldd` 解析回系统 libusb、符号错误消失。本轮**未**改动 `run.sh`、后端或导航仓库去绕过它：这属于本机环境配置问题，且隔离实验室如实复现真实规划器的失败是正确行为。**未进行真实浏览器视觉验收；修复后仍未在真实规划成功的前提下验证 MINCO 轨迹输出**。

## 2026-09-23 — 新增 terrain 全局/MINCO 轨迹验证工作区

- 操作面：`web/index.html`、`web/styles.css`、`web/map-editor.js` 和新增的 `web/trajectory-lab.js` 加入第四个“轨迹验证”栏目。工作区只读导入 `data/map` 中的 MPE2 terrain `label + direction` 双通道，可在图上设置车体当前位置与目标位置，并显示**真实规划器**算出的全局搜索折线与 MINCO 轨迹。白色质点按弧长沿真实 MINCO 路径回放，可开始/暂停/重置并调整回放速度；运行中仍能用笔刷放置或擦除临时障碍，松开指针后整份障碍会送进隔离 ROGMap 并触发真实重规划。
- 真实隔离规划，而非浏览器近似：`backend/trajectory_lab_core.py`（新增，无 ROS 依赖）与 `backend/mapping_server.py` 把项目自己的 `map_server` 与 `mas2027_nav_executor` 作为受管子进程**再启动一份**，全部话题重映射到 `/mapping/trajectory_lab/*`：隔离执行器的 `cmd_vel`、`/opt_path`、terrain cost/direction、planning constraints 与 RViz 可视化话题都改到该命名空间，车体位姿、目标与 brush 障碍也只发布到隔离话题，因此不订阅也不发布实车的 `/goal_pose`、`/Odometry`、`/cloud_registered`、`/cmd_vel`。启动前逐项复核了导航仓库 `path_planner.cpp` 的 ROGMap 参数前缀（`planner.rog_map`）与 28 个 `-p` 覆盖名，确认每个覆盖都对应导航端真实声明/加载的参数，不存在“覆盖名写错→静默退回实车话题”的路径。
- 契约与安全边界：参数滑块（`safe_dist`、`collision_dist`、`max_velocity`、`max_acceleration`、`penalty_weight_time`、`esdf_weight`）只作为隔离进程的临时 ROS 参数覆盖，绝不写 `planner_params.yaml`、`node_params.yaml` 或 terrain/YAML；参数白名单同时挡住了借滑块注入任意 ROS 参数（例如把 `node.topics.cmd_vel_pub` 指回 `/cmd_vel`）的路径。场景守卫在启动前拒绝地图名路径穿越、缺少 `<名称>.yaml` 或 `<名称>_terrain.msgpack`、车体/目标落在 `label 1` 硬障碍格或越界、以及放在地图外的临时障碍。状态机为 `STOPPED → STARTING → PLANNING → READY`，受管子进程意外退出转 `ERROR`；停止时只对本项目子进程组依次 `SIGINT`/`SIGTERM`/`SIGKILL`，不做任何按名字的批量终止，后端退出时同样停掉隔离规划器，日志留在 `.ros/trajectory_lab/`。前端新增 `OBSTACLE_LIMIT = 20000`（`addObstacles()` 纯函数）与后端 `MAX_LAB_OBSTACLES` 对齐：每次笔刷都会重发整份障碍集，达到上限即停止新增并提示，避免构造出必被后端拒绝的请求体；`/api/trajectory` 轮询改为只在“轨迹验证”工作区可见时发出（切回该页立即刷新一次），与既有 ROGMap 工作区一致——运行中的实验室一次可返回上万点路径，原先的 2 Hz 常驻轮询在操作者看别的页时也在持续拉取。`README.md` 补全隔离边界、状态机、日志位置与四个新接口；`AGENTS.md` 的架构清单补齐 `trajectory_lab_core.py` 等既有模块，新增“Isolated trajectory laboratory”接口定义与生命周期约束，并把 5 个前端 `node --check`、5 个 harness 与新增后端测试纳入最简验证集。
- 回归与验证：新增 `tests/test_trajectory_lab_server.py`（16 项）覆盖此前完全无测试的新增后端代码（`backend/mapping_server.py` 净增 379 行）——场景守卫的通行/障碍格/越界/穿越名/缺 YAML/缺 terrain、旋转原点下世界→栅格与浏览器 `cellToWorld` 一致、停止时只杀自己的进程组而同名无关进程存活、无进程时停止为空操作、子进程退出上报 `ERROR`、停止状态下拒绝障碍更新，以及 HTTP 路由与 400/404/409 映射和超限请求体在到达控制器前被拒。`tests/trajectory_lab_harness.mjs` 扩充到 15 项，新增临时障碍上限与后端常量的一致性检查、轮询可见性门控，两处均已实测“改坏即失败”。全部通过：`python3 -m compileall -q backend tests`；`python3 -m unittest discover -s tests -v`（76 项）；5 个前端文件的 `node --check`；`point_cloud_viewer`、`editor_pointer`、`app_status`、`rogmap_projection`、`trajectory_lab` 5 个 Node harness；`bash -n run.sh`。另在 `ROS_DOMAIN_ID=97` / 隔离端口 18797 / 临时输出目录启动真实后端：`/api/status` 为 `IDLE`，`/api/cloud` 为合法空 `MAP1`（magic `MAP1`、点数 0），`GET /api/trajectory` 返回 `STOPPED` 且 `isolated: true`、`namespace: /mapping/trajectory_lab`、`managed_processes: []`，空转 `POST /api/trajectory/stop` 返回 200；`POST /api/trajectory/start` 对缺失地图返回 404（`找不到地图 YAML`）、对 `../escape` 返回 400（名称非法）、对注入 `node.topics.cmd_vel_pub` 的参数返回 400（`不支持的轨迹参数`）；SIGINT 后按精确 PID 退出码 0、无残留进程、临时目录已清理。另就一次关停时出现的 rclpy `wait set ... context is not valid` 回溯，用 HEAD（无轨迹实验室）与当前树在同一隔离端口各跑 3 次空转启停对比，两者均全部干净，确认它不是本轮引入的回归。**未启动隔离规划器子进程，未连接真实雷达，未验证真实 `nav_executor` 在隔离命名空间下能否成功规划出 MINCO 轨迹，也未核对实车话题上确实没有新增发布者**；未进行真实浏览器视觉验收。

## 2026-09-22 — 网页新增 ROGMap projection_layer 逐格 tunnel 诊断

- 实时数据与完整性：`backend/mapping_server.py` 用 best-effort QoS 订阅 `/rog_map/layer_type`、`/rog_map/layer_height_delta` 和 `/rog_map/occupied`，不改动导航仓库或 ROGMap 本体。新增 ROS 无关的 `backend/rogmap_debug.py`，把四分类格、占据最高点与 `height_delta` 对齐到同一滑动窗口，再按唯一 Z 体素层数重建 `vertical_occupancy_ratio=(occupied_count-1)*resolution/height_delta`。只有三路快照在 0.25 s 内对齐、且占据云同时覆盖柱内最低/最高端点时才标记 ratio 有效；可视化裁切、active-list 缺层或数据不同步时显式返回“ratio 不可用”，不生成假数值。同源 HTTP 新增 `GET /api/rogmap/projection` 的固定步长 `ROG1` 载荷，`/api/status.rogmap_projection` 回报三路新鲜度、几何、话题名和分类基线。
- 操作面：`web/index.html`、`web/styles.css`、`web/map-editor.js` 和新增的 `web/rogmap-projection.js` 加入第三个“ROGMap 分类”工作区。格网可拖动、缩放、自适应，可切换实际四类、分类原因和只突出 tunnel；点选桌子所在格后显示世界坐标、实际 `layer_type`、`height_delta`、占据高度范围、占据层数、ratio 与每条 tunnel 判定的通过/失败。右侧可本地 what-if 试调 surface/wall/tunnel 的 6 个阈值，实时重算整图与类别数量，并按 ROGMap 启动校验拦截非法参数关系；该操作明确不写 ROS 参数，真正生效仍需改 YAML 并重启。`README.md` 补充了话题、公式、可信边界与 HTTP 兼容模式说明。
- 回归与验证：`tests/test_rogmap_debug.py` 覆盖参数派生约束、墙分支优先于 tunnel、两端覆盖时 ratio 重建、缺端点/不同步时拒绝报 ratio 以及 `ROG1` 字节布局；`tests/rogmap_projection_harness.mjs` 直接加载真实前端模块，验证解码、tunnel 判定、ratio 缺失和非法参数。全部通过：`python3 -m compileall -q backend tests`；`python3 -m unittest discover -s tests -v`（57 项）；4 个前端文件的 `node --check`；4 个 Node harness；`bash -n run.sh`；`git diff --check`。另在隔离端口 18794 / `ROS_DOMAIN_ID=94` 启动真实后端：`/api/status` 正常返回离线 ROGMap 状态与 6 个基线值，`/api/cloud` 为合法空 `MAP1`，未有 ROGMap 数据时新接口正确返回 HTTP 503，随后精确停止后端并清理临时目录。未连接真实雷达/导航链，因此未用现场桌子实测 ratio 与类型一致率；未进行真实浏览器视觉验收。

## 2026-09-22 — terrain 产物对齐导航端 MessagePack ARRAY

- 根因与修正：`backend/terrain_core.py` 原先把 `terrain` / `direction` 编码为 MessagePack BIN，但当前 `/home/mas/mas_nav_2027_native` 的 `map_server::load_terrain_msgpack()` 直接通过 `via.array` 逐项读取，两者不再兼容。新写入的 terrain 现在使用 fixarray/array16/array32，0–127 使用 positive fixint，128–255 使用 uint8，可被导航端当前加载器原样读取。读取端仍保留旧 BIN 兼容，历史地图可继续打开并在下次保存时无损迁移。
- 性能与测试：元素流由 NumPy 向量化生成，避免百万网格的 Python 逐项循环。`tests/test_terrain_core.py` 覆盖 fixarray、array16、大于 127 的 uint8 元素以及历史 BIN 读取；`README.md` 明确记录与导航端 `via.array` 的格式合同。
- 验证（全通过）：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（52 项）、三个 `node --check`、`node tests/point_cloud_viewer_harness.mjs`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 和 `git diff --check`。另将现有 `lab_map_20260921_211523_terrain.msgpack` 在 `/tmp` 重编码后，用与导航仓库当前源码相同的 msgpack-cxx `via.array.ptr[j].as<uint8_t>()` 探针成功读取 `772×308`、`0.05 m/px` 及两个各 237,776 项的通道。已将该现有 terrain 原子迁移为 array32，迁移前后 metadata 及两通道 SHA-256 完全一致；旧 BIN 原文备份于 `/tmp/lab_map_20260921_211523_terrain.legacy-bin.msgpack`。在隔离 ROS domain 中按正式 launch 的系统库优先级启动已安装的真实 `map_server_node`，节点成功加载 `772×308 @ 0.05 m/px` 的迁移后 terrain；`/cost_map` 实测为 `772×308`、分辨率 `0.05`、原点 `(-5.2370148, -7.8741794)`，与同名 YAML 完全一致。同样以隔离 domain 启动真实 `odom_localizer`，成功加载同名 PCD 的 1,435,901 点并完成先验点云预处理，日志明确确认该点云是 mapping-session odom/map 且不再应用雷达外参；无 `/cloud_registered` 时只按预期等待配准。两个节点均已精确停止并清理临时 ROS 日志。未连接真实雷达，未启动完整导航链/实车，未进行真实浏览器视觉检查；本轮未改动后端传输，因此未重复 HTTP `/api/status` / `/api/cloud` 冒烟。

## 2026-09-22 — 二维投影跟随局部地面起伏

- 根因与核心修正：`backend/mapping_core.py` 原先只用一个 RANSAC 平面校正整张点云的俯仰/横滚；它比全局 Z 切片明显改善，但长走廊中的局部地面起伏、LIO 轨迹缓慢弯曲以及地面回波厚度仍可能越过默认 5 cm 障碍下限，形成大片黑色假障碍。`ground` 模式现在保留整体平面校正，再在最多 500,000 个均匀抽样点上建立 0.4 m 局部地面网格：从距离主平面不超过 6 cm 的严格种子出发，只沿相邻高度缓变单元生长，在 2 m 内以最近四个可信网格插值局部偏移；低位包络最多向回波带中心抬 4 cm。所有完整点仍用 NumPy 向量化映射到网格，原始 PCD 不变，二维预览与最终 PGM/YAML 继续共用 `slice_points_to_occupancy()`。种子和生长步长刻意低于常见 10 cm 路沿，避免宽而密的低平台被当成地面擦除。
- 可观测性与界面：`backend/mapping_server.py` 的预览响应新增 `X-Map-Ground-Local-Cells`、`X-Map-Ground-Local-Anchors` 和局部偏移最小/最大值响应头；`web/app.js` 在预览统计中显示实际参与校正的“局部地面 N 格”，`web/index.html` 与 `README.md` 同步说明整体倾斜 + 局部缓变地面的两阶段语义和保护低矮障碍的边界。`tests/test_mapping_core.py` 新增 ±16 cm 缓慢弯曲地面与 10 cm 平台回归，`tests/test_mapping_server.py` / `tests/app_status_harness.mjs` 覆盖新统计链路。
- 实测与验证：对现有 `data/pcd/lab_map.pcd`（1,435,901 点）只读运行默认配置，耗时约 0.7 秒；局部模型采用 892 个可信网格，切片点由旧算法的 404,964 降至 387,393，占据格由 57,145 降至 50,345，中部大块地面残留明显消退。`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（51 项）、三个前端 `node --check`、`node tests/point_cloud_viewer_harness.mjs`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 全通过。另在隔离端口 18796 启动当前后端：`/api/status` 正常返回 `IDLE`，`/api/cloud` 验证为 `MAP1` + 点数 0；真实 `POST /api/pcd/map-preview` 对 `lab_map` 返回 HTTP 200、合法 788×308 PGM 以及 892 个局部地面网格等新响应头，随后精确停止测试进程。项目默认 8765 端口当时没有运行中的服务。未连接真实雷达、未启动受管建图链，也未用真实浏览器对现场地图进行人工视觉验收。

## 2026-09-22 — 实时与文件点云拖动优先保证相机帧

- 根因与交互：`web/point-cloud-viewer.js` 原先在每个实时 `MAP1` 快照到达时立即遍历全部点并上传 GPU，该重工作会插入鼠标拖动的两帧之间；文件预览即使没有实时更新，拖动时每帧绘制最多 220,000 点也会在较弱 GPU 上拉低帧率。现在拖动期间保持当前 GPU 点缓冲不变，到达的多帧实时点云只保留最新一帧，松手后再统一上传；相机帧绘制时用 WebGL attribute stride 对大于 90,000 的点集做等间隔临时降采样，不分配第二份点云、不改磁盘或完整预览数据，松手立即恢复全密度。
- 手感与修正：指针合并事件现在以最新坐标直接驱动下一个动画帧；旋转方向和按键分工对齐 RViz 2 官方 `OrbitViewController`：左键拖动旋转（`yaw -= dx×0.005`、`pitch += dy×0.005`），中键或 `Shift+左键` 平移焦点，右键向上/向下拖动放大/缩小，滚轮缩放。平移根据当前距离、视场角与画布高度在相机平面内换算；滚轮统一了像素/行/页三种 `deltaMode`；WebGL shader 的 attribute/uniform 位置改为初始化时缓存。同时修正原“俯视”将 pitch 设为接近水平的反向错误，现在是真正从 +Z 上方看向地图。`web/index.html` 的底部手势提示已同步。
- 回归覆盖：新增 `tests/point_cloud_viewer_harness.mjs`，用真实查看器和 Canvas/WebGL 桩验证拖动中不上传新点云、两帧到达合并为最新帧、120,000 点在拖动时等间隔绘制 60,000 点、相机在下一动画帧更新、RViz 旋转方向、中键平移、右键向上拖动放大、正确俯视与滚轮归一化。`AGENTS.md` 与 `README.md` 已将它纳入无浏览器验证流程。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（48 项）、三个前端 `node --check`、`node tests/point_cloud_viewer_harness.mjs`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 与 `git diff --check` 全通过。未连接真实雷达，未启动 ROS 建图链，未使用真实浏览器/GPU 对超大 PCD 的实际帧率与手感做主观验收。

## 2026-09-22 — 二维地图编辑器叠加坐标对齐的俯视点云

- 操作与显示：`web/index.html` / `web/styles.css` / `web/map-editor.js` 在地图画布标题栏新增“点云 P”开关，鼠标点击或非输入框聚焦时按 `P` 可显示/隐藏。叠加层复用同名 PCD 的 `GET /api/pcd/preview` `MAP1` 载荷，按 `/api/status` 当前的 `height_mode` / `z_min` / `z_max` 先取障碍高度切片，再仅将 XY 投影为半透明青色点层；不增加三维视角，不写入或改动 PCD。
- 坐标与数据安全：每个点使用 YAML 原点、分辨率和 origin yaw 转到 PGM 栅格，并在离屏画布中预先栅格化，因此缩放、平移和窗口变尺寸时不会与地图错位，也避免每帧重画数十万个点。切换地图会丢弃旧缓存；重新定义 map 坐标系后会重新读取已成组变换的 PCD；并发请求带序号守卫，迟到的旧地图响应不会覆盖新图。`README.md` 补充了用青色点层复核黑色障碍格的操作方法。
- 测试：`tests/editor_pointer_harness.mjs` 新增 `MAP1` 点云桩，验证后端默认高度窗口传入预览请求、一个已知世界 XY 点精确落在预期的 `(10, 20)` 栅格，以及 `P` 快捷键关闭叠加层。验证通过：`python3 -m compileall -q backend tests`；`python3 -m unittest discover -s tests -v`（48 项）；三个 `node --check`；`node tests/editor_pointer_harness.mjs`；`node tests/app_status_harness.mjs`；`bash -n run.sh`；`git diff --check`。未连接真实雷达，未启动 ROS 建图链，也未用真实浏览器对大型 PCD 进行视觉/帧率验收。

## 2026-09-22 — 本地 small_point_lio 覆盖工作区纳入版本管理

- 背景：`e4f1be4` 已把受管 LIO 改为启动项目内 `ros2_ws/install/small_point_lio` 的可执行文件，`run.sh` 也会 source `ros2_ws/install/setup.bash`，但 `ros2_ws/src/small_point_lio` 与 `ros2_ws/README.md` 当时漏了 `git add`，一直处于未跟踪状态；只克隆本仓库时本地覆盖工作区缺源码，无法按 `ros2_ws/README.md` 重建。
- 变更：把 `ros2_ws/README.md` 与 `ros2_ws/src/small_point_lio`（47 个文件：C++ 源码、launch、包内示例配置、MIT 许可证、ankerl/liblzf 第三方源码与 CI 文件，约 440 KiB）提交入库，内容与上游复制时逐字节一致，本次未做任何修改。`.gitignore` 新增 `ros2_ws/.cache/`（clangd 索引缓存）、`.vscode/`（只含本机绝对路径的编辑器设置）和 `data/pcd/*.mapping-report.json`；后者是建图产出的审计报告，按“生成物只留在 `data/`”的约定不入库。构建产物 `ros2_ws/build`、`install`、`log` 的忽略规则不变。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests`（48 项）、`node --check` 三个前端文件、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 与 `git diff --check` 全通过。提交前用 `git status --ignored` 与 `git add -An` 核对暂存清单，确认没有 `ros2_ws/build|install|log|.cache` 进入索引，且本次未改动任何运行时代码。未重新执行 `colcon build`，未连接真实雷达，未启动 ROS 建图链或浏览器验证。

## 2026-09-21 — 二维投影新增 ROGMap 风格的结构体素杂点滤波

- 核心行为：`backend/mapping_core.py` 新增离线结构体素分类。切片点先按可配置的三维体素去重，再按 XY 列分类：同列占有至少两个 Z 体素的垂直结构保留，有任一八邻域占据列支撑的水平结构也保留，只删除没有垂直或水平支撑的孤立单体素列。分类使用唯一占据体素而非原始点数，避免近处高密度单体素被误当成结构，同时保护细墙、路沿和杆状障碍。该策略借鉴 ROGMap 投影层的体素列/八邻域思路；因最终 PCD 不含逐射线 hit/miss 时序，没有冒充完整的 `OCCUPIED / KNOWN_FREE / UNKNOWN` 概率分类。
- 参数、传输与界面：`MapExportConfig` 新增 `filter_mode`（`voxel`/`radius`/`none`）与 `filter_voxel_size`（0.02–1.0 m）；`backend/mapping_server.py` 默认使用 `voxel` + 0.10 m，新增 `--map-filter-mode` / `--map-filter-voxel-size`，并通过 `status.map_export`、预览/转换请求与 `X-Map-Filter-*` 响应头传递模式、删除点数及体素列统计。`web/index.html` / `web/app.js` 增加“结构体素（推荐）/半径邻域（兼容）/不滤波”选择和体素尺寸，自动禁用非当前模式的参数，二维预览统计直接显示去除点数；半径滤波和完全关闭均保留为可回退模式。预览和最终 PGM/YAML 继续共用同一栅格化函数。`README.md` 同步算法边界、API 和启动参数。
- 测试：`tests/test_mapping_core.py` 覆盖“同一孤立体素内多点仍删除”、水平相邻单体素保留、孤立垂直多 Z 体素保留、输入不被修改、元数据和预览/写盘像素一致；`tests/test_mapping_server.py` 覆盖 override 校验与响应头；`tests/app_status_harness.mjs` 覆盖默认值、模式联动禁用、预览/转换请求与删除统计显示。
- 验证（全通过）：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（48 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 和 `git diff --check`。未连接真实雷达，未启动 ROS 建图链，也未用真实浏览器对大型 PCD 做视觉/性能验收；鉴于本轮机器曾因无关的 `gevfilter` 内核模块页错误重启，本次只运行了有界的纯单元/无头前端验证，未触发硬件链路或整张 `lab_map.pcd` 预览。

## 2026-09-21 — 二维投影按拟合地面校正高度

- 根因与行为变更：原有二维转换只能用全图统一的绝对 Z 窗口，LIO 输出有轻微俯仰/横滚偏差时，远处地面会进入障碍切片并在 PGM 中变黑。现在默认使用 `ground` 高度基准：在 0.5 m XY 网格内取低位高度样本，通过确定性 RANSAC 拟合主地面 `z=ax+by+c`，然后用每个完整 PCD 点相对该平面的高度做切片。拟合阶段最多均匀取 500,000 点以约束多百万点会话的峰值内存，平面仍应用到全部点；原 PCD 不会被变换或改写。半径滤波同时从 SciPy `workers=-1` 改为单 worker + 50,000 点分块，不改变精确邻域判定，但避免一次预览占满全部 CPU 而令本地/远程操作面假死。`absolute` 模式保留旧的全局 Z 语义，拟合样本不足、无稳定平面或地面倾斜超过 20° 时会明确提示切换。核心实现位于 `backend/mapping_core.py`。
- 后端与传输：`backend/mapping_server.py` 新增 `--height-mode ground|absolute`（默认 `ground`），`status.map_export`、`POST /api/pcd/map-preview`、`POST /api/pcd/convert` 和 `GET /api/pcd/preview` 都传递并校验同一 `height_mode`，使三维切片预览、二维 PGM 预览和最终写盘使用同一选点结果。预览响应新增 `X-Map-Height-Mode`、`X-Map-Ground-A/B/C`、`X-Map-Ground-Tilt` 和拟合内点网格数，便于现场审核。
- 操作界面与文档：`web/index.html` / `web/styles.css` / `web/app.js` 在「点云转二维图」增加「自动地面（推荐）/全局 Z（兼容）」选择，把 Z 上下限明确标为障碍高度，二维预览统计显示实际拟合倾角；操作者改动依然不会被状态轮询覆盖。页面内智能体工具 schema 与 `README.md` 同步新参数、响应头和相对高度语义。
- 测试：`tests/test_mapping_core.py` 新增合成 4.88° 倾斜地面 + 墙体回归，确认自动模式删掉全部地面但逐点保留墙体，并覆盖非法模式和 120,001 点分块时始终只请求一个 cKDTree worker；`tests/test_mapping_server.py` 覆盖 API override 的允许值/拒绝值；`tests/app_status_harness.mjs` 覆盖后端默认值、预览/转换请求、三维切片 URL 与倾角统计显示。对实际 `data/pcd/lab_map.pcd`（1,435,901 点）做了只读分析：拟合主地面倾斜 2.21°，旧绝对 Z 切片 360,416 点中有 44,374 点（12.3%）落在拟合地面的 `-0.10–0.05 m` 范围，与远处地面变黑现象一致。
- 验证（全通过）：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（43 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 与 `git diff --check`。未启动 ROS/雷达/真实建图会话，未进行真实浏览器视觉检查或 HTTP 端到端冒烟；本轮机器反复重启已另行定位为开机自动加载的海康 MVS `gevfilter` 树外内核模块页错误，因此未再启动任何硬件链路。

## 2026-09-21 — 异常 HTTP 请求不再使错误响应崩溃

- `backend/mapping_server.py`：修复请求行格式错误或不完整时的二次异常。Python HTTP 处理器可能在 `parse_request()` 尚未设置 `path` 时就调用 `send_error()` / `end_headers()`，现在安全读取该属性，并为这类错误响应照常添加安全与禁缓存响应头；正常 `/ws` 升级路径的行为不变。
- `tests/test_mapping_server.py`：新增未建立 `path` 的处理器错误响应回归，直接覆盖本次回溯中的失败阶段并核对三个响应头。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（39 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 与 `git diff --check` 全通过。另在临时本机端口向真实 `ThreadingHTTPServer` 发送 65,537 字节的超长请求行：服务器正常返回 `HTTP/1.0 414 Request-URI Too Long`，包含安全响应头，且 stderr 为空。未连接真实雷达，未运行 ROS 建图会话，未进行真实浏览器交互检查。

## 2026-09-21 — 保存阶段加入保守的 ERASOR 风格区域去动态

- `backend/mapping_core.py`：新增 `BinDisappearanceConfig` 驱动的极坐标“环 × 扇区”消失投票接入函数。它先保留现有关键帧射线 free/hit 回放的结果，再比较当前关键帧扫描与累计地图在同一极坐标格中的竖直结构跨度；只有地图高结构已消失、至少达到最少负票且负票多于确认静态的正票时，才提出删除。两条路径的合计删除比例再次受既有全图安全上限约束；若区域票会超限，只回退区域票而保留已安全通过的射线清理。新增原子 `<名称>.mapping-report.json`，记录输入/输出点数、坐标系、体素大小与射线/区域票的完整统计，供现场审计但不改变 PCD 数据格式。
- `backend/mapping_server.py`：保存线程改为调用组合式离线清理；新参数 `--polar-disappearance-filter`（默认开启）/`--no-polar-disappearance-filter` 与距离范围、环/扇区、格尺寸、高度比例、投票数和区域删除上限参数可调，均在启动前校验。`/api/status` 新增 `polar_disappearance_filter` 配置，已有 `keyframe_history.offline_filter` 现在发布总结果及 `ray`、`polar` 分项；网页状态提示会显示保存阶段的射线/区域删除数量。
- `README.md`：补充组合式清理的执行顺序、总安全回退语义、审计报告位置和参数表。
- 测试：`tests/test_mapping_core.py` 新增三项纯 NumPy/临时关键帧历史回归：多帧确认的高处动态结构被极坐标票删除而低矮地面和独立静态结构保留、合计删除超过总安全上限时完整回退区域票、JSON 报告原子写入且不遗留临时文件。
- 验证：`source /opt/ros/jazzy/setup.bash && python3 backend/mapping_server.py --help` 确认极坐标参数可见；`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（38 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`、`bash -n run.sh` 与 `git diff --check` 全通过。另在隔离端口 18795、`ROS_DOMAIN_ID=96`、临时输出目录启动后端：`/api/status` 返回默认开启的 `polar_disappearance_filter` 全量配置，`/api/cloud` 的 8 字节空云仍是合法 `MAP1` + 点数 0；已用精确 PID 停止后端并删除临时目录。未连接真实雷达、未运行实际 ROS 建图并保存会话或浏览器视觉交互验证；新默认阈值尚需用真实行人/推车场景进行保守调参。

## 2026-09-21 — 点云保存与二维图生成解耦，改为可调参数的自助切片

- 行为变更（用户要求）：`结束并保存` 现在**只**写三维 PCD（`data/pcd/<名称>.pcd`），不再顺带生成 PGM/YAML。二维图变成独立的、可反复重来的自助步骤：在 `data/pcd` 里选点云 → 调切片参数 → 先看二维预览 → 确认后再生成 PGM/YAML。ROS 服务 `/mapping/stop_and_save` 走同一个会话方法，因此命令行调用同样只落 PCD。
- `backend/mapping_core.py`：把原 `write_occupancy_map()` 里的“切片 + 半径滤波 + 栅格化”抽成纯函数 `slice_points_to_occupancy()`，让浏览器预览与磁盘导出共用同一份计算，二者不可能算出不同栅格；新增 `encode_pgm()`（唯一 P5 编码器）、`pool_occupancy_pixels()`（整数倍 block-min 降采样，细墙不会在预览里消失）、`build_pcd_map_preview()`（只读地生成预览载荷，名称先经地图名校验并钉在 `data/pcd` 内）、`slice_by_height()`（三维预览的高度窗口）与 `export_pcd_session()`（只写 PCD）。`export_session()` 随之移除，`write_occupancy_map()` 保留给转换路径。栅格上限与预览上限提为 `MAX_OCCUPANCY_CELLS`、`PREVIEW_MAX_PIXELS` 常量。
- `backend/mapping_server.py`：`begin_save()` 去掉 `z_max` 参数与保存期的 `_export_z_max` 状态，`_save_worker()` 只调用 `export_pcd_session()`，`dispatch()` 也不再携带 `z_max`；`status.map_export` 改为发布完整的六个切片默认值（resolution/z_min/z_max/radius/min_neighbors/padding），供网页预填。新增 `export_config_from_overrides()`：逐项校验操作者传入的切片参数（数值、整数、有限、`z_min<z_max`、非负），错误直接给出中文字段名，空字段表示沿用后端默认值。新增 `POST /api/pcd/map-preview`：只读返回 PGM(P5) 预览与 `X-Map-*` 元数据，不写任何文件，与转换共用 `MAPPING`/`SAVING` 状态守卫（返回 409），切片刻意在文件锁之外进行以免大点云阻塞编辑器保存。`POST /api/pcd/convert` 接受同一套参数与可选 `output_name`；`GET /api/pcd/preview` 新增 `z_min`/`z_max` 查询参数，只过滤传输帧、不动磁盘文件。
- 同名转换命名修正（冒烟测试中发现）：保存时会写下同名 `<名称>_frame.json` 身份坐标系记录，原先它被当作“名字已占用”，导致刚保存的 PCD 转二维图只能拿到带时间戳的名字，破坏“同名 PCD + PGM/YAML”成组约定。现在 `_available_*_conversion_name()` 只把 PCD/PGM/YAML/terrain 视为占用，并在坐标系记录**确实带操作者对正**（x/y/z/yaw 非零）时才避让，避免覆盖已对正的坐标系。
- `web/index.html`、`web/app.js`、`web/styles.css`：“建图会话”卡片移除“二维投影高度上限”滑块，按钮副标题改为“只写入三维 PCD”，并明确提示二维图在下方按需生成；“最近保存”卡片只列 PCD。原“已有 PCD 转二维图”卡片重写为“点云转二维图”：PCD 列表 + 六个参数输入 + 输出地图名 + 「恢复默认值」「预览二维切片」「在三维预览中查看当前切片」→「生成 PGM · YAML」。预览画布解码 P5 后按占据/空闲着色，支持滚轮缩放、拖动平移、双击复位；参数一旦被操作者改动就不再被每秒状态轮询覆盖。三维预览按钮会把当前 Z 窗口作为查询参数传给 `/api/pcd/preview`，直接看清切片切到了什么。智能体工具同步更新：`stop_mapping_and_save` 改为只保存点云，`convert_existing_pcd_to_pgm` 接受全部切片参数与 `output_name`，新增只读的 `preview_pcd_occupancy_map`。
- 测试：`tests/test_mapping_core.py` 把导出测试改为 `test_session_export_writes_only_the_pcd`（断言不再产生 PGM/YAML），新增切片窗口过滤不修改入参、预览与导出的栅格逐像素一致且预览不落盘、block-min 降采样不丢细墙、自定义切片参数进入 YAML 共 4 项，共 30 项。`tests/app_status_harness.mjs` 增加第 7–10 组共 11 项检查：默认参数来自 `status.map_export`、操作者改动与「恢复默认值」、PGM 预览解码与统计行、转换请求体、三维预览的 Z 过滤、以及保存指令不再携带 `z_max`（合计 33 项）。
- 验证（全通过）：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests`（30 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`node tests/app_status_harness.mjs`（33 项）、`bash -n run.sh`、`git diff --check`。另在隔离端口 18799 + `ROS_DOMAIN_ID=88` + 临时输出目录做端到端冒烟（脚本 `/tmp/mw_smoke_run.sh`）：`/api/status` 的 `map_export` 六字段正确；自定义参数（Z 0.3–1.0、分辨率 0.1、无滤波）预览出 123×83 栅格、405 个占据格，且 `data/map` 仍为空；非法参数分别返回 400（分辨率非数值、最小邻点非整数、`z_min≥z_max`、路径穿越名）与 404（文件缺失）；转换生成的 PGM 与预览逐像素一致、YAML 写对分辨率与原点、源 PCD 的 MD5 未变；同名二次转换自动加时间戳；`/api/stop` 在待机时返回 409；带 Z 窗口的三维预览帧为 6,522/11,660 点；用合成点云跑完整会话累计 15,439 点后保存，落盘**只有** `smoke_2d_flow.pcd` 与坐标系记录（无 PGM/YAML），随后转换该 PCD 得到同名的 `smoke_2d_flow.pgm/.yaml`；最后 reset 回 `IDLE` 并关停隔离后端，仓库 `data/` 未被改动。
- 未验证：真实浏览器里的视觉与交互检查（预览画布缩放/平移手感、移动端断点、参数网格排版）、真实雷达数据下的切片效果，以及 HTTP 兼容模式下的实际页面操作（接口本身是普通同源 HTTP，冒烟测试已覆盖）。

## 2026-09-21 — 保存后地图名称输入框不再被状态刷新覆盖

- 症状：一次建图保存完成后，不关网页想再建一张并改用新名字，输入框里的名字改不动，总是跳回上一张已保存的名称。
- 根因：`web/app.js` 的 `renderStatus()` 在 `MAPPING`/`SAVING`/`SAVED` 三种状态下都执行 `dom.mapName.value=s.session_name`，而状态每秒经 HTTP 轮询与 WebSocket 推送各渲染一次；`SAVED` 时后端回报的 `session_name` 已是带时间戳的落盘名，于是操作者的每次输入都在下一次渲染被回写。后端 `start_session()` 本来就允许从 `SAVED` 直接开启新会话，问题只在前端。
- 修复：`renderStatus()` 只在发现**新的** `session_name` 时才同步一次（新增 `state.sessionName` 记录已同步过的名称），同步之后输入框交还操作者编辑；`MAPPING`/`SAVING` 期间输入框本就禁用，仍显示会话名。重置不清空操作者已输入的名字，方便直接改名复用。
- 新增 `tests/app_status_harness.mjs`：用 DOM、WebSocket、fetch 桩加载真实 `web/app.js`，经 HTTP 轮询与 WebSocket 推送两条路径回放 `IDLE → MAPPING → SAVED → 再次建图 → 再次保存 → 重置` 的完整状态序列，共 14 项检查，重点覆盖“保存后改名不被后续渲染覆盖”。`AGENTS.md` 的最简验证集补上该命令并说明用途。
- 验证：新 harness 对修复后的代码 14 项全过；把修复的那一行回退成原实现后重跑同一个 harness，恰好 5 项失败且失败详情就是原 bug 现象（值停留在 `venue_map_20260921_184409`），证明该测试确实能抓住此回归。另有 `python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests`（26 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`bash -n run.sh`、`git diff --check` 全部通过。
- 未验证：真实浏览器里手动确认（本次只改前端，刷新页面即生效，后端无需重启）；未重跑真实雷达建图流程。

## 2026-09-21 — 实时建图卡片内的文件夹点云预览

- `backend/mapping_server.py` 新增 `GET /api/pcd/preview?name=<名称>`：把 `data/pcd/<名称>.pcd` 按 `--web-max-points` 抽稀后，用与实时点云完全相同的 `MAP1` 帧返回，并用 `X-Preview-Points`、`X-Preview-Total` 响应头分别给出显示点数和原始点数；`_send_bytes` 增加可选 `extra_headers`。名称非法或路径穿越返回 400，文件不存在返回 404，磁盘文件只读不写。
- `backend/mapping_core.py` 新增三个可脱离 ROS 测试的纯函数：`decimate_points()` 等间隔抽稀、`build_cloud_frame()` 统一打包 `MAP1` 帧并返回 `(帧, 显示点数, 原始点数)`、`resolve_pcd_input()` 校验名称并把路径钉在 `data/pcd` 内。三处重复的 `MAP1` 打包（WebSocket 广播、`/api/cloud`、新预览接口）合并到 `build_cloud_frame()`；`VoxelAccumulator.snapshot()` 改用 `decimate_points()`，抽稀语义与原先完全一致。预览读取刻意放在地图文件锁之外，避免解析大 PCD 时阻塞保存；导出是原子替换，因此读到的必然是完整的旧版或新版文件。
- `web/index.html`、`web/app.js`、`web/styles.css`：累计点云卡片新增“点云预览”开关，打开后卡片纵向分成实时视图与预览视图两块，两套相机互不影响，实时建图与 HTTP 轮询照常运行。预览 WebGL 视图在首次展开时才创建，下拉框与“已有 PCD 转二维图”共用一份 `/api/pcd-files` 列表，状态行显示“原始点数 → 抽稀后点数”。`web/point-cloud-viewer.js` 新增公开 `resize()`，用于面板展开后重算画布尺寸（原先只在窗口 resize 时重算）。实时与预览共用新的 `parseCloudFrame()`，实时路径行为未变。
- `tests/test_mapping_core.py` 新增 4 项回归：帧头与抽稀步长且不修改入参数组、未超上限时逐点保真、`data/pcd` 内路径解析拒绝穿越/空名/缺失文件、预览载荷符合浏览器协议。`README.md` 新增“文件夹点云预览”一节。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests`（26 项，新增 4 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`bash -n run.sh`、`git diff --check` 全部通过；另用脚本核对 `app.js` 里 67 个 `$()` 元素引用在 `index.html` 中全部存在。
- 在隔离端口 18790（`ROS_DOMAIN_ID=77`、临时输出目录）做端到端验证：92,203 点文件原样返回（`X-Preview-Total = X-Preview-Points = 92203`，帧长 1,106,444 字节自洽）；1,600,389 点文件抽稀到 200,049 点（低于 220,000 上限）、帧长 2.4 MB；`../outside`、`..`、`.`、`a b`、`/etc/passwd`、`sub/file`、`../../etc/passwd` 全部 400，缺失文件 404，空名称 400；预览前后源文件 MD5 不变；`/api/status`、`/api/cloud`（空累积器 8 字节帧）、`/api/pcd-files`、首页均 200。隔离后端与临时目录已清理。
- 未验证：真实浏览器里的视觉与交互检查（面板折叠布局、双画布相机、移动端断点）、真实雷达数据预览、百万点级 PCD 在前端的实际帧率。
- 随后发现生产后端与受管建图链路已自行退出（机器处于空场），因此在 8765 上用真实地图文件补做了上线验证：启动日志同时打印「本机打开 http://127.0.0.1:8765」与「同一网络的其他电脑请打开 http://192.168.77.141:8765」；`data/pcd/venue_map_20260921_184409.pcd`（92,203 点）返回 1,106,444 字节自洽帧，`lab_map.pcd`（1,600,389 点）抽稀为 200,049 点；`../outside`、`../../etc/passwd`、`a b` 均 400，缺失文件 404；`/api/status`、`/api/pcd-files` 与局域网地址首页均 200。重启发生在无建图会话、无受管节点运行时，没有中断任何建图。

## 2026-09-21 — 启动时给出可从其他电脑打开的局域网网址

- `backend/mapping_server.py` 新增 `ssh_peer_address()` 与 `console_urls()`：绑定通配地址时，除 `http://127.0.0.1:<port>` 外，再用 `SSH_CONNECTION` 的对端地址做一次 UDP connect，由内核选出本机在该网络中的源地址（不发送任何报文），从而打印 `http://192.168.77.141:8765` 这类真正可从外部浏览器打开的地址。直接查询默认路由会命中本机的 FlClash 代理网卡（198.18.0.1），因此没有采用该做法。启动日志由单行 `Open http://127.0.0.1:8765` 改为「本机打开 …」加「同一网络的其他电脑请打开 …」。
- `run.sh` 用同一规则（`ip route get $SSH_CONNECTION 对端` 取 `src`）计算 `LAN_URL`，在 `--no-browser`、自动打开失败和网页就绪三条分支上通过 `print_access_hint` 额外打印局域网地址，并提示复制的 `127.0.0.1` 只有本机能打开。自动打开浏览器的行为保持不变。
- 起因：在 VS Code Remote SSH 下复制启动日志里的 `http://127.0.0.1:8765` 到另一台电脑的浏览器会指向那台电脑自己，因而打不开；只有本机地址会被复制成不可用网址。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests`（22 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`、`bash -n run.sh`、`git diff --check` 全部通过。在 `SSH_CONNECTION=192.168.77.150 …` 的当前会话中直接调用新函数，通配绑定返回 `['http://127.0.0.1:8765', 'http://192.168.77.141:8765']`，显式 `127.0.0.1` 与 `::1` 分别返回单条正确地址。改动只在下次启动生效：当时后端正处于 `MAPPING` 会话（`venue_map`，114,253 点），为避免打断建图未重启进程，因此未做启动日志的端到端复验（真实浏览器打开局域网地址由操作者自行确认）。该次重启随后在机器空场时完成，两行启动提示与局域网地址首页 200 均已实测通过。

## 2026-09-21 — 有界关键帧历史与保存时离线去动态

- `backend/mapping_server.py` 新增关键帧历史记录器：平移 0.10 m、旋转 5° 或静止 0.75 s 触发，以 0.20 s 最小间隔抑制抖动。ROS 回调只向默认深度 4 的有界队列提交数据；后台线程保留每角度格最近回波，按最多 64 帧一块原子写入 `data/session_history` 中的版本化二进制 chunk。单会话默认硬上限 2 GiB，队列拥塞或到达限额不会阻塞点云回调或中断建图。
- `backend/mapping_core.py` 新增关键帧 chunk 原子读写、流式迭代和全历史 DDA 过滤。保存时一次只读一帧，对最终占据体素统计分帧去重的 free/hit 证据；默认需要至少 5 个空闲帧且 free 不少于 hit 的 2 倍才删除。候选删除超过全图 35% 时自动回退原点云；离线清理后会同步更新累积器与网页点云，PCD/PGM/YAML 使用同一份清理结果。
- 成功导出后默认删除临时历史，`--keep-keyframe-history` 可保留可重放的 `.history` 目录；导出失败或建图中后端退出会保留未完成历史。`/mapping/status` 和 `web/app.js` 显示已写关键帧、磁盘字节数、丢帧/异常与离线删除结果；`.gitignore` 忽略生成的历史目录，`README.md` 记录数据语义、安全限额、恢复行为和全部调参项。
- 新增关键帧二进制往返、“动态物体最终停留位置”利用早期自由空间证据清理、以及离线结果替换累积器的回归覆盖。验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（22 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`（11 项）、`bash -n run.sh` 和 `git diff --check` 全部通过；另直接验证保留历史的非隐藏 `.history` 目录可重放，以及小磁盘限额下无阻塞拒绝入队。
- 在隔离端口 18785/18786 和临时输出目录运行两轮合成 ROS 点云端到端验证：10 个关键帧原子写成 4 个 chunk（480,408 字节），保存时流式回放删除 9,820 个候选拖影点。最终 PCD、`/api/status` 和 `GET /api/cloud` 的 `MAP1` 点数一致为 36,795，PCD/PGM/YAML/frame 均生成；保留模式的 chunk 可读，默认模式的临时历史在成功导出后已删除。隔离后端和临时产物已清理；未连接真实雷达，未进行行人移动实景对比或真实浏览器视觉检查。

## 2026-09-21 — 建图累积层动态拖影清理

- 按 `nav_opensource/HW/HWSentryNav26/utils/cpp/offline_mapping_optimizer/src/raycasting_filter.cpp` 复核并替换自由空间遍历内核：`backend/mapping_core.py` 现在用向量化 Amanatides–Woo 3D-DDA 逐个体素边界前进，与 HW 一样排除回波终点体素并保持相同的边界并列判定顺序。在线路径保留每角度格最近回波、同帧去重、命中清零 miss 和 5 个不同处理帧确认，没有把 HW 适用于位姿优化后离线点云的激进阈值 2 直接套到实时建图。
- `backend/mapping_core.py` 将原本只增不减的体素并集扩展为可逆累积器：当前帧命中会重置 miss，同一体素在默认 5 个不同后续帧中被射线观测为空后才主动删除，未观测体素永不衰减。删除槽位会重用，保持 `--max-points` 硬上限且不累积无界历史存储；`snapshot()`、点数和 bounds 只反映当前有效体素。
- 新增保守的角度分箱可见性射线：每个角度格只取最近回波，射线不把终点所在体素计为空闲，且当前帧任何命中都优先于空闲证据。`backend/mapping_server.py` 按时间戳从有界 odom 队列选择射线原点，应用项目 URDF 的 `base_link -> lidar_link` 平移，并在独立的最新帧后台线程中默认以 3 Hz、4000 条射线、0.38–10 m 运行，不阻塞 ROS 点云回调。会话 epoch 保证上一会话的延迟任务不会删除新会话点云。
- 去动态默认开启，可用 `--no-dynamic-removal` 关闭；新增 miss 阈值、射线距离/频率/数量/角分辨率、odom 时间容差和传感器原点偏移参数。`/mapping/status` 回报处理帧数、清理点数、候选体素、耗时和异常；`web/index.html` / `web/app.js` 在“数据链路”显示累计清理点数。`README.md` 记录算法语义、调参方式和“最后停留位置/无时序单 PCD 无法清理”的边界；导航主项目未修改。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（20 项，含 HW DDA 斜向边界穿越/终点排除、拖影删除、同帧 miss 去重、hit 重置、未观测保留、最近表面遮挡保护、bounds/槽位重用与并发快照）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`（11 项）、`bash -n run.sh` 和 `git diff --check` 全部通过。500 条随机射线与逐射线标量 HW DDA oracle 的体素集合完全一致；30000 个合成回波/4000 条射线的向量化 DDA 实测约 78–80 ms。
- 在隔离端口 18784 和临时输出目录启动后端，`GET /api/status` 确认去动态默认开启且统计字段完整，`GET /api/cloud` 验证为 8 字节空云帧（`MAP1` + point count 0）。隔离后端和临时产物已清理；本次 DDA 内核替换未重跑真实雷达、行人移动前后对比或真实浏览器视觉检查。
- 在隔离端口 18782/18783 和临时输出目录完成 HTTP/合成 ROS 端到端验证：`/api/status` 回报默认开启与全部统计，合成发布器的云后 odom 间隔约 0.13 s，因此该测试单独使用 `--dynamic-odom-tolerance 0.20`（生产默认仍为 0.12 s）；后台线程实际处理 3 帧、489236 个候选体素，最后一帧约 69.4 ms、无异常。12 帧合成数据只提供 3 个去动态处理帧，未达 5 帧阈值时预期删除 0 点。`GET /api/cloud` 保持 `MAP1` 头和正确长度，会话均保存到 `SAVED`，PCD/PGM/YAML/frame 均产生；隔离后端和临时产物已清理。
- 实雷达验证尝试未能进入建图：项目内 `small_point_lio` 和 `robot_state_publisher` 成功上线，但 `mid360_driver` 立即因 `bind: Cannot assign requested address` 退出，因而没有 `/cloud_registered`/`/Odometry` 数据，无法做行人移动前后的实景对比。两个受管节点已通过 `stop_stack` 停止，后端、临时目录均已清理，最终 ROS 图无节点且没有活动会话。未进行真实浏览器视觉检查。

## 2026-09-21 — 内置并优先启动项目 Small Point-LIO

- 将 `/home/mas/mas_nav_2027_native/src/mas2027_perception/Odometry/small_point_lio` 的完整包原样复制到 `ros2_ws/src/small_point_lio`：共 47 个文件、约 440 KiB，包含 C++ 源码、launch、包内示例配置、MIT 许可证、ankerl/liblzf 第三方源码与 CI 文件；源/目标 `diff -qr` 一致，没有符号链接或生成物。导航主项目只读取，未修改任何文件。
- `run.sh` 保留 ROS Jazzy 和导航 underlay，在本地 overlay 存在时最后 source `ros2_ws/install/setup.bash`；`backend/mapping_server.py` 的网页受管 LIO 命令改为直接启动项目内 `small_point_lio_node`，不再通过 `ros2 run` 回退到导航工作区的同名包。后端状态新增本地源码/可执行路径、`ready` 与 `stale` 标记；未构建或 C/C++ 源码比可执行文件新时拒绝启动并提示重建。网页对受管 LIO 显示“项目内启动”。
- 新增 `ros2_ws/README.md`，并更新根 `README.md`、`.gitignore`：说明使用导航安装空间作为只读 underlay 构建本地包、源码修改后的重建流程、点云预处理/估计器/已配准输出的集成点，以及忽略 `ros2_ws/build`、`install`、`log`。`run.sh` 仍不自动构建，LIO 未构建时 HTTP 后端仍可正常启动。
- 验证：本地 `colcon build --symlink-install --packages-select small_point_lio` 成功（1 包，约 31 s），`ros2 pkg prefix small_point_lio` 指向本项目 install，直接启动节点时 `/small_point_lio` 提供 `/Odometry`、`/cloud_registered`、`/cloud_registered_full` 和 `/map_save`。构建保留了上游 Eigen/GCC `-Warray-bounds` 告警，但无编译错误。`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（15 项）、三个 `node --check`、`node tests/editor_pointer_harness.mjs`（11 项）、`bash -n run.sh` 和 `git diff --check` 全部通过。
- 在隔离端口 18780 实测网页 `start_stack`：`robot_state_publisher`、`mid360_driver`、项目内 `small_point_lio` 全部上线，进程命令行为本项目 install 下的精确可执行文件；两台 MID360 在线并启用双雷达配对，`/cloud_registered` 与 `/Odometry` 均实时，后端测得点云约 20.15 Hz。随后通过 `stop_stack` 正常停止三个受管进程，状态保持 `IDLE`，日志除预期的 SIGINT 停止记录外无启动警告。最后在隔离端口 18781 复查 `/api/status` 的本地 LIO 路径/ready/stale 状态和 `/api/cloud` 的 `MAP1` 头；两个隔离后端与临时文件均已清理。未开始建图会话、未生成测试地图，未进行真实浏览器视觉检查。

## 2026-09-21 — 点云去动态物体方案交接文档

- 新增 `docs/dynamic-removal-handoff.md`：记录建图累积管线去除动态物体的调研结论与定稿方案，供后续实现者交接使用。内容包含当前 `VoxelAccumulator` 纯并集累积导致拖影的根因定位（`backend/mapping_core.py` 的 `add()` 无删除路径）、验收标准（允许保留动态对象最后位置、禁止误删静态结构）、方法调研对比、以及读取 `mas_nav_2027_native` 只读仓库得到的可复用资产坐标——`rog_map` 的 `raycaster` DDA 实现与 `hit/miss` 双计数判据（实测参数换算为 1 次命中判占据、5 次穿越判释放），以及 `map_server` 的 map-based 变化检测判据。
- 定稿方案：把 `rog_map` 的 hit/miss 判据搬进 `VoxelAccumulator`，改用全局哈希、关闭 decay、去掉滑动窗口、未观测体素保持原样，从而得到"最近一次观测优先"语义，使动态对象只在最终停留位置留下残留。文档同时列出实现陷阱（同一体素覆盖无法删除轨迹、单次 free 判据会误删静态结构）、分阶段实施顺序、禁止事项、验证要求与未决问题。
- 本次为文档改动，未修改任何源码、配置、脚本或测试。验证：仅通读现有实现核对文档中的行号与参数引用，未运行 `AGENTS.md` 的最小验证集（无代码变更）；未连接雷达/LIO，未进行硬件或浏览器验证。方案本身尚未实现，文档中的阶段 0 原型与后续阶段均未开始。

## 2026-09-21 — 已有 PCD 转换为 PGM/YAML

- 实时建图工作区新增“已有 PCD 转二维图”卡片：可刷新并选择项目 `data/pcd` 下的点云，沿用当前 Z 高度上限和后端导出参数生成 Nav2 PGM/YAML；转换在同源 HTTP 上执行，不依赖 WebSocket。涉及 `web/index.html`、`web/styles.css`、`web/app.js`、`web/map-editor.js` 与 `backend/mapping_server.py`。
- `backend/mapping_core.py` 的 PCD 读取扩展为支持 `DATA ascii` 和未压缩 `DATA binary`，允许多种数值类型及 `intensity`/`ring` 等附加字段，只提取有限 `x/y/z`；`binary_compressed` 会返回明确错误。新转换流程不重写原 PCD，二维切片失败不留下半套文件；同名地图已存在时自动选择最长 48 字符的时间戳名，并写出同名规范 XYZ binary PCD，保持坐标系编辑的成组语义。`README.md` 同步增加操作流程和格式边界。
- 新增 ASCII/附加字段、binary/附加字段、规范化输出与失败原子性回归用例。验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（15 项）、`node --check web/app.js`、`node --check web/point-cloud-viewer.js`、`node --check web/map-editor.js`、`node tests/editor_pointer_harness.mjs`（11 项）、`bash -n run.sh` 全部通过。在隔离端口 18777 和临时输出目录上验证 PCD 列表、首次转换（65,196 点，189×170）、不同 Z 上限的二次时间戳转换（187×170）、成组 PCD/PGM/YAML/frame 输出、`/api/status` 和 `/api/cloud` 的 `MAP1` 头；隔离后端已停止，临时产物已清理。未连接真实雷达/LIO，未进行硬件验证；未进行真实浏览器视觉/交互检查。

## 2026-09-21 — 可调 Z 高度的二维俯视投影

- 实时建图控制栏新增“二维投影高度上限”滑块，显示实际保留的 `Z` 区间；点击“结束并保存”时，通过 WebSocket 或 HTTP 把所选上限传给同一个会话保存方法，高于上限的点不进入 PGM，完整 PCD 仍保存全部累计点。涉及 `web/index.html`、`web/styles.css`、`web/app.js` 与 `backend/mapping_server.py`。
- 每次保存会冻结独立的 `MapExportConfig`，状态与导出结果同时记录实际 `z_min/z_max`；ROS `/mapping/stop_and_save` Trigger 继续使用启动参数默认值。地图导出配置新增非有限数值校验，非法高度不会结束正在进行的会话；`README.md` 同步说明滑块和 PCD/PGM 数据语义。
- 新增二维投影高度回归用例，确认超过截止高度且 XY 距离很远的点不会扩大 PGM 边界，并覆盖 `NaN` 参数拒绝行为。验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（12 项）、`node --check web/app.js`、`node --check web/point-cloud-viewer.js`、`node --check web/map-editor.js`、`node tests/editor_pointer_harness.mjs`（11 项）、`bash -n run.sh` 全部通过。
- 在隔离端口 18776 与临时输出目录运行合成点云端到端烟测：以 `Z≤0.80 m` 保存后，完整 PCD 含 46,615 点且最高 `Z=1.47 m`，二维切片采用 24,801 点并生成 167×124 PGM，`/api/status` 回报实际高度范围，`/api/cloud` 保持 `MAP1` 头和 46,615 点；另确认 `Z≤0.01 m` 被接口拒绝且不会开始保存。烟测会话已保存为 `SAVED` 后停止隔离后端，临时输出已清理。未启动真实雷达/LIO，未进行硬件验证；环境没有可用图形浏览器，因此未做真实浏览器视觉/交互检查。

## 2026-09-20 — 成组统一 map 坐标系并支持拖拽取帧

- 新增“map 坐标系”工作区卡片：在二维图上定义新 `map` 原点与 `map +X`，应用时对同名完整 PCD 施加同一二维刚体变换，把 PGM/YAML 与已有 terrain 最近邻重采样到 `yaw=0` 的轴对齐栅格并同步旋转 terrain 方向；涉及 `web/index.html`、`web/styles.css`、`web/map-editor.js`、`web/app.js` 与新增的 `backend/map_frame_core.py`。
- 取帧交互支持三种手势：空白处按下＝该点成为新原点并拖出 `+X`；拖动原点圆圈＝平移且保持朝向；拖动 `+X` 箭头＝只转向且保持原点。手柄在画布上以圆圈和箭头端点显示，悬停时切换 `move`/`grab` 光标，单击手柄不会误改数值，输入框仍可直接键入。此前按下即固定原点、只能靠拖动改朝向，重新定位还会把朝向重置为 0°。
- `backend/terrain_core.py` 的编辑器协议升级为 `MPE2`（头部新增 `origin_yaw`，读取时兼容旧 `MPE1`），YAML 的 `origin` 读写改为完整 `[x, y, yaw]`，并让 origin 三元组始终以浮点样式回写；`backend/mapping_core.py` 新增 `read_binary_pcd`，仅接受本工具生成的 `x/y/z float32` binary PCD 并校验 `POINTS` 与实际长度。
- 新增 `POST /api/editor/frame`：`MAPPING`/`SAVING` 期间拒绝执行，PCD、PGM、YAML、terrain、frame 元数据通过备份-回滚事务成组替换，任一失败即恢复原文件；操作记录写入 `<名称>_frame.json`（`source_to_map`/`map_to_source`、`revision`、上次定义与栅格信息），重复定义会在元数据中累计组合。
- 修正栅格重算精度：变换后的目标栅格改用包围盒精确最小角作为原点、按 `ceil(范围/分辨率)` 取整，不再把原点吸附到分辨率整数倍，因此“原点与朝向都不变”的对齐对 PGM/YAML/PCD 是逐字节空操作，亚栅格平移也不再凭空多出一行一列填充。
- 新增 `tests/test_map_frame_core.py`（含旧实现会失败的对齐/尺寸回归用例）与 `tests/editor_pointer_harness.mjs`：后者用极简 DOM 桩加载真实 `web/map-editor.js`，在无浏览器、无构建工具的情况下回放取帧的按下、平移、转向手势。`AGENTS.md` 与 `README.md` 同步更新验证命令和手势说明。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（10 项）、`node --check web/app.js`、`node --check web/point-cloud-viewer.js`、`node --check web/map-editor.js`、`node tests/editor_pointer_harness.mjs`（11 项）、`bash -n run.sh` 全部通过；把 `venue_map`（189×170、65196 点）复制到隔离目录后实测：恒等对齐保持 189×170 且 PGM/YAML/PCD 逐字节不变，90° 旋转尺寸精确换成 170×189、PCD 与解析解偏差为 0、占用与未知像元数量分别保持 5546/26584，PCD↔PGM 归属 65118/65119 点一致（唯一分歧点恰好落在栅格边界），30° 旋转得到解析预期的 249×242 且 YAML yaw 写为 0；隔离端口 18774 上验证 `/api/status`、`/api/cloud` 的 `MAP1` 头、`/api/maps` 的 frame 字段与 `POST /api/editor/frame` 的 404/400 分支。回归用例均确认能拒绝旧实现。未做真实浏览器检查（本机 `firefox`、`geckodriver` 都是无法在此环境运行的 snap 包，也无图形浏览器），未启动雷达或 LIO 会话；测试用的隔离后端与临时目录已清理，本机 8765 上原有的控制台进程未被触碰，浏览器需刷新以加载新的 `web/map-editor.js`。

## 2026-09-20 — 启动后自动打开控制台网页

- 更新 `run.sh`：解析 `--host`/`--port` 后启动就绪探测，`/api/status` 可访问时自动打开对应控制台地址；VS Code Remote SSH 优先使用客户端 `code --openExternal`，普通 Linux 桌面依次使用 `xdg-open`、`gio open` 或 `sensible-browser`，无法调用浏览器时保留后端并打印手动访问地址。
- 新增 `--no-browser` 启动选项，供无桌面服务器、systemd 和自动测试显式关闭自动打开行为；该选项只由启动脚本消费，其余参数继续原样传给 Python/ROS 后端。同步更新 `README.md` 的运行说明与参数表。
- 验证：`bash -n run.sh` 通过；在隔离端口启动真实后端并等待状态就绪，使用临时 `code` 探针确认调用参数严格为 `--openExternal http://127.0.0.1:18771`，`curl /api/status` 返回 `IDLE`；另验证 `--host=127.0.0.1 --port=18772 --no-browser` 能启动且不会执行打开命令。两个验证后端均已停止，临时探针已清理。未执行实际浏览器视觉检查，也未开始硬件建图会话。

## 2026-09-20 — 集成二维与 terrain 地图编辑器

- 网页新增“地图编辑”工作区：可从项目 `data/map` 加载 PGM/YAML，使用笔刷、矩形、画线修整障碍/空闲/未知栅格，并提供平移、缩放、坐标查看、有限内存撤销和原子保存；涉及 `web/index.html`、`web/styles.css`、`web/map-editor.js` 与 `web/app.js`。
- 新增 PGM/YAML → HW terrain msgpack 转换和 terrain 语义标注，完整支持 `0–6` 地形标签、方向滑块、画线左法向、方向箭头及已有 terrain 覆盖确认；MessagePack 的 `terrain`/`direction` 使用 HW C++ `std::vector<uint8_t>` 可读取的 BIN 通道。
- 新增 `backend/terrain_core.py` 和地图编辑 REST 接口，包含路径/名称校验、地图尺寸上限、PGM P2/P5 读取、YAML `negate` 正确处理、项目内文件约束、临时文件原子替换，以及对上游数组型和 HW BIN 型 msgpack 的兼容读取；`backend/mapping_server.py` 保持 ROS 回调与文件编辑请求隔离。
- 新增 `tests/test_terrain_core.py`，并更新 `README.md` 的编辑流程、输出布局和模块说明；导航仓库与 `nav_opensource/HW` 仅作为只读格式/行为参考，未被修改。
- 验证：`python3 -m compileall -q backend tests`、`python3 -m unittest discover -s tests -v`（7 项）、`node --check web/app.js`、`node --check web/map-editor.js`、`node --check web/point-cloud-viewer.js`、`bash -n run.sh` 均通过；在隔离端口和临时输出目录实测 `/api/status`、`/api/cloud` 的 `MAP1` 头、地图列表、PGM/terrain 二进制读取、二维→terrain 转换、两个图层保存及重复转换 409 防覆盖均通过，测试产物已清理；另用系统 `msgpack-cxx` 探针成功按 HW C++ 同款 `as<std::vector<uint8_t>>()` 读取生成文件。未进行浏览器视觉/交互检查，未启动新的硬件建图会话；尝试直接启动导航仓库 `map_server_node` 时被该安装环境现有的 `libpcl_io.so.1.14`/`libusb_set_option` 符号错误阻断。

## 2026-09-20 — 添加后续开发说明

- 新增根目录 `AGENTS.md`，记录独立项目边界、模块职责、ROS/HTTP/WebSocket 接口、建图会话语义、节点进程管理安全规则、性能约束和验证流程。
- 明确后续所有源码、配置、测试及文档变更都需要在本文件顶部追加记录，且不得隐式修改导航主仓库。
- 验证：人工检查文档路径、当前目录结构和已有接口名称；本次只修改文档，未重启网页服务，也未重复执行硬件建图测试。

## 2026-09-20 — 初始独立版建图控制台

- 新增 ROS 2 点云会话累计器、浏览器 3D 点云显示和开始/结束/清空控制。
- 新增二进制 PCD 与 Nav2 PGM/YAML 输出，默认保存到项目内 `data/`。
- 新增 ROS 状态、累计点云及 Trigger 服务接口，并提供 REST/WebSocket 控制链路。
- 新增核心数据路径单元测试和中文运行说明。
- 新增无需雷达的短时合成点云发布脚本，用于端到端联调。
- 网页新增建图链路管理，可启动缺失的机器人 TF、MID360 驱动和 Small Point-LIO；停止时只清理网页自行启动的进程。
- 建图按钮会等待真实 `/cloud_registered` 数据在线，避免在 LIO 未就绪时误开始空会话。
- 新增普通 HTTP 状态与点云快照回退；VS Code Remote SSH 预览不支持 WebSocket 时仍可完整控制和查看累计点云。
