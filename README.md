# MAS 三维建图控制台

这是一个放在 `/home/mas/mapping_web_ui` 的独立 ROS 2 网页工具。它不修改导航仓库，也不依赖 rosbridge：后端直接订阅 Small Point-LIO 的实时点云与里程计，浏览器通过同一个服务查看三维累计点云并控制建图会话。

## 能做什么

- 点击“开始建图”后，从空会话开始累计 `/cloud_registered`；
- 网页以 3D 方式显示降采样后的累计点云，支持旋转、平移、缩放和俯视；
- 实时显示累计点数、覆盖范围、地图尺寸、建图时长、行驶里程与输入频率；
- 默认用时间对齐的里程计原点做自由空间射线清理，删除行人走过后被后续观测否定的拖影体素；
- 点击“结束并保存”只写入三维 PCD：完整累计点全部落盘，保存本身不再生成二维图；
- 点云与二维图完全解耦：在“点云转二维图”卡片里自选 PCD、调切片参数、先看二维预览，再按需生成 Nav2 可读的 PGM/YAML；
- 支持 `data/pcd` 中已有的 ASCII 或未压缩 binary PCD（允许 intensity 等附加字段），可在三维预览中只看当前 Z 切片范围内的点；
- 在“地图编辑”工作区用笔刷、矩形或画线修整二维 PGM 占据图，并可撤销、缩放和平移；
- 可直接在二维图上点选新 `map` 原点并拖出 `map +X` 方向，成组变换完整 PCD、PGM/YAML 和已有 terrain，为后续重定位发布 `map→odom` TF 固定统一的地图基准；
- 把 PGM/YAML 一键转换为导航 `map_server` 使用的 terrain msgpack，再标注平地、障碍、上坡、隧道和起伏路段；
- 在“ROGMap 分类”工作区直接查看 `projection_layer` 四分类，点选桌子所在格后查看 `height_delta`、占据高度、占据层数与竖直占据率，并在浏览器中 what-if 试调 surface/wall/tunnel 阈值；
- 在“轨迹验证”工作区只读导入已有 terrain，用一份完全隔离的真实 `map_server` + `nav_executor` 复现全局折线与 MINCO 轨迹，并在质点运行中放置临时障碍观察真实重规划；
- 同时发布 `/mapping/accumulated_cloud` 与 `/mapping/status`，仍可在 RViz/Foxglove 中观察；
- 提供 `/mapping/start`、`/mapping/stop_and_save`、`/mapping/reset` 三个 `std_srvs/srv/Trigger` 服务；
- 输出文件已存在时自动在新文件名后加时间戳，不覆盖旧地图。

## 运行

雷达接通后，先启动控制台：

```bash
cd /home/mas/mapping_web_ui
./run.sh
```

后端状态接口就绪后，脚本会自动打开：

```text
http://127.0.0.1:8765
```

在普通 Ubuntu 桌面终端中会调用系统默认浏览器；在 VS Code Remote SSH 终端中会通过 VS Code 在本地客户端打开地址。若当前环境没有可用的浏览器打开方式，终端会打印地址供手动访问，但后端仍会继续运行。

无桌面服务器、systemd 或自动测试不需要打开浏览器时，可以使用：

```bash
./run.sh --no-browser
```

在网页“建图链路”区域点击“启动建图节点”，页面会依次补齐：

```text
robot_state_publisher → mid360_driver → small_point_lio
```

等“点云输入”和“里程计”都变成绿色后，再点击“开始建图”。如果这些节点已经从其他终端启动，网页会显示“外部运行”且不会重复启动；“停止节点”也只停止网页自己启动的进程。

同一局域网的平板或其他电脑可打开 `http://<机器人IP>:8765`。该服务默认监听所有网卡，没有登录鉴权；只应在可信局域网内使用。如只允许本机访问，请改用：

```bash
./run.sh --host 127.0.0.1
```

通过 VS Code Remote SSH 的端口转发或内置预览访问时，如果代理不支持 WebSocket，页面会自动切换为“HTTP 兼容模式”：节点状态、控制按钮和三维点云仍然可用，只是点云刷新频率降低到约 1 Hz。

不需要把 Small Point-LIO 的 `save_pcd` 改成 `true`。本工具只在点击开始后累计 `/cloud_registered`，不会重启 LIO，也不会改变 LIO 的 odom 原点。

网页启动的 ROS 进程日志位于 `.ros/managed/`。关闭网页标签不会停止节点；结束 `run.sh` 时，会清理由该网页后端启动的节点。

## ROGMap 投影分类诊断

页头的“ROGMap 分类”是一个只读诊断面，不修改导航仓库或正在运行的 ROGMap。后端使用 best-effort QoS 订阅：

```text
/rog_map/layer_type          nav_msgs/msg/OccupancyGrid
/rog_map/layer_height_delta  sensor_msgs/msg/PointCloud2 (z=占据最高点, intensity=height_delta)
/rog_map/occupied            sensor_msgs/msg/PointCloud2
```

`layer_type` 的实际四类仍以 ROGMap 发布值为准：`-1=UNKNOWN`、`33=FREE`、`66=PASSABLE`、`100=OCCUPIED`。页面另外复刻 `classifyCell()` 的判定顺序，将有占据高度的柱细分为薄面、实心墙、中空 tunnel 和模糊障碍。点选一格后会显示：

- `height_delta = (最高占据层号 - 最低占据层号) × resolution`；
- `vertical_occupancy_ratio = (occupied_count - 1) × resolution / height_delta`；
- 是否同时满足 `tunnel_height_delta_min ≤ height_delta ≤ tunnel_height_delta_max` 和 `ratio ≤ tunnel_occupancy_ratio_max`；
- 实际 `layer_type` 与当前 what-if 阈值推演出的原始分支。

竖直占据率不是 ROGMap 现有可视化话题的直接字段。后端只在三路快照时间对齐，且 `/rog_map/occupied` 同时覆盖该柱的最低/最高占据端点时，才按唯一 Z 体素层数重建 ratio。可视化范围裁切、decay active list 或快照不同步时，格子会明确标成“ratio 不可用”，不会用不完整占据云猜测。

右侧 6 个阈值仅用于浏览器 what-if 重上色；非法关系（例如 `surface_max ≥ tunnel_min` 或 `tunnel_ratio_max ≥ wall_ratio_min`）会当场拦截。后端默认基线对齐当前 `planner_params.yaml`，也可用 `--rogmap-*-...` 启动参数覆盖。真正调参仍要修改 `planner.rog_map.projection.*` 并重启 `nav_executor`：ROGMap 当前没有将这组参数热更新到分类器的回调。

二进制诊断接口为 `GET /api/rogmap/projection`，返回固定步长的 `ROG1` 格网；它和页面轮询都走同源 HTTP，因此 VS Code Remote SSH 不转发 WebSocket 时仍可用。

## terrain 轨迹验证

页头“轨迹验证”不是浏览器内的近似推演，而是一次**真实规划器的隔离复现**：后端把项目自己的 `map_server` 与 `mas2027_nav_executor` 作为受管子进程再启动一份，全部话题改到 `/mapping/trajectory_lab/*` 命名空间，页面显示的是这两个真实节点算出来的全局折线与 MINCO 轨迹。使用流程是：

1. 选择 `data/map` 中同时具备 `<名称>.yaml` 与 `<名称>_terrain.msgpack` 的地图，点“导入 terrain（只读）”。
2. 选“车体位置”并在图上点击当前位置，再选“目标位置”点击目标。
3. 点“启动真实规划”：后端先校验场景，再拉起隔离 `map_server` 与隔离 `nav_executor`，并向隔离命名空间发布车体位姿、目标与障碍点云。
4. 白色质点按回放速度沿真实 MINCO 轨迹运行。运行中可切换“放障碍”笔刷；松开指针后整份临时障碍会送进隔离 ROGMap 并触发真实重规划。

隔离边界（页面上有同样的说明）：

- 该工作区只使用 `/mapping/trajectory_lab/*`，不订阅也不发布实车的 `/goal_pose`、`/Odometry`、`/cloud_registered` 或 `/cmd_vel`；
- 隔离执行器的 `cmd_vel`、`/opt_path`、cost/terrain label 地图、planning constraints 与可视化话题全部被重映射进该命名空间；
- 车体位姿、目标与 brush 障碍只发布到隔离话题，绝不进入实车 ROGMap；
- 右侧 6 个滑块（`safe_dist`、`collision_dist`、`max_velocity`、`max_acceleration`、`penalty_weight_time`、`esdf_weight`）作为**临时 ROS 参数覆盖**传给隔离执行器，不写 `planner_params.yaml`、`node_params.yaml` 或 terrain；改参数后点“重新启动并规划”生效。

场景守卫在启动前拒绝：地图名路径穿越、缺少 `<名称>.yaml` 或 `<名称>_terrain.msgpack`、车体或目标落在 `label 1` 硬障碍格上或越出地图范围、临时障碍放在地图外。临时障碍上限为 20000 个格（与后端 `MAX_LAB_OBSTACLES` 一致），浏览器达到上限时停止新增并提示。

规划器状态机为 `STOPPED → STARTING → PLANNING → READY`，任一受管子进程意外退出即转为 `ERROR`。停止时后端只对本项目启动的子进程组依次发 `SIGINT`、`SIGTERM`、`SIGKILL`，不使用任何按名字的批量终止；后端退出时同样会停掉隔离规划器。两个子进程日志分别写在 `.ros/trajectory_lab/map_server.log` 与 `.ros/trajectory_lab/nav_executor.log`。

依赖外部导航工作区：默认从 `/home/mas/mas_nav_2027_native/src/mas2027_nav_executor/config` 读取 `planner_params.yaml`、`node_params.yaml`、`mpc_params.yaml`（`--trajectory-lab-config-dir` 可改），日志目录用 `--trajectory-lab-log-dir` 可改。相关接口为 `GET /api/trajectory` 与 `POST /api/trajectory/start|obstacles|stop`。

### 海康 MVS 动态库冲突（`run.sh` 已处理）

装了海康 MVS 的机器上，`/etc/profile` 与 `~/.bashrc` 会把 `/opt/MVS/lib/64:/opt/MVS/lib/32` 前置进 `LD_LIBRARY_PATH`。MVS 自带的 `libusb-1.0.so.0` 比系统版旧、不含 `libusb_set_option`，会遮蔽系统 libusb，使任何加载系统 `libpcl_io` 的进程直接以 `undefined symbol: libusb_set_option` 退出——受管建图链、隔离轨迹实验室的 `nav_executor`，以及从同一 shell 启动的实车导航链都会中招。

`LD_LIBRARY_PATH` 是**整体先于** `ld.so` 缓存搜索的，所以只在末尾追加 MVS 没有用，必须把系统目录显式排在它前面。`run.sh` 在 source 完 ROS 工作区后会检测并处理：若 MVS 那份 libusb 存在且系统目录不在首位，就把 `/usr/lib/x86_64-linux-gnu` 置为 `LD_LIBRARY_PATH` 首项并打印一行说明。该处理幂等，未装 MVS 的机器上完全空转，且只影响本脚本启动的后端与其受管子进程，单独运行的 MVS 客户端不受影响。

换用系统 libusb 是安全的：两者 soname 相同，系统版是 MVS 那份的**严格超集**（97 个 API 对 83 个，多出的都是 libusb 1.0.23+ 的标准 API，无厂商私有符号），且只有 `libMvUsb3vTL.so`（USB3 传输层）链接 libusb，GigE 那套 `libMVGigEVisionSDK.so` 根本不加载它。已实测 MVS SDK 换库后仍能正常加载与枚举设备；若 USB3 取流出现异常，删掉 `run.sh` 里那段判断即可回到原状。

从普通终端（而非 `run.sh`）启动实车导航链时不受此处理影响，仍需自行把系统目录排在 MVS 前面，例如：

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

## 本地 Small Point-LIO 源码

项目内置了从导航工作区原样复制的 Small Point-LIO 包：

```text
ros2_ws/src/small_point_lio
```

该副本可在本项目中独立修改，不需要改动 `/home/mas/mas_nav_2027_native`。初始副本包含完整 C++ 源码、launch、包内示例配置、MIT 许可证和必要的第三方源码。

需要使用本地副本时，只构建这一个包：

```bash
cd /home/mas/桌面/mapping_web_ui/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/mas/mas_nav_2027_native/install/setup.bash
colcon build --symlink-install --packages-select small_point_lio \
  --allow-overriding small_point_lio
```

`run.sh` 会先加载导航 overlay，再在 `ros2_ws/install/setup.bash` 存在时加载本地 overlay。网页的“启动建图节点”不再通过包名模糊解析 LIO，而是直接启动项目内 `ros2_ws/install/small_point_lio/lib/small_point_lio/small_point_lio_node`。如果本地包尚未构建，或者源码比可执行文件新，页面会拒绝启动并提示重新构建，不会悄悄回退到导航主项目的同名包。`run.sh` 本身不会自动执行 `colcon build`，所以网页后端在 LIO 未构建时仍然可用。

默认运行参数仍以导航工作区的 `small_point_lio_params.yaml` 为真源，副本内的 `config/mid360.yaml` 只是包自带示例。更详细的 overlay 说明见 `ros2_ws/README.md`。

以后在项目里增加点云处理时，常用集成点是：

- `src/small_point_lio/preprocess.cpp`：配准前的距离裁减、下采样和原始点预处理；
- `src/small_point_lio/small_point_lio.cpp`：去畸变、状态估计与建图主数据路径；
- `src/small_point_lio_node.cpp` 的 `set_pointcloud_callback`：已配准到 odom 系、即将发布为 `/cloud_registered` 的点云。

修改 C/C++ 源码后必须重新执行上述 `colcon build` 命令，再从网页启动建图节点。

## 动态拖影清理

去动态处理位于本项目的全局体素累积层，不改 Small Point-LIO 的位姿估计，也不修改导航主项目。核心算法参考 `nav_opensource/HW` 的 `offline_mapping_optimizer` 射线滤波器：每个处理帧从与点云时间戳最接近的 `/Odometry` 位姿换算真实雷达原点，再用 Amanatides–Woo 3D-DDA 精确遍历射线穿过的体素。为适应边建图边清理，在 HW 离线累计穿越次数的基础上增加了“分帧去重 + 命中优先”保护：

- 当前帧命中的体素继续保留，并清零其空闲计数；
- 每个角度格只取最近回波，3D-DDA 排除回波终点体素，防止用远点穿透近处表面；
- 被射线穿过、但当前帧没有命中的历史体素累积一次 miss；
- 同一体素在 5 个不同处理帧中都被观测为空才删除；任意新命中会重置计数；
- 没有被射线观测的区域不衰减、不忘记，避免因机器人离开而删除墙体。

为了不阻塞 ROS 点云回调，射线计算在独立的“最新帧”后台工作线程中默认以 3 Hz 运行，每帧最多 4000 条射线，只处理 0.38–10 m 范围。HW 离线工具默认穿越阈值为 2，本项目在线默认为 5 个不同处理帧，避免配准波动或稀疏回波过早删除静态结构。网页“数据链路”会显示功能是否开启、累计清理点数和处理帧数。

为了利用动态物体出现之前的自由空间证据，新建会话默认还会记录有界的磁盘关键帧历史：

- 平移达到 0.10 m、旋转达到 5°，或距上一关键帧达到 0.75 s 时记录；最小间隔 0.20 s，因此机器人静止时的行人移动也会留下观测证据；
- 点云回调只向有界队列提交关键帧，后台线程每角度格保留最近回波，每帧最多 4000 条射线；
- 历史以最多 64 帧一块的二进制 chunk 原子写入 `data/session_history/`，默认硬上限 2 GiB；队列拥塞或达到上限时只丢弃/停止历史记录，不阻塞 ROS 回调，也不中断建图；
- 结束建图时逐块回放历史，对最终点云同时统计命中帧和空闲穿越帧。默认至少 5 个关键帧判空，且空闲帧数至少是命中帧数的 2 倍才删除；
- 如果候选删除超过全图 35%，认为可能存在位姿/参数异常，整次离线清理自动放弃，保留原累计点云；
- 成功导出后默认删除临时历史；使用 `--keep-keyframe-history` 可保留 `.history` 目录以便重新调参。导出失败或后端建图中异常退出时会保留历史，便于恢复和排查。

因此，新会话中动态物体即使最后停在某处，也可以被它出现之前的历史自由空间证据清理。已有的单个 PCD 仍然没有时间序列和观测原点，无法追溯执行这种离线清理。

保存时默认还会执行一层保守的 ERASOR 风格极坐标格投票，用来补足“射线没有恰好穿过物体外壳”时的残影：每个关键帧以雷达原点为中心分成环 × 扇区，比较累计地图与当前扫描在同一区域中的竖直结构高度跨度。地图结构明显更高、扫描只剩低矮地面、且这种差异得到至少两个关键帧确认时，才把高于局部地面的体素作为消失候选。它在射线回放之后执行；极坐标票与射线票合计会再次受 `--keyframe-max-removal-fraction` 总安全上限约束，超限时会**只放弃极坐标票**并保留已经安全通过的射线清理结果。

每次保存会在 PCD 旁原子写入 `<名称>.mapping-report.json`。其中记录输入/输出点数、体素大小、坐标系、射线清理与极坐标投票的分项统计、跳过/回退原因，便于发现误删时复核；它不改变 PCD 格式，也不影响二维图的独立转换。

## 点云转二维图（自助）

保存会话只负责三维点云：“结束并保存”写入 `data/pcd/<名称>.pcd`，**不会**再顺带生成 PGM/YAML。二维图是一个独立的、可反复重来的步骤，参数与预览都归操作者：

1. 把 `.pcd` 文件放入本项目的 `data/pcd/`；文件名主体需符合地图名规则（字母、数字、点、短横线或下划线，最长 48 字符）。
2. 在“实时建图”右侧滚动到“点云转二维图”，点击刷新并选择文件。
3. 调整切片参数：默认高度基准为「自动地面」。后端先在 0.5 m XY 网格内取低位高度样本，用确定性 RANSAC 拟合主地面平面以校正 LIO 俯仰/横滚；随后在 0.4 m 网格上从严格的地面种子向相邻缓变单元生长，并在 2 m 范围内插值局部高度，继续补偿长走廊中的缓慢起伏与轨迹弯曲。种子带和相邻步长刻意低于常见 10 cm 路沿，避免把台阶或低平台吸收到地面中。「障碍下限–障碍上限」据此按**相对局部地面高度**判定。「全局 Z」保留原有绝对坐标语义，供无法稳定拟合地面或需要固定 Z 带的场景使用。离群点默认用「结构体素」滤波，另可选原有的「半径邻域」或完全关闭。数值留空表示沿用后端启动参数。首次进入时以后端默认值填充，操作者一旦改动就不再被状态轮询覆盖，点“恢复默认值”可拉回。
4. 点“在三维预览中查看当前切片”：累计点云卡片下方会加载该 PCD，并使用与二维转换相同的高度基准和上下限选点，用来判断切片是否切到墙、地面或天花板。
5. 点“预览二维切片”：后端在内存里完成切片、离群点滤波与栅格化，返回一张 PGM 预览和完整元数据（栅格尺寸、分辨率、原点、切片点数、滤波后点数、去除点数、占据格数）。预览不写任何文件；栅格过大时按整数倍做“取最暗值”降采样，只影响显示，细墙不会在预览里消失。
6. 确认无误后点“生成 PGM · YAML”。可填“输出地图名”，留空则沿用点云名称；同名地图已存在时自动加时间戳，绝不覆盖旧图。

预览走 `POST /api/pcd/map-preview`，请求体为 `{"map_name":"...", "height_mode":"ground", "filter_mode":"voxel", "filter_voxel_size":0.10, "z_min":…, "z_max":…, "resolution":…, "radius":…, "min_neighbors":…, "padding":…}`。`height_mode` 可为 `ground`/`absolute`；`filter_mode` 可为 `voxel`/`radius`/`none`。响应体是二进制 PGM（P5），几何与统计通过 `X-Map-*` 响应头返回，包括 `X-Map-Filter-Mode` 和 `X-Map-Filter-Removed`；自动地面模式另用 `X-Map-Ground-A/B/C`、`X-Map-Ground-Tilt` 以及 `X-Map-Ground-Local-*` 回报整体平面、倾角和局部地面网格统计。转换走 `POST /api/pcd/convert`，请求体在上述切片参数之外可再加 `"output_name"`。两个接口都只在 `IDLE`/`SAVED`/`ERROR` 状态可用，建图或保存进行中返回 409。

「结构体素」借鉴 ROGMap 投影层的体素列和八邻域分类：先按三维体素去重，再保留同一 XY 列中占有至少两个 Z 体素的垂直结构，或有任一八邻域占据列支撑的水平结构；只删除两者都不满足的孤立单体素列。这能保留细墙、路沿和杆状障碍，同时比“某个半径内必须有 N 个点”更不受近密远疏的点云密度影响。它不是 ROGMap 完整的 `OCCUPIED / KNOWN_FREE / UNKNOWN` 概率分类：单个已保存 PCD 没有每条激光射线的 hit/miss 时序证据，因而无法在离线转换时真实重建那三类状态。

转换在普通同源 HTTP 上运行，因此 HTTP 兼容模式也可用。输入 PCD 支持 `DATA ascii` 和 `DATA binary`，只提取 `x/y/z`，自动忽略 `intensity`、`ring` 等其他字段；`binary_compressed` 会给出明确的不支持提示。转换不会重写原 PCD。预览与生成共用后端同一个切片函数，因此预览到的栅格就是最终写盘的内容。生成完成后可直接进入“地图编辑”工作区。

## 文件夹点云预览

在“累计点云”卡片右上角点击“点云预览”，卡片会分成上下两块：上面继续显示实时累计点云，下面是被动加载的预览画布。

1. 下拉框列出 `data/pcd/` 中全部 `.pcd`（文件名主体需符合地图名规则，最长 48 字符），点“预览”加载，点“清除”清空预览画布，“✕”收起面板。
2. 预览使用独立 WebGL 视图和自己的相机，旋转、平移、缩放、适应视图（进入后自动取景）互不影响，实时建图画面照常刷新。
3. 预览按 `--web-max-points`（默认 220,000）等间隔抽稀后传输，状态行会同时给出原始点数和实际显示点数；磁盘上的 PCD 只被读取，从不改写。
4. 带 `height_mode`、`z_min`/`z_max` 查询参数时，使用与二维转换相同的自动地面或全局 Z 高度窗口选点（“点云转二维图”卡片的“在三维预览中查看当前切片”就是这么调的），过滤只作用于传输的帧，不动磁盘文件。

预览走 `GET /api/pcd/preview?name=<名称>`，返回与实时点云完全相同的 `MAP1` 帧格式，并用 `X-Preview-Points`、`X-Preview-Total` 响应头给出显示点数和原始点数。名称先经过地图名校验，再做目录归属校验，路径分隔符与穿越名称一律返回 400；文件不存在返回 404。读取发生在文件锁之外，因此预览一个大 PCD 不会阻塞保存流程；导出使用原子替换，预览读到的必然是完整的旧版或新版文件。因为只是同源 HTTP 与 WebGL，预览在 HTTP 兼容模式下同样可用。

## 地图编辑流程

完成一次建图并保存后，点击页头的“地图编辑”：

1. 从 `data/map` 选择地图，点击“编辑二维 PGM”。二维层提供障碍物、可通行、未知区域三类像素，可用笔刷、矩形和画线修图；保存只原子替换该地图 YAML 引用的 PGM，不改变 YAML 的分辨率或原点。
2. 点画布右上角“点云 P”，或在非输入框聚焦时按 `P`，可显示/隐藏同名 PCD 的青色俯视点层。网页先按后端当前的障碍高度窗口取点，再只使用 XY 投影；点云与 PGM 共用 YAML 原点、分辨率和 yaw，缩放或平移时保持重合。青色点仍存在的黑格可视为有点云支撑；只有黑格而没有青色点的区域可重点复核，再用“可通行”笔刷清掉不存在的障碍。此叠加层只读 PCD，不会修改点云或地图。
3. 在“map 坐标系”中点击“在图上拖出原点与 +X”后有三种手势：在空白处按下＝该点成为新 `(0,0)` 并拖出 `map +X`；拖动原点圆圈＝平移坐标系且保持朝向；拖动 +X 箭头＝只转向且保持原点。也可以在输入框中直接填写原点在当前地图中的 X/Y 和 +X 朝向角。点击“统一 PCD 与二维图坐标系”并确认后，后端会对同名完整 PCD 应用相同二维刚体变换，把 PGM 和已有 terrain 最近邻重采样到 yaw=0 的轴对齐栅格。建议在精修 PGM/terrain 前先定义坐标系，避免重复重采样。
4. 点击“生成 terrain MSG”。后端按 YAML 的 `occupied_thresh` 和 `negate` 把二维图转换为 `<地图名称>_terrain.msgpack`，随后自动打开 terrain 图层。
5. 在 terrain 图层选择 `0` 平地、`1` 障碍，或与底盘 mode 同号的 `5` 上坡、`6` 隧道、`7` 起伏路段。笔刷、矩形和画线只修改标签。
6. 点击“保存当前图层”。输出采用 `mas_nav_2027_native` 当前 `map_server` 可直接读取的 `width`、`height`、`resolution`、`terrain` MessagePack 字段；标签通道按 MessagePack ARRAY 写入，与导航端的 `via.array` 加载方式一致。网页仍可读取旧 BIN 格式，保存时会移除旧方向字段并将标签迁移为 ARRAY。

坐标系设置需要 `data/pcd/<名称>.pcd` 与 PGM/YAML 同名存在；缺少 PCD 时网页会禁用应用按钮，避免只改二维图。操作会生成 `<名称>_frame.json`，其中 `source_to_map` 是把原始 LIO/odom 点坐标变换到固定 map 坐标的变换，也就是后续 TF 中 `map→odom` 所需的平面变换语义；`map_to_source` 是其逆变换。重复定义会在元数据中累计组合，但每次都需要重采样二维栅格，因此应尽量一次定准。

重新从 PGM 生成已存在的 terrain 文件前，网页会明确提示该操作会覆盖已有地形标签。地图编辑和坐标系设置使用普通同源 HTTP 接口，不依赖 WebSocket，因此在 VS Code Remote SSH 的 HTTP 兼容模式下也能使用。

## 输出

默认保存到本项目内：

```text
data/
├── pcd/
│   ├── <地图名称>.pcd
│   └── <地图名称>.mapping-report.json # 本次清理的可审计统计与参数
├── map/
    ├── <地图名称>.pgm
    ├── <地图名称>.yaml
    ├── <地图名称>_terrain.msgpack  # 在地图编辑工作区生成
    └── <地图名称>_frame.json       # source↔map 变换与坐标系修订记录
└── session_history/                 # 临时或 --keep-keyframe-history 保留的关键帧
```

刚保存时 PCD 与 `/cloud_registered` 使用相同数值坐标，`map` 与点云源坐标系按单位变换记录。保存只写 `<地图名称>.pcd`；PGM/YAML 由“点云转二维图”按当时的切片参数生成。默认先拟合主地面并跟随局部缓变地面，再保留相对局部地面 `0.05–1.50 m` 的点向 XY 平面投影；仅在选择「全局 Z」时，上下限才是原始点云的绝对 Z。完整 PCD 始终保存关键帧离线清理后的所有保留点，地面估计和切片都不会改写它。二维图的有点栅格为占用，其余栅格为空闲，与现有 `pcd2pgm` 的投影语义一致。自定义 map 基准后，PCD 点、二维栅格和 terrain 都处于同一个 `map` 坐标系，YAML 栅格 yaw 保持为 0，以兼容只接收 `origin_x/origin_y` 的 HW terrain server。

## 常用参数

```bash
./run.sh \
  --cloud-topic /cloud_registered \
  --odom-topic /Odometry \
  --voxel-size 0.05 \
  --height-mode ground \
  --z-min 0.05 \
  --z-max 1.50 \
  --map-resolution 0.05 \
  --port 8765
```

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `--voxel-size` | `0.05` | 累计点云体素尺寸；越小越细，也越占内存 |
| `--max-points` | `5000000` | 后端累计点数硬上限 |
| `--web-max-points` | `220000` | 每次发给网页显示的最大点数，不影响保存精度 |
| `--ros-max-points` | `500000` | `/mapping/accumulated_cloud` 单帧最大点数 |
| `--dynamic-removal` / `--no-dynamic-removal` | 开启 | 开启或禁用基于可见空闲的拖影清理 |
| `--dynamic-miss-threshold` | `5` | 体素被多少个不同处理帧判空后才删除；越大越保守 |
| `--dynamic-min-range` / `--dynamic-max-range` | `0.38` / `10.0` | 去动态射线距离范围（米） |
| `--dynamic-removal-rate` | `3.0` | 后台可见性处理最高频率（Hz） |
| `--dynamic-max-rays` | `4000` | 每个处理帧的角度均匀射线上限 |
| `--dynamic-odom-tolerance` | `0.12` | 点云与里程计时间戳允许的最大差值（秒） |
| `--keyframe-history` / `--no-keyframe-history` | 开启 | 记录磁盘关键帧并在保存时回放去动态 |
| `--keyframe-translation` / `--keyframe-rotation-deg` | `0.10` / `5.0` | 位移/旋转关键帧阈值 |
| `--keyframe-min-interval` / `--keyframe-max-interval` | `0.20` / `0.75` | 关键帧最小间隔和静止时最长间隔（秒） |
| `--keyframe-max-rays` / `--keyframe-chunk-frames` | `4000` / `64` | 每关键帧射线上限与每个原子 chunk 帧数 |
| `--keyframe-max-disk-gb` | `2.0` | 单会话关键帧历史硬上限 |
| `--keyframe-free-threshold` / `--keyframe-free-hit-ratio` | `5` / `2.0` | 离线删除的最小判空帧数与 free/hit 比例 |
| `--keyframe-max-removal-fraction` | `0.35` | 单次离线清理可删除的全图最大比例 |
| `--polar-disappearance-filter` / `--no-polar-disappearance-filter` | 开启 | 保存时启用/关闭 ERASOR 风格极坐标格消失投票；关闭后仅用射线回放 |
| `--polar-min-range` / `--polar-max-range` | `0.4` / `10.0` | 极坐标比较的有效距离范围（米） |
| `--polar-rings` / `--polar-sectors` | `10` / `72` | 极坐标网格环数与扇区数；越细越灵敏，也更容易受稀疏回波影响 |
| `--polar-cell-size` / `--polar-min-votes` | `0.2` / `2` | 投票用全局体素格尺寸与删除候选至少所需的负票数 |
| `--polar-scan-ratio-threshold` | `0.25` | 当前扫描与地图的高度跨度比例阈值；越小越保守 |
| `--polar-structure-span` / `--polar-height-threshold` | `0.5` / `0.4` | 候选结构最小高度跨度及其相对局部地面的最小高度（米） |
| `--polar-max-removal-fraction` | `0.20` | 极坐标投票本身最多可删除当前输入的比例；之后仍受总安全上限限制 |
| `--keep-keyframe-history` | 关闭 | 成功导出后保留关键帧 `.history` 目录 |
| `--height-mode` | `ground` | 二维投影高度基准；`ground` 校正整体倾斜并跟随局部缓变地面，`absolute` 使用原始全局 Z |
| `--z-min` / `--z-max` | `0.05` / `1.50` | 障碍切片高度范围；`ground` 模式下为相对拟合地面的高度，`absolute` 模式下为全局 Z |
| `--radius-filter` | `0.50` | 二维切片离群点滤波半径；设为 `0` 可关闭 |
| `--min-neighbors` | `10` | 半径内最少邻点数 |
| `--map-filter-mode` | `voxel` | 二维投影离群点滤波；`voxel`=结构体素，`radius`=半径邻域，`none`=关闭 |
| `--map-filter-voxel-size` | `0.10` | 结构体素分类尺寸（m，范围 0.02–1.0） |
| `--output-dir` | `./data` | PCD 和地图输出根目录 |
| `--lio-params` | 导航仓库中的参数 YAML | 驱动与 LIO 共用的参数文件 |
| `--local-lio-source` | `./ros2_ws/src/small_point_lio` | 项目内 LIO 源码目录，用于检查是否需要重新构建 |
| `--local-lio-executable` | `./ros2_ws/install/.../small_point_lio_node` | 网页启动的项目内 LIO 可执行文件 |
| `--no-stack-control` | 关闭 | 禁用网页启动/停止 ROS 节点的能力 |
| `--no-browser` | 关闭 | 仅启动后端，不自动打开网页；此参数由 `run.sh` 消费，不传给 ROS 后端 |

如果雷达或里程计话题名不同，只需通过启动参数覆盖，不用改源码。

## ROS 2 接口

```text
订阅  /cloud_registered             sensor_msgs/msg/PointCloud2
订阅  /Odometry                    nav_msgs/msg/Odometry
发布  /mapping/accumulated_cloud   sensor_msgs/msg/PointCloud2
发布  /mapping/status              std_msgs/msg/String（JSON）
服务  /mapping/start               std_srvs/srv/Trigger
服务  /mapping/stop_and_save       std_srvs/srv/Trigger（只写三维 PCD）
服务  /mapping/reset               std_srvs/srv/Trigger
```

命令行也可以控制：

```bash
ros2 service call /mapping/start std_srvs/srv/Trigger '{}'
ros2 service call /mapping/stop_and_save std_srvs/srv/Trigger '{}'
ros2 service call /mapping/reset std_srvs/srv/Trigger '{}'
```

通过 ROS 服务开始时，地图名自动使用当前时间；网页开始时使用输入框中的名称。

## 验证与排错

运行不依赖 ROS 图的核心测试：

```bash
cd /home/mas/mapping_web_ui
python3 -m unittest discover -s tests -v
node tests/editor_pointer_harness.mjs
node tests/point_cloud_viewer_harness.mjs
node tests/app_status_harness.mjs
node tests/trajectory_lab_harness.mjs
```

`tests/editor_pointer_harness.mjs` 用一个极简 DOM 桩加载真实的 `web/map-editor.js`，回放取帧的按下、平移、转向手势，因此不需要浏览器或构建工具即可回归地图编辑器的指针交互。

`tests/point_cloud_viewer_harness.mjs` 用最小 Canvas/WebGL 桩驱动真实的 `web/point-cloud-viewer.js`，回归拖动时的相机跟随、交互期降采样、实时点云延后上传、俯视、平移和滚轮缩放。

`tests/app_status_harness.mjs` 用 DOM、WebSocket、fetch 桩加载真实的 `web/app.js`，经 HTTP 轮询与 WebSocket 推送回放状态序列，并覆盖自助二维切片：默认参数来自 `status.map_export`、操作者改动不被轮询覆盖、PGM 预览解码与统计行、转换请求体、以及保存指令不再带 `z_max`。

`tests/trajectory_lab_harness.mjs` 直接加载真实的 `web/trajectory-lab.js`，覆盖 MPE2 双通道解码、旋转地图的栅格/世界坐标可逆、真实规划请求体、路径弧长回放，以及临时障碍上限与后端 `MAX_LAB_OBSTACLES` 的一致性。

暂时没有连接雷达时，可以在控制台点击“开始建图”后，用合成房间点云检查完整链路：

```bash
python3 tests/publish_synthetic_cloud.py
```

这只向当前 ROS 2 Domain 临时发布 12 帧测试数据，不会修改导航仓库或雷达配置。

检查后端状态：

```bash
curl http://127.0.0.1:8765/api/status
```

- “点云输入”显示无数据：先检查 `/cloud_registered` 是否在发布，以及 ROS Domain ID 是否一致。
- 网页能连但点数不增长：必须先点击“开始建图”；待机状态不会累计，以免常驻占内存。
- 二维切片预览报错“切片后只有 N 个点”：调整“点云转二维图”里的 `Z 下限/上限` 重新预览，按雷达安装高度取值。
- 保存后只看到 PCD：这是预期行为。二维图不再随保存生成，请在“点云转二维图”里按需生成 PGM/YAML。
- 累计点数达到上限：结束保存，或在内存允许时增大 `--max-points`；不要盲目减小体素尺寸。

## 文件结构

```text
backend/mapping_server.py   ROS 2 节点、HTTP/WebSocket 服务与会话控制
backend/mapping_core.py     体素累计、PCD 与 PGM/YAML 生成
backend/map_frame_core.py   PCD/二维/terrain 成组坐标变换与 frame 元数据
backend/terrain_core.py     PGM 编辑、二维转 terrain、MessagePack 读写
web/                       无外部 CDN 的网页前端与 WebGL 3D 查看器
tests/                     核心数据路径测试
ros2_ws/src/small_point_lio 项目内的 Small Point-LIO ROS 2 源码副本
run.sh                     ROS 环境加载与一键启动脚本
```
