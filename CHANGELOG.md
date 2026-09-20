# Change history

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
