# Change history

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
