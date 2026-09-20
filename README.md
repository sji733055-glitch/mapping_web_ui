# MAS 三维建图控制台

这是一个放在 `/home/mas/mapping_web_ui` 的独立 ROS 2 网页工具。它不修改导航仓库，也不依赖 rosbridge：后端直接订阅 Small Point-LIO 的实时点云与里程计，浏览器通过同一个服务查看三维累计点云并控制建图会话。

## 能做什么

- 点击“开始建图”后，从空会话开始累计 `/cloud_registered`；
- 网页以 3D 方式显示降采样后的累计点云，支持旋转、平移、缩放和俯视；
- 实时显示累计点数、覆盖范围、地图尺寸、建图时长、行驶里程与输入频率；
- 点击“结束并保存”后生成三维 PCD 和 Nav2 可读的 PGM/YAML；
- 在“地图编辑”工作区用笔刷、矩形或画线修整二维 PGM 占据图，并可撤销、缩放和平移；
- 可直接在二维图上点选新 `map` 原点并拖出 `map +X` 方向，成组变换完整 PCD、PGM/YAML 和已有 terrain，为后续重定位发布 `map→odom` TF 固定统一的地图基准；
- 把 PGM/YAML 一键转换为 HW `map_server` 使用的 terrain msgpack，再标注平地、障碍、斜坡、各级台阶、飞坡及其方向；
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

## 地图编辑流程

完成一次建图并保存后，点击页头的“地图编辑”：

1. 从 `data/map` 选择地图，点击“编辑二维 PGM”。二维层提供障碍物、可通行、未知区域三类像素，可用笔刷、矩形和画线修图；保存只原子替换该地图 YAML 引用的 PGM，不改变 YAML 的分辨率或原点。
2. 在“map 坐标系”中点击“在图上拖出原点与 +X”后有三种手势：在空白处按下＝该点成为新 `(0,0)` 并拖出 `map +X`；拖动原点圆圈＝平移坐标系且保持朝向；拖动 +X 箭头＝只转向且保持原点。也可以在输入框中直接填写原点在当前地图中的 X/Y 和 +X 朝向角。点击“统一 PCD 与二维图坐标系”并确认后，后端会对同名完整 PCD 应用相同二维刚体变换，把 PGM 和已有 terrain 最近邻重采样到 yaw=0 的轴对齐栅格，并同步旋转 terrain 方向。建议在精修 PGM/terrain 前先定义坐标系，避免重复重采样。
3. 点击“生成 terrain MSG”。后端按 YAML 的 `occupied_thresh` 和 `negate` 把二维图转换为 `<地图名称>_terrain.msgpack`，随后自动打开 terrain 图层。
4. 在 terrain 图层选择 `0–6` 标签；斜坡、台阶和飞坡可设置方向。画线工具会使用线段左侧法向作为通过方向，笔刷和矩形使用方向滑块的值。
5. 点击“保存当前图层”。输出采用 `nav_opensource/HW/HWSentryNav26/map_server` 兼容的 `width`、`height`、`resolution`、`terrain`、`direction` MessagePack 字段。

坐标系设置需要 `data/pcd/<名称>.pcd` 与 PGM/YAML 同名存在；缺少 PCD 时网页会禁用应用按钮，避免只改二维图。操作会生成 `<名称>_frame.json`，其中 `source_to_map` 是把原始 LIO/odom 点坐标变换到固定 map 坐标的变换，也就是后续 TF 中 `map→odom` 所需的平面变换语义；`map_to_source` 是其逆变换。重复定义会在元数据中累计组合，但每次都需要重采样二维栅格，因此应尽量一次定准。

重新从 PGM 生成已存在的 terrain 文件前，网页会明确提示该操作会覆盖已有语义和方向标注。地图编辑和坐标系设置使用普通同源 HTTP 接口，不依赖 WebSocket，因此在 VS Code Remote SSH 的 HTTP 兼容模式下也能使用。

## 输出

默认保存到本项目内：

```text
data/
├── pcd/<地图名称>.pcd
└── map/
    ├── <地图名称>.pgm
    ├── <地图名称>.yaml
    ├── <地图名称>_terrain.msgpack  # 在地图编辑工作区生成
    └── <地图名称>_frame.json       # source↔map 变换与坐标系修订记录
```

刚保存时 PCD 与 `/cloud_registered` 使用相同数值坐标，`map` 与点云源坐标系按单位变换记录。PGM/YAML 默认用 `Z=0.05–1.50 m` 的切片生成；二维图的有点栅格为占用，其余栅格为空闲，与现有 `pcd2pgm` 的投影语义一致。自定义 map 基准后，PCD 点、二维栅格和 terrain 都处于同一个 `map` 坐标系，YAML 栅格 yaw 保持为 0，以兼容只接收 `origin_x/origin_y` 的 HW terrain server。

## 常用参数

```bash
./run.sh \
  --cloud-topic /cloud_registered \
  --odom-topic /Odometry \
  --voxel-size 0.05 \
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
| `--z-min` / `--z-max` | `0.05` / `1.50` | 生成二维地图时保留的高度范围 |
| `--radius-filter` | `0.50` | 二维切片离群点滤波半径；设为 `0` 可关闭 |
| `--min-neighbors` | `10` | 半径内最少邻点数 |
| `--output-dir` | `./data` | PCD 和地图输出根目录 |
| `--lio-params` | 导航仓库中的参数 YAML | 驱动与 LIO 共用的参数文件 |
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
服务  /mapping/stop_and_save       std_srvs/srv/Trigger
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
```

`tests/editor_pointer_harness.mjs` 用一个极简 DOM 桩加载真实的 `web/map-editor.js`，回放取帧的按下、平移、转向手势，因此不需要浏览器或构建工具即可回归地图编辑器的指针交互。

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
- 二维地图保存失败：通常是 `z-min/z-max` 切片后没有点，按雷达安装高度调整范围。
- 累计点数达到上限：结束保存，或在内存允许时增大 `--max-points`；不要盲目减小体素尺寸。

## 文件结构

```text
backend/mapping_server.py   ROS 2 节点、HTTP/WebSocket 服务与会话控制
backend/mapping_core.py     体素累计、PCD 与 PGM/YAML 生成
backend/map_frame_core.py   PCD/二维/terrain 成组坐标变换与 frame 元数据
backend/terrain_core.py     PGM 编辑、二维转 terrain、MessagePack 读写
web/                       无外部 CDN 的网页前端与 WebGL 3D 查看器
tests/                     核心数据路径测试
run.sh                     ROS 环境加载与一键启动脚本
```
