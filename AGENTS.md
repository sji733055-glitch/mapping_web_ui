# Repository Instructions

## Project purpose and boundary

This repository is the standalone web mapping console for the MAS ROS 2 Jazzy
stack. It owns the browser UI, the ROS-facing mapping supervisor, point-cloud
session accumulation, map export, and the optional lifecycle management of the
minimal mapping nodes.

Keep this project independent from `/home/mas/mas_nav_2027_native`. Do not edit
the navigation repository as part of a mapping-web task unless the user
explicitly asks for a cross-repository change. The navigation workspace may be
read to understand message types, node parameters, transforms, or launch
behavior.

The normal integration workspace is:

```text
/home/mas/mas_nav_2027_native
```

The default Small Point-LIO parameter file is:

```text
/home/mas/mas_nav_2027_native/src/mas2027_nav_bringup/config/small_point_lio_params.yaml
```

## Architecture

Keep the implementation dependency-light and preserve these boundaries:

- `backend/mapping_server.py`
  - ROS 2 node and subscriptions;
  - HTTP, REST, and WebSocket server;
  - mapping-session state machine;
  - managed ROS child-process lifecycle;
  - publication of accumulated cloud and JSON status.
- `backend/mapping_core.py`
  - ROS-independent voxel accumulation;
  - map-name validation;
  - binary PCD generation;
  - height slicing, block-min preview pooling and PGM/YAML occupancy-map
    generation.
- `backend/terrain_core.py`
  - ROS-independent occupancy/terrain editor protocol (MPE2, PGM, YAML);
  - MessagePack `terrain`/`direction` channels the navigation `map_server`
    reads.
- `backend/trajectory_lab_core.py`
  - ROS-independent validation and command construction for the isolated
    trajectory laboratory;
  - owns the `/mapping/trajectory_lab` topic namespace and the parameter
    whitelist. Keep it importable without ROS so the isolation rules stay
    unit-testable.
- `backend/map_frame_core.py`, `backend/rogmap_debug.py`
  - ROS-independent map-frame alignment and ROGMap projection diagnostics.
- `web/index.html`
  - semantic structure of the single-page operator console.
- `web/styles.css`
  - responsive visual design and interaction states.
- `web/app.js`
  - backend connection, commands, status rendering, and transport fallback.
- `web/point-cloud-viewer.js`
  - dependency-free WebGL point-cloud rendering and camera interaction.
- `web/map-editor.js`, `web/rogmap-projection.js`, `web/trajectory-lab.js`
  - the map editor, the ROGMap classification workspace and the trajectory
    laboratory surface. Each must stay loadable by the Node harnesses without
    a browser.
- `tests/`
  - ROS-independent unit tests, the Node front-end harnesses, and the optional
    synthetic ROS data source.

Do not introduce rosbridge, a CDN, or a JavaScript build tool unless a new
requirement clearly needs one. The current site is intentionally served as
plain static assets by the Python backend and works without Internet access.

## Runtime contract

The supported start command is:

```bash
cd /home/mas/mapping_web_ui
./run.sh
```

The default operator URL is:

```text
http://127.0.0.1:8765
```

`run.sh` must continue to:

- source `/opt/ros/jazzy/setup.bash`;
- source the navigation overlay when available;
- place ROS logs under this project instead of relying on `~/.ros`;
- start the Python backend without requiring a colcon build.

`run.sh` also keeps `/usr/lib/x86_64-linux-gnu` at the head of
`LD_LIBRARY_PATH` when the Hikvision MVS SDK is installed. MVS ships an older
`libusb-1.0.so.0` that lacks `libusb_set_option`, and the shell profiles prepend
it, which makes every process loading the system `libpcl_io` die with
`undefined symbol: libusb_set_option` — the managed mapping chain and the
isolated trajectory laboratory included. Because `LD_LIBRARY_PATH` is searched
entirely ahead of the `ld.so` cache, appending MVS is not enough; the system
directory must come first. Keep this fix idempotent, a no-op where MVS is
absent, and limited to this script's own process tree, and do not "fix" it by
deleting the vendor library.

The backend must remain usable through VS Code Remote SSH. Preserve both
transport paths:

1. WebSocket for low-latency JSON state and binary point-cloud snapshots;
2. same-origin HTTP polling for status, commands, and point-cloud snapshots
   when the VS Code proxy does not pass WebSocket traffic.

Do not make the UI depend exclusively on WebSocket. The header should clearly
distinguish `实时连接` from `HTTP 兼容模式`.

## ROS 2 interfaces

Preserve the current defaults unless the user requests a breaking change:

```text
Subscribe  /cloud_registered             sensor_msgs/msg/PointCloud2
Subscribe  /Odometry                    nav_msgs/msg/Odometry
Publish    /mapping/accumulated_cloud   sensor_msgs/msg/PointCloud2
Publish    /mapping/status              std_msgs/msg/String containing JSON
Service    /mapping/start               std_srvs/srv/Trigger
Service    /mapping/stop_and_save       std_srvs/srv/Trigger (3-D PCD only)
Service    /mapping/reset               std_srvs/srv/Trigger
```

The input cloud is already in the LIO odom frame. Do not apply a second
coordinate transformation before accumulation or PCD export unless a new input
topic explicitly requires it.

The browser binary-cloud protocol is:

```text
bytes 0..3   ASCII magic `MAP1`
bytes 4..7   little-endian uint32 point count
bytes 8..    tightly packed little-endian float32 XYZ triples
```

Keep WebSocket and `GET /api/cloud` payloads compatible with this format.

### Isolated trajectory laboratory

The trajectory workspace launches a **second**, fully remapped copy of the real
`map_server` and `mas2027_nav_executor` as backend-managed children. It must
never touch the vehicle's topics:

```text
Namespace  /mapping/trajectory_lab/*     (every lab topic, see LAB_TOPICS)
Publish    .../odometry                  nav_msgs/msg/Odometry
Publish    .../goal                      geometry_msgs/msg/PoseStamped
Publish    .../obstacles                 sensor_msgs/msg/PointCloud2
Subscribe  .../global_plan               nav_msgs/msg/Path
Subscribe  .../minco_path                nav_msgs/msg/Path
Subscribe  .../planning_constraints      nav_msgs/msg/OccupancyGrid
Subscribe  .../cmd_vel                   geometry_msgs/msg/Twist (observation only)
```

The vehicle's `/goal_pose`, `/Odometry`, `/cloud_registered` and `/cmd_vel` must
stay unused by the laboratory. `backend/trajectory_lab_core.py` owns the topic
map and the parameter whitelist; when changing them, re-check the parameter
prefixes in the navigation sources (ROGMap loads under `planner.rog_map`) so an
override can never be silently dropped and fall back to a vehicle topic.

Operator parameter sliders are ephemeral `-p` overrides on the isolated child.
They must never be written to `planner_params.yaml`, `node_params.yaml` or any
terrain/YAML file.

## Mapping-session semantics

The LIO process stays alive across mapping sessions. `开始建图` and `结束并保存`
control only this application's accumulator and output session; they must not
restart LIO or reset its odom origin.

Preserve the state machine:

```text
IDLE -> MAPPING -> SAVING -> SAVED
                       \-> ERROR
SAVED/ERROR -> IDLE through reset
```

Important behavior:

- Do not accumulate while the application is idle.
- Starting a session clears only the in-memory application accumulator.
- The start control remains unavailable until `/cloud_registered` is online.
- Saving must snapshot the full accumulated cloud, not the browser-decimated
  view.
- `结束并保存` writes the 3-D PCD only. The 2-D map is always a separate,
  repeatable operator step over a `data/pcd` file, so the height slice and the
  filters can be previewed and retuned without touching the disk. Keep the
  browser preview and the on-disk export on one shared slicing function.
- The offline slice preview must never write files and must stay available in
  HTTP fallback mode; it shares the session-state guard with the conversion
  (`MAPPING`/`SAVING` returns 409).
- Never overwrite an existing map silently. Add a timestamp suffix when the
  requested output name already exists.
- Validate map names before using them as paths. Path separators and traversal
  names must remain rejected.
- A failed 2-D slice must not corrupt an existing PCD, PGM, or YAML file.
- The slice conversion accepts operator parameters (Z window, resolution,
  radius, min_neighbors, padding, output name); validate every one of them
  before it can reach the rasterizer. An empty field means "use the configured
  default".
- Browser and ROS service commands must use the same session methods.

Default output layout:

```text
data/pcd/<name>.pcd
data/map/<name>.pgm
data/map/<name>.yaml
```

Generated map files belong in `data/` and stay ignored by Git except for the
`.gitkeep` placeholders.

## Managed mapping-node lifecycle

The web UI may start the minimal mapping chain:

```text
robot_state_publisher -> mid360_driver -> small_point_lio
```

Keep these safety guarantees:

- Check the ROS graph before starting processes; do not create duplicate nodes.
- Clearly report externally started nodes as external.
- Stop only child processes that were launched by this backend.
- Never use broad `pkill`, `killall`, or name-based termination.
- Start every managed child in its own process group and signal only its exact
  process group.
- Refuse to stop the mapping chain while a session is `MAPPING` or `SAVING`.
- Shut down backend-managed children when the backend exits.
- Preserve managed-process logs under `.ros/managed/`.
- Surface launch failures in status while keeping the HTTP server responsive.

Changing the mapping-node set or launch order requires checking the upstream
navigation parameters and TF requirements first. Small Point-LIO needs the
`base_link -> lidar_link` transform from `robot_state_publisher`.

The isolated trajectory laboratory follows the same rules with its own logs:

- it may only ever start its own `map_server` + `nav_executor` pair, and must
  stop an existing pair before starting a new one;
- stopping signals only the exact process groups it created, escalating
  `SIGINT` → `SIGTERM` → `SIGKILL`, and never matches on process name;
- the backend stops the laboratory on exit, and a child that exits on its own
  is surfaced as `ERROR` in status;
- logs stay under `.ros/trajectory_lab/`.

## Performance and data integrity

- Keep point ingestion vectorized with NumPy; avoid per-point Python work before
  frame-level deduplication.
- Preserve the fixed-voxel accumulator and its configurable hard point limit.
- Browser and ROS publications may be decimated, but PCD export must use all
  retained accumulator points.
- Avoid repeatedly serializing the full cloud more often than configured.
- Keep blocking map export and managed-process waits off the ROS callback path.
- Continue using atomic temporary-file replacement for PCD, PGM, and YAML
  output.
- Treat long mapping sessions and multi-million-point clouds as normal; do not
  add unbounded copies or history buffers.

## UI behavior

The application is an operator working surface, not a marketing site. The first
viewport must continue to expose:

- the 3-D accumulated point cloud;
- mapping-node health;
- start/stop mapping controls;
- mapping progress and output state.

Preserve keyboard/touch accessibility, mobile responsiveness, Chinese operator
labels, and clear offline/error states. A transport failure must not be shown as
a ROS-node failure when the HTTP fallback can still read the backend.

Do not replace the WebGL viewer with a 2-D canvas. It must remain possible to
rotate, pan, zoom, fit the cloud, and switch to a top view.

## Verification

For every code change, run the relevant subset and report anything not tested.
The minimum non-hardware verification is:

```bash
cd /home/mas/mapping_web_ui
python3 -m compileall -q backend tests
python3 -m unittest discover -s tests -v
node --check web/app.js
node --check web/point-cloud-viewer.js
node --check web/map-editor.js
node --check web/rogmap-projection.js
node --check web/trajectory-lab.js
node tests/point_cloud_viewer_harness.mjs
node tests/editor_pointer_harness.mjs
node tests/app_status_harness.mjs
node tests/rogmap_projection_harness.mjs
node tests/trajectory_lab_harness.mjs
bash -n run.sh
```

`tests/editor_pointer_harness.mjs` drives the real `web/map-editor.js` through a
minimal DOM stub, so map-editor pointer interactions stay covered without a
browser.

`tests/point_cloud_viewer_harness.mjs` drives the real WebGL viewer through a
minimal canvas/GL stub. It covers camera movement, interaction-time point
decimation, deferred live-cloud uploads, top view, panning, and wheel zoom.

`tests/app_status_harness.mjs` drives the real `web/app.js` through a minimal
DOM, WebSocket and fetch stub, replaying the status sequence over both
transports, so console state rendering stays covered without a browser. It also
covers the self-service 2-D slice flow: defaults coming from `status.map_export`
without clobbering operator edits, PGM (P5) preview decoding and statistics, the
conversion request body, and a save command that no longer carries `z_max`.

`tests/trajectory_lab_harness.mjs` drives the real `web/trajectory-lab.js`
through a minimal DOM stub. Only the exported `TrajectoryLabCore` helpers run,
so keep pure protocol/geometry/limit rules there and keep the DOM wiring behind
the `dom.canvas` guard.

`tests/test_trajectory_lab_server.py` covers the isolated-laboratory backend
without ROS: the scene guard, the process-group shutdown behaviour and the HTTP
status mapping. Extend it whenever the laboratory gains an endpoint or a
child process.

For backend or transport changes, also verify:

```bash
curl http://127.0.0.1:8765/api/status
curl http://127.0.0.1:8765/api/cloud
```

Do not write binary cloud output to the terminal during verification; inspect
its `MAP1` header and point count through a small parser or save it to an
explicit temporary path.

When no radar is connected, the optional synthetic test path is:

```bash
python3 tests/publish_synthetic_cloud.py
```

When a radar is connected, hardware verification should check:

- all three mapping nodes are visible;
- both `/cloud_registered` and `/Odometry` are live;
- the web backend reports the real point-cloud frequency;
- a short named smoke session accumulates nonzero points;
- ending the session produces the PCD only;
- converting that PCD to a 2-D map then produces PGM and YAML;
- any driver/LIO warnings that occurred during startup are reported.

Do not leave a mapping session active after automated verification. End and
save it or reset the application to `IDLE`. Use distinctive `*_smoke_*` names
for test outputs so they cannot be confused with production maps.

## Change history

Every task that changes source code, configuration, scripts, assets, tests, or
documentation must update `CHANGELOG.md` before handoff.

Add new entries at the top of the history, immediately below the main heading.
Each entry must include:

- the date and a short title;
- the behavior changed and the main files/components involved;
- verification performed, including hardware/browser checks not performed.

Do not rewrite or delete older entries. Documentation-only changes, including
changes to this file, also require a history entry.
