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
  - PGM/YAML occupancy-map generation.
- `web/index.html`
  - semantic structure of the single-page operator console.
- `web/styles.css`
  - responsive visual design and interaction states.
- `web/app.js`
  - backend connection, commands, status rendering, and transport fallback.
- `web/point-cloud-viewer.js`
  - dependency-free WebGL point-cloud rendering and camera interaction.
- `tests/`
  - ROS-independent unit tests and the optional synthetic ROS data source.

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
Service    /mapping/stop_and_save       std_srvs/srv/Trigger
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
- Never overwrite an existing map silently. Add a timestamp suffix when the
  requested output name already exists.
- Validate map names before using them as paths. Path separators and traversal
  names must remain rejected.
- A failed 2-D slice must not corrupt an existing PCD, PGM, or YAML file.
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
bash -n run.sh
```

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
- PCD, PGM, and YAML outputs are all produced;
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
