# Local ROS 2 overlay

This workspace contains the mapping console's local copy of
`small_point_lio`:

```text
ros2_ws/src/small_point_lio
```

The package was copied without source changes from:

```text
/home/mas/mas_nav_2027_native/src/mas2027_perception/Odometry/small_point_lio
```

The navigation workspace remains a read-only underlay. Build only the local
package from this directory:

```bash
cd /home/mas/桌面/mapping_web_ui/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/mas/mas_nav_2027_native/install/setup.bash
colcon build --symlink-install --packages-select small_point_lio \
  --allow-overriding small_point_lio
```

`../run.sh` sources the navigation overlay first and this workspace's
`install/setup.bash` second when it exists. The web supervisor launches the
exact executable under this workspace's `install/small_point_lio`; it does not
fall back to the navigation workspace's same-name executable. If the local
package is missing or its C/C++ source is newer than the executable, the web
start command reports that a rebuild is required. The HTTP server itself still
starts normally, and `run.sh` never invokes `colcon` itself.

The mapping console still uses the navigation bringup parameter file by
default:

```text
/home/mas/mas_nav_2027_native/src/mas2027_nav_bringup/config/small_point_lio_params.yaml
```

The copied `small_point_lio/config/mid360.yaml` is the package's upstream
sample and is not the console's default runtime configuration.

Useful point-cloud processing integration points are:

```text
src/small_point_lio/preprocess.cpp          raw/pre-registration filtering
src/small_point_lio/small_point_lio.cpp     deskew and estimator data path
src/small_point_lio_node.cpp                registered odom-frame cloud output
```

Re-run the build command after changing C or C++ source. The web page will
label a managed LIO process as `项目内启动`.
