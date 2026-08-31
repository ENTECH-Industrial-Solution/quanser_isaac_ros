# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single ROS 2 Jazzy `ament_cmake` package, `qcar2_isaac_nav2`, that drives a
Quanser QCar2 (Ackermann-steered car) autonomously inside NVIDIA Isaac Sim using
Nav2 1.3.12. Not a git repository.

`src/qcar2_isaac_nav2/README.md` (Thai) is the user-facing doc: file-by-file roles,
run instructions, and the rationale behind every tuned parameter. Read it before
changing config — most values there are the result of measurement, not defaults.

## Build

```bash
cd ~/entech_quanser_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qcar2_isaac_nav2
source install/setup.bash
```

There are no tests. `colcon test` only runs `ament_lint_auto`, with copyright and
cpplint explicitly disabled in `CMakeLists.txt`. `config/`, `launch/`, `rviz/`, `behavior_trees/` and `maps/`
are installed by `install(DIRECTORY ...)`, so **any edit to a YAML, lua, launch,
rviz, BT or map file requires a rebuild before it takes effect** — a very easy
mistake to make when a change appears to do nothing.

`setup.py` / `setup.cfg` are vestigial leftovers from an `ament_python` skeleton
(they reference a `resource/` dir that does not exist). The package builds through
`CMakeLists.txt`; ignore them.

## Verifying changes

Nothing here can be validated by unit tests — correctness means "the car reaches
the goal in Isaac Sim". The working loop:

```bash
# Isaac Sim must already be PLAYING.
ros2 launch qcar2_isaac_nav2 qcar2_navigation_launch.py use_rviz:=false \
    > /tmp/nav.log 2>&1 &

# Bringup succeeded only when BOTH lifecycle managers report this:
grep -E "Managed nodes are active|Failed to bring up" /tmp/nav.log   # expect 2 hits

# Then drive a goal and measure it.
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

`SUCCEEDED` alone is a weak signal. The useful metric for this Ackermann robot is
**path driven ÷ straight-line distance** and **total heading change**, sampled from
`map -> base_link` while the goal runs. Healthy is roughly 0.9–1.3x and under ~80°;
circling shows up as 3–5x and 300°+ long before the goal reports failure.

## Architecture: the control path

```
Isaac Sim ──/clock /scan /imu /odom, TF odom->base_link──> Nav2
Nav2 controller_server ──/cmd_vel_nav (TwistStamped, yaw rate rad/s)──┐
                                                                      ▼
                                              src/twist_stamped_to_twist.py
                                                                      │
Isaac Sim QCar2 drive graph <──/cmd_vel_twist (Twist, STEERING ANGLE rad)──┘
```

Two phases, deliberately separate. `qcar2_mapping_launch.py` runs Cartographer and
supplies `map -> odom`; `qcar2_navigation_launch.py` runs `map_server` + AMCL on a
saved map and AMCL supplies `map -> odom` instead. Isaac Sim always owns
`odom -> base_link` and everything below it. The legacy
`qcar2_slam_and_nav_bringup_launch.py` runs SLAM and Nav2 together with no saved map.

### Two sensing routes, one Nav2 half

There is a second, complete route that uses the RealSense instead of the lidar:

| | mapping | navigation | `map -> odom` from |
|---|---|---|---|
| lidar | `qcar2_mapping_launch.py` | `qcar2_navigation_launch.py` | Cartographer / AMCL |
| camera | `qcar2_vslam_mapping_launch.py` | `qcar2_vslam_navigation_launch.py` | RTAB-Map (both) |

Only the sensing half differs — planner, controller, behavior trees and the twist
bridge are shared, and `config/qcar2_nav2_vslam.yaml` is `qcar2_nav2_amcl.yaml`
with `/scan` removed and the obstacle layer swapped for a VoxelLayer fed by the
depth cloud. Keep the two params files in step when tuning anything else.

The camera route needs an Isaac-side change first: `scripts/isaac_add_rgbd_camera.py`
adds a camera under `realsenseRGB` so colour and depth come off ONE render product
(registered, identical stamps — `approx_sync: false` depends on this). The asset's
own `/realsense_depth` is 37 mm off-axis from the colour camera and is not usable
for RGB-D SLAM, though it stays correct for `depthimage_to_laserscan`.

The camera route produces **two** artifacts and needs both: `~/.ros/qcar2_vslam.db`
(pose graph + visual words, not in the repo, not reproducible without re-driving)
and `maps/qcar2_vslam_map.pgm` served by `map_server`. Letting RTAB-Map serve `/map`
itself instead means holding every node's grid in working memory — 6 GB on this map,
enough for the OOM killer to take Isaac Sim with it.

Two RTAB-Map settings fail silently if changed: `RGBD/MaxOdomCacheSize: 1` (the
default 10 waits for several consistent localizations, which a *parked* car never
produces — `map -> odom` is then never published and every goal fails with
"map does not exist", with nothing logged as an error), and `Mem/InitWMWithAllNodes`,
which must track `start_at_origin` — an empty working memory with no given pose also
never publishes `map -> odom`. `README.md` has the full list.

The V-SLAM sources have gone missing from `src/` once already while the built copies
survived in `install/`. If a `qcar2_vslam_*` file is referenced but absent, check
`install/qcar2_isaac_nav2/` before rewriting it.

### A third route: the CSI ring + YOLO

`qcar2_yolo_launch.py` runs `src/yolo_detector.py` over the four CSI cameras
(`csi_front/back/left/right`, 97.6 deg each, 390 deg total). Pure sensing: it
publishes `/csi_<pos>/detections`, `/csi_<pos>/annotated` and a tiled
`/csi/mosaic`, and touches no TF, costmap or cmd_vel, so it can start and stop
under a running Nav2 stack.

The asset ships **four complete, correctly wired CSI graphs**; three are merely
`active = False`. An inactive prim composes away entirely - its children are never
composed, so a stage traversal for OmniGraph prims does not list it and it reads
as absent rather than disabled. Do not "build the missing graphs": that layers
duplicates onto prims that already exist in the referenced `qcar2.usd`.
`scripts/isaac_add_csi_cameras.py` activates them and patches topicName,
frameSkipCount and the render product's `enabled` flag - pure USD attributes, no
omni.graph runtime, which is why it can be verified against a copied stage with
plain pxr. `active` is authored as a root-layer override; `qcar2.usd` is untouched.
No camera_info is published: YOLO is 2D and an RViz *Image* display does not read it.

**Render products are the GPU budget, and `frameSkipCount` does not touch them.**
A render product renders every frame regardless of subscribers; only disabling it
reclaims GPU. `scripts/isaac_camera_streams.py` is therefore preset-based
(`QCAR2_CAMERA_PRESET`): `vslam` (1 product), `csi` (4), `csi_front` (1), `both`
(5), `none`. Four cameras starved the RGB-D pair to 1.6 Hz on this laptop, so the
CSI ring and V-SLAM are meant to take turns; `csi_front` + `cameras:=front` is the
way to run both.

**`cv_bridge` segfaults on this machine and must not be used.** Jazzy's cv_bridge
is built against NumPy 1.x while `~/.bashrc` sources Isaac's `setup_python_env.sh`,
putting NumPy 2.5.2 first on PYTHONPATH. One conversion exits 139 with no Python
traceback, which reads like a driver fault rather than a dependency clash.
`yolo_detector.py` converts `sensor_msgs/Image` with plain NumPy instead
(`decode_rgb`/`encode_rgb`) - do not "simplify" that back.

Other things that cost time here: Isaac writes
`omni:rtx:autoExposure:enabled = False` onto any camera it attaches a render
product to, and a blown-out white image yields zero detections on a
healthy-looking pipeline (the add-CSI script forces it back on); ultralytics 8.4
replaced `half` with `quantize` and warns on *every* predict call otherwise; and
`ros2 run` leaves an orphan Python child when its own PID is killed, so a stale
node keeps publishing - `pgrep -af yolo_detector` before believing a result.

ultralytics is a pip package in `~/.local` (`pip install --user
--break-system-packages ultralytics`), deliberately not a `package.xml` dep.
Weights cache in `~/.cache/qcar2_yolo/`.

Measured: 4 cameras x ~9.5 Hz, 0% dropped, on yolo11n fp16 - but with the sim
stopped, so treat it as an upper bound.

### The bridge is not a format shim

`src/twist_stamped_to_twist.py` performs a **unit conversion**, not just a message
retype. Nav2 publishes `angular.z` as a yaw rate; the Isaac Sim QCar2 drive graph
reads `angular.z` as a front-wheel **steering angle** in radians, clamped near
0.5 rad. Measured on this scene by holding a constant Twist and reading `/odom`:
commanding 0.30 yields a 0.797 m radius, matching `0.258/tan(0.30)`, not `v/ω`.

The bridge applies the bicycle model `δ = atan(ω·L/v)` with `L = 0.258 m`. Removing
or bypassing it makes the car steer several times harder than Nav2 asked, so it
overshoots, gets corrected, overshoots back — the car drives in circles while every
Nav2 log line looks healthy. If a future change reintroduces circling from open
space, re-measure this relationship first.

## Invariants that break the stack silently

- **`enable_stamped_cmd_vel: true` on every Nav2 node** that touches `cmd_vel`.
  Jazzy defaults to `false` (plain `Twist`) while the bridge subscribes as
  `TwistStamped`. Two incompatible types on one topic deliver zero messages: Nav2
  plans perfectly and the car never moves. Check with
  `ros2 topic info -v /cmd_vel_nav` — one type, not two.
- **Both behavior trees must be overridden**, not just `navigate_to_pose`.
  `spin` is absent from `behavior_server.behavior_plugins` (an Ackermann car cannot
  rotate in place), and `bt_navigator` loads `navigate_to_pose` *and*
  `navigate_through_poses` at activation. Leaving either on the stock tree fails
  activation with `Exception when loading BT: Action server spin not available`.
- **No empty YAML lists.** `polygons: []`, `observation_sources: []`, `docks: []`
  have no type; `collision_monitor` and `docking_server` abort with
  `parameter_value_from failed ... No parameter value set`. Use typed entries with
  `enabled: False`.
- **The map path is substituted into the params file**, never passed as
  `localization_launch.py`'s `map:=`. That argument arrives as a second params file
  scoped to `/**:`, and ROS 2 gives an explicit `map_server:` key priority over a
  `/**:` wildcard regardless of order — the empty `yaml_filename` wins and
  `map_server` starts blank. The substitution also must happen inside the
  `OpaqueFunction` in `qcar2_navigation_launch.py`, because
  `IncludeLaunchDescription` applies `launch_arguments` in the *child* scope where
  `map` has already been reset, so a lazily-evaluated `LaunchConfiguration('map')`
  reads the wrong value.
- **Robot geometry is measured, not assumed.** `base_link -> hub_frontLeft` and
  `base_link -> wheel_rearLeft` give wheelbase 0.258 m and a ~0.39 x 0.19 m body, so
  `footprint`, `minimum_turning_radius`/`min_turning_r` (0.5) and the bridge's
  `wheelbase` param must stay consistent with each other and with Isaac Sim.

## Operational gotchas

**Never press Stop/Play in Isaac Sim while ROS is running.** `/clock` resets to 0;
Cartographer dies on the time jump (`Check failed: imu_data.time >= ...`),
`map -> odom` disappears, and goals then abort in ~13 ms with
`Could not find a connection between 'map' and 'base_link'`. Restart the ROS side
after every sim replay.

**Never leave two stacks running.** Duplicate nodes make the second lifecycle
manager abort (`Failed to change state for node: route_server`) and several `/map`
publishers of differing size produce endless
`Received map message is malformed. Rejecting.`. `ros2 node list | sort | uniq -d`
must print nothing.

**A stalled car is usually wedged in inflation, not broken config.** A car that
previously crashed sits in a costmap cell valued `253` (inscribed); from there the
planner emits 15 m paths and MPPI commands ~0 m/s indefinitely. Teleop it back into
free space before concluding a config change failed.

**Killing processes: `pkill -f` matches your own shell's command line.** A command
containing both a kill pattern and the launch string kills the shell mid-script
(exit 144, later commands silently skipped). Collect PIDs first, or put the kill
and the relaunch in separate invocations:

```bash
for p in $(pgrep -f "/opt/ros/jazzy/lib/nav2"; pgrep -f "twist_stamped_to_twist.py"); do
  [ "$p" != "$$" ] && kill -9 $p 2>/dev/null
done
```

Kill only ROS processes — leave Isaac Sim (`release/kit/kit`) alone, it takes
minutes to restart and holds the scene state.

`/scan` from Isaac Sim runs at only ~4 Hz, which constrains mapping speed (drive
below ~0.5 m/s) and is worth remembering before blaming SLAM tuning.
