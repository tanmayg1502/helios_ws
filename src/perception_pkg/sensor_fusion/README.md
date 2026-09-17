# sensor_fusion

Combines the wheel odometry and the camera's visual-inertial odometry into one
position estimate, and provides the launch file that brings up the entire
sensing layer.

This package is the boundary of the perception layer. It produces
`/odometry/filtered` and the `odom → base_link` transform, and stops there; it
starts no mapper.

**Read [the root README](../../../README.md) first** for the system overview.

---

## Why fuse at all

Two things on this robot independently estimate motion, and both are wrong in
different ways:

| Source | Good at | Bad at |
|---|---|---|
| Wheel encoders (`/wheel/odometry`) | Always available, smooth, accurate short-term | Slips, especially sideways on mecanum wheels |
| ZED visual-inertial (`/zed/odom_with_cov`) | No slip, low drift over distance | Fails in dark or featureless spaces; can lose tracking entirely |

An **EKF** (Extended Kalman Filter) merges them. The short version of how: it
keeps a running estimate of the robot's state plus a measure of how uncertain
that estimate is; each new measurement pulls the estimate toward itself in
proportion to how much that source is trusted. A source declared noisy moves the
estimate less. The result tracks better than either input alone, and degrades
gracefully when one input goes bad.

We use `robot_localization`, the standard ROS implementation. This package only
supplies its configuration.

### Two decisions that shape the config

**Only velocities are fused, never absolute positions.** Both sources report
position *and* velocity. Their positions drift, and fusing two independently
drifting positions produces a fight between them. Velocities do not accumulate
error. So the EKF takes `vx, vy, ωz` from both sources and integrates the
position itself: one consistent estimate rather than two competing ones.

**The IMU is deliberately not a separate input.** The ZED's `/odom` is already
visual-*inertial*: the SDK fuses camera and IMU internally. Adding the raw IMU
as a third input would count the same physical measurement twice, making the
filter overconfident and prone to overshoot. The IMU block in `ekf.yaml` is
present but commented out, with a note on the one condition under which it
should be enabled.

---

## Transform ownership

This is the rule that most often breaks a ROS robot, so it is worth being
explicit. Every transform must have exactly one publisher. Two publishers do not
produce an error; the position simply flickers between two answers and
everything downstream behaves strangely.

```
map ──[mapping layer]──► odom ──[this package's EKF]──► base_link ──[URDF]──► sensors
    (NOT owned here)
```

| Transform | Sole publisher |
|---|---|
| `map → odom` | `mapping_localization_pkg`: slam_toolbox **or** RTAB-Map, never both |
| `odom → base_link` | `ekf_filter_node`, from this package |
| `base_link → sensors, wheels` | `robot_state_publisher`, from `helios_description` |

Two nodes are capable of publishing `odom → base_link` and are actively
prevented from doing so:

- **`wheel_odometry_node`** is started with `publish_tf: False`.
- **The ZED wrapper** is started with `publish_tf:=false`.

Both still publish their measurements as *topics*, which is what the EKF
consumes. They just are not allowed to state where the robot is.

---

## Files

```
config/
  ekf.yaml               All EKF tuning. The substance of this package.
launch/
  bringup.launch.py      Starts the whole sensing layer. The one you run.
  ekf.launch.py          Starts only the EKF.
rviz/
  visual_odometry_with_lidar.rviz  Layout for camera + laser together.
CMakeLists.txt           Installs config/, launch/ and rviz/ into share/.
package.xml              Metadata; declares the dependency on robot_localization.
```

### Per-sensor content lives with its sensor, not here

Anything specific to one sensor sits beside that sensor's vendor submodule, so
everything for a given device is in one place:

| | where | what |
|---|---|---|
| Camera | [`camera/custom_covariance/`](../camera/custom_covariance/README.md) | Republishes the ZED's odometry with the twist covariance the wrapper never sets |
| Camera | [`camera/zed_custom_tuning/`](../camera/zed_custom_tuning/README.md) | Our parameter overrides on the ZED wrapper, passed as `ros_params_override_path` |
| LiDAR | [`lidar/custom_config/`](../lidar/custom_config/README.md) | Hokuyo driver parameters, its launch file, and the laser-only RViz layout |

All three are **ours**, deliberately placed *outside* the submodule they sit
next to, which is the entire point. Editing vendor content inside a pinned
submodule gets silently reverted by `git submodule update`.

`bringup.launch.py` pulls all three in. What remains in this package is the fusion
layer itself: the EKF, its config, and the layout that shows both sensors at
once.

### `config/ekf.yaml`

This package has no source code: the EKF is `robot_localization`'s node, and
this file is what makes it a *mecanum rover* EKF rather than a generic one.

| Parameter | Value | Meaning |
|---|---|---|
| `frequency` | 30.0 | Output rate, Hz |
| `sensor_timeout` | 0.2 | Seconds before an input is treated as stale |
| `transform_time_offset` | 0.02 | Future-dates the published transform. **See below before changing it** |
| `smooth_lagged_data` | true | Rewind and re-apply a measurement that arrives late |
| `history_length` | 0.3 | Seconds of state history the rewind can reach back through |
| `two_d_mode` | true | Force z, roll, pitch to zero |
| `publish_tf` | true | This node owns `odom → base_link` |
| `world_frame` | `odom` | Makes this a *local* filter |

**`two_d_mode: true`** because the rover drives on flat floors and the laser
scanner sees a flat slice. Forcing z, roll and pitch to zero removes three
noise-only degrees of freedom rather than estimating quantities that cannot
usefully be observed.

**`world_frame: odom`** makes this a *local* filter: it produces a smooth,
continuous estimate that drifts slowly, which is what you want for control. The
non-drifting global estimate is the mapping layer's job.

**Input configuration.** Each source has a 15-element boolean mask over
`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`.
Both `odom0` (wheels) and `odom1` (ZED) enable exactly `vx`, `vy` and `vyaw`,
which is the "velocities only" decision made concrete.

`odom1_twist_rejection_threshold: 5.0` discards ZED velocity readings that jump
implausibly. It is a Mahalanobis gate, so its meaning depends on the ZED's
measurement covariance: at `vyaw` variance 0.01 (sigma 0.1 rad/s), 5.0 rejects
disagreements beyond 0.5 rad/s, about 28.6 deg/s. That is permissive enough to
survive the transient disagreement at the start and end of every turn, and still
tight enough to catch a gross VIO failure.

It was raised from 2.0, which rejected past only 11.5 deg/s and so threw the ZED
out during exactly the manoeuvres it is best at. Against a de-biased gyro over a
99.7 s spin, the ZED twist was the most accurate yaw source on the robot
(+0.03%) while the wheels read 3.3% low. Note that `odom0` (wheels) has **no**
gate at all, so an encoder spike enters unfiltered.

**`transform_time_offset: 0.02`, and keep it small.** This parameter does
**not** extrapolate the pose forward. It relabels the same pose with a later
timestamp, so every millisecond of it is pure systematic lag: a consumer asking
for time T is handed the pose the robot was actually at some time earlier.

It was 0.1, and that was measured causing real harm. The published
`odom → base_link` came out future-dated by +100.6 ms relative to
`/odometry/filtered`. At 0.83 rad/s that misplaces every laser scan by 4.8 deg,
which is what smeared the map and produced doubled wall lines during rotation.
The visible symptom on the robot was the scan appearing to swing backwards for
about a second when you started a turn, then catching up.

0.02 is the smallest value that still keeps a small timing margin for consumers
whose data is stamped fresher than the EKF's newest fused measurement. If RViz
starts reporting "extrapolation into the future" on camera data, the fix is
`smooth_lagged_data` and the queue settings, **not** a larger offset here.

**`smooth_lagged_data: true` with `history_length: 0.3`.** The two inputs do not
arrive with the same delay. Measured header-stamp to arrival:

| Input | median | p90 | max |
|---|---|---|---|
| `/wheel/odometry` | 1.6 ms | | |
| `/zed/odom_with_cov` | 58.1 ms | 69 ms | 315 ms |

That is roughly 1.7 filter cycles of skew. Without the rewind, the ZED
measurement is folded in with a clamped zero time delta rather than at the
instant it describes. 0.3 s clears the measured p90 by 4x, and is deliberately
not larger: every lagged measurement re-integrates the whole window.

Process and initial covariances are left at `robot_localization` defaults.

### `launch/bringup.launch.py`

The whole sensing layer in one command, in order: robot description → wheel
odometry → ZED → `zed_odom_covariance_node` → laser → EKF. Two of its settings
are non-obvious and are documented at length inside the file:

- **`enable_ipc:=false` on the ZED wrapper is required, not an optimisation.**
  With intra-process communication enabled, the wrapper cannot use a static
  transform broadcaster and instead republishes the camera's *geometrically
  fixed* internal frames as dynamic transforms at frame rate, from the same
  thread doing depth processing. Under load that thread slips, those transforms
  go stale, and every timestamped lookup into the camera chain fails, which was
  making RTAB-Map discard about a third of all frames.

- **`node_name` is passed explicitly to the laser driver.** Launch
  configurations leak between sibling includes in a single launch description,
  and the ZED include sets `node_name` first. Without this, the laser driver
  inherited it and came up as `/zed_node`, silently breaking anything
  addressing it by name.

Arguments: `camera:=false`, `lidar:=false`, `rviz:=true`.

### `launch/ekf.launch.py`

Just `ekf_filter_node` with `ekf.yaml`. Use it when the drivers are already
running and you want to restart only the filter, for instance after editing
tuning values. Included by `bringup.launch.py` as its final step.

---

## Build

```bash
cd ~/helios_ws
colcon build --packages-select sensor_fusion --symlink-install
source install/setup.bash
```

Requires `robot_localization`:

```bash
sudo apt install ros-jazzy-robot-localization
```

With `--symlink-install`, edits to `ekf.yaml` take effect on the next launch
with no rebuild.

## Run

```bash
# The whole sensing layer; this is the normal command
ros2 launch sensor_fusion bringup.launch.py

# Wheels only, no camera (useful when the ZED is unavailable)
ros2 launch sensor_fusion bringup.launch.py camera:=false

# Just the EKF, drivers already running
ros2 launch sensor_fusion ekf.launch.py
```

**Keep the rover still for the first ~5 seconds.** The ZED aligns its sense of
"down" against gravity at startup; moving during that logs a realignment warning
and starts tracking from a worse estimate.

A one-off "failed to meet update rate" warning at startup is normal: the ZED
SDK is loading its depth model onto the GPU and briefly starves the CPU.

To add mapping, run one of these separately on top:

```bash
ros2 launch mapping_localization_pkg slam_toolbox.launch.py   # 2D
ros2 launch mapping_localization_pkg rtabmap.launch.py        # 3D
```

---

## Verify

**1. Both inputs are arriving.** The EKF publishes output even when starved of
input, so check the inputs first:

```bash
ros2 topic hz /wheel/odometry          # matches encoder rate, ~30 Hz
ros2 topic hz /zed/odom_with_cov       # ~30 Hz, what the EKF actually fuses
ros2 node info /ekf_filter_node        # both must appear as subscriptions
```

`/zed/odom_with_cov` silent while `/zed/zed_node/odom` publishes means
`zed_odom_covariance_node` is not running, and `odom1` is starved. The raw
wrapper topic must never be fused directly; see
[`custom_covariance`](../camera/custom_covariance/README.md) for why.

**2. Output is running:**

```bash
ros2 topic hz /odometry/filtered       # ~30 Hz, steady
```

**3. The transform tree is correct.** The single most important check:

```bash
ros2 run tf2_tools view_frames         # writes frames.pdf
```

Expect one connected tree: `odom → base_link → sensors and wheels` (plus
`map → odom` once a mapper runs). No frame may appear twice.

To confirm nobody else is publishing `odom → base_link`:

```bash
ros2 topic info /tf --verbose | grep "Node name"
```

`ekf_filter_node` should be the only odometry-related publisher. Seeing
`wheel_odometry` or a ZED node there means a `publish_tf` override did not take.

**4. It tracks real motion.** With the rover stationary, `/odometry/filtered`
velocities should read ~0 and the pose should not creep. Then push it forward
about a metre:

```bash
ros2 topic echo /odometry/filtered --field pose.pose.position
```

`x` should increase by roughly a metre. Rotating in place should change
orientation while position stays put.

**5. Fusion is actually helping.** Compare the two inputs against the output
during the same motion. Plot all three in RViz as *Odometry* displays. The
fused track should sit between the two inputs and be smoother than either. If it
follows one input exactly, the other is being rejected: check its covariances
and the rejection threshold.

**6. Visually,** with `rviz:=true`, set Fixed Frame to `odom`. The laser scan
should stay locked to the world as the robot moves, not swim. Scan points
sliding across static walls means the fused estimate disagrees with reality.

---

## Tuning

- **The ZED is unavailable:** `camera:=false` runs wheel-only. Expect faster
  drift, especially in heading.
- **`vy` looks noisy:** mecanum slips sideways. Its covariance in
  `wheel_odometry.yaml` is already 5× the longitudinal one for exactly this
  reason. Raise it further if the fused estimate still pulls off-axis when
  strafing.
- **Heading drifts in `odom`:** expected, and corrected by the mapping layer at
  the `map` level. Tightening it before SLAM means enabling the IMU block in
  `ekf.yaml`, which is only valid if you first disable the ZED's internal IMU
  fusion so `/zed/zed_node/odom` becomes purely visual.
- **Estimate is jumpy:** tune `process_noise_covariance`, currently at defaults.
  Raise it to trust measurements more, lower it to trust the model more.
- **The map smears or walls double during turns:** check
  `transform_time_offset` first. It is lag, not extrapolation, so a large value
  misplaces every scan by `yaw_rate x offset`. Keep it at 0.02.
- **RViz reports "extrapolation into the future" on camera data:** raise
  `history_length` and check `smooth_lagged_data` is on. Do **not** raise
  `transform_time_offset` to paper over it.

---

## Related

- [`wheel_odometry`](../wheel_odometry/README.md): one of the two inputs
- [`perception_pkg`](../README.md): sensor hardware setup
- [`helios_description`](../../helios_description/README.md): provides
  `base_link` and below
- [`mapping_localization_pkg`](../../mapping_localization_pkg/README.md):
  consumes `/odometry/filtered`
