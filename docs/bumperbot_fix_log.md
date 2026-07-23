# Bumperbot — Fix & Progress Log

> Track the state of the `ChepuriNatraj/bumperbot` ROS 2 (Jazzy) workspace.
> Workspace: `/home/botforgelabs2/Desktop/bumperbot`
> Robot name: `agv` (differential-drive mobile robot).

---

## 1. Initial stage (what we started with)

- Cloned `ChepuriNatraj/bumperbot` to `~/Desktop/bumperbot`.
- Machine already had the full environment: **Ubuntu 24.04 + ROS 2 Jazzy + colcon + Gazebo (`gz`) + Docker**.
- All required ROS packages were already installed:
  `ros-jazzy-velocity-controllers`, `ros-jazzy-diff-drive-controller`,
  `ros-jazzy-slam-toolbox`, `ros-jazzy-nav2-bringup`,
  `ros-jazzy-teleop-twist-keyboard`, `ros-jazzy-ros2-control`,
  `ros-jazzy-robot-localization`, `ros-jazzy-navigation2`.
- `colcon build` succeeded — **7/7 packages** built (only harmless RPLiDAR SDK compiler warnings).
- The repo's own `TROUBLESHOOTING.md` already documented the `catkin_pkg`/`PATH` issue and
  several fixes applied on the previous dev machine.

### Packages in the workspace
| Package | Role |
| --- | --- |
| `bumperbot_description` | URDF/Xacro model, Gazebo worlds, RViz, ros2_control HW interface, world `models/` |
| `bumperbot_controller` | ros2_control controllers (DiffDrive, simple/noisy) + `twist_relay` + joystick/twist_mux teleop |
| `robot_firmware` | Real-hardware serial drivers (`imu_serial`, `motor_serial`, `led_control`) + Arduino `.ino` firmware |
| `wheel_odometry` | Encoders → `/wheel/odom` |
| `wheel_imu_localization` | EKF fusing wheel odom + IMU → `/odom`; Nav2 AMCL owns `map→odom` |
| `robot_bringup` | Top-level launch (`bringup` / `slam_bringup`) + Nav2 params |
| `rplidar_ros` | Vendored RPLiDAR A2 driver → `/scan` |

### TF / data-flow chain (important for the bug below)
```
map → odom → base_footprint → base_link → laser_link
  ^AMCL      ^EKF
/scan is rigidly fixed to laser_link, so "laser drift" is ALWAYS a transform
issue, never the LiDAR itself.
```

---

## 2. The issue reported

**Feedback from the person who tested it (real robot):**

> "When [running] without `navigation.launch.py` I tried and it works fine with
> teleop. Laser point didn't drift. But when it comes to `navigation.launch.py`
> and giving a goal, then the laser drifts."

**Symptom:** Under plain teleop → no drift. Under Nav2 + a set goal → the laser
"drifts" on the map in RViz.

---

## 3. Root cause (diagnosed)

**Duplicate `map→odom` transform publisher.**

On the real robot, `bringup.launch.py` starts BOTH:
- the **EKF** (`wheel_imu_localization/ekf.yaml`) with `publish_tf: true` **and**
  `map_frame: map` → so the EKF also broadcasts `map→odom`;
- **AMCL** (`nav2_params_default.yaml`) with `tf_broadcast: true`, global `map`,
  odom `odom` → AMCL also broadcasts `map→odom`.

Under teleop only, AMCL isn't running, so only the EKF owns `odom` → no conflict → no drift.
Under Nav2, AMCL joins in and the two disagree, so the `map→odom` transform
flicker-fights and drags the laser (and all `/scan` points) around in RViz.

**Contributing factor:** frame mismatch — EKF / `amcl` / `DiffDriveController` use
`base_footprint`, but Nav2 costmaps/controller used `base_link`. A node whose
`robot_base_frame` doesn't match the frame the TF graph carries extrapolates with
`transform_tolerance`, so the scan appears offset while rotating toward a goal.

**Also blocking Nav2 from even loading:** `bringup.launch.py` hardcoded
`/home/amr/...` paths for the map and Nav2 params — those don't exist on this
machine.

---

## 4. Fixes applied (committed)

Commit `bbf4ccd` on `main` (ahead of `origin/main`, not yet pushed).

**FIX 1 — stop the EKF fighting AMCL (the actual drift cause)**
`src/wheel_imu_localization/config/ekf.yaml`
- `map_frame: map` → `map_frame: odom`
- EKF is now odom-level only (`odom→base_footprint`); **AMCL is the sole
  `map→odom` publisher** (standard Nav2 design).

**FIX 2 — unify the base frame**
`src/robot_bringup/config/nav2_params_default.yaml`
- all 4 `robot_base_frame: base_link` → `base_footprint` (bt_navigator + both
  costmaps + behavior server), matching what the EKF and `DiffDriveController`
  produce.

**FIX 3 — repoint hardcoded dev paths**
`src/robot_bringup/launch/bringup.launch.py`
- `map` `/home/amr/maps/my_map.yaml` → `/home/botforgelabs2/Desktop/bumperbot/maps/my_map.yaml`
- `params_file` `/home/amr/robot_ws/src/robot_bringup/config/nav2_params_default.yaml`
  → `/home/botforgelabs2/Desktop/bumperbot/src/robot_bringup/config/nav2_params_default.yaml`
  (×2 — localization and navigation includes).

**Verify:** rebuilt clean (`colcon build --packages-select wheel_imu_localization
robot_bringup`), no stale `/home/amr` refs remain.

---

## 5. To test on the real robot
```bash
cd ~/Desktop/bumperbot
source install/setup.bash
ros2 launch robot_bringup bringup.launch.py
```
Set a goal in RViz → laser should stay locked to the map (no drift).

---

## 6. Open items / watch-list (not yet done)
- [ ] **Hardware serial ports:** `robot_firmware` defaults to `/dev/ttyACM0`,
      `/dev/ttyACM1`, `/dev/arduino` — confirm they match real wiring.
- [ ] **If drift persists:** tighten `transform_tolerance` in AMCL (currently `1.0`s —
      high) vs the EKF.
- [ ] **`robot_display.launch.py`** starts both `joint_state_publisher_gui` and the
      `DiffDriveController` (via `controller.launch.py`) → duplicate `joint_states`
      publishers. Minor; consider removing the GUI from the real-robot display.
- [ ] **Firmware improvements available but NOT yet merged** (seen in a reviewed
      `src_latest` copy, since deleted): quaternion-based BNO055 IMU (`imu.ino` +
      `imu_serial.py` rewrite), motor/encoder tuning, odometry calibration
      (`wheel_radius 0.0595`, `wheel_base 0.540`, `encoder_cpr 46343`), URDF geometry
      tweaks. Cherry-pick these into `src` when ready — they are additive and safe.
- [ ] **Push** commit `bbf4ccd` to `origin/main` when you want it public.

---

## 7. Change history
| Date | Change | Commit |
| --- | --- | --- |
| 2026-07-23 | Cloned repo, set up env, `colcon build` 7/7 OK | (uncommitted baseline) |
| 2026-07-23 | Fixed laser drift: EKF no longer fights AMCL; base-frame + path fixes | `bbf4ccd` |
| 2026-07-23 | Reviewed & removed `src_latest` (older/buggy copy missing 440 world-model files, reverted drift fix) | (untracked, deleted) |
