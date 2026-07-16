# Bumperbot — Setup & Troubleshooting Notes

These are the issues encountered while building and running this ROS 2 (Jazzy)
workspace on this machine, and how each was resolved.

---

## 1. `colcon build` fails: `ModuleNotFoundError: No module named 'catkin_pkg'`

**Symptom**
```
File ".../ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 22, in <module>
    from catkin_pkg.package import parse_package_string
ModuleNotFoundError: No module named 'catkin_pkg'
CMake Error ... ament_package_xml.cmake:95
```

**Root cause**
- ROS 2 Jazzy is installed for **system Python 3.12** (`/usr/bin/python3`).
- Your `PATH` puts `/home/botforgelabs2/.local/bin` first, where **uv** installed a
  separate **Python 3.11** interpreter (`python3.11`).
- CMake's `FindPython3` finds the 3.11 executable first and uses it to run ROS's
  build helper scripts, which can't find `catkin_pkg` (only present in 3.12).
- A stale `build/` directory caches the bad Python path, so the error persists
  even after fixing `PATH`.

**Fix**
1. Clean the cached build dirs (clears the cached interpreter path):
   ```bash
   rm -rf build install log
   ```
2. Build with system Python first on `PATH`:
   ```bash
   PATH=/usr/bin:/usr/local/bin:$PATH colcon build
   ```
3. Make it permanent — add to `~/.bashrc` (already done):
   ```bash
   export PATH=/usr/bin:/usr/local/bin:$PATH
   ```
   After this, a plain `colcon build` works in new terminals.

---

## 2. `bumperbot_description` install fails: missing `models` directory

**Symptom**
```
CMake Error at cmake_install.cmake:46 (file):
  file INSTALL cannot find
  ".../bumperbot_description/models": No such file or directory.
```
The `CMakeLists.txt` installs a `models/` directory that did not exist in the repo.

**Fix**
Created the directory (with a `.gitkeep` so it is tracked):
```
src/bumperbot_description/models/.gitkeep
```
It is referenced by `robot_gazebo.launch.py` for Gazebo model resources.

---

## 3. Controller spawn fails: `velocity_controllers` / `diff_drive_controller` not installed

**Symptom**
```
[spawner-2] [FATAL] Failed loading controller simple_velocity_controller
# or
[controller_manager] Loader for controller 'simple_velocity_controller'
  (type 'velocity_controllers/JointGroupVelocityController') not found.
```
`ros2 pkg prefix velocity_controllers` → `Package not found`.

**Root cause**
The workspace's `bumperbot_controllers.yaml` references controller types from
`velocity_controllers` and `diff_drive_controller`, but those ROS packages were
not installed on this machine (only the `ros2_control` core was present).

**Fix**
```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-velocity-controllers ros-jazzy-diff-drive-controller
```
(Installed version: `4.40.1`.)

---

## 4. Robot does not move when sending `/cmd_vel` (topic + message-type mismatch)

**Symptom**
`teleop_twist_keyboard` publishes commands, but the robot never moves.
`/bumperbot_controller/odom` stays at ~0 and wheel joint positions don't change.

**Root cause**
With the default launch (`controller.launch.py use_simple_controller:=False`),
the `DiffDriveController` (Jazzy 4.40.1) expects:
- message type **`geometry_msgs/msg/TwistStamped`**
- topic **`/bumperbot_controller/cmd_vel`** (named after the controller node)

But `teleop_twist_keyboard` publishes **`geometry_msgs/msg/Twist`** on **`/cmd_vel`**.
So commands never reach the controller.

> Note: `diff_drive_controller` 4.40.1 does **not** support the `cmd_vel_topic`
> or `use_stamped_vel` YAML parameters (those were added in a later release), so
> setting them in `bumperbot_controllers.yaml` is silently ignored.

**Fix**
Use the existing `twist_relay.py` node (already in the package) to convert
`Twist` → `TwistStamped`:

1. `src/bumperbot_controller/bumperbot_controller/twist_relay.py`
   - Changed its input subscription from `/bumperbot_controller/cmd_vel_unstamped`
     to **`/cmd_vel`** (the topic teleop publishes to).
   - It already publishes `TwistStamped` to `/bumperbot_controller/cmd_vel`.

2. `src/bumperbot_controller/launch/controller.launch.py`
   - Added `twist_relay_node` to the `LaunchDescription` so it starts with the
     controllers.

3. `src/bumperbot_controller/config/bumperbot_controllers.yaml`
   - Reverted the unsupported `cmd_vel_topic` / `use_stamped_vel` params.

**Resulting data flow**
```
teleop (/cmd_vel, Twist)
   → twist_relay.py
   → /bumperbot_controller/cmd_vel (TwistStamped)
   → DiffDriveController
   → wheels move ✓
```

**Verified**: publishing `Twist {linear.x: 0.3}` to `/cmd_vel` yields
`/bumperbot_controller/odom.twist.twist.linear.x ≈ 0.315`.

---

## 5. Gazebo fails to load `small_house` / `small_warehouse` world

**Symptom A** — `gz sim` dies immediately:
```
[Wrn] [gz.cc:102] Fuel world download failed because Fetch failed. Other errors
[gazebo-2] Unable to find or download file
[ERROR] [gazebo-2]: process has died
```
**Root cause**: `robot_gazebo.launch.py` unconditionally appended `.world` to
`world_name`, so `world_name:=small_house.world` resolved to the nonexistent
`small_house.world.world`, and Gazebo fell back to (failed) Fuel download.
**Fix (applied)**: the launch file now strips a trailing `.world` before
re-appending it, so both `world_name:=small_house` and
`world_name:=small_house.world` resolve correctly.

**Symptom B** — world loads but every prop is missing:
```
Error Code 14: ... Unable to find uri[model://aws_robomaker_residential_...]
```
followed by the server shutting itself down.
**Root cause**: `small_house.world` / `small_warehouse.world` reference ~63/13
external AWS RoboMaker models via `model://` URIs. `gz sim` only resolves bare
`model://name` URIs against `GZ_SIM_RESOURCE_PATH` — it does **not** fall back
to Fuel for these. The `models/` directory in this repo previously only held a
`.gitkeep` placeholder.
**Fix (applied)**: the required model directories are now vendored under
`src/bumperbot_description/models/` (copied from the `ros2` branches of
`aws-robotics/aws-robomaker-small-house-world` and
`aws-robomaker-small-warehouse-world` on GitHub). `robot_gazebo.launch.py`
already points `GZ_SIM_RESOURCE_PATH` at this directory, so no further setup
is needed — just rebuild (`colcon build --packages-select
bumperbot_description`) after pulling.

---

## 6. Saving a map: `save_map_timeout` type error

**Symptom**
```
[map_saver_cli]: Unexpected problem appear: parameter 'save_map_timeout'
has invalid type: Wrong parameter type ... is of type {double} ...
```

**Fix** — pass the timeout as a float, not an integer:
```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_map \
  --ros-args -p save_map_timeout:=10000.0
```

---

## How to run (verified working)

```bash
cd ~/Desktop/bumperbot
colcon build
source install/setup.bash

# Terminal 1 — simulation
ros2 launch bumperbot_description robot_gazebo.launch.py world_name:=empty

# Terminal 2 — controllers (DiffDriveController + twist_relay)
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=False

# Terminal 3 — drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Known caveats / next steps

- `src/robot_bringup/launch/bringup.launch.py` contains **hardcoded paths** from
  the original dev machine (`/home/amr/...` for the map and Nav2 params). These
  must be updated to this workspace before the real-robot / Nav2 launch works.
- Real-hardware nodes (`robot_firmware`) default to serial ports
  `/dev/ttyACM0`, `/dev/ttyACM1`, and the `ros2_control` interface uses
  `/dev/arduino` — verify these match your actual hardware.
- When iterating, kill previous launches before relaunching to avoid duplicate
  nodes (e.g. two `/twist_relay` nodes):
  ```bash
  pkill -f controller.launch.py; pkill -f robot_gazebo.launch.py; pkill -f gz
  ```
