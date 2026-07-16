# Bumperbot ROS 2

A complete ROS 2 (Jazzy) workspace for a differential-drive mobile robot
(**`bumperbot`**, URDF robot name `agv`). It supports both **Gazebo simulation**
and **real hardware**, providing the robot model, sensor/actuator drivers,
odometry, sensor fusion, LiDAR, and full autonomous navigation + SLAM.

## Packages

| Package | Description |
| --- | --- |
| `bumperbot_description` | Robot URDF/Xacro model, Gazebo worlds (`empty`, `small_house`, `small_warehouse`), RViz configs, `ros2_control` hardware interface. |
| `bumperbot_controller` | `ros2_control` controllers for simulation — `simple_velocity_controller`, `noisy_controller`, plus a `Twist`→`TwistStamped` relay. C++ & Python variants. |
| `robot_firmware` | Real-hardware drivers over serial: `imu_serial`, `motor_serial`, `led_control`, and the `ros2_control` hardware interface. |
| `wheel_odometry` | Converts `/encoder_ticks` → `/wheel/odom`. |
| `wheel_imu_localization` | EKF (`robot_localization`) fusing wheel odometry + IMU → `/odom`. |
| `robot_bringup` | Top-level launch files tying everything together (navigation + SLAM). |
| `rplidar_ros` | Vendored RPLiDAR A2 driver → `/scan`. |

## System requirements

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Python 3.12 (the version ROS 2 Jazzy targets)

## Build

```bash
cd ~/bumperbot
colcon build
source install/setup.bash
```

> **Note:** if your `PATH` puts a non-system Python (e.g. uv's `python3.11`)
> first, the build fails with `ModuleNotFoundError: No module named 'catkin_pkg'`.
> Ensure system Python 3.12 is found first, or add
> `export PATH=/usr/bin:/usr/local/bin:$PATH` to your `~/.bashrc`.
> See `TROUBLESHOOTING.md` for details.

Required ROS controller packages (install if missing):
```bash
sudo apt-get install -y ros-jazzy-velocity-controllers ros-jazzy-diff-drive-controller
```

## Run in simulation

```bash
# Terminal 1 — robot in Gazebo
ros2 launch bumperbot_description robot_gazebo.launch.py world_name:=empty

# Terminal 2 — controllers (DiffDriveController + Twist relay)
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=False

# Terminal 3 — drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`teleop_twist_keyboard` publishes `Twist` on `/cmd_vel`; the `twist_relay` node
converts it to `TwistStamped` for the `DiffDriveController`.

> **Note on worlds:** `empty.world` loads instantly. `small_house.world` and
> `small_warehouse.world` reference external AWS RoboMaker models via
> `model://` URIs; the required model directories are vendored under
> `src/bumperbot_description/models/` (pulled from the `ros2` branches of
> `aws-robotics/aws-robomaker-small-house-world` and
> `aws-robomaker-small-warehouse-world`), so no network access is needed at
> launch time.

## Build a map with SLAM and save it

Requires `ros-jazzy-slam-toolbox` and `ros-jazzy-nav2-bringup`:
```bash
sudo apt-get install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup
```

```bash
# Terminal 1 — robot in Gazebo (empty world loads fastest)
ros2 launch bumperbot_description robot_gazebo.launch.py world_name:=empty

# Terminal 2 — controllers
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=False

# Terminal 3 — SLAM
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 4 — drive the robot to explore (teleop, or publish /cmd_vel)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When the map looks complete, save it:
```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/my_map --ros-args -p save_map_timeout:=10000.0
```
This writes `maps/my_map.pgm` and `maps/my_map.yaml`. The saved map can then be
loaded by `robot_bringup bringup.launch.py` (update its `map` argument to point
at your `my_map.yaml`).

## Run on the real robot

Full autonomous navigation (loads a saved map):
```bash
ros2 launch robot_bringup bringup.launch.py
```

Build a map with SLAM:
```bash
ros2 launch robot_bringup slam_bringup.launch.py
```

> The `robot_bringup` launch files contain paths and serial-port settings that
> must be adjusted for your machine/hardware. See `TROUBLESHOOTING.md`.

## Documentation

See [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) for build/runtime issues and how
they were resolved.

## License

TODO: add a license.
