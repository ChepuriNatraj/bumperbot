# Bumperbot — Systematic Root-Cause Analysis: Laser/Map Drift Under Nav2

> Role: Senior ROS 2 Jazzy + Nav2 + TF2 + Localization engineer.
> Method: **No guessing.** Static evidence collected from the workspace on this
> machine; remaining gaps must be closed with live runtime commands on the robot.
> Workspace: `/home/botforgelabs2/Desktop/bumperbot`
> Robot name: `agv` (differential-drive).

---

## Problem Statement

A differential-drive robot works perfectly during manual teleoperation, but as soon
as a **Nav2 goal is sent**, the laser/map **drifts or rotates incorrectly — mainly
during rotation**. The robot *physically* moves correctly; straight navigation is
mostly fine; **rotation causes localization inconsistency.**

### Observed behavior (given)
- Teleop works correctly.
- Laser scan stable during teleop.
- Robot drives straight correctly.
- Odometry appears normal.
- Issue starts **only after a Nav2 goal is sent**.
- Issue is **worst during rotation**.
- Robot physically moves correctly (so it is a *perception/transform* problem, not a motion problem).

---

## A. Static evidence already collected (from `src/`)

| Source | Finding | Implication |
| --- | --- | --- |
| `bumperbot_controller/config/bumperbot_controllers.yaml` | `DiffDriveController` has **`enable_odom_tf: true`** and **`base_frame_id: base_footprint`** | It **publishes `odom→base_footprint`** |
| `wheel_imu_localization/config/ekf.yaml` (current, post-fix) | **`publish_tf: true`**, `world_frame: odom`, `base_link_frame: base_footprint`, **`map_frame: odom`** | It **publishes `odom→base_footprint`** (not `map→odom` anymore) |
| `ekf.yaml` (original `f938521` / pre-fix) | had **`map_frame: map`** | It **also published `map→odom`** → conflicted with AMCL |
| `nav2_params_default.yaml` | `amcl` has **`tf_broadcast: true`**, `global_frame_id: map`, `odom_frame_id: odom` | AMCL **publishes `map→odom`** |
| `nav2_params_default.yaml` | `robot_base_frame: base_footprint` everywhere (post-fix); `docking_server.base_frame: "base_link"` still at line 414 | frame-id mismatch mostly fixed; one stale `base_link` on an (unused) server |
| `robot.urdf.xacro` | `base_footprint`(empty)→`base_link` fixed joint; `laser_link` child of `base_link` | laser rigidly fixed; "laser drift" = transform-chain artifact, never the LiDAR |
| `bringup.launch.py` | EKF + AMCL + DiffDriveController + controller all launched together | all TF publishers co-exist at runtime |

### Two facts already proven from config (not guesswork)
1. **There is a duplicate publisher of `odom→base_footprint`:** both
   `DiffDriveController` (`enable_odom_tf: true`) and the `EKF`
   (`publish_tf: true`, `world_frame: odom`) broadcast the exact same transform.
2. **The original `map→odom` duplicate (EKF `map_frame: map` + AMCL) has been
   removed** in the current `src` (commit `bbf4ccd`).

---

## B. Ranked hypotheses

| # | Hypothesis | Why it fits the symptom | Static evidence | Live evidence still needed |
| --- | --- | --- | --- | --- |
| **H1** | Duplicate `map→odom` publisher: EKF (`map_frame: map`) **and** AMCL both broadcast it | AMCL runs **only under Nav2**; under teleop it's off → no fight → fine. Worst during rotation (fast `map→odom` change). **Matches "teleop fine / Nav2 drifts" most precisely.** | Original `ekf.yaml` HAD `map_frame: map` → confirmed this bug existed. Current `src` has `map_frame: odom` → **should be fixed**. | Confirm the *running* EKF actually uses `map_frame: odom` (not an old install) |
| **H2** | Duplicate `odom→base_footprint` publisher: `DiffDriveController` **and** EKF both broadcast it; they diverge in **yaw** (EKF uses IMU, controller uses wheel integration) | "Drift mainly during rotation" = yaw is the axis both publish and disagree on. TF last-writer-wins → jitter during rotation. | **Both configs confirmed publishing `odom→base_footprint`** (smoking gun in static evidence) | Confirm at runtime both actually publish + diverge under rotation |
| **H3** | IMU yaw **sign/axis error** inside the EKF (sub-cause of H2) | If IMU yaw is flipped, EKF's `odom→base_footprint` yaw is wrong → rotation looks inverted/drifted | `imu.ino` (reviewed copy) admits sign/axis correction is "applied on the Python side" — risk it's missing | Echo `/imu/data` vs actual yaw during rotation |
| **H4** | `laser_link` mounting offset wrong in URDF → rotates about wrong pivot | Any rotation swings the laser around a wrong point | URDF shows `laser_link` is a child of `base_link`; need its `origin` | `view_frames`; inspect `laser_link` joint origin |
| **H5** | `base_frame_id` mismatch (some node still `base_link`) → TF extrapolation during rotation | Nav2 extrapolates with `transform_tolerance` when frame mismatches | `nav2_params_default.yaml` now `base_footprint`; `docking_server.base_frame: "base_link"` remains (likely unused) | `ros2 param dump /controller_server` |
| **H6** | `transform_tolerance` too high (AMCL=1.0s) → stale `map→odom` during rotation | Rotation + 1s lag → laser visibly lags | `amcl.transform_tolerance: 1.0` present | Monitor `/amcl_pose` vs `/odom` during spin |
| **H7** | `use_sim_time` / clock mismatch → stale transforms | Mixed clocks → buffer holds old TF | All launches `use_sim_time: false` → low prob | Compare `/tf` header stamps |
| **H8** | DDS/WiFi delay on `/tf` or `/scan` (real robot, wireless) | Stale pose rendered → apparent drift | N/A (env) | `ros2 topic delay /tf`, `ros2 topic delay /scan` |
| **H9** | Costmap `robot_radius`/inflation mismatch → recovery spins misinterpreted as drift | Behavior, not pose | params look sane | observe `/cmd_vel` during "drift" |

### Eliminated so far
- Nothing fully eliminated yet.
- **H1 is "should be fixed"** (needs runtime confirmation that the deployed EKF uses `map_frame: odom`).
- **H2 is statically confirmed as present.**

---

## C. Iteration 1 — investigate the duplicate-TF publishers (H1, then H2)

This is the highest-probability class and both are cheap to confirm. **Do not proceed
to H3–H9 until these are resolved.**

### Step 1 — confirm which node owns `map→odom` and `odom→base_footprint`

```bash
# Full TF tree snapshot (look for WHO publishes map->odom and odom->base_footprint)
ros2 run tf2_tools view_frames          # produces frames.pdf / frames.gv
# Inspect the graph:
grep -E "map|odom|base_footprint" frames.gv
```

```bash
# List nodes and what TF they publish
ros2 node list
ros2 node info /ekf_filter_node          # look for "/tf" under "Publishes"
ros2 node info /bumperbot_controller      # (controller_manager) look for "/tf"
ros2 node info /amcl                       # look for "/tf"
```

```bash
# Live echo of the two suspect transforms (run WHILE rotating the robot)
ros2 run tf2_ros tf2_echo map odom            # EXPECT: if H1 true -> jumps/conflicts
ros2 run tf2_ros tf2_echo odom base_footprint # EXPECT: if H2 true -> two sources disagree in yaw
```

**Expected if H1 is still true (old install running):**
`view_frames`/`tf2_echo map odom` shows the transform being published by **two** nodes
(`/ekf_filter_node` and `/amcl`), and `tf2_echo` output flickers between two very
different poses during rotation.

**Expected if H1 is false (fix applied):**
only `/amcl` publishes `map→odom`; `tf2_echo map odom` is smooth.

**Expected if H2 is true:**
`odom base_footprint` is published by **both** `/bumperbot_controller` and
`/ekf_filter_node`; during a rotation the two diverge in yaw (e.g., one says +0.3 rad,
the other says −0.3 rad) and the rendered laser jitters.

**Expected if H2 is false:**
exactly one publisher of `odom→base_footprint`.

### Step 2 — confirm the *running* EKF config (catch an un-applied fix)

```bash
ros2 param get /ekf_filter_node map_frame
ros2 param get /ekf_filter_node publish_tf
ros2 param get /ekf_filter_node world_frame
# Also confirm the controller isn't double-publishing:
ros2 param get /bumperbot_controller enable_odom_tf 2>/dev/null
# (or dump the controller's params:)
ros2 param dump /bumperbot_controller
```

**Expected if H1/H2 are the cause:**
`map_frame == odom`, `publish_tf == true`, `world_frame == odom` →
confirms EKF publishes `odom→base_footprint` (H2) and does **not** publish
`map→odom` (H1 fixed).

**Expected if the fix never took:**
`map_frame == map` → H1 is alive; redeploy commit `bbf4ccd` and rebuild.

---

## D. Decision tree after Step 1–2 outputs

```
view_frames + tf2_echo
|
├─ map→odom published by 2 nodes (ekf + amcl)?
|     YES -> H1 ALIVE (un-applied fix). Rebuild/redeploy bbf4ccd. STOP, re-test.
|     NO  -> H1 eliminated (fix confirmed).
|
├─ odom→base_footprint published by 2 nodes (controller + ekf)?
|     YES -> H2 CONFIRMED. This is the residual root cause.
|     |        -> Sub-investigate H3 (IMU yaw sign): echo /imu/data during
|     |           rotation; compare sign of angular_velocity.z to true motion.
|     |        -> FIX: set EKF publish_tf: false (let DiffDriveController own
|     |           odom→base_footprint) OR set DiffDriveController
|     |           enable_odom_tf: false (let EKF own it). Pick ONE.
|     NO  -> H2 eliminated.
|
└─ If BOTH eliminated but symptom persists -> move to H3/H4/H5/H6.
```

---

## E. Remaining commands (only if H1 & H2 are eliminated)

```bash
# H3 — IMU sign/axis
ros2 topic echo /imu/data --field angular_velocity  # during a CCW rotation, is z positive?
ros2 topic echo /imu/data --field orientation

# H4 — laser mount
ros2 run tf2_ros tf2_echo base_link laser_link        # expect a small fixed offset, not a rotation pivot error
# + read robot.urdf.xacro <joint name="laser_joint"> origin

# H5 — frame mismatch live
ros2 param dump /controller_server | grep robot_base_frame
ros2 param dump /local_costmap | grep robot_base_frame
ros2 param dump /global_costmap | grep robot_base_frame

# H6 — AMCL lag
ros2 topic hz /amcl_pose
ros2 topic echo /amcl_pose --field pose  # compare yaw vs /odom during a spin

# H8 — DDS delay (wireless real robot)
ros2 topic delay /tf
ros2 topic delay /scan
```

---

## F. Current standing (honest, evidence-based)

- **H1 (duplicate `map→odom`)** is the classic cause of "teleop fine, Nav2 drifts"
  and **was present** in the original config; the current `src` (commit `bbf4ccd`)
  **removes it**. *Needs runtime confirmation that the deployed EKF actually uses
  `map_frame: odom`.*
- **H2 (duplicate `odom→base_footprint`: DiffDriveController + EKF)** is
  **statically confirmed present right now** and is the best explanation for
  "drift mainly during **rotation**." It would also explain why the H1 fix alone may
  not have fully cured the problem.
- Everything else (H3–H9) is lower probability until H1/H2 are disproven at runtime.

### Next action
Run the Step-1 / Step-2 block on the robot (Nav2 launched, then command a rotation).
Paste back:
1. the `view_frames` grep output,
2. the `tf2_echo odom base_footprint` output **during rotation**,
3. the `ros2 param get /ekf_filter_node map_frame` result.

That collapses this to a single root cause within one iteration.

---

## G. TF / data-flow chain (reference)

```
map → odom → base_footprint → base_link → laser_link
  ^AMCL      ^EKF (+ DiffDriveController = DUPLICATE)
/scan is rigidly fixed to laser_link, so "laser drift" is ALWAYS a transform
issue, never the LiDAR itself.
```

---

## H. Change history (context)

| Date | Change | Commit |
| --- | --- | --- |
| 2026-07-23 | Cloned repo, set up env, `colcon build` 7/7 OK | (uncommitted baseline) |
| 2026-07-23 | Fixed laser drift: EKF no longer fights AMCL (`ekf.yaml map_frame: map → odom`); `robot_base_frame base_link → base_footprint`; repointed `/home/amr` paths to workspace | `bbf4ccd` |
| 2026-07-23 | Reviewed & removed `src_latest` (older/buggy copy missing 440 world-model files, reverted drift fix) | (untracked, deleted) |
| 2026-07-23 | RCA documented (this file) | (uncommitted) |
