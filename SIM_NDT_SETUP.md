# Simulation Full-Stack Setup (NDT Localisation + Perception)

How to run the crowd-matching test scene in AWSIM with the **full
Autoware stack** — real NDT scan matching against a baked point-cloud
map, and either the dedicated `perception_cuda` container's CenterPoint
pipeline or the in-process `perception_minimal` PoC.

This is the sim counterpart to the bus deployment in `DEPLOYMENT.md`.
Previously the sim ran with a ground-truth pose relay and a minimal
in-process perception shim (both are kept as fallbacks); this guide
covers the default path that matches what the bus runs.

---

## TL;DR

```bash
# Host terminal 1 — AWSIM with dense-landmark scene
export ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
/home/harry/Unity/AWSIM2.0/Exec/SfmTest/SfmPedestrianTest.x86_64 &

# Host terminal 2 — start the simulation container
cd /home/harry/Lee_autoware_ws/docker
docker compose up -d simulation   # (add "perception_cuda" if using PERCEPTION_MODE=cuda)
docker exec -it simulation_container bash

# Inside container — default is ndt + minimal
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh campus_tuned_sfm
```

The launcher will bring up NDT, fake-occupancy-grid publisher,
topic relay, and the SFM module patches. After ~25 s it publishes the
initial pose automatically. After ~40 s engage with
`python3 .../scripts/set_goal_engage.py --distance 10`.

---

## Launch-script Modes

`scripts/launch_sfm_test.sh` now takes three orthogonal env vars plus
the existing config positional argument:

| Env var | Values | Default | Effect |
|---------|--------|---------|--------|
| `LOCALIZATION_MODE` | `ndt` \| `gt` | `ndt` | `ndt` uses real NDT scan matching; `gt` falls back to `ground_truth_relay.py` |
| `PERCEPTION_MODE`   | `cuda` \| `minimal` \| `none` | `cuda` | `cuda` expects perception_cuda container; `minimal` runs `perception_minimal.launch.xml` + `object_relay.py` in-process; `none` disables both |
| `MAP_PATH`          | any path | `/autoware_map_sfm` | Lets the same script test against the Shinjuku map for regression |
| `INITIAL_POSE_X/Y/YAW` | floats | `0.0 / 2.53 / 0.0` | Overrides the initial-pose hint published to `/initialpose` in `ndt` mode |

**Valid combinations used during development:**

- `LOCALIZATION_MODE=ndt PERCEPTION_MODE=minimal` — current default. Bus sees
  pedestrians via the `object_relay` shim; NDT locks against the baked PCD.
- `LOCALIZATION_MODE=ndt PERCEPTION_MODE=cuda` — target state. Needs
  `perception_cuda` container up (see `docker/compose.yaml`) and the front→concat
  lidar relay (done automatically by the launch script).
- `LOCALIZATION_MODE=gt PERCEPTION_MODE=minimal` — legacy path used for the
  8-way comparative study. Preserves historical behaviour.
- `LOCALIZATION_MODE=gt PERCEPTION_MODE=none` — for bag-record-only runs when
  nothing else matters.

---

## Unity Scene Prerequisites

NDT needs matching geometry in the runtime scene and in the baked PCD.
Two scenes live in `Assets/Awsim/Scenes/`:

- **`SfmPedestrianTest_NDT.unity`** — runtime scene built into
  `SfmPedestrianTest.x86_64`. Inherits the original crowd-matching layout +
  adds the `SfmLandmarks/` parent (dense static geometry for NDT).
- **`SfmPcdBake.unity`** — bake-only scene. Same landmarks, plus the
  `PcdGenerator` + `PcdGenerationDemo` wrapper from the stock
  `PcdGenerationDemo.unity`. `AutowareSimulationDemo`, `EgoVehicle`, and
  `CampusPathway` GameObjects are disabled so the bake is not polluted by
  the bus body, moving pedestrians, or ROS2-node conflicts.

### Landmark generator

`Assets/Awsim/Scripts/Editor/Common/GenerateSfmLandmarks.cs` is an Editor
script exposing two top-menu items:

- **AWSIM → Generate SFM Landmarks** — creates/overwrites a `SfmLandmarks/`
  GameObject under the active scene with a deterministic (seed=1729) layout
  of anchors, continuous walls, buildings, tree rows, bushes, street furniture,
  and signboards. Covers X = −200..+200, matching the lanelet extent.
- **AWSIM → Remove SFM Landmarks** — wipes the hierarchy.

Current density (~450 objects total) targets Shinjuku-style PCD quality.
If iterating on the recipe:
1. Tweak `GenerateSfmLandmarks.cs`, save.
2. Open `SfmPedestrianTest_NDT.unity` → `Remove` → `Generate` → Ctrl+S.
3. Open `SfmPcdBake.unity` → `Remove` → `Generate` → Ctrl+S.
4. Rebake PCD (press Play in `SfmPcdBake`).
5. Copy baked PCD into the map dir (see below).
6. **Rebuild the standalone exe** — otherwise the runtime LiDAR scans the
   old layout and NDT mismatches.

### PCD bake

```bash
# After pressing Play in SfmPcdBake and the auto-exit prints "PCL data saved"
cp /home/harry/Unity/AWSIM2.0/AWSIM/Assets/sim_pointcloud_map.pcd \
   /home/harry/autoware_map_sfm_test/pointcloud_map.pcd
```

Current bake is ~22 MB / 1.85 M points, ~12 % ground, ~82 % above 1 m.

---

## Fake Occupancy Grid

`scripts/fake_occupancy_grid.py` publishes an empty 100 m × 100 m
`/perception/occupancy_grid_map/map` at 10 Hz.

**Why it exists:** `behavior_path_planner` blocks indefinitely on
`waiting for occupancy_grid` when no perception stack produces that topic.
`PERCEPTION_MODE=minimal` and `none` do not publish it; only the full
`perception_cuda` pipeline does. The launch script auto-starts the fake
publisher whenever `PERCEPTION_MODE != cuda`.

---

## perception_cuda Container (PERCEPTION_MODE=cuda path)

`docker/compose.yaml` defines a `perception_cuda` service that runs
CenterPoint + tracker + prediction from the
`universe-sensing-perception-devel-cuda` image. It subscribes to
`/sensing/lidar/concatenated/pointcloud` and publishes
`/perception/object_recognition/objects` with PEDESTRIAN labels.

**Key details:**

- The nuway-sensor-kit concat node won't load when
  `launch_sensing_driver=false` (container `pointcloud_container` is never
  created). Instead, `launch_sfm_test.sh` starts a `topic_tools relay`
  from `/sensing/lidar/front/pointcloud_raw_ex` →
  `/sensing/lidar/concatenated/pointcloud`. That publication is visible to
  both simulation_container (for NDT) and perception_cuda (for
  CenterPoint) via `network_mode: host`.
- `tier4_sensing_component.launch.xml` is **not** run inside the
  perception_cuda service (would conflict with the relay). Only
  `tier4_perception_component.launch.xml`.
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set on both services so DDS
  peers line up.

Start both containers:

```bash
docker compose -f /home/harry/Lee_autoware_ws/docker/compose.yaml up -d
```

---

## Initial Pose

In `ndt` mode the launch script publishes `/initialpose` 25 s after
`ros2 launch` is invoked, giving Autoware's `autoware_initial_pose_adaptor`
time to register. The default is `(x=0.0, y=2.53, yaw=0.0)` — overridable
via env vars.

If the automatic publish doesn't trigger convergence, call the AD API
service directly from inside the container:

```bash
ros2 service call /api/localization/initialize \
  autoware_adapi_v1_msgs/srv/InitializeLocalization \
  '{pose: [{header: {frame_id: map},
    pose: {pose: {position: {x: 0.0, y: 2.53, z: 0.0},
                  orientation: {w: 1.0}},
           covariance: [0.25, 0,0, 0,0,0, 0,0.25,0, 0,0,0,
                        0,0,0.06, 0,0,0, 0,0,0, 0.06,0,0,
                        0,0,0,0, 0.06,0, 0,0,0,0,0,0.06]}}]}'
```

Expect `success=True` in response.

---

## Verification Checklist

After launch, confirm in order:

```bash
# Lidar input present
ros2 topic hz /sensing/lidar/front/pointcloud_raw_ex       # ~10 Hz
ros2 topic hz /sensing/lidar/concatenated/pointcloud       # ~10 Hz (via relay)

# NDT up and publishing
ros2 topic hz /localization/pose_estimator/pose_with_covariance   # ~10 Hz

# EKF activated + TF tree
ros2 topic hz /localization/kinematic_state                # ~50 Hz
ros2 run tf2_ros tf2_echo map base_link                    # finite values

# Initialisation succeeded
ros2 topic echo --once /api/localization/initialization_state
# → state: 3  (INITIALIZED)

# Trajectory being planned (needs fake occupancy grid)
ros2 topic hz /planning/scenario_planning/trajectory       # ~10 Hz

# Perception feed (minimal mode)
ros2 topic hz /perception/object_recognition/objects       # ~20 Hz
```

Set goal and engage:

```bash
python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/set_goal_engage.py --distance 10
```

Expect `change_to_autonomous: success=True`. Bus should move.

---

## Clean Stop Between Runs

Full restart before every test — zombie processes from a previous run
will skew results:

```bash
# In the host
pkill -f SfmPedestrianTest
docker restart simulation_container     # (add perception_cuda if it was up)
```

---

## Known Limitations / Residuals

- **NDT pose has a small Z offset (~0.65 m) and pitch (~9.6°)** compared
  to ground truth. These are artifacts of the tilted base_link →
  velodyne_front_link URDF static TF vs the level-lidar bake vehicle.
  The X/Y error is <10 cm, which is fine for the campus test.
- **Baked PCD is vantage-specific.** A bake from the car's roof-mounted
  Velodyne matches runtime just well enough when landmarks are dense
  (as in the current generator). If density is reduced, matching score
  falls below threshold and NDT wanders.
- **Fake occupancy grid is a sim-only hack.** The real bus deployment
  uses full perception and never hits this path.
- **Unity AWSIM repo changes are not tracked in Lee_autoware_ws.** The
  `SfmPedestrianTest_NDT.unity`, `SfmPcdBake.unity`, and
  `GenerateSfmLandmarks.cs` edits belong to that separate project —
  commit there if you want them versioned.

---

## Files Added / Modified (this branch)

### `Lee_autoware_ws`

| File | Change |
|------|--------|
| `docker/compose.yaml` | `perception_cuda` service re-enabled; concat handled by launch-script relay; `RMW_IMPLEMENTATION` added |
| `scripts/launch_sfm_test.sh` | `LOCALIZATION_MODE`, `PERCEPTION_MODE`, `MAP_PATH`, `INITIAL_POSE_*` env vars; auto-starts concat relay + fake-grid; gated GT/object/minimal fallbacks |
| `scripts/fake_occupancy_grid.py` (new) | 10 Hz empty `/perception/occupancy_grid_map/map` publisher |
| `scripts/object_relay.py` | Tracking tweaks (match_threshold 2.0, vel_alpha 0.5, min_track_age 1) |
| `launch/perception_minimal.launch.xml` (from earlier bus PoC) | Minimal clustering pipeline; reused as the `PERCEPTION_MODE=minimal` fallback |
| `config/perception_minimal/ground_cropbox.param.yaml` | `processing_time_threshold_sec` fix |

### Unity (`/home/harry/Unity/AWSIM2.0/AWSIM/`)

| File | Change |
|------|--------|
| `Assets/Awsim/Scenes/SfmPedestrianTest_NDT.unity` (new) | Runtime scene with dense landmarks |
| `Assets/Awsim/Scenes/SfmPcdBake.unity` (new) | Bake-only scene with PcdGenerator wired for `/home/harry/autoware_map_sfm_test/lanelet2_map.osm` |
| `Assets/Awsim/SfmTest/lanelet2_map.osm` (copied) | Imported so PcdGenerator can read it |
| `Assets/Awsim/Scripts/Editor/Common/GenerateSfmLandmarks.cs` (new) | Landmark generator Editor script |
