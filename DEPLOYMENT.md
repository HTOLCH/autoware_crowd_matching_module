# Crowd-Matching Module Deployment Guide

Deployment instructions for the pedestrian-aware velocity planning module
on the nUWAy campus shuttle bus (Nvidia Orin, ARM64).

## Quick Reference (copy-paste)

```bash
# On the Orin (outside container)
cd /home/nvidia/Workspace/nUWAy_autoware_ws
git checkout -b harry/crowd-matching
cd src
git clone https://github.com/HTOLCH/autoware_crowd_matching_module.git autoware_behavior_velocity_sfm_module
cd ..
git add src/autoware_behavior_velocity_sfm_module/
git commit -m "Add crowd-matching velocity planning module"
git push -u origin harry/crowd-matching

# Start the container (check IMAGE/TAG env vars or .env file first)
cd /home/nvidia/Workspace/autoware_on_nUWAy
docker compose up -d
docker exec -it devel_container bash

# Inside container — build, deploy, launch
cd /workspace
colcon build --packages-select autoware_behavior_velocity_sfm_module
source /workspace/install/setup.bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh enable
ros2 launch autoware_launch autoware.launch.xml data_path:=/autoware_data map_path:=/autoware_map

# OPTIONAL — only if the bus's full perception stack is broken/unavailable.
# Run in a second shell inside the container after Autoware has come up:
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_perception.sh start
```

## Architecture Overview

```
Desktop (x86_64, RTX 3080)          Bus (Nvidia Orin, ARM64)
 ├─ AWSIM simulation                 ├─ Real LiDARs, IMU, GNSS, CAN
 ├─ simulation_container             ├─ devel_container
 ├─ Lee_autoware_ws/                 ├─ nUWAy_autoware_ws/
 ├─ object_relay.py (sim only)       ├─ Full perception pipeline
 └─ ground_truth_relay.py            └─ Real localization stack
```

The module itself requires ZERO code changes between sim and bus.
It reads from `/perception/object_recognition/objects` (PredictedObjects),
which is the standard Autoware perception output on both platforms.

## Important: ARM64 vs x86_64

The Orin is ARM64. You CANNOT copy compiled binaries (install/ or build/)
from your x86_64 desktop. You must copy SOURCE code and compile on the Orin.

## Pre-Deployment: Git Safety (on the bus)

Before making any changes on the Orin, create a branch so you can
always revert to Lee's clean state if anything goes wrong.

### 1. Create a safety branch on the bus workspace

```bash
cd /home/nvidia/Workspace/nUWAy_autoware_ws
git checkout -b harry/crowd-matching
```

### 2. Clone the module into the workspace

```bash
cd /home/nvidia/Workspace/nUWAy_autoware_ws/src
git clone https://github.com/HTOLCH/autoware_crowd_matching_module.git autoware_behavior_velocity_sfm_module
```

The directory must be named `autoware_behavior_velocity_sfm_module` to match the
CMake package name and deploy script expectations.

### 3. Commit and push the branch

```bash
cd /home/nvidia/Workspace/nUWAy_autoware_ws
git add src/autoware_behavior_velocity_sfm_module/
git commit -m "Add crowd-matching velocity planning module"
git push -u origin harry/crowd-matching
```

Now you have a remote backup. If anything goes wrong:
```bash
# First restore Autoware configs
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh disable

# Then revert to Lee's clean state
git checkout main
```

IMPORTANT: Always run `deploy_sfm.sh disable` BEFORE switching branches.
The deploy script patches Autoware install files outside of git tracking
(in /opt/autoware/). Switching branches without disabling would leave
those files in a patched state.

## Deployment (on the bus)

### 4. Start the container

The compose.yaml uses `${IMAGE}:${TAG}` for the Docker image. Check that these
env vars are set (or that an `.env` file exists) before starting. Ask Lee if unsure
which image is on the Orin. You can check with `docker images`.

```bash
cd /home/nvidia/Workspace/autoware_on_nUWAy
docker compose up -d
docker exec -it devel_container bash
```

The container entrypoint (`ros_entrypoint.sh`) automatically sources ROS Humble,
Autoware, and the workspace install, so build tools and dependencies are available
immediately.

### 5. Build the module

```bash
cd /workspace
colcon build --packages-select autoware_behavior_velocity_sfm_module
source /workspace/install/setup.bash
```

If the build fails with missing dependencies, run:
```bash
rosdep install --from-paths src/autoware_behavior_velocity_sfm_module --ignore-src -y
```

### 6. Enable the module

```bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh enable
```

This does five things:
1. Patches behavior_planning.launch.xml to load the crowd-matching plugin
2. Installs campus-tuned motion configs (1.2m stop margins instead of 5m)
3. Tunes velocity smoother (engage_exit_ratio=0.9 for low-speed campus driving)
4. Disables pedestrian/bicycle surround departure check (always triggered on shared pathway)
5. Enables obstacle_stop + road_user_stop modules in nuway_preset (Autoware safety backup)

### 7. Verify deployment

```bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh status
```

Expected output:
```
=== Crowd-Matching Module Status ===
  Crowd-matching plugin: ENABLED (patched in behavior_planning.launch.xml)
  Motion configs: Campus-tuned (backups present)
  Velocity smoother: Campus-tuned (exit_ratio=0.9)
  Planning preset: Modified (safety modules enabled, backup present)
  obstacle_stop_module: ENABLED
  road_user_stop_module: ENABLED
```

### 8. Launch Autoware

```bash
ros2 launch autoware_launch autoware.launch.xml \
    data_path:=/autoware_data \
    map_path:=/autoware_map
```

Or use the alias:
```bash
autolaunch
```

If the bus's full perception stack is running, no relay scripts are needed —
the real perception pipeline (LiDAR detection, multi-object tracker, map-based
prediction) publishes pedestrians to the same topic the module reads from.

If the full perception stack is broken or unavailable on the bus, use the
minimum perception fallback — see [Bus Perception Fallback](#bus-perception-fallback)
below.

### 9. Verify the module is running

In a second terminal inside the container:
```bash
ros2 topic echo /rosout --once | grep -i "crowd-matching"
```

You should see:
```
Crowd-Matching Module is executing (mode: crowd_matching)!
```

Check the module's live output:
```bash
cat /tmp/cm_target.csv
```

## Collecting Test Data

### First test (validation run)

The goal is to confirm it builds, deploys, and doesn't crash. Run these in a
second terminal inside the container while the bus is driving.

**1. Record a rosbag:**
```bash
ros2 bag record -o /workspace/rosbags/bus_test_$(date +%Y%m%d_%H%M%S) \
    /vehicle/status/velocity_status \
    /localization/kinematic_state \
    /perception/object_recognition/objects \
    /planning/scenario_planning/trajectory
```

**2. Check the module loaded:**
```bash
ros2 topic echo /rosout --once | grep -i "crowd-matching"
```

**3. Check pedestrians are being detected:**
```bash
ros2 topic echo /perception/object_recognition/objects --once | grep -c "label: 7"
```
Label 7 = PEDESTRIAN. If this returns 0, the perception pipeline is not
classifying pedestrians. Check LiDAR data and detection model.

**4. Watch the module's live decisions:**
```bash
tail -f /tmp/cm_target.csv
```
Columns: time, target velocity, raw target, co_flow count, min_front distance.
If co_flow stays at 0, the module is not detecting co-flow pedestrians.

**5. Screen record** RViz with your phone as visual evidence for the thesis.

**6. Note down on your phone:**
- Did Autoware start without errors?
- Did the module load? (rosout message)
- Were pedestrians detected with PEDESTRIAN labels?
- Did the bus slow for pedestrians or hard-stop?
- Any crashes or unexpected behaviour?

### Controlled test (next session)

Once the basics work, do a proper comparative run:

**Run 1 — Module enabled (Config D):**
```bash
ros2 bag record -o /workspace/rosbags/bus_crowd_match_$(date +%Y%m%d_%H%M%S) \
    /vehicle/status/velocity_status \
    /localization/kinematic_state \
    /perception/object_recognition/objects \
    /planning/scenario_planning/trajectory
```
Drive the shared pathway stretch. Copy `/tmp/cm_target.csv` off the container
after the run:
```bash
docker cp devel_container:/tmp/cm_target.csv ~/bus_cm_target_enabled.csv
```

**Run 2 — Module disabled (stock/campus-tuned baseline):**
```bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh disable
# Restart Autoware, record another rosbag on the same stretch
```

**Analyse on desktop later:**
```bash
# Copy rosbags from the bus
scp -r nvidia@<bus-ip>:/home/nvidia/Workspace/nUWAy_autoware_ws/rosbags/ ~/bus_rosbags/

# Run analysis (inside sim container on desktop)
docker exec simulation_container python3 \
    /workspace/src/autoware_behavior_velocity_sfm_module/scripts/analyze_results.py \
    --bag-dir /workspace/rosbags/bus_test
```
The analysis script works on any rosbag with the 4 topics above, sim or real.

## Bus Perception Fallback

When the full Autoware perception stack (CenterPoint ML detection, multi-object
tracker, map-based prediction) is not running on the bus, the crowd-matching
module receives nothing on `/perception/object_recognition/objects` and cannot
react to pedestrians. This section brings up a minimum perception layer that
mirrors the simulation approach: rule-based LiDAR clustering plus the
`object_relay.py` script that promotes clusters to PEDESTRIAN-classified
`PredictedObjects`.

### Pipeline

```
raw lidar (Velodyne VLP16)
   -> CropBoxFilter        (drop ground + sky points)
   -> EuclideanCluster     (voxel-grid based, no ML, no GPU)
   -> FeatureRemover       (tier4 -> autoware_perception_msgs)
   -> object_relay.py      (corridor filter + PEDESTRIAN tagging)
   -> /perception/object_recognition/objects  --> crowd-matching module
```

Localization is NOT touched; the bus's own GNSS/INS continues to feed
`/localization/kinematic_state`. Only the perception side is replaced.

### Pre-flight checks (run inside container BEFORE starting the fallback)

```bash
# (a) Raw lidar alive?
ros2 topic list | grep sensing/lidar
ros2 topic hz /sensing/lidar/concatenated/pointcloud   # default; verify on bus

# (b) Localization alive? (required for base_link -> map transform in the relay)
ros2 topic hz /localization/kinematic_state            # expect ~50 Hz

# (c) Planner trajectory alive? (required for the corridor filter)
ros2 topic hz /planning/scenario_planning/trajectory   # expect ~10 Hz
```

If (a) is missing the fallback cannot run — escalate to Lee. If (c) is missing
the corridor filter auto-falls-back to pass-through (every cluster becomes a
"pedestrian"), and the bus will conservatively slow for any obstacle until the
trajectory comes back. The relay logs a throttled warning when this happens.

### Start

```bash
# After deploy_sfm.sh enable and Autoware launch, in a separate container shell:
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_perception.sh start
```

The script starts a composable container (crop + cluster + feature remover)
and `object_relay.py`. Logs go to `/tmp/cm_perception.log` and
`/tmp/cm_object_relay.log`. PIDs stored in `/tmp/cm_perception.pids`.

Optional environment overrides:
```bash
CM_LIDAR_TOPIC=/sensing/lidar/top/pointcloud_raw_ex \
CM_CORRIDOR_WIDTH=2.5 \
CM_MAX_FORWARD=20.0 \
    deploy_perception.sh start
```

### Verify

```bash
deploy_perception.sh status
```

Should report all four topics with non-zero Hz:
- `/sensing/lidar/concatenated/pointcloud`              (sensor)
- `/perception/obstacle_segmentation/pointcloud`        (after crop)
- `/perception/object_recognition/detection/clustering/objects`  (after cluster + remover)
- `/perception/object_recognition/objects`              (after relay; PEDESTRIAN PredictedObjects)

Watch the relay's filter stats:
```bash
tail -f /tmp/cm_object_relay.log
```
Each second it prints `kept N, corridor-reject N, forward-reject N, ground-reject N`
(or a `corridor filter DISABLED` warning if no trajectory has been received).

### Walk-test

1. Drive onto a stretch of shared pathway with the module enabled and the
   fallback perception running.
2. Have someone walk inside the planned trajectory corridor (~2 m strip ahead
   of the bus). The relay's `kept` count should rise above zero.
3. `tail -f /tmp/cm_target.csv` — `co_flow` should go above zero, `target`
   should drop from 1.39 m/s towards crowd speed (~1.0-1.2 m/s).
4. Then have the same person step OUTSIDE the corridor (e.g. next to a wall
   2 m off the path). The cluster should now be `corridor-reject`-ed and the
   bus should NOT slow down for them.

### Stop

```bash
deploy_perception.sh stop
```

Cleans up the launch, the relay, and any straggling composable container
processes. After stopping you can switch back to the full perception stack
(if/when Lee fixes it) without any other changes — the crowd-matching module
itself doesn't care which producer publishes the PedestrianObjects.

### Known PoC Limitations

- **Stationary objects:** clusters with zero velocity (people standing still,
  dogs, kids in the corridor) WILL slow the bus down via proximity attenuation.
  This is the desired behaviour for safety, but means the bus will also slow
  for any cluster mistaken as pedestrian inside the corridor.
- **No real classifier:** every kept cluster is tagged as PEDESTRIAN. The
  corridor filter is the only thing keeping the bus from treating walls and
  parked cars as pedestrians, so trajectories must be reliable.
- **No real tracker:** velocity estimates come from a single-frame
  nearest-neighbour matcher with EMA smoothing. Acceptable for slow campus
  speeds but noisier than the real Autoware multi-object tracker.
- **Single corridor width:** does not currently widen the corridor at curves
  or intersections.

### Recommended rosbag topics (fallback runs)

In addition to the four planning/perception topics already in DEPLOYMENT.md's
validation run, also record:

```
/perception/object_recognition/detection/clustering/objects   # pre-relay clusters
/perception/object_recognition/objects                         # post-relay (what module sees)
/planning/scenario_planning/trajectory                         # corridor reference
```

These let you post-mortem any "why did the bus slow / why didn't it slow"
question.

## Sensors and CAN

These are handled by the standard Autoware launch. No extra steps needed.

| Sensor | Interface | Launched by |
|--------|-----------|-------------|
| Front Velodyne VLP16 | 192.168.5.28:2369 | sensing.launch.xml -> lidar.launch.xml |
| Rear Velodyne VLP16 | 192.168.5.27:2368 | sensing.launch.xml -> lidar.launch.xml |
| SBG Ellipse IMU/GNSS | /dev/ttyUSB0 (921600 baud) | imu.launch.xml + gnss.launch.xml |
| CAN interface | /dev/ttyUSB0 (shared) | vehicle_interface or can_container |

The crowd-matching module sits in the planning pipeline and has no direct
interaction with sensors or CAN. It reads from the perception output and
adjusts the planned velocity.

## Dual-Layer Safety Architecture

```
Layer 1 (module):   Crowd-matching velocity planning
                    ├─ Crowd speed matching (detection_radius: 15m)
                    ├─ Proximity attenuation (10m onset, 3m zero target)
                    └─ Relaxation smoothing (tau=2.0s)

Layer 2 (Autoware): Backup emergency stops
                    ├─ obstacle_stop at 1.2m (hard stop)
                    └─ road_user_stop at 1.2m (hard stop)
```

Both layers must be active. The deploy script enables the Autoware backup
modules in the nuway_preset automatically.

## Disabling and Restoring

To fully restore the bus to its pre-module state:

```bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh disable
```

This restores every modified file from .sfm_bak backups:
- behavior_planning.launch.xml (crowd-matching plugin removed)
- Motion configs restored to stock (5m margins)
- Velocity smoother restored to stock
- Surround obstacle checker restored
- nuway_preset.yaml restored (safety modules back to original state)

Restart Autoware after disabling.

To clean up backup files (e.g. before deploying to a different machine):
```bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh clean
```

## Troubleshooting

### Bus will not start driving (steering_converged error)
```bash
ros2 param set /control/trajectory_follower/controller_node_exe \
    enable_keep_stopped_until_steer_convergence false
```

### Module not loading
Check that the package is installed:
```bash
ros2 pkg list | grep sfm
```
Should show: `autoware_behavior_velocity_sfm_module`

If not, rebuild and re-source:
```bash
cd /workspace
colcon build --packages-select autoware_behavior_velocity_sfm_module
source /workspace/install/setup.bash
```

### No pedestrians detected
Check perception output:
```bash
ros2 topic echo /perception/object_recognition/objects --once
```
Verify objects have `classification.label = 7` (PEDESTRIAN) and non-zero
twist values.

### Module runs but bus does not slow for pedestrians
Check that the module is receiving predicted objects:
```bash
ros2 topic hz /perception/object_recognition/objects
```
Should be publishing at 10-20 Hz.

Check module log:
```bash
ros2 topic echo /rosout | grep "CM:"
```
The `co_flow` count and `min_front` distance show what the module sees.

### Known issue: RMW typo in bus entrypoint
The bus ros_entrypoint.sh has a typo on line 12:
```bash
export RMW_IMPLMENTATION=rmw_cyclonedds_cpp   # missing 'E'
```
Should be `RMW_IMPLEMENTATION`. This means the bus may be using FastDDS
instead of CycloneDDS. Check with Lee whether this matters.

## Key Differences: Simulation vs Bus

| Aspect | Simulation | Real Bus |
|--------|-----------|----------|
| Container | simulation_container | devel_container |
| Architecture | x86_64 | ARM64 (Orin) |
| Perception | object_relay.py (sim only) | Full Autoware pipeline |
| Localization | ground_truth_relay.py (sim only) | EKF + GNSS + LiDAR |
| Map | autoware_map_sfm_test (400m straight) | FullCampus |
| Launch script | launch_sfm_test.sh (sim only) | Standard autoware.launch.xml |
| Workspace path | src/autoware_nuway_launch/autoware_launch_demo/ | src/launcher/autoware_launch/ |

## File Reference

| File | Purpose |
|------|---------|
| deploy_sfm.sh | Enable/disable/status/clean (main deployment tool) |
| patch_behavior_planning.sh | Patches Autoware launch XML for crowd-matching plugin |
| config/sfm.param.yaml | Module parameters (mode, detection_radius, braking_distance, etc.) |
| configs/campus_tuned_sfm/ | Campus-tuned Autoware configs installed by deploy script |
| deploy_perception.sh | Bus PoC: start/stop/status for the minimum perception fallback |
| launch/perception_minimal.launch.xml | Crop + euclidean cluster + feature remover composable pipeline |
| config/perception_minimal/ | Param files for the cropbox and voxel-grid clustering nodes |
| scripts/object_relay.py | Cluster -> PEDESTRIAN PredictedObjects, with trajectory corridor filter |
