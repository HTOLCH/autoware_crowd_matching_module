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

No relay scripts are needed on the bus. The real perception pipeline
(LiDAR detection, multi-object tracker, map-based prediction) publishes
pedestrians to the same topic the module reads from.

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
