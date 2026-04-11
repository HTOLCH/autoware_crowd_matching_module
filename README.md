# Crowd-Matching Velocity Planning Module for Autoware

A behavior_velocity_planner plugin that enables smooth pedestrian-aware driving on shared pathways. Instead of Autoware's default binary stop/go behaviour, this module matches co-flow pedestrian speed and maintains safe following distance via proximity-based velocity attenuation.

Developed for the nUWAy campus shuttle bus as part of a Masters thesis at the University of Western Australia.

## Problem

Stock Autoware places 5m stop margins before all detected pedestrians, making shared-pathway driving impossible. The bus stops repeatedly and cannot make progress on campus footpaths where pedestrians are always present.

## Solution

The module provides two velocity planning strategies:

- **Crowd-matching** (default, recommended): Detects co-flow pedestrians, computes their mean speed, and smoothly converges to match it. Combined with proximity-based velocity attenuation that targets zero velocity at a configurable minimum clearance distance.
- **Classical SFM**: Helbing & Molnar (1995) repulsive force model. Included for comparison but not recommended due to oscillation from speed-proportional buffer feedback.

A dual-layer safety architecture ensures safe operation:
1. **Module layer**: Proximity attenuation (10m onset, 3m zero target) with exponential relaxation smoothing
2. **Autoware backup layer**: obstacle_stop + road_user_stop at 1.2m margins (emergency only)

## Where It Fits in the Autoware Planning Pipeline

```
Mission Planner -> Behavior Path Planner -> Behavior Velocity Planner -> Motion Velocity Planner -> Velocity Smoother -> Control
                                                    ^
                                            This module runs HERE
                                      (modifies velocity along existing path)
```

The module modifies velocity only along the existing lanelet-following path. It does not alter steering. Autoware's behavior_path_planner still handles lane following.

## Quick Start (Deployment on nUWAy Bus)

See [DEPLOYMENT.md](DEPLOYMENT.md) for full step-by-step instructions including git safety workflow, sensor reference, and troubleshooting.

```bash
# 1. Clone into the Autoware workspace src/ directory
cd /path/to/autoware_ws/src
git clone https://github.com/HTOLCH/autoware_crowd_matching_module.git autoware_behavior_velocity_sfm_module

# 2. Build (inside Docker container)
cd /workspace
colcon build --packages-select autoware_behavior_velocity_sfm_module
source /workspace/install/setup.bash

# 3. Deploy (patches Autoware launch files + configs)
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh enable

# 4. Launch Autoware normally
ros2 launch autoware_launch autoware.launch.xml data_path:=/autoware_data map_path:=/autoware_map
```

To disable and restore all files:
```bash
/workspace/src/autoware_behavior_velocity_sfm_module/scripts/deploy_sfm.sh disable
```

## Algorithm Modes

### Crowd-Matching (mode: "crowd_matching")

1. **Pedestrian filtering**: Detect pedestrians within `detection_radius` (15m) from `/perception/object_recognition/objects`
2. **Co-flow detection**: Project each pedestrian's velocity onto ego forward direction. Pedestrians with forward speed > `co_flow_threshold` (0.3 m/s) are co-flow
3. **Speed matching**: If enough co-flow pedestrians exist (>= `min_crowd_size`), target velocity = mean co-flow speed
4. **Crowd hold**: After crowd disappears, gradually ramp back to desired velocity over `crowd_hold_time` (1.5s)
5. **Proximity attenuation**: Quadratic distance scaling from `braking_distance` (10m) to `min_clearance` (3m), targeting zero velocity at min_clearance
6. **Relaxation smoothing**: Exponential convergence with time constant `relaxation_time` (2.0s)

### Classical SFM (mode: "classical")

Helbing & Molnar (1995) repulsive force with speed-proportional safety buffer. Included for comparative study. Exhibits oscillation due to buffer feedback loop.

## Parameters

All parameters in `config/sfm.param.yaml` under the `sfm` namespace:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | "crowd_matching" | Algorithm: crowd_matching, classical, crowd_only, following_only, classical_following, crowd_matching_pid |
| `desired_velocity` | 1.39 | Target speed when no crowd (m/s, = 5 km/h) |
| `relaxation_time` | 2.0 | Velocity convergence time constant (s) |
| `detection_radius` | 15.0 | Max distance to consider pedestrians (m) |
| `braking_distance` | 10.0 | Proximity attenuation onset distance (m) |
| `min_clearance` | 3.0 | Module targets zero velocity at this distance (m) |
| `co_flow_threshold` | 0.3 | Min pedestrian forward speed for co-flow (m/s) |
| `min_crowd_size` | 1 | Min co-flow pedestrians to trigger crowd matching |
| `crowd_hold_time` | 1.5 | Hold crowd speed after crowd disappears (s) |
| `vehicle_half_width` | 1.2 | Bus half-width for lateral path check (m) |
| `pedestrian_radius` | 0.3 | Pedestrian collision radius (m) |

## Package Structure

```
autoware_behavior_velocity_sfm_module/
├── CMakeLists.txt          # Build config
├── package.xml             # ROS 2 package manifest
├── plugins.xml             # pluginlib class registration
├── DEPLOYMENT.md           # Bus deployment guide
├── config/
│   └── sfm.param.yaml     # Tunable parameters
├── src/
│   ├── manager.hpp         # Plugin manager + required subscriptions
│   ├── manager.cpp         # Parameter loading, PLUGINLIB_EXPORT_CLASS
│   ├── scene.hpp           # SfmParameters struct + SfmModule class
│   └── scene.cpp           # Core algorithm (243 lines)
└── scripts/
    ├── deploy_sfm.sh       # Enable/disable/status/clean (main deployment tool)
    ├── patch_behavior_planning.sh  # Patches Autoware launch XML
    ├── launch_sfm_test.sh  # Simulation launcher (8 configs)
    ├── object_relay.py     # Sim-only: tracking + velocity estimation relay
    ├── ground_truth_relay.py   # Sim-only: ground truth localization
    ├── analyze_results.py  # Rosbag analysis + comparison plots
    ├── set_goal_engage.py  # Automated goal setting + engage
    ├── run_evaluation.sh   # Single config evaluation
    ├── run_density_sweep.sh    # Density sweep (5-25 peds)
    ├── run_crossing_sweep.sh   # Crossing scenario sweep
    ├── run_failure_modes.sh    # Stationary pedestrian tests
    ├── run_jumpout_tests.sh    # Timed pedestrian spawn tests
    ├── run_merge_test.sh       # Merge-onto-pathway transition test
    └── configs/            # Per-config parameter sets (8 configs A-H)
```

## 8-Way Comparative Study

| Config | Velocity Model | Following Controller | Key Result |
|--------|---------------|---------------------|------------|
| A: Stock | None (5m stops) | N/A | 3 stops, 36% stationary |
| B: Campus-Tuned | None (0.8m stops) | N/A | Drives but no crowd awareness |
| C: Classical | Helbing repulsive | Off | Oscillates (feedback loop) |
| D: Crowd-Match+Prox | Crowd-matching | Proximity attenuation | Best: StdDev=0.048, 0 stops |
| E: Crowd-Match Only | Crowd-matching | Off | Good but no safe following |
| F: Following Only | None | Proximity attenuation | Follows but no speed matching |
| G: Classical+Follow | Helbing repulsive | Proximity attenuation | Still oscillates |
| H: Crowd-Match+PID | Crowd-matching | PID controller | 5x worse jerk |

## Dependencies

- `autoware_behavior_velocity_planner_common` (experimental plugin API)
- `autoware_perception_msgs` (PredictedObjects)
- `autoware_motion_utils`, `autoware_utils`
- `pluginlib`, `rclcpp`, `geometry_msgs`, `visualization_msgs`

## References

- Helbing, D. & Molnar, P. (1995). Social force model for pedestrian dynamics. Physical Review E, 51(5), 4282-4286.

## License

Apache License 2.0
