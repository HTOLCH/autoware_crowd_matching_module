# SFM Module — Autoware Architecture Diagrams

## 1. Full Autoware Stack with SFM Module Placement

```
 ============================================================================================
 |                              AUTOWARE AUTONOMOUS DRIVING STACK                            |
 ============================================================================================

                    AWSIM (Unity Simulator)                   Real Vehicle
                    ~~~~~~~~~~~~~~~~~~~~~~                    ~~~~~~~~~~~~
                    | LiDAR pointclouds  |                    | Sensors   |
                    | Camera images      |                    | CAN bus   |
                    | GNSS/IMU           |                    | GNSS/IMU  |
                    | Vehicle status     |                    |           |
                    +--------+-----------+                    +-----+-----+
                             |          ROS 2 DDS                   |
 ============================|==========================================|====================
 |                           v                                         v                    |
 |  +------------------+  +-------------------+  +--------------------------------------------+
 |  |   LOCALIZATION   |  |     SENSING       |  |                PERCEPTION                  |
 |  |                  |  |                   |  |                                            |
 |  | NDT Scan Match   |  | Pointcloud concat |  | LiDAR: CenterPoint 3D detection           |
 |  | EKF Localizer    |  | Ground removal    |  | Camera: YoloX 2D detection                |
 |  | Pose Initializer |  | Crop box filter   |  | Multi-object tracker                      |
 |  |                  |  |                   |  | Map-based prediction                      |
 |  +--------+---------+  +-------------------+  |                                            |
 |           |                                    +---------------------+----------------------+
 |           |                                                          |
 |           v                                                          v
 |  /localization/kinematic_state                  /perception/object_recognition/objects
 |  (PoseStamped + TwistStamped)                   (PredictedObjects)
 |           |                                                          |
 |           +------------------------+  +------------------------------+
 |                                    |  |
 |  +=========================================================================+
 |  |                            PLANNING                                     |
 |  |                                                                         |
 |  |  +------------------------------------------------------------------+  |
 |  |  |  Mission Planner                                                  |  |
 |  |  |  - Receives goal pose from RViz / API                            |  |
 |  |  |  - Computes route along lanelet graph                            |  |
 |  |  +----+-------------------------------------------------------------+  |
 |  |       | /planning/mission_planning/route                                |
 |  |       v                                                                 |
 |  |  +------------------------------------------------------------------+  |
 |  |  |  Behavior Path Planner                                            |  |
 |  |  |  - Lane following along lanelets                                  |  |
 |  |  |  - Lane changes, avoidance, start/goal planning                   |  |
 |  |  |  - Outputs: lateral path + reference velocities                   |  |
 |  |  +----+-------------------------------------------------------------+  |
 |  |       | path_with_lane_id (PathWithLaneId)                              |
 |  |       v                                                                 |
 |  |  +==================================================================+  |
 |  |  ||              Behavior Velocity Planner                         ||  |
 |  |  ||                                                                ||  |
 |  |  ||  Dynamically loads plugins via pluginlib. Each plugin          ||  |
 |  |  ||  receives the path and can MODIFY POINT VELOCITIES.           ||  |
 |  |  ||                                                                ||  |
 |  |  ||  Loaded modules (from nuway_preset.yaml):                     ||  |
 |  |  ||                                                                ||  |
 |  |  ||    +------------------+  +------------------+                  ||  |
 |  |  ||    | crosswalk_module |  | traffic_light    |                  ||  |
 |  |  ||    | Stop at occupied |  | Stop at red      |                  ||  |
 |  |  ||    | crosswalks       |  | lights           |                  ||  |
 |  |  ||    +------------------+  +------------------+                  ||  |
 |  |  ||    +------------------+  +------------------+                  ||  |
 |  |  ||    | stop_line_module |  | intersection     |                  ||  |
 |  |  ||    | Stop at map stop |  | Yield/priority   |                  ||  |
 |  |  ||    | lines            |  | at intersections |                  ||  |
 |  |  ||    +------------------+  +------------------+                  ||  |
 |  |  ||    +------------------+  +------------------+                  ||  |
 |  |  ||    | blind_spot       |  | detection_area   |                  ||  |
 |  |  ||    | ...              |  | ...              |                  ||  |
 |  |  ||    +------------------+  +------------------+                  ||  |
 |  |  ||                                                                ||  |
 |  |  ||    +====================================================+     ||  |
 |  |  ||    ||            *** SFM MODULE ***                     ||     ||  |
 |  |  ||    ||                                                   ||     ||  |
 |  |  ||    ||  Input:  PredictedObjects (pedestrians)           ||     ||  |
 |  |  ||    ||          EgoPose, EgoVelocity                     ||     ||  |
 |  |  ||    ||          PathWithLaneId                           ||     ||  |
 |  |  ||    ||                                                   ||     ||  |
 |  |  ||    ||  Process: Helbing & Molnar SFM forces             ||     ||  |
 |  |  ||    ||           + speed-proportional safety buffer      ||     ||  |
 |  |  ||    ||           + approach velocity amplification       ||     ||  |
 |  |  ||    ||           + exponential smoothing                 ||     ||  |
 |  |  ||    ||                                                   ||     ||  |
 |  |  ||    ||  Output: same path geometry, only speeds reduced   ||     ||  |
 |  |  ||    +====================================================+     ||  |
 |  |  ||                                                                ||  |
 |  |  +==================================================================+  |
 |  |       | path (Path — lane IDs stripped)                                 |
 |  |       v                                                                 |
 |  |  +------------------------------------------------------------------+  |
 |  |  |  Motion Velocity Planner                                          |  |
 |  |  |  - obstacle_stop: emergency stop for obstacles on path            |  |
 |  |  |  - obstacle_cruise: adaptive cruise control                       |  |
 |  |  |  - obstacle_slow_down: decelerate near obstacles                  |  |
 |  |  |  - dynamic_obstacle_stop: stop for dynamic obstacles              |  |
 |  |  |  - out_of_lane: slow for out-of-lane hazards                      |  |
 |  |  +----+-------------------------------------------------------------+  |
 |  |       | trajectory                                                      |
 |  |       v                                                                 |
 |  |  +------------------------------------------------------------------+  |
 |  |  |  Velocity Smoother (JerkFiltered)                                 |  |
 |  |  |  - Enforces jerk/acceleration limits                              |  |
 |  |  |  - Produces dynamically feasible trajectory                       |  |
 |  |  +----+-------------------------------------------------------------+  |
 |  |       | /planning/scenario_planning/trajectory                          |
 |  +=========================================================================+
 |           |
 |           v
 |  +------------------------------------------------------------------+
 |  |                         CONTROL                                   |
 |  |  - MPC lateral controller (steering)                              |
 |  |  - PID longitudinal controller (accel/brake)                      |
 |  |  - Vehicle command gateway                                        |
 |  +----+-------------------------------------------------------------+
 |       | /control/command/control_cmd
 |       v
 ============================================================================================
         |
         v
    Vehicle Interface (AWSIM / CAN bus)
```

## 2. SFM Module Internal Processing Pipeline

```
   modifyPathVelocity(PathWithLaneId * path) — called every planning cycle (~10-20 Hz)
   ======================================================================

   planner_data_->predicted_objects
         |
         |  (1) FILTER PEDESTRIANS
         |  - Check classification[0].label == PEDESTRIAN
         |  - Check distance to ego < detection_radius (15 m)
         |
         v
   std::vector<PedestrianInfo>    (poses + twists of nearby pedestrians)
         |
         |  (2) FOR EACH PATH POINT
         |
         v
   +-----+-----------------------------------------------------------+
   |  path point [i]                                                  |
   |  original_velocity = point.longitudinal_velocity_mps             |
   |                                                                  |
   |  FOR EACH PEDESTRIAN:                                            |
   |  +-----------------------------------------------------------+  |
   |  |                                                           |  |
   |  |  d = distance(path_point, pedestrian)                     |  |
   |  |                                                           |  |
   |  |  Base force:                                              |  |
   |  |    F = A * exp((r_sum - d) / B)                           |  |
   |  |                                                           |  |
   |  |  Speed-proportional buffer (if d - k*v_ego < r_sum):      |  |
   |  |    F = A * exp((r_sum - (d - k*v_ego)) / B)               |  |
   |  |                                                           |  |
   |  |  Approach velocity amplification:                         |  |
   |  |    v_approach = ped_velocity projected toward ego          |  |
   |  |    if v_approach > 0:  F *= (1 + v_approach)              |  |
   |  |                                                           |  |
   |  |  Saturation:                                              |  |
   |  |    F = min(F, force_saturation)                           |  |
   |  |                                                           |  |
   |  +-------------------+---------------------------------------+  |
   |                      |                                          |
   |  total_force += F    | (sum over all pedestrians)               |
   |                      v                                          |
   |  FORCE-TO-VELOCITY MAPPING:                                     |
   |    scale_raw = max(0, 1 - total_force / force_saturation)       |
   |                                                                  |
   |  EXPONENTIAL SMOOTHING:                                          |
   |    scale = alpha * scale_prev + (1-alpha) * scale_raw            |
   |                                                                  |
   |  APPLY:                                                          |
   |    v_new = max(original_velocity * scale, min_velocity)          |
   |    point.longitudinal_velocity_mps = v_new                       |
   |                                                                  |
   +------------------------------------------------------------------+
         |
         v
   Same path returned (geometry unchanged, only point velocities modified)
```

## 3. Force Response Curve

Shows how repulsive force magnitude varies with pedestrian distance for different ego velocities.

```
   Force
   (capped at force_saturation = 10.0)

   10 |****                          * = v_ego = 1.39 m/s (5 km/h)
      |    ***                        o = v_ego = 0.5 m/s
      |  oo   ***                     . = v_ego = 0.0 m/s (stationary)
    8 | o  ..    **
      |o  .        **
      |  .           **
    6 |o .             **
      | .               **
      |.                  *
    4 |.              oo   **
      |            ooo       **
      |         ooo            **
    2 |      ooo          ..     ***
      |   ooo          ...         ****
      |ooo          ...                *****
    0 +------+------+------+------+------+------+-------> distance (m)
      0    1.5     3      5      7      9     11    13
           ^r_sum
           |
      Safety buffer shifts curve LEFT at higher speeds:
        v=0.0: buffer = 0.0 m (rightmost curve)
        v=0.5: buffer = 1.0 m
        v=1.4: buffer = 2.8 m (leftmost curve, reacts earliest)
```

## 4. Velocity Profile Along Path — Example Scenario

Single pedestrian standing 5 m from path, ego travelling at 1.39 m/s:

```
   velocity (m/s)
   1.4 |==========\                                  /===========
       |           \                                /
   1.2 |            \                              /
       |             \                            /
   1.0 |              \                          /
       |               \                        /
   0.8 |                \                      /
       |                 \                    /
   0.6 |                  \                  /
       |                   \                /
   0.4 |                    \              /
       |                     \            /
   0.2 |                      \          /
       |                       \________/
   0.0 +---+---+---+---+---+---+---+---+---+---+---+----> path (m)
       0   5  10  15  20  25  30  35  40  45  50  55
                               ^
                          pedestrian location
                          (5 m lateral offset)


   Stock Autoware (crosswalk module only):

   1.4 |================================================== (no crosswalk = no reaction)
       |
   0.0 +-------------------------------------------------> path (m)

                  OR if on crosswalk:

   1.4 |==========\
       |           |  <-- binary stop
   0.0 |           |_________|===========================> path (m)
                   ^         ^
              stop line    ped clears crosswalk
```

## 5. Plugin Loading Sequence

```
   Docker Container Start
         |
         v
   colcon build --packages-select autoware_behavior_velocity_sfm_module
         |
         |  installs:
         |    - lib/libautoware_behavior_velocity_sfm_module.so
         |    - share/.../plugins.xml  -->  pluginlib ament_index registration
         |    - share/.../config/sfm.param.yaml
         |
         v
   ros2 launch autoware_launch nuway_simulator.launch.xml
         |
         v
   nuway_preset.yaml
     launch_sfm_module: "true"
         |
         v
   behavior_planning.launch.xml
     +--  assembles launch_modules list:
     |      [..., "autoware::behavior_velocity_planner::SfmModulePlugin", ...]
     |
     +--  loads sfm.param.yaml into node parameter space
         |
         v
   BehaviorVelocityPlannerNode::init()
     for (name : launch_modules):
       planner_manager_.launchScenePlugin(*this, name)
         |
         v
   pluginlib::ClassLoader  --->  loads libautoware_behavior_velocity_sfm_module.so
         |
         v
   SfmModulePlugin::init()
     creates SfmModuleManager(node)
       reads params: sfm.desired_velocity, sfm.interaction_strength, ...
         |
         v
   SfmModuleManager::launchNewModules()
     creates SfmModule(module_id=0, params)
         |
         v
   [RUNNING] SfmModule::modifyPathVelocity() called every planning cycle
```

## 6. Interaction with Other Velocity Modules

The behavior_velocity_planner runs all enabled modules **sequentially** on the same path.
Each module can only **reduce** velocities (never increase above the original).

```
   PathWithLaneId from behavior_path_planner
   (all points at lanelet speed limit, e.g., 1.39 m/s)
         |
         v
   +--crosswalk_module-----+  Sets v=0 at occupied crosswalk stop lines
   |  v: [1.39, ..., 0, 0, ..., 1.39]
   +------------------------+
         |
         v
   +--traffic_light_module--+  Sets v=0 at red traffic lights
   |  v: [1.39, ..., 0, 0, ..., 1.39]
   +------------------------+
         |
         v
   +--stop_line_module------+  Sets v=0 at map-defined stop lines
   |  v: [1.39, ..., 0, ..., 1.39]
   +------------------------+
         |
         v
   +--sfm_module------------+  Continuously scales velocities near pedestrians
   |  v: [1.39, 1.2, 0.8, 0.4, 0.2, 0.4, 0.8, 1.2, 1.39]
   +------------------------+    ^-- smooth gradient, not binary
         |
         v
   +--other modules---------+  (blind_spot, detection_area, etc.)
   +------------------------+
         |
         v
   Path output to motion_velocity_planner
   (velocities are the MINIMUM of all module modifications)
```
