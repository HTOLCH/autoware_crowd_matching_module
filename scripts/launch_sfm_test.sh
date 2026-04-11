#!/bin/bash
# Launch Autoware for crowd-matching pedestrian testing with ground truth localization.
# Run this INSIDE the Docker container after building the SFM module.
#
# Three test configurations for thesis comparison:
#   A) stock           — Stock Autoware (5m margins, no crowd-matching). Expected: bus stops, cannot navigate.
#   B) campus_tuned    — Reduced margins (1.5m), no crowd-matching. Expected: bus drives but stop/go behavior.
#   C) campus_tuned_sfm — Reduced margins (1.5m) + crowd-matching module. Expected: smooth pedestrian negotiation.
#
# Prerequisites:
#   - AWSIM SfmPedestrianTest scene running with ROS_DOMAIN_ID=5
#   - Docker container running with compose.yaml
#
# Usage:
#   docker exec -it -e DISPLAY=:0 simulation_container bash
#   /workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh [stock|campus_tuned|campus_tuned_sfm]
#
# Default: campus_tuned_sfm

set -e

# === Parse config mode ===
CONFIG="${1:-campus_tuned_sfm}"
case "$CONFIG" in
    stock|campus_tuned|campus_tuned_sfm|campus_tuned_classical_sfm|campus_tuned_crowd_only|campus_tuned_following_only|campus_tuned_classical_following|campus_tuned_sfm_pid)
        ;;
    *)
        echo "ERROR: Unknown config '$CONFIG'"
        echo "Usage: $0 [config]"
        echo ""
        echo "  stock                         — Config A: Stock Autoware (5m margins, no crowd-matching)"
        echo "  campus_tuned                  — Config B: Reduced margins, no crowd-matching"
        echo "  campus_tuned_classical_sfm    — Config C: Classical only"
        echo "  campus_tuned_sfm              — Config D: Crowd-Matching + Following (best)"
        echo "  campus_tuned_crowd_only       — Config E: Crowd-Matching only (no following)"
        echo "  campus_tuned_following_only   — Config F: Following only (no velocity model)"
        echo "  campus_tuned_classical_following — Config G: Classical + Following"
        echo "  campus_tuned_sfm_pid          — Config H: Crowd-Matching + PID Following"
        exit 1
        ;;
esac

MAP_PATH="/autoware_map_sfm"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/configs/$CONFIG"
DIAG_DIR="/workspace/install/autoware_launch/share/autoware_launch/config/system/diagnostics"
MOTION_DIR="/workspace/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner"

echo "============================================"
echo "  Crowd-Matching Test Launcher — Config: $CONFIG"
echo "============================================"

case "$CONFIG" in
    stock)
        echo "  Config A: Stock Autoware (5m margins, no crowd-matching)"
        echo "  Expected: Bus stops and cannot navigate campus pathway"
        ;;
    campus_tuned)
        echo "  Config B: Campus-tuned margins (1.5m), no crowd-matching"
        echo "  Expected: Bus drives but with binary stop/go behavior"
        ;;
    campus_tuned_classical_sfm)
        echo "  Config C: Campus-tuned margins + Classical Helbing (repulsive force)"
        echo "  Expected: Speed oscillation due to force-velocity feedback loop"
        ;;
    campus_tuned_sfm)
        echo "  Config D: Campus-tuned margins + Crowd-Matching (proposed solution)"
        echo "  Expected: Smooth crowd-matching velocity"
        ;;
esac
echo ""

# 1. Check map is mounted
if [ ! -f "$MAP_PATH/lanelet2_map.osm" ]; then
    echo "ERROR: Map not found at $MAP_PATH"
    echo "Make sure autoware_map_sfm_test is mounted in compose.yaml"
    exit 1
fi

# 2. Source Autoware
source /opt/autoware/setup.bash
# Source workspace overlay if built
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# 3. Apply SFM behavior planning patch (only for campus_tuned_sfm)
SFM_CONFIGS="campus_tuned_sfm campus_tuned_classical_sfm campus_tuned_crowd_only campus_tuned_following_only campus_tuned_classical_following campus_tuned_sfm_pid"
if echo "$SFM_CONFIGS" | grep -qw "$CONFIG"; then
    echo "[3] Applying crowd-matching behavior planning patch..."
    bash "$SCRIPT_DIR/patch_behavior_planning.sh"
    # Set mode based on config
    # Install per-config SFM parameters (each config has its own tuned sfm.param.yaml)
    SFM_PARAM="/workspace/install/autoware_behavior_velocity_sfm_module/share/autoware_behavior_velocity_sfm_module/config/sfm.param.yaml"
    CONFIG_SFM_PARAM="$CONFIG_DIR/sfm.param.yaml"
    if [ -f "$CONFIG_SFM_PARAM" ]; then
        cp "$CONFIG_SFM_PARAM" "$SFM_PARAM"
        MODE=$(grep 'mode:' "$SFM_PARAM" | awk '{print $2}' | tr -d '"')
        echo "    Crowd-matching config installed from $CONFIG (mode: $MODE)"
    else
        # Fallback: set mode via sed
        case "$CONFIG" in
            campus_tuned_classical_sfm)      MODE="classical" ;;
            campus_tuned_crowd_only)         MODE="crowd_only" ;;
            campus_tuned_following_only)     MODE="following_only" ;;
            campus_tuned_classical_following) MODE="classical_following" ;;
            campus_tuned_sfm_pid)            MODE="crowd_matching_pid" ;;
            *)                               MODE="crowd_matching" ;;
        esac
        if [ -f "$SFM_PARAM" ]; then
            sed -i "s/mode: .*/mode: \"$MODE\"/" "$SFM_PARAM"
            echo "    Crowd-matching mode: $MODE (fallback)"
        fi
    fi
else
    echo "[3] Skipping crowd-matching patch (config: $CONFIG)"
fi

# 4. Install simulation-only diagnostic config (removes localization checks)
echo "[4] Installing sim diagnostic config..."
# Restore from previous unclean shutdown
if [ -f "$DIAG_DIR/autoware-awsim.yaml.bak" ]; then
    mv "$DIAG_DIR/autoware-awsim.yaml.bak" "$DIAG_DIR/autoware-awsim.yaml"
fi
cp "$SCRIPT_DIR/diagnostics-sfm-sim.yaml" "$DIAG_DIR/diagnostics-sfm-sim.yaml"
cp "$DIAG_DIR/autoware-awsim.yaml" "$DIAG_DIR/autoware-awsim.yaml.bak"
cat > "$DIAG_DIR/autoware-awsim.yaml" << 'EOF'
files:
  - { path: $(dirname)/diagnostics-sfm-sim.yaml }
EOF
echo "    Diagnostic config updated (localization checks disabled for sim)"

# 5. Install motion planning params for this config
echo "[5] Installing motion planning params ($CONFIG)..."
# First restore any leftover .bak files from a previous unclean shutdown
for f in obstacle_stop.param.yaml road_user_stop.param.yaml dynamic_obstacle_stop.param.yaml; do
    if [ -f "$MOTION_DIR/$f.bak" ]; then
        mv "$MOTION_DIR/$f.bak" "$MOTION_DIR/$f"
    fi
done
# Now backup originals and install config
for f in obstacle_stop.param.yaml road_user_stop.param.yaml dynamic_obstacle_stop.param.yaml; do
    if [ -f "$CONFIG_DIR/$f" ]; then
        if [ -f "$MOTION_DIR/$f" ]; then
            cp "$MOTION_DIR/$f" "$MOTION_DIR/$f.bak"
        fi
        cp "$CONFIG_DIR/$f" "$MOTION_DIR/$f"
    fi
done
if [ "$CONFIG" = "stock" ]; then
    echo "    obstacle_stop: 5.0m (stock), road_user_stop: 5.0m (stock)"
else
    echo "    obstacle_stop: 1.5m (tuned), road_user_stop: 1.5m (tuned)"
fi

# 5b. Set velocity smoother engage_velocity for campus speed (all non-stock configs)
if [ "$CONFIG" != "stock" ]; then
    VS_NUWAY="/workspace/src/autoware_nuway_launch/autoware_launch_demo/config/planning/scenario_planning/common/autoware_velocity_smoother"
    # Restore from previous unclean shutdown
    if [ -f "$VS_NUWAY/velocity_smoother.param.yaml.bak" ]; then
        mv "$VS_NUWAY/velocity_smoother.param.yaml.bak" "$VS_NUWAY/velocity_smoother.param.yaml"
    fi
    if [ -f "$VS_NUWAY/velocity_smoother.param.yaml" ]; then
        cp "$VS_NUWAY/velocity_smoother.param.yaml" "$VS_NUWAY/velocity_smoother.param.yaml.bak"
        sed -i 's/engage_velocity: .*/engage_velocity: 0.25  # keep default for startup/' "$VS_NUWAY/velocity_smoother.param.yaml"
        sed -i 's/engage_acceleration: .*/engage_acceleration: 1.0  # campus: faster ramp/' "$VS_NUWAY/velocity_smoother.param.yaml"
        sed -i 's/engage_exit_ratio: .*/engage_exit_ratio: 0.9  # exit engage almost immediately (at 0.225 m\/s)/' "$VS_NUWAY/velocity_smoother.param.yaml"
        echo "    Velocity smoother: engage=0.25, exit_ratio=0.9, accel=1.0 (fast engage exit)"
    fi
fi

# 5c. Install campus surround obstacle checker (disable ped/bicycle departure block)
# On a shared pathway, pedestrians are always within 0.5m — stock config permanently blocks departure.
SURROUND_DIR="/opt/autoware/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker"
# Restore from previous unclean shutdown
if [ -f "$SURROUND_DIR/surround_obstacle_checker.param.yaml.bak" ]; then
    mv "$SURROUND_DIR/surround_obstacle_checker.param.yaml.bak" "$SURROUND_DIR/surround_obstacle_checker.param.yaml"
fi
cp "$SURROUND_DIR/surround_obstacle_checker.param.yaml" "$SURROUND_DIR/surround_obstacle_checker.param.yaml.bak"
cp "$SCRIPT_DIR/surround_obstacle_checker.param.yaml" "$SURROUND_DIR/surround_obstacle_checker.param.yaml"
echo "    Surround obstacle checker: pedestrian/bicycle departure check disabled"

# 6. Start ground truth relay in background
echo "[6] Starting ground truth localization relay..."
python3 "$SCRIPT_DIR/ground_truth_relay.py" &
RELAY_PID=$!
echo "    Relay PID: $RELAY_PID"

# 7. Start object detection relay (promotes clustering to PEDESTRIAN PredictedObjects)
echo "[7] Starting object detection relay..."
python3 "$SCRIPT_DIR/object_relay.py" &
OBJ_RELAY_PID=$!
echo "    Object relay PID: $OBJ_RELAY_PID"

# Give relays a moment to start
sleep 1

# 7b. Set rviz view to TopDownOrtho with base_link target frame
RVIZ_FILE="/workspace/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz"
if [ -f "$RVIZ_FILE" ]; then
    sed -i 's/Target Frame: viewer/Target Frame: base_link/' "$RVIZ_FILE"
    sed -i 's/Class: rviz_default_plugins\/ThirdPersonFollower/Class: rviz_default_plugins\/TopDownOrtho/' "$RVIZ_FILE"
fi

# 8. Disable steering convergence check (blocks startup on straight paths)
echo "[8] Setting post-launch params (will apply once controller starts)..."
(sleep 20 && ros2 param set /control/trajectory_follower/controller_node_exe enable_keep_stopped_until_steer_convergence false 2>/dev/null && echo "    Steering convergence check disabled") &

# 9. Launch Autoware
echo "[9] Launching Autoware ($CONFIG)..."
echo ""
ros2 launch autoware_launch nuway_simulator.launch.xml \
    vehicle_model:=nuway_vehicle \
    sensor_model:=nuway_sensor_kit \
    map_path:=$MAP_PATH \
    data_path:=/autoware_data \
    use_sim_time:=true \
    localization:=false \
    rviz:=true

# === Cleanup ===
echo ""
echo "Shutting down..."
kill $RELAY_PID 2>/dev/null || true
kill $OBJ_RELAY_PID 2>/dev/null || true
# Restore original configs
if [ -f "$DIAG_DIR/autoware-awsim.yaml.bak" ]; then
    mv "$DIAG_DIR/autoware-awsim.yaml.bak" "$DIAG_DIR/autoware-awsim.yaml"
fi
for f in obstacle_stop.param.yaml road_user_stop.param.yaml dynamic_obstacle_stop.param.yaml; do
    if [ -f "$MOTION_DIR/$f.bak" ]; then
        mv "$MOTION_DIR/$f.bak" "$MOTION_DIR/$f"
    fi
done
if [ -f "$SURROUND_DIR/surround_obstacle_checker.param.yaml.bak" ]; then
    mv "$SURROUND_DIR/surround_obstacle_checker.param.yaml.bak" "$SURROUND_DIR/surround_obstacle_checker.param.yaml"
fi
VS_NUWAY="/workspace/src/autoware_nuway_launch/autoware_launch_demo/config/planning/scenario_planning/common/autoware_velocity_smoother"
if [ -f "$VS_NUWAY/velocity_smoother.param.yaml.bak" ]; then
    mv "$VS_NUWAY/velocity_smoother.param.yaml.bak" "$VS_NUWAY/velocity_smoother.param.yaml"
fi
echo "Done. Config '$CONFIG' cleaned up."
