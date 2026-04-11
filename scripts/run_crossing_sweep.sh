#!/bin/bash
# =============================================================================
# Crossing Scenario Sweep — Run all 7 configs on crossing pedestrian scene
# =============================================================================
#
# Usage (on HOST):
#   ./run_crossing_sweep.sh [duration_sec] [goal_distance_m]
#
# Prerequisites:
#   - AWSIM SfmCrossingTest built with crossing pedestrians enabled
#   - Docker container exists

set -e

DURATION="${1:-60}"
GOAL_DISTANCE="${2:-40}"
AWSIM_BIN="/home/harry/Unity/AWSIM2.0/Exec/SfmCrossingTest/SfmCrossingTest.x86_64"
HOST_BAG_DIR="/home/harry/Lee_autoware_ws/rosbags"
SWEEP_DIR="$HOST_BAG_DIR/crossing_sweep_$(date +%Y%m%d)"

# All 7 configs in order
CONFIGS=(
    stock
    campus_tuned
    campus_tuned_classical_sfm
    campus_tuned_sfm
    campus_tuned_crowd_only
    campus_tuned_following_only
    campus_tuned_classical_following
)

CONFIG_LABELS=(
    "A: Stock Autoware"
    "B: Campus-Tuned"
    "C: Classical"
    "D: Crowd-Match+Follow"
    "E: Crowd-Match Only"
    "F: Following Only"
    "G: Classical+Follow"
)

# --- Helper: clean shutdown of both AWSIM and Autoware ---
clean_shutdown() {
    echo "    Shutting down..."
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    docker exec simulation_container bash -c "pkill -9 -f 'ros2\|python3'" 2>/dev/null || true
    sleep 1
    docker stop simulation_container 2>/dev/null || true
    sleep 2
    killall -9 SfmCrossingTest.x86_64 2>/dev/null || true
    killall -9 SfmPedestrianTest.x86_64 2>/dev/null || true
    sleep 2
    if pgrep -f "SfmCrossingTest\|SfmPedestrianTest" > /dev/null 2>&1; then
        pkill -9 -f "SfmCrossingTest\|SfmPedestrianTest" 2>/dev/null || true
        sleep 2
    fi
    echo "    Both AWSIM and Autoware stopped."
}

echo "============================================"
echo "  Crossing Scenario — 7-Way Config Sweep"
echo "  Configs: ${#CONFIGS[@]}"
echo "  Duration per run: ${DURATION}s"
echo "  Goal distance: ${GOAL_DISTANCE}m"
echo "  Output: $SWEEP_DIR"
echo "============================================"
echo ""

# Check AWSIM binary exists
if [ ! -f "$AWSIM_BIN" ]; then
    echo "ERROR: AWSIM crossing build not found at: $AWSIM_BIN"
    echo "Build the SfmCrossingTest scene in Unity first."
    exit 1
fi

mkdir -p "$SWEEP_DIR"

for idx in "${!CONFIGS[@]}"; do
    CONFIG="${CONFIGS[$idx]}"
    LABEL="${CONFIG_LABELS[$idx]}"

    echo ""
    echo "======== $LABEL ($CONFIG) ========"
    echo ""

    # 1. Clean shutdown
    clean_shutdown

    # 2. Clear AWSIM log
    > ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null || true

    # 3. Launch AWSIM (crossing scene, default pedestrian settings from Inspector)
    echo "[1] Launching AWSIM (crossing scene)..."
    export ROS_DOMAIN_ID=5
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    "$AWSIM_BIN" &
    AWSIM_PID=$!
    echo "    AWSIM PID: $AWSIM_PID"
    echo "    Waiting for AWSIM startup..."
    for i in $(seq 1 30); do
        if grep -q "CampusPathwaySimulator: Initialized" ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null; then
            echo "    AWSIM ready after ${i}s"
            break
        fi
        sleep 1
    done
    sleep 2

    # 4. Launch Autoware with this config
    echo "[2] Starting container and launching Autoware ($CONFIG)..."
    docker start simulation_container 2>/dev/null || true
    sleep 3
    docker exec -d simulation_container bash -c \
        "/workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh $CONFIG 2>&1 | tee /tmp/autoware_launch.log"
    echo "    Waiting 45s for Autoware startup..."
    sleep 45

    # 5. Start rosbag recording
    BAG_NAME="${CONFIG}_crossing_$(date +%H%M%S)"
    echo "[3] Recording rosbag: $BAG_NAME..."
    TOPICS="/vehicle/status/velocity_status /localization/kinematic_state /planning/scenario_planning/trajectory /perception/object_recognition/objects /autoware/state"
    docker exec -d simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         mkdir -p /workspace/rosbags && \
         ros2 bag record -o /workspace/rosbags/$BAG_NAME $TOPICS"

    # 6. Set goal and engage
    echo "[4] Setting goal (${GOAL_DISTANCE}m) and engaging..."
    docker exec simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/set_goal_engage.py --distance $GOAL_DISTANCE" 2>&1 | sed 's/^/    /'

    # 7. Wait for test duration
    echo "[5] Recording for ${DURATION}s..."
    sleep "$DURATION"

    # 8. Stop recording and save
    echo "[6] Stopping recording..."
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    sleep 2
    docker exec simulation_container bash -c "chown -R 1000:1000 /workspace/rosbags/" 2>/dev/null

    if [ -d "$HOST_BAG_DIR/$BAG_NAME" ]; then
        mv "$HOST_BAG_DIR/$BAG_NAME" "$SWEEP_DIR/"
        echo "    Saved: $SWEEP_DIR/$BAG_NAME"
    else
        echo "    WARNING: Bag not found at $HOST_BAG_DIR/$BAG_NAME"
    fi

    echo "======== DONE: $LABEL ========"
done

# Final cleanup
clean_shutdown

echo ""
echo "============================================"
echo "  Crossing scenario sweep complete!"
echo "  Results: $SWEEP_DIR"
echo ""
echo "  To analyze:"
echo "  docker exec simulation_container python3 \\"
echo "    /workspace/src/autoware_behavior_velocity_sfm_module/scripts/analyze_results.py \\"
echo "    --bag-dir /workspace/rosbags/crossing_sweep_$(date +%Y%m%d)"
echo "============================================"
