#!/bin/bash
# =============================================================================
# Failure Mode Testing — Stationary pedestrian at various distances
# =============================================================================
#
# Usage (on HOST):
#   ./run_failure_modes.sh [duration_sec]
#
# Tests:
#   1. Stationary pedestrian at 15m (normal braking distance)
#   2. Stationary pedestrian at 8m (braking onset)
#   3. Stationary pedestrian at 5m (emergency zone)
#   4. No pedestrians (baseline)
#
# P-key tests (manual):
#   Run AWSIM with --jump-dist 5/8/15 and press P during the run.

set -e

DURATION="${1:-60}"
GOAL_DISTANCE="150"
AWSIM_BIN="/home/harry/Unity/AWSIM2.0/Exec/SfmTest/SfmPedestrianTest.x86_64"
HOST_BAG_DIR="/home/harry/Lee_autoware_ws/rosbags"
SWEEP_DIR="$HOST_BAG_DIR/failure_modes_$(date +%Y%m%d)"
CONFIG="campus_tuned_sfm"

# Test cases: name:max-peds:stationary-dist (0 = no stationary)
declare -a TESTS=(
    "stationary_15m:10:15"
    "stationary_8m:10:8"
    "stationary_5m:10:5"
    "no_peds:0:0"
)

# --- Helper: clean shutdown of both AWSIM and Autoware ---
clean_shutdown() {
    echo "    Shutting down..."
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    docker exec simulation_container bash -c "pkill -9 -f 'ros2\|python3'" 2>/dev/null || true
    sleep 1
    docker stop simulation_container 2>/dev/null || true
    sleep 2
    killall -9 SfmPedestrianTest.x86_64 2>/dev/null || true
    sleep 2
    if pgrep -f SfmPedestrianTest > /dev/null 2>&1; then
        pkill -9 -f SfmPedestrianTest 2>/dev/null || true
        sleep 2
    fi
    echo "    Both AWSIM and Autoware stopped."
}

echo "============================================"
echo "  Failure Mode Testing — Config D"
echo "  Tests: ${#TESTS[@]} scenarios"
echo "  Duration per run: ${DURATION}s"
echo "  Output: $SWEEP_DIR"
echo "============================================"
echo ""

mkdir -p "$SWEEP_DIR"

for TEST in "${TESTS[@]}"; do
    IFS=':' read -r NAME PEDS STAT_DIST <<< "$TEST"

    echo ""
    echo "======== TEST: $NAME ========"
    echo ""

    # 1. Clean shutdown
    clean_shutdown

    # 2. Clear AWSIM log
    > ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null || true

    # 3. Build AWSIM args and launch
    AWSIM_ARGS="--max-peds $PEDS"
    if [ "$STAT_DIST" != "0" ]; then
        AWSIM_ARGS="$AWSIM_ARGS --stationary-dist $STAT_DIST"
    fi

    echo "[1] Launching AWSIM with: $AWSIM_ARGS..."
    export ROS_DOMAIN_ID=5
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    "$AWSIM_BIN" $AWSIM_ARGS &
    AWSIM_PID=$!
    echo "    Waiting for AWSIM startup..."
    for i in $(seq 1 30); do
        if grep -q "CampusPathwaySimulator: Initialized" ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null; then
            echo "    AWSIM ready after ${i}s"
            break
        fi
        sleep 1
    done
    sleep 2

    # 4. Launch Autoware
    echo "[2] Starting container and launching Autoware..."
    docker start simulation_container 2>/dev/null || true
    sleep 3
    docker exec -d simulation_container bash -c \
        "/workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh $CONFIG 2>&1 | tee /tmp/autoware_launch.log"
    echo "    Waiting 45s for Autoware startup..."
    sleep 45

    # 5. Record rosbag
    BAG_NAME="${NAME}_$(date +%H%M%S)"
    echo "[3] Recording rosbag: $BAG_NAME..."
    TOPICS="/vehicle/status/velocity_status /localization/kinematic_state /planning/scenario_planning/trajectory /perception/object_recognition/objects /autoware/state"
    docker exec -d simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         mkdir -p /workspace/rosbags && \
         ros2 bag record -o /workspace/rosbags/$BAG_NAME $TOPICS"

    # 6. Set goal and engage
    echo "[4] Setting goal and engaging..."
    docker exec simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/set_goal_engage.py --distance $GOAL_DISTANCE" 2>&1 | sed 's/^/    /'

    # 7. Wait
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
        echo "    WARNING: Bag not found"
    fi

    echo "======== DONE: $NAME ========"
done

# Final cleanup
clean_shutdown

echo ""
echo "============================================"
echo "  Failure mode tests complete!"
echo "  Results: $SWEEP_DIR"
echo "============================================"
