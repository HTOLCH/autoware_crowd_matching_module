#!/bin/bash
# =============================================================================
# Jump-Out Tests — Pedestrian appears suddenly at various distances
# while bus is at cruising speed (timed spawn after 15s delay)
# =============================================================================
#
# Usage (on HOST):
#   ./run_jumpout_tests.sh [duration_sec]
#
# Requires AWSIM build with --jump-delay support

set -e

DURATION="${1:-45}"
GOAL_DISTANCE=150
AWSIM_BIN="/home/harry/Unity/AWSIM2.0/Exec/SfmTest/SfmPedestrianTest.x86_64"
HOST_BAG_DIR="/home/harry/Lee_autoware_ws/rosbags"
SWEEP_DIR="$HOST_BAG_DIR/jumpout_tests_$(date +%Y%m%d)"
CONFIG="campus_tuned_sfm"
JUMP_DELAY=15  # seconds after start — bus should be at ~1.39 m/s by then

# Test cases: name:jump_distance:max_peds
declare -a TESTS=(
    "jumpout_15m:15:10"
    "jumpout_8m:8:10"
    "jumpout_5m:5:10"
    "jumpout_3m:3:10"
)

clean_shutdown() {
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    docker exec simulation_container bash -c "pkill -9 -f 'ros2\|python3'" 2>/dev/null || true
    sleep 1
    docker stop simulation_container 2>/dev/null || true
    sleep 2
    killall -9 SfmPedestrianTest.x86_64 2>/dev/null || true
    sleep 2
    echo "    Shutdown complete."
}

echo "============================================"
echo "  Jump-Out Tests — Config D"
echo "  Tests: ${#TESTS[@]} distances"
echo "  Jump delay: ${JUMP_DELAY}s (bus at cruising speed)"
echo "  Duration per run: ${DURATION}s"
echo "  Output: $SWEEP_DIR"
echo "============================================"

mkdir -p "$SWEEP_DIR"

for TEST in "${TESTS[@]}"; do
    IFS=':' read -r NAME DIST PEDS <<< "$TEST"

    echo ""
    echo "======== TEST: $NAME (spawn at ${DIST}m after ${JUMP_DELAY}s) ========"
    echo ""

    clean_shutdown
    > ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null || true

    echo "[1] Launching AWSIM with --max-peds $PEDS --jump-delay $JUMP_DELAY --jump-dist $DIST..."
    export ROS_DOMAIN_ID=5
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    "$AWSIM_BIN" --max-peds "$PEDS" --jump-delay "$JUMP_DELAY" --jump-dist "$DIST" &
    for i in $(seq 1 30); do
        grep -q "CampusPathwaySimulator: Initialized" ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null && break
        sleep 1
    done
    sleep 2

    echo "[2] Launching Autoware ($CONFIG)..."
    docker start simulation_container 2>/dev/null || true
    sleep 3
    docker exec -d simulation_container bash -c \
        "/workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh $CONFIG 2>&1 | tee /tmp/autoware_launch.log"
    sleep 45

    BAG_NAME="${NAME}_$(date +%H%M%S)"
    echo "[3] Recording: $BAG_NAME..."
    TOPICS="/vehicle/status/velocity_status /localization/kinematic_state /planning/scenario_planning/trajectory /perception/object_recognition/objects /autoware/state"
    docker exec -d simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         mkdir -p /workspace/rosbags && \
         ros2 bag record -o /workspace/rosbags/$BAG_NAME $TOPICS"

    echo "[4] Setting goal and engaging..."
    docker exec simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/set_goal_engage.py --distance $GOAL_DISTANCE" 2>&1 | sed 's/^/    /'

    echo "[5] Recording for ${DURATION}s (pedestrian spawns at ${JUMP_DELAY}s)..."
    sleep "$DURATION"

    echo "[6] Stopping..."
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    sleep 2
    docker exec simulation_container bash -c "chown -R 1000:1000 /workspace/rosbags/" 2>/dev/null

    if [ -d "$HOST_BAG_DIR/$BAG_NAME" ]; then
        mv "$HOST_BAG_DIR/$BAG_NAME" "$SWEEP_DIR/"
        echo "    Saved: $SWEEP_DIR/$BAG_NAME"
    fi

    echo "======== DONE: $NAME ========"
done

clean_shutdown

echo ""
echo "============================================"
echo "  Jump-out tests complete!"
echo "  Results: $SWEEP_DIR"
echo "============================================"
