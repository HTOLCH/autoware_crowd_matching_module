#!/bin/bash
# =============================================================================
# Density Sweep — Run Config D at varying pedestrian counts
# =============================================================================
#
# Usage (on HOST):
#   ./run_density_sweep.sh [duration_sec] [goal_distance_m]
#
# Prerequisites:
#   - AWSIM SfmPedestrianTest built with command-line override support
#   - Docker container exists

set -e

DURATION="${1:-60}"
GOAL_DISTANCE="${2:-150}"
AWSIM_BIN="/home/harry/Unity/AWSIM2.0/Exec/SfmTest/SfmPedestrianTest.x86_64"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_BAG_DIR="/home/harry/Lee_autoware_ws/rosbags"
SWEEP_DIR="$HOST_BAG_DIR/density_sweep_$(date +%Y%m%d)"
DENSITIES=(5 10 15 20 25)

# --- Helper: clean shutdown of both AWSIM and Autoware ---
clean_shutdown() {
    echo "    Shutting down..."
    # Stop rosbag recording
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    # Stop all ROS processes inside container
    docker exec simulation_container bash -c "pkill -9 -f 'ros2\|python3'" 2>/dev/null || true
    sleep 1
    # Stop the container (kills Autoware completely)
    docker stop simulation_container 2>/dev/null || true
    sleep 2
    # Kill AWSIM
    killall -9 SfmPedestrianTest.x86_64 2>/dev/null || true
    sleep 2
    # Verify both are down
    if pgrep -f SfmPedestrianTest > /dev/null 2>&1; then
        echo "    WARNING: AWSIM still running, force killing..."
        pkill -9 -f SfmPedestrianTest 2>/dev/null || true
        sleep 2
    fi
    echo "    Both AWSIM and Autoware stopped."
}

# --- Helper: launch AWSIM and wait for it to be ready ---
launch_awsim() {
    local PEDS=$1
    echo "[1] Launching AWSIM with --max-peds $PEDS..."
    export ROS_DOMAIN_ID=5
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    "$AWSIM_BIN" --max-peds "$PEDS" &
    AWSIM_PID=$!
    echo "    AWSIM PID: $AWSIM_PID"
    # Wait for AWSIM to initialize (check Player.log for initialization)
    echo "    Waiting for AWSIM startup..."
    for i in $(seq 1 30); do
        if grep -q "CampusPathwaySimulator: Initialized" ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null; then
            echo "    AWSIM ready after ${i}s"
            break
        fi
        sleep 1
    done
    sleep 2
}

# --- Helper: launch Autoware and wait for it to be ready ---
launch_autoware() {
    echo "[2] Starting container and launching Autoware..."
    docker start simulation_container 2>/dev/null || true
    sleep 3
    docker exec -d simulation_container bash -c \
        "/workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh campus_tuned_sfm 2>&1 | tee /tmp/autoware_launch.log"
    echo "    Waiting 45s for Autoware startup..."
    sleep 45
    # Verify Autoware launched
    if docker exec simulation_container bash -c "pgrep -f 'ros2' > /dev/null 2>&1"; then
        echo "    Autoware running."
    else
        echo "    WARNING: Autoware may not have started."
    fi
}

echo "============================================"
echo "  Density Sweep — Config D (crowd_match+follow)"
echo "  Densities: ${DENSITIES[*]}"
echo "  Duration per run: ${DURATION}s"
echo "  Goal distance: ${GOAL_DISTANCE}m"
echo "  Output: $SWEEP_DIR"
echo "============================================"
echo ""

mkdir -p "$SWEEP_DIR"

for PEDS in "${DENSITIES[@]}"; do
    echo ""
    echo "======== DENSITY: $PEDS pedestrians ========"
    echo ""

    # 1. Clean shutdown of everything from previous run
    clean_shutdown

    # 2. Clear AWSIM log so we can detect fresh startup
    > ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null || true

    # 3. Launch AWSIM first, wait for it to be ready
    launch_awsim "$PEDS"

    # 4. Launch Autoware (clean container start)
    launch_autoware

    # 5. Start rosbag recording
    BAG_NAME="density_${PEDS}_$(date +%H%M%S)"
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

    # 7. Wait for test duration
    echo "[5] Recording for ${DURATION}s..."
    sleep "$DURATION"

    # 8. Stop recording and fix permissions
    echo "[6] Stopping recording..."
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    sleep 2
    docker exec simulation_container bash -c "chown -R 1000:1000 /workspace/rosbags/" 2>/dev/null

    # 9. Copy bag to sweep directory
    if [ -d "$HOST_BAG_DIR/$BAG_NAME" ]; then
        mv "$HOST_BAG_DIR/$BAG_NAME" "$SWEEP_DIR/"
        echo "    Saved: $SWEEP_DIR/$BAG_NAME"
    else
        echo "    WARNING: Bag not found at $HOST_BAG_DIR/$BAG_NAME"
    fi

    echo "======== DONE: $PEDS pedestrians ========"
done

# Final cleanup
clean_shutdown

echo ""
echo "============================================"
echo "  Density sweep complete!"
echo "  Results: $SWEEP_DIR"
echo ""
echo "  To analyze:"
echo "  docker exec simulation_container python3 \\"
echo "    /workspace/src/autoware_behavior_velocity_sfm_module/scripts/analyze_results.py \\"
echo "    --bag-dir /workspace/rosbags/density_sweep_$(date +%Y%m%d)"
echo "============================================"
