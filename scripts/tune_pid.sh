#!/bin/bash
# =============================================================================
# PID Tuning — Run Config H with different PID gains on straight pathway
# =============================================================================
set -e

DURATION=60
GOAL_DISTANCE=150
AWSIM_BIN="/home/harry/Unity/AWSIM2.0/Exec/SfmTest/SfmPedestrianTest.x86_64"
HOST_BAG_DIR="/home/harry/Lee_autoware_ws/rosbags"
TUNE_DIR="$HOST_BAG_DIR/pid_tuning_$(date +%Y%m%d)"
CONFIG="campus_tuned_sfm_pid"
# Per-config yaml that launch_sfm_test.sh copies to install dir at launch
SFM_SOURCE="/home/harry/Lee_autoware_ws/src/autoware_behavior_velocity_sfm_module/scripts/configs/campus_tuned_sfm_pid/sfm.param.yaml"

# Tuning iterations: desired_dist:kp:ki:kd:label
declare -a TUNES=(
    "5.0:0.15:0.02:0.05:baseline"
    "5.0:0.25:0.02:0.05:higher_kp"
    "5.0:0.15:0.05:0.05:higher_ki"
    "5.0:0.15:0.02:0.10:higher_kd"
    "5.0:0.20:0.03:0.08:balanced"
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
echo "  PID Tuning — Config H on straight pathway"
echo "  Iterations: ${#TUNES[@]}"
echo "  Duration: ${DURATION}s per run"
echo "  Output: $TUNE_DIR"
echo "============================================"

mkdir -p "$TUNE_DIR"

for TUNE in "${TUNES[@]}"; do
    IFS=':' read -r DIST KP KI KD LABEL <<< "$TUNE"

    echo ""
    echo "======== TUNE: $LABEL (Kp=$KP Ki=$KI Kd=$KD dist=$DIST) ========"
    echo ""

    # 1. Set PID params in source config BEFORE launching anything
    echo "[1] Setting PID params in source config..."
    sed -i "s/pid_desired_distance: .*/pid_desired_distance: $DIST/" "$SFM_SOURCE"
    sed -i "s/pid_kp: .*/pid_kp: $KP/" "$SFM_SOURCE"
    sed -i "s/pid_ki: .*/pid_ki: $KI/" "$SFM_SOURCE"
    sed -i "s/pid_kd: .*/pid_kd: $KD/" "$SFM_SOURCE"
    echo "    Set: dist=$DIST Kp=$KP Ki=$KI Kd=$KD"

    # 2. Clean shutdown
    clean_shutdown
    > ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null || true

    # 3. Launch AWSIM
    echo "[2] Launching AWSIM..."
    export ROS_DOMAIN_ID=5
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    "$AWSIM_BIN" &
    for i in $(seq 1 30); do
        grep -q "CampusPathwaySimulator: Initialized" ~/.config/unity3d/TIERIV/AWSIM/Player.log 2>/dev/null && break
        sleep 1
    done
    sleep 2

    # 4. Launch Autoware (will install the updated sfm.param.yaml)
    echo "[3] Launching Autoware ($CONFIG)..."
    docker start simulation_container 2>/dev/null || true
    sleep 3
    docker exec -d simulation_container bash -c \
        "/workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh $CONFIG 2>&1 | tee /tmp/autoware_launch.log"
    sleep 45

    # 5. Record
    BAG_NAME="campus_tuned_sfm_pid_${LABEL}_$(date +%H%M%S)"
    echo "[4] Recording: $BAG_NAME..."
    TOPICS="/vehicle/status/velocity_status /localization/kinematic_state /planning/scenario_planning/trajectory /perception/object_recognition/objects /autoware/state"
    docker exec -d simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         mkdir -p /workspace/rosbags && \
         ros2 bag record -o /workspace/rosbags/$BAG_NAME $TOPICS"

    # 6. Engage
    echo "[5] Setting goal and engaging..."
    docker exec simulation_container bash -c \
        "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
         python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/set_goal_engage.py --distance $GOAL_DISTANCE" 2>&1 | sed 's/^/    /'

    # 7. Wait
    echo "[6] Recording for ${DURATION}s..."
    sleep "$DURATION"

    # 8. Stop and save
    echo "[7] Stopping..."
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null || true
    sleep 2
    docker exec simulation_container bash -c "chown -R 1000:1000 /workspace/rosbags/" 2>/dev/null

    if [ -d "$HOST_BAG_DIR/$BAG_NAME" ]; then
        mv "$HOST_BAG_DIR/$BAG_NAME" "$TUNE_DIR/"
        echo "    Saved: $TUNE_DIR/$BAG_NAME"
    fi

    echo "======== DONE: $LABEL ========"
done

clean_shutdown

# Restore baseline PID params
sed -i "s/pid_desired_distance: .*/pid_desired_distance: 5.0/" "$SFM_SOURCE"
sed -i "s/pid_kp: .*/pid_kp: 0.15/" "$SFM_SOURCE"
sed -i "s/pid_ki: .*/pid_ki: 0.02/" "$SFM_SOURCE"
sed -i "s/pid_kd: .*/pid_kd: 0.05/" "$SFM_SOURCE"

echo ""
echo "============================================"
echo "  PID Tuning complete! Results: $TUNE_DIR"
echo "============================================"
