#!/bin/bash
# =============================================================================
# Crowd-Matching Evaluation Runner — Records rosbag + screen for A/B/C config comparison
# =============================================================================
#
# Usage (on HOST, not in container):
#   ./run_evaluation.sh [stock|campus_tuned|campus_tuned_sfm] [duration_sec] [goal_distance_m]
#
# Example — run all 3 configs:
#   ./run_evaluation.sh stock 60 10
#   ./run_evaluation.sh campus_tuned 60 10
#   ./run_evaluation.sh campus_tuned_sfm 60 10
#
# Then analyze:
#   docker exec simulation_container python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/analyze_results.py
#
# Prerequisites:
#   - AWSIM SfmPedestrianTest running with ROS_DOMAIN_ID=5
#   - Docker container exists (will be restarted by this script)

set -e

CONFIG="${1:-campus_tuned_sfm}"
DURATION="${2:-60}"
GOAL_DISTANCE="${3:-10}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="${CONFIG}_${TIMESTAMP}"
BAG_DIR="/workspace/rosbags"
HOST_BAG_DIR="/home/harry/Lee_autoware_ws/rosbags"
SCREEN_DIR="/home/harry/sfm_evaluation"
SCRIPT_DIR="/workspace/src/autoware_behavior_velocity_sfm_module/scripts"

echo "============================================"
echo "  Crowd-Matching Evaluation — Config: $CONFIG"
echo "  Duration: ${DURATION}s, Goal: ${GOAL_DISTANCE}m ahead"
echo "  Bag: $BAG_NAME"
echo "============================================"
echo ""

# Create output directories
mkdir -p "$HOST_BAG_DIR" "$SCREEN_DIR"

# 1. Restart container for clean state
echo "[1] Restarting container..."
docker restart simulation_container
sleep 5

# 2. Launch Autoware
echo "[2] Launching Autoware ($CONFIG)..."
docker exec -d simulation_container bash -c \
    "export DISPLAY=:0 && $SCRIPT_DIR/launch_sfm_test.sh $CONFIG 2>&1 | tee /tmp/autoware_launch.log"

# Wait for Autoware to initialize
echo "    Waiting 40s for Autoware startup..."
sleep 40

# Check if launch succeeded
if ! docker exec simulation_container bash -c "grep -q 'Launching Autoware' /tmp/autoware_launch.log 2>/dev/null"; then
    echo "ERROR: Autoware may not have started. Check /tmp/autoware_launch.log"
    exit 1
fi
echo "    Autoware launched."

# 3. Start rosbag recording
echo "[3] Starting rosbag recording..."
TOPICS=(
    /vehicle/status/velocity_status
    /localization/kinematic_state
    /planning/scenario_planning/trajectory
    /perception/object_recognition/objects
    /autoware/state
)

docker exec -d simulation_container bash -c \
    "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
     mkdir -p $BAG_DIR && \
     ros2 bag record -o $BAG_DIR/$BAG_NAME ${TOPICS[*]} 2>&1 | tee /tmp/rosbag_record.log"
echo "    Recording to $BAG_DIR/$BAG_NAME"

# 4. Start screen recording (rviz + AWSIM)
echo "[4] Starting screen recording..."
SCREEN_FILE="$SCREEN_DIR/${BAG_NAME}.mp4"
ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i :0 \
    -c:v libx264 -preset ultrafast -crf 23 \
    "$SCREEN_FILE" \
    < /dev/null > /dev/null 2>&1 &
FFMPEG_PID=$!
echo "    Screen recording PID: $FFMPEG_PID"
echo "    Output: $SCREEN_FILE"

# 5. Set goal and engage
echo "[5] Setting goal ${GOAL_DISTANCE}m ahead and engaging..."
docker exec simulation_container bash -c \
    "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
     python3 $SCRIPT_DIR/set_goal_engage.py --distance $GOAL_DISTANCE" 2>&1 | sed 's/^/    /'

# 6. Wait for test duration
echo ""
echo "[6] Recording for ${DURATION}s... (Ctrl+C to stop early)"
echo "    Started at $(date +%H:%M:%S)"

# Trap Ctrl+C for clean shutdown
cleanup() {
    echo ""
    echo "[7] Stopping..."

    # Stop screen recording
    kill $FFMPEG_PID 2>/dev/null
    wait $FFMPEG_PID 2>/dev/null
    echo "    Screen recording saved: $SCREEN_FILE"

    # Stop rosbag recording and fix permissions
    docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null
    sleep 2
    docker exec simulation_container bash -c "chown -R 1000:1000 $BAG_DIR/" 2>/dev/null
    echo "    Rosbag saved: $HOST_BAG_DIR/$BAG_NAME"

    # Take rviz screenshot
    echo "    Taking screenshot..."
    import -window root "$SCREEN_DIR/${BAG_NAME}_final.png" 2>/dev/null || true

    echo ""
    echo "============================================"
    echo "  Test complete: $CONFIG"
    echo "  Rosbag: $HOST_BAG_DIR/$BAG_NAME"
    echo "  Video:  $SCREEN_FILE"
    echo "============================================"
    exit 0
}
trap cleanup SIGINT SIGTERM

sleep "$DURATION"
cleanup
