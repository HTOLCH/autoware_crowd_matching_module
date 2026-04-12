#!/bin/bash
# =============================================================================
# Crowd-Matching Module Minimum Perception Layer
# =============================================================================
#
# Brings up a stripped-down LiDAR perception pipeline (crop + euclidean
# clustering + feature remover + object_relay) that feeds the crowd-matching
# module on the bus when the full Autoware perception stack is unavailable.
#
# Usage:
#   ./deploy_perception.sh start    # Start launch + relay in background
#   ./deploy_perception.sh stop     # Kill the launch + relay processes
#   ./deploy_perception.sh status   # Check Hz on the relevant topics
#
# Optional environment overrides:
#   CM_LIDAR_TOPIC=/sensing/lidar/concatenated/pointcloud   raw lidar input
#   CM_CORRIDOR_WIDTH=2.0                                   relay corridor (m)
#   CM_MAX_FORWARD=25.0                                     relay max-forward (m)
# =============================================================================

set -e

ACTION="${1:-status}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PID_FILE="/tmp/cm_perception.pids"

LIDAR_TOPIC="${CM_LIDAR_TOPIC:-/sensing/lidar/concatenated/pointcloud}"
CORRIDOR_WIDTH="${CM_CORRIDOR_WIDTH:-2.0}"
MAX_FORWARD="${CM_MAX_FORWARD:-25.0}"

cluster_in_topic="/perception/object_recognition/detection/clustering/objects"
relay_out_topic="/perception/object_recognition/objects"

start() {
    if [ -f "$PID_FILE" ]; then
        echo "Already running (PID file exists). Run 'stop' first."
        exit 1
    fi
    echo "[1/2] Launching minimum perception pipeline..."
    echo "      lidar input: $LIDAR_TOPIC"
    ros2 launch autoware_behavior_velocity_sfm_module perception_minimal.launch.xml \
         input_pointcloud:="$LIDAR_TOPIC" > /tmp/cm_perception.log 2>&1 &
    LAUNCH_PID=$!
    echo "      launch PID: $LAUNCH_PID  (log: /tmp/cm_perception.log)"

    # Give the container time to come up before starting the relay
    sleep 3

    echo "[2/2] Starting object_relay.py with corridor filter..."
    echo "      corridor width: ${CORRIDOR_WIDTH} m, max forward: ${MAX_FORWARD} m"
    python3 "$SCRIPT_DIR/object_relay.py" \
        --ros-args \
        -p "lane_corridor_width:=$CORRIDOR_WIDTH" \
        -p "max_forward_distance:=$MAX_FORWARD" \
        > /tmp/cm_object_relay.log 2>&1 &
    RELAY_PID=$!
    echo "      relay PID:  $RELAY_PID  (log: /tmp/cm_object_relay.log)"

    echo "$LAUNCH_PID $RELAY_PID" > "$PID_FILE"
    echo ""
    echo "Done. Verify with: $0 status"
}

stop() {
    if [ ! -f "$PID_FILE" ]; then
        echo "No PID file — perception not running via this script."
        # Try to clean up any stragglers anyway
        pkill -f perception_minimal.launch.xml 2>/dev/null || true
        pkill -f "object_relay.py" 2>/dev/null || true
        exit 0
    fi
    read -r LAUNCH_PID RELAY_PID < "$PID_FILE"
    echo "Stopping launch ($LAUNCH_PID) and relay ($RELAY_PID)..."
    kill "$LAUNCH_PID" 2>/dev/null || true
    kill "$RELAY_PID" 2>/dev/null || true
    sleep 1
    # Belt and braces: the launched composable container child might survive
    pkill -f perception_minimal.launch.xml 2>/dev/null || true
    pkill -f "object_relay.py" 2>/dev/null || true
    pkill -f cm_perception_container 2>/dev/null || true
    rm -f "$PID_FILE"
    echo "Stopped."
}

status() {
    echo "=== Crowd-Matching Minimum Perception Status ==="
    if [ -f "$PID_FILE" ]; then
        read -r LAUNCH_PID RELAY_PID < "$PID_FILE"
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "  perception launch: RUNNING (pid $LAUNCH_PID)"
        else
            echo "  perception launch: NOT RUNNING (stale PID $LAUNCH_PID)"
        fi
        if kill -0 "$RELAY_PID" 2>/dev/null; then
            echo "  object_relay:      RUNNING (pid $RELAY_PID)"
        else
            echo "  object_relay:      NOT RUNNING (stale PID $RELAY_PID)"
        fi
    else
        echo "  Not started via this script."
    fi
    echo ""
    echo "Topic Hz checks (5s timeout each — Ctrl-C to skip):"
    for topic in "$LIDAR_TOPIC" \
                "/perception/obstacle_segmentation/pointcloud" \
                "$cluster_in_topic" \
                "$relay_out_topic"; do
        echo "  $topic"
        timeout 5 ros2 topic hz "$topic" 2>&1 | head -2 | sed 's/^/    /'
    done
}

case "$ACTION" in
    start)  start ;;
    stop)   stop ;;
    status) status ;;
    *)
        echo "Usage: $0 {start|stop|status}"
        exit 1
        ;;
esac
