#!/bin/bash
# Clean restart of AWSIM + Autoware for iterative testing
# Usage: ./restart_sim.sh [config]
set -e

CONFIG="${1:-campus_tuned_sfm}"
AWSIM_EXEC="$HOME/Unity/AWSIM2.0/Exec/SfmTest/SfmPedestrianTest.x86_64"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Stopping everything ==="
pkill -9 -f SfmPedestrianTest 2>/dev/null || true
docker stop simulation_container 2>/dev/null || true
sleep 3

echo "=== Launching AWSIM ==="
export DISPLAY=:0 ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
"$AWSIM_EXEC" -screen-fullscreen 0 -screen-width 1280 -screen-height 720 > /tmp/awsim.log 2>&1 &
AWSIM_PID=$!
echo "  PID: $AWSIM_PID"
echo "  Waiting 20s for AWSIM to initialize..."
sleep 20

if ! kill -0 $AWSIM_PID 2>/dev/null; then
    echo "ERROR: AWSIM died"
    exit 1
fi
echo "  AWSIM running"

echo "=== Launching Autoware ($CONFIG) ==="
cd "$HOME/Lee_autoware_ws/docker" && docker compose up -d simulation
sleep 5
docker exec -d simulation_container bash -c \
    "export DISPLAY=:0 && /workspace/src/autoware_behavior_velocity_sfm_module/scripts/launch_sfm_test.sh $CONFIG 2>&1 | tee /tmp/autoware_launch.log"
echo "  Waiting 45s for Autoware to start..."
sleep 45

echo "=== Verifying connection ==="
EGO=$(docker exec simulation_container bash -c \
    "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null; \
     timeout 5 ros2 topic echo /localization/kinematic_state --once 2>/dev/null" | grep 'x:' | head -1)
if [ -z "$EGO" ]; then
    echo "ERROR: No ego pose — AWSIM not connected"
    exit 1
fi
echo "  Connected: $EGO"

echo "=== Setting goal and engaging ==="
docker exec simulation_container bash -c \
    "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null; \
     python3 /workspace/src/autoware_behavior_velocity_sfm_module/scripts/set_goal_engage.py --distance 100" 2>&1

echo ""
echo "=== Ready ==="
