#!/bin/bash
# =============================================================================
# Crowd-Matching Module Deployment — Enable or disable the crowd-matching module
# =============================================================================
#
# Usage:
#   ./deploy_sfm.sh enable    # Enable crowd-matching module + apply campus configs
#   ./deploy_sfm.sh disable   # Disable crowd-matching module + restore stock configs
#   ./deploy_sfm.sh status    # Check if crowd-matching module is currently enabled
#   ./deploy_sfm.sh clean     # Remove all .sfm_bak files (run before deploying to a new machine)
#
# This script:
#   1. Patches/unpatches behavior_planning.launch.xml for the crowd-matching plugin
#   2. Installs/restores motion planning configs (obstacle_stop, road_user_stop, etc.)
#   3. Installs/restores velocity smoother tuning
#   4. Installs/restores surround obstacle checker config
#   5. Enables/restores safety modules in the planning preset (dual-layer safety)
#
# All changes are reversible — 'disable' restores every file from .sfm_bak backups.

set -e

ACTION="${1:-status}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── Path resolution ──────────────────────────────────────────────────────────
# Workspace overlay takes precedence over /opt/autoware/ base install.
# Supports both bus (autoware_launch.nuway) and sim (Lee_autoware_ws) workspaces.

BEHAVIOR_LAUNCH="/opt/autoware/share/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml"

# Detect workspace overlay (bus or sim)
WS_OVERLAY=""
if [ -d "/workspace/install/autoware_launch/share/autoware_launch/config/planning" ]; then
    WS_OVERLAY="/workspace/install/autoware_launch/share/autoware_launch"
    echo "  [env] Using workspace overlay: $WS_OVERLAY"
fi

# Motion planning configs
if [ -n "$WS_OVERLAY" ]; then
    MOTION_DIR="$WS_OVERLAY/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner"
else
    MOTION_DIR="/opt/autoware/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner"
fi

# Velocity smoother
if [ -n "$WS_OVERLAY" ]; then
    VS_DIR="$WS_OVERLAY/config/planning/scenario_planning/common/autoware_velocity_smoother"
else
    VS_DIR="/opt/autoware/share/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother"
fi

# Surround obstacle checker
if [ -n "$WS_OVERLAY" ]; then
    SURROUND_DIR="$WS_OVERLAY/config/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker"
else
    SURROUND_DIR="/opt/autoware/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/surround_obstacle_checker"
fi

# Planning preset (for enabling safety modules)
PRESET_FILE=""
if [ -n "$WS_OVERLAY" ] && [ -f "$WS_OVERLAY/config/planning/preset/nuway_preset.yaml" ]; then
    PRESET_FILE="$WS_OVERLAY/config/planning/preset/nuway_preset.yaml"
elif [ -f "/opt/autoware/share/autoware_launch/config/planning/preset/nuway_preset.yaml" ]; then
    PRESET_FILE="/opt/autoware/share/autoware_launch/config/planning/preset/nuway_preset.yaml"
fi

# Additional velocity smoother paths (Lee's sim workspace source — only if it exists)
VS_NUWAY="/workspace/src/autoware_nuway_launch/autoware_launch_demo/config/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml"

CONFIG_DIR="$SCRIPT_DIR/configs/campus_tuned_sfm"

is_enabled() {
    grep -q "launch_sfm_module" "$BEHAVIOR_LAUNCH" 2>/dev/null
}

do_enable() {
    echo "=== Enabling Crowd-Matching Module ==="

    # 1. Patch behavior planning launch
    echo "[1] Patching behavior_planning.launch.xml..."
    bash "$SCRIPT_DIR/patch_behavior_planning.sh"

    # 2. Backup and install motion planning configs
    echo "[2] Installing motion planning configs (campus-tuned)..."
    for f in obstacle_stop.param.yaml road_user_stop.param.yaml dynamic_obstacle_stop.param.yaml; do
        if [ -f "$CONFIG_DIR/$f" ] && [ -f "$MOTION_DIR/$f" ]; then
            if [ ! -f "$MOTION_DIR/$f.sfm_bak" ]; then
                cp "$MOTION_DIR/$f" "$MOTION_DIR/$f.sfm_bak"
            fi
            cp "$CONFIG_DIR/$f" "$MOTION_DIR/$f"
            echo "    $f installed"
        fi
    done

    # 3. Velocity smoother tuning
    echo "[3] Tuning velocity smoother..."
    VS_FILE="$VS_DIR/velocity_smoother.param.yaml"
    if [ -f "$VS_FILE" ]; then
        if [ ! -f "$VS_FILE.sfm_bak" ]; then
            cp "$VS_FILE" "$VS_FILE.sfm_bak"
        fi
        sed -i 's/engage_velocity: .*/engage_velocity: 0.25/' "$VS_FILE"
        sed -i 's/engage_acceleration: .*/engage_acceleration: 1.0/' "$VS_FILE"
        sed -i 's/engage_exit_ratio: .*/engage_exit_ratio: 0.9/' "$VS_FILE"
        echo "    engage_exit_ratio=0.9, engage_acceleration=1.0"
    fi

    # Also patch Lee's sim workspace source if it exists
    if [ -f "$VS_NUWAY" ]; then
        if [ ! -f "$VS_NUWAY.sfm_bak" ]; then
            cp "$VS_NUWAY" "$VS_NUWAY.sfm_bak"
        fi
        sed -i 's/engage_velocity: .*/engage_velocity: 0.25/' "$VS_NUWAY"
        sed -i 's/engage_acceleration: .*/engage_acceleration: 1.0/' "$VS_NUWAY"
        sed -i 's/engage_exit_ratio: .*/engage_exit_ratio: 0.9/' "$VS_NUWAY"
    fi

    # 4. Surround obstacle checker
    echo "[4] Installing surround obstacle checker config..."
    if [ -f "$SURROUND_DIR/surround_obstacle_checker.param.yaml" ]; then
        if [ ! -f "$SURROUND_DIR/surround_obstacle_checker.param.yaml.sfm_bak" ]; then
            cp "$SURROUND_DIR/surround_obstacle_checker.param.yaml" "$SURROUND_DIR/surround_obstacle_checker.param.yaml.sfm_bak"
        fi
        cp "$SCRIPT_DIR/surround_obstacle_checker.param.yaml" "$SURROUND_DIR/surround_obstacle_checker.param.yaml"
        echo "    Pedestrian/bicycle departure check disabled"
    fi

    # 5. Enable safety modules in planning preset (dual-layer safety)
    if [ -n "$PRESET_FILE" ] && [ -f "$PRESET_FILE" ]; then
        echo "[5] Enabling safety modules in preset..."
        if [ ! -f "$PRESET_FILE.sfm_bak" ]; then
            cp "$PRESET_FILE" "$PRESET_FILE.sfm_bak"
        fi
        # Enable obstacle_stop and road_user_stop as backup safety layer
        sed -i '/launch_obstacle_stop_module/{n;s/default: "false"/default: "true"/}' "$PRESET_FILE" 2>/dev/null || \
        sed -i 's/\(launch_obstacle_stop_module\).*default: "false"/\1\n      default: "true"/' "$PRESET_FILE" 2>/dev/null || true
        sed -i '/launch_road_user_stop_module/{n;s/default: "false"/default: "true"/}' "$PRESET_FILE" 2>/dev/null || \
        sed -i 's/\(launch_road_user_stop_module\).*default: "false"/\1\n      default: "true"/' "$PRESET_FILE" 2>/dev/null || true
        # Verify
        if grep -A1 'launch_obstacle_stop_module' "$PRESET_FILE" | grep -q '"true"'; then
            echo "    obstacle_stop_module: ENABLED"
        else
            echo "    WARNING: Could not enable obstacle_stop_module — check preset format"
        fi
        if grep -A1 'launch_road_user_stop_module' "$PRESET_FILE" | grep -q '"true"'; then
            echo "    road_user_stop_module: ENABLED"
        else
            echo "    WARNING: Could not enable road_user_stop_module — check preset format"
        fi
    else
        echo "[5] No nuway_preset.yaml found — skipping safety module activation"
        echo "    Ensure obstacle_stop and road_user_stop modules are enabled manually"
    fi

    echo ""
    echo "Crowd-matching module ENABLED. Restart Autoware to apply."
    echo ""
    echo "NOTE: If the bus won't start driving (steering_converged error), run:"
    echo "  ros2 param set /control/trajectory_follower/controller_node_exe enable_keep_stopped_until_steer_convergence false"
}

do_disable() {
    echo "=== Disabling Crowd-Matching Module ==="

    # 1. Unpatch behavior planning launch + planning component
    echo "[1] Removing crowd-matching module from launch files..."
    if is_enabled; then
        sed -i '/launch_sfm_module/d' "$BEHAVIOR_LAUNCH"
        sed -i '/SfmModulePlugin/d' "$BEHAVIOR_LAUNCH"
        sed -i '/sfm_module_param_path/d' "$BEHAVIOR_LAUNCH"
        echo "    behavior_planning.launch.xml cleaned"
    else
        echo "    Already disabled"
    fi
    # Clean planning component — check all possible locations
    for pc in \
        "/workspace/install/autoware_launch/share/autoware_launch/launch/components/tier4_planning_component.launch.xml" \
        "/workspace/src/autoware_nuway_launch/autoware_launch_demo/launch/components/tier4_planning_component.launch.xml" \
        "/opt/autoware/share/autoware_launch/launch/components/tier4_planning_component.launch.xml"; do
        if [ -f "$pc" ]; then
            sed -i '/sfm_module_param_path/d' "$pc"
        fi
    done
    echo "    tier4_planning_component.launch.xml cleaned"

    # 2. Restore motion planning configs
    echo "[2] Restoring motion planning configs..."
    for f in obstacle_stop.param.yaml road_user_stop.param.yaml dynamic_obstacle_stop.param.yaml; do
        if [ -f "$MOTION_DIR/$f.sfm_bak" ]; then
            mv "$MOTION_DIR/$f.sfm_bak" "$MOTION_DIR/$f"
            echo "    $f restored"
        fi
    done

    # 3. Restore velocity smoother
    echo "[3] Restoring velocity smoother..."
    for vs in "$VS_DIR/velocity_smoother.param.yaml" "$VS_NUWAY"; do
        if [ -f "$vs.sfm_bak" ]; then
            mv "$vs.sfm_bak" "$vs"
            echo "    Restored: $vs"
        fi
    done

    # 4. Restore surround obstacle checker
    echo "[4] Restoring surround obstacle checker..."
    if [ -f "$SURROUND_DIR/surround_obstacle_checker.param.yaml.sfm_bak" ]; then
        mv "$SURROUND_DIR/surround_obstacle_checker.param.yaml.sfm_bak" "$SURROUND_DIR/surround_obstacle_checker.param.yaml"
        echo "    Restored"
    fi

    # 5. Restore planning preset
    echo "[5] Restoring planning preset..."
    if [ -n "$PRESET_FILE" ] && [ -f "$PRESET_FILE.sfm_bak" ]; then
        mv "$PRESET_FILE.sfm_bak" "$PRESET_FILE"
        echo "    nuway_preset.yaml restored"
    else
        echo "    No preset backup found — skipping"
    fi

    echo ""
    echo "Crowd-matching module DISABLED. All configs restored to stock. Restart Autoware to apply."
}

do_status() {
    echo "=== Crowd-Matching Module Status ==="
    if is_enabled; then
        echo "  Crowd-matching plugin: ENABLED (patched in behavior_planning.launch.xml)"
    else
        echo "  Crowd-matching plugin: DISABLED"
    fi

    if [ -f "$MOTION_DIR/obstacle_stop.param.yaml.sfm_bak" ]; then
        echo "  Motion configs: Campus-tuned (backups present)"
    else
        echo "  Motion configs: Stock"
    fi

    if grep -q 'engage_exit_ratio: 0.9' "$VS_DIR/velocity_smoother.param.yaml" 2>/dev/null; then
        echo "  Velocity smoother: Campus-tuned (exit_ratio=0.9)"
    else
        echo "  Velocity smoother: Stock"
    fi

    if [ -n "$PRESET_FILE" ] && [ -f "$PRESET_FILE.sfm_bak" ]; then
        echo "  Planning preset: Modified (safety modules enabled, backup present)"
    else
        echo "  Planning preset: Stock"
    fi

    if [ -n "$PRESET_FILE" ] && [ -f "$PRESET_FILE" ]; then
        if grep -A1 'launch_obstacle_stop_module' "$PRESET_FILE" | grep -q '"true"'; then
            echo "  obstacle_stop_module: ENABLED"
        else
            echo "  obstacle_stop_module: DISABLED"
        fi
        if grep -A1 'launch_road_user_stop_module' "$PRESET_FILE" | grep -q '"true"'; then
            echo "  road_user_stop_module: ENABLED"
        else
            echo "  road_user_stop_module: DISABLED"
        fi
    fi
}

do_clean() {
    if is_enabled; then
        echo "ERROR: Crowd-matching module is currently enabled. Run './deploy_sfm.sh disable' first."
        echo "Otherwise you'll lose the stock defaults that are stored in .sfm_bak files."
        exit 1
    fi
    echo "=== Cleaning crowd-matching backups ==="
    echo "This removes all .sfm_bak files so the next 'enable' captures fresh defaults."
    local count=0
    # Motion configs
    for f in "$MOTION_DIR"/*.sfm_bak; do
        if [ -f "$f" ]; then rm "$f"; echo "  Removed: $f"; count=$((count + 1)); fi
    done
    # Velocity smoother
    for f in "$VS_DIR"/*.sfm_bak; do
        if [ -f "$f" ]; then rm "$f"; echo "  Removed: $f"; count=$((count + 1)); fi
    done
    # Surround obstacle checker
    for f in "$SURROUND_DIR"/*.sfm_bak; do
        if [ -f "$f" ]; then rm "$f"; echo "  Removed: $f"; count=$((count + 1)); fi
    done
    # Planning preset
    if [ -n "$PRESET_FILE" ] && [ -f "$PRESET_FILE.sfm_bak" ]; then
        rm "$PRESET_FILE.sfm_bak"; echo "  Removed: $PRESET_FILE.sfm_bak"; count=$((count + 1))
    fi
    # Lee's sim workspace
    if [ -f "$VS_NUWAY.sfm_bak" ]; then
        rm "$VS_NUWAY.sfm_bak"; echo "  Removed: $VS_NUWAY.sfm_bak"; count=$((count + 1))
    fi
    echo "Cleaned $count backup files."
}

case "$ACTION" in
    enable)
        do_enable
        ;;
    disable)
        do_disable
        ;;
    status)
        do_status
        ;;
    clean)
        do_clean
        ;;
    *)
        echo "Usage: $0 [enable|disable|status|clean]"
        echo ""
        echo "  enable   Enable crowd-matching module + apply campus configs"
        echo "  disable  Disable crowd-matching module + restore stock configs"
        echo "  status   Check if crowd-matching module is currently enabled"
        echo "  clean    Remove all .sfm_bak files (run before deploying to a new machine)"
        exit 1
        ;;
esac
