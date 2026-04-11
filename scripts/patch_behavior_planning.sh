#!/bin/bash
# Patch Autoware launch files to add crowd-matching module support.
# Run this inside the Docker container before launching Autoware.

set -e

TARGET="/opt/autoware/share/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml"

if [ ! -f "$TARGET" ]; then
  echo "ERROR: $TARGET not found"
  exit 1
fi

# Check if already patched
if grep -q "launch_sfm_module" "$TARGET"; then
  echo "Crowd-matching module already patched in behavior_planning.launch.xml"
else
  echo "Patching $TARGET for crowd-matching module..."
  python3 - "$TARGET" << 'PYEOF'
import sys

target = sys.argv[1]
with open(target, 'r') as f:
    lines = f.readlines()

new_lines = []
found_no_drivable = False
for i, line in enumerate(lines):
    new_lines.append(line)

    if 'NoDrivableLaneModulePlugin' in line:
        found_no_drivable = True

    # 1. After launch_no_drivable_lane_module arg, add SFM arg
    if 'launch_no_drivable_lane_module' in line and 'default="true"' in line:
        new_lines.append('  <arg name="launch_sfm_module" default="true"/>\n')

    # 2. After NoDrivableLaneModulePlugin block closes with />, add SFM plugin <let>
    if found_no_drivable and '/>' in line.strip():
        found_no_drivable = False
        new_lines.append(
            '  <let\n'
            '    name="behavior_velocity_planner_launch_modules"\n'
            '    value="$(eval &quot;\'$(var behavior_velocity_planner_launch_modules)\' + \'autoware::behavior_velocity_planner::experimental::SfmModulePlugin, \'&quot;)"\n'
            '    if="$(var launch_sfm_module)"\n'
            '  />\n'
        )

    # 3. After no_drivable_lane param, add SFM param
    if 'no_drivable_lane_module_param_path' in line and '<param' in line:
        new_lines.append('      <param from="$(var behavior_velocity_planner_sfm_module_param_path)"/>\n')

with open(target, 'w') as f:
    f.writelines(new_lines)

print("  behavior_planning.launch.xml patched")
PYEOF
fi

# File 2: tier4_planning_component.launch.xml — add crowd-matching param path definition
# Check all possible locations: bus workspace overlay, sim source, base Autoware
for PLANNING_COMP in \
  "/workspace/install/autoware_launch/share/autoware_launch/launch/components/tier4_planning_component.launch.xml" \
  "/workspace/src/autoware_nuway_launch/autoware_launch_demo/launch/components/tier4_planning_component.launch.xml" \
  "/opt/autoware/share/autoware_launch/launch/components/tier4_planning_component.launch.xml"; do

  if [ ! -f "$PLANNING_COMP" ]; then
    continue
  fi

  if ! grep -q "sfm_module_param_path" "$PLANNING_COMP"; then
    echo "Patching $PLANNING_COMP for crowd-matching param path..."
    sed -i '/no_drivable_lane_module_param_path/a\    <arg name="behavior_velocity_planner_sfm_module_param_path" value="$(find-pkg-share autoware_behavior_velocity_sfm_module)/config/sfm.param.yaml"/>' "$PLANNING_COMP"
    echo "  tier4_planning_component.launch.xml patched"
  else
    echo "Crowd-matching param path already in $PLANNING_COMP"
  fi
done

echo "Crowd-matching module patch applied successfully!"
echo "Verifying..."
grep -n "sfm\|SfmModule" "$TARGET" || echo "WARNING: Patch verification failed"
