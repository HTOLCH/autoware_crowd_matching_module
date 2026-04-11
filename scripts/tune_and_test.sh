#!/bin/bash
# Quick tune-and-test cycle for a single config
# Usage: ./tune_and_test.sh <config_name> <iteration_label>
set -e

CONFIG="$1"
LABEL="${2:-test}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Tuning test: $CONFIG ($LABEL) ==="

# Stop everything
pkill -9 -f SfmPedestrianTest 2>/dev/null || true
docker stop simulation_container 2>/dev/null || true
sleep 3

# Start sim
"$SCRIPT_DIR/restart_sim.sh" "$CONFIG" 2>&1 | tail -3

# Disable steering check
docker exec simulation_container bash -c \
    "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null; \
     ros2 param set /control/trajectory_follower/controller_node_exe enable_keep_stopped_until_steer_convergence false" 2>/dev/null
sleep 10

# Record 60s
BAG_NAME="${CONFIG}_tune_${LABEL}"
docker exec -d simulation_container bash -c \
    "source /opt/autoware/setup.bash && source /workspace/install/setup.bash 2>/dev/null && \
     ros2 bag record -o /workspace/rosbags/$BAG_NAME \
     /vehicle/status/velocity_status /localization/kinematic_state /perception/object_recognition/objects /autoware/state"
sleep 65
docker exec simulation_container bash -c "pkill -f 'ros2 bag record'" 2>/dev/null
sleep 2
docker exec simulation_container bash -c "chown -R 1000:1000 /workspace/rosbags/" 2>/dev/null

# Quick metrics
docker exec simulation_container bash -c "source /opt/autoware/setup.bash && python3 -c \"
import sqlite3, struct, os
bag = '/workspace/rosbags/$BAG_NAME'
db = None
for f in os.listdir(bag):
    if f.endswith('.db3'): db = os.path.join(bag, f); break
if not db: print('NO DATA'); exit()
conn = sqlite3.connect(db)
c = conn.cursor()
c.execute('SELECT id, name FROM topics')
topics = {r[1]: r[0] for r in c.fetchall()}
tid = topics.get('/vehicle/status/velocity_status')
if not tid: print('NO VEL TOPIC'); exit()
c.execute('SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,))
vels = []
for ts, blob in c.fetchall():
    try:
        o = 4+8; sl = struct.unpack_from('<I', bytes(blob), o)[0]; o += 4+sl; o=(o+3)&~3
        vels.append(abs(struct.unpack_from('<f', bytes(blob), o)[0]))
    except: pass
conn.close()
# trim
s,cn = 0,0
for i,v in enumerate(vels):
    if v>0.05: cn+=1
    else: cn=0
    if cn>=5: s=i-4; break
vels = vels[s:s+1800]
if not vels: print('NO DATA'); exit()
avg = sum(vels)/len(vels)
std = (sum((v-avg)**2 for v in vels)/len(vels))**0.5
dt = 60.0/max(len(vels)-1,1)
ac = [(vels[i+1]-vels[i])/dt for i in range(len(vels)-1)]
jk = [(ac[i+1]-ac[i])/dt for i in range(len(ac)-1)]
ajk = sum(abs(j) for j in jk)/len(jk) if jk else 0
st = sum(1 for i in range(1,len(vels)) if vels[i]<=0.05 and vels[i-1]>0.05)
print(f'avg={avg:.3f} std={std:.3f} jerk={ajk:.3f} stops={st}')
\"" 2>&1

echo "=== Done: $CONFIG ($LABEL) ==="
