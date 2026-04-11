#!/usr/bin/env python3
"""Generate full 3-page report with all metrics for thesis."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os, sqlite3, struct, math

CONFIG_ORDER = ["stock", "campus_tuned", "campus_tuned_classical_sfm",
                "campus_tuned_crowd_only", "campus_tuned_following_only",
                "campus_tuned_classical_following", "campus_tuned_sfm"]

CONFIG_LABELS = {
    "stock": "A: Stock", "campus_tuned": "B: Tuned",
    "campus_tuned_classical_sfm": "C: Classical",
    "campus_tuned_sfm": "D: Crowd+Follow",
    "campus_tuned_crowd_only": "E: Crowd Only",
    "campus_tuned_following_only": "F: Follow Only",
    "campus_tuned_classical_following": "G: Class+Follow",
}

CONFIG_COLORS = {
    "stock": "#e74c3c", "campus_tuned": "#f39c12",
    "campus_tuned_classical_sfm": "#9b59b6", "campus_tuned_sfm": "#27ae60",
    "campus_tuned_crowd_only": "#3498db", "campus_tuned_following_only": "#e67e22",
    "campus_tuned_classical_following": "#1abc9c",
}

def read_velocity(bag_path):
    db_path = None
    for f in os.listdir(bag_path):
        if f.endswith(".db3"): db_path = os.path.join(bag_path, f); break
    if not db_path: return []
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT id, name FROM topics")
    topics = {r[1]: r[0] for r in c.fetchall()}
    tid = topics.get("/vehicle/status/velocity_status")
    if not tid: conn.close(); return []
    c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
    data = []
    for ts, blob in c.fetchall():
        try:
            o = 4+8; sl = struct.unpack_from("<I", bytes(blob), o)[0]; o += 4+sl; o=(o+3)&~3
            data.append((ts/1e9, abs(struct.unpack_from("<f", bytes(blob), o)[0])))
        except: pass
    conn.close()
    s, cn = 0, 0
    for i, (t, v) in enumerate(data):
        if v > 0.05: cn += 1
        else: cn = 0
        if cn >= 5: s = i-4; break
    if s > 0: data = data[s:]
    if data:
        t0 = data[0][0]
        data = [(t, v) for t, v in data if t-t0 <= 60.0]
    return data

def extract_clearance_ttc(bag_path):
    try:
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
        from autoware_perception_msgs.msg import PredictedObjects
    except: return {}
    db_path = None
    for f in os.listdir(bag_path):
        if f.endswith(".db3"): db_path = os.path.join(bag_path, f); break
    if not db_path: return {}
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    c.execute("SELECT id, name, type FROM topics")
    topics = {r[1]: {"id": r[0]} for r in c.fetchall()}
    ego_data = {}
    if "/localization/kinematic_state" in topics:
        tid = topics["/localization/kinematic_state"]["id"]
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
        for ts, blob in c.fetchall():
            try:
                msg = deserialize_message(bytes(blob), Odometry)
                ego_data[ts/1e9] = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)
            except: pass
    clearances, ttcs = [], []
    if "/perception/object_recognition/objects" in topics:
        tid = topics["/perception/object_recognition/objects"]["id"]
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
        for ts, blob in c.fetchall():
            try:
                msg = deserialize_message(bytes(blob), PredictedObjects)
                t = ts/1e9
                ego_t = min(ego_data.keys(), key=lambda et: abs(et-t)) if ego_data else None
                if not ego_t or abs(ego_t-t) > 0.5: continue
                ex, ey, ev = ego_data[ego_t]
                for obj in msg.objects:
                    px = obj.kinematics.initial_pose_with_covariance.pose.position.x
                    py = obj.kinematics.initial_pose_with_covariance.pose.position.y
                    pvx = obj.kinematics.initial_twist_with_covariance.twist.linear.x
                    pvy = obj.kinematics.initial_twist_with_covariance.twist.linear.y
                    dist = math.hypot(px-ex, py-ey)
                    clearances.append(dist)
                    if dist > 0.1:
                        nx, ny = (px-ex)/dist, (py-ey)/dist
                        closing = ev*nx - (pvx*nx + pvy*ny)
                        if closing > 0.05:
                            ttc = dist/closing
                            if ttc < 60: ttcs.append(ttc)
            except: pass
    conn.close()
    r = {}
    if clearances: r["min_clearance"] = min(clearances); r["mean_clearance"] = sum(clearances)/len(clearances)
    if ttcs: r["min_ttc"] = min(ttcs); r["mean_ttc"] = sum(ttcs)/len(ttcs)
    return r

def compute_all(data, ext):
    vels = [v for _, v in data]
    times = [t for t, _ in data]
    dur = times[-1]-times[0]
    avg = sum(vels)/len(vels)
    mx = max(vels)
    std = (sum((v-avg)**2 for v in vels)/len(vels))**0.5
    dt = dur/max(len(times)-1,1)
    stops = sum(1 for i in range(1,len(vels)) if vels[i]<=0.05 and vels[i-1]>0.05)
    pct_stop = sum(dt for v in vels if v<=0.05)/dur*100 if dur>0 else 0
    accels = [(vels[i+1]-vels[i])/dt for i in range(len(vels)-1)] if dt>0 else []
    jerks = [(accels[i+1]-accels[i])/dt for i in range(len(accels)-1)] if accels else []
    return {
        "avg_vel": avg, "max_vel": mx, "std": std, "stops": stops, "pct_stop": pct_stop,
        "avg_jerk": sum(abs(j) for j in jerks)/len(jerks) if jerks else 0,
        "max_jerk": max(abs(j) for j in jerks) if jerks else 0,
        "energy": sum(abs(a)*dt for a in accels), "decel_area": sum(abs(vels[i+1]-vels[i])*dt for i in range(len(vels)-1) if vels[i+1]<vels[i]),
        "distance": sum(v*dt for v in vels), "duration": dur,
        "min_clear": ext.get("min_clearance", 0), "mean_clear": ext.get("mean_clearance", 0),
        "min_ttc": ext.get("min_ttc", 0), "mean_ttc": ext.get("mean_ttc", 0),
    }

def bar_chart(ax, title, key, ylabel, fmt, configs, results, higher_better=None):
    labels = [CONFIG_LABELS.get(c, c) for c in configs]
    colors = [CONFIG_COLORS.get(c, "#999") for c in configs]
    vals = [results[c]["metrics"][key] for c in configs]
    bars = ax.bar(labels, vals, color=colors, alpha=0.85, edgecolor="white", linewidth=0.5)
    ax.set_ylabel(ylabel, fontsize=9)
    suffix = ""
    if higher_better is True: suffix = " (higher = better)"
    elif higher_better is False: suffix = " (lower = better)"
    ax.set_title(title + suffix, fontsize=10, fontweight="bold")
    for bar, val in zip(bars, vals):
        yo = max(vals)*0.05 if max(vals)>0 else 0.01
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+yo,
                f'{val:{fmt}}', ha="center", va="bottom", fontsize=8, fontweight="bold")
    if vals and max(vals)>0: ax.set_ylim(top=max(vals)*1.25)
    ax.grid(True, axis="y", alpha=0.2)
    ax.tick_params(axis="x", labelsize=7, rotation=15)

# ── Read all bags ──
bag_dir = "/workspace/rosbags"
results = {}
for entry in sorted(os.listdir(bag_dir)):
    path = os.path.join(bag_dir, entry)
    if not os.path.isdir(path) or "archive" in entry or "analysis" in entry or "final_comparison" in entry: continue
    config = None
    for c in ["campus_tuned_classical_following", "campus_tuned_classical_sfm",
              "campus_tuned_crowd_only", "campus_tuned_following_only",
              "campus_tuned_sfm", "campus_tuned", "stock"]:
        if entry.startswith(c): config = c; break
    if not config: continue
    data = read_velocity(path)
    if not data: continue
    print(f"Reading {entry} ({config})...")
    ext = extract_clearance_ttc(path)
    metrics = compute_all(data, ext)
    results[config] = {"data": data, "metrics": metrics}

configs = [c for c in CONFIG_ORDER if c in results]
out_dir = "/workspace/rosbags/analysis_full_metrics"
os.makedirs(out_dir, exist_ok=True)

# ── Page 1: Velocity + Smoothness ──
fig = plt.figure(figsize=(14, 16))
gs = gridspec.GridSpec(4, 2, hspace=0.5, wspace=0.35, height_ratios=[1.3, 1, 1, 1])

ax1 = fig.add_subplot(gs[0, :])
for c in configs:
    d = results[c]["data"]; t0 = d[0][0]
    ax1.plot([t-t0 for t,_ in d], [v for _,v in d], color=CONFIG_COLORS[c],
             label=CONFIG_LABELS[c], alpha=0.8, linewidth=1.2)
ax1.axhline(y=1.39, color="black", linestyle="--", alpha=0.3, label="5 km/h")
ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Velocity (m/s)")
ax1.set_title("Measured Vehicle Velocity", fontsize=12, fontweight="bold")
ax1.set_ylim(-0.1, 2.0); ax1.set_xlim(0, 60)
ax1.legend(loc="upper right", fontsize=7); ax1.grid(True, alpha=0.3)

bar_chart(fig.add_subplot(gs[1,0]), "Average Velocity", "avg_vel", "m/s", ".2f", configs, results)
bar_chart(fig.add_subplot(gs[1,1]), "Velocity StdDev", "std", "m/s", ".3f", configs, results, higher_better=False)
bar_chart(fig.add_subplot(gs[2,0]), "Average |Jerk|", "avg_jerk", "m/s\u00b3", ".2f", configs, results, higher_better=False)
bar_chart(fig.add_subplot(gs[2,1]), "Max |Jerk|", "max_jerk", "m/s\u00b3", ".1f", configs, results, higher_better=False)
bar_chart(fig.add_subplot(gs[3,0]), "Energy Proxy", "energy", "m/s", ".1f", configs, results, higher_better=False)
bar_chart(fig.add_subplot(gs[3,1]), "Deceleration Area", "decel_area", "m\u00b2/s", ".3f", configs, results, higher_better=False)

fig.suptitle("7-Way Comparative Study \u2014 Smoothness & Efficiency Metrics", fontsize=14, fontweight="bold", y=0.99)
plt.savefig(os.path.join(out_dir, "full_report_page1_smoothness.png"), dpi=150, bbox_inches="tight")
plt.close()
print("Page 1 saved")

# ── Page 2: Safety Metrics ──
fig = plt.figure(figsize=(14, 12))
gs2 = gridspec.GridSpec(3, 2, hspace=0.5, wspace=0.35)

bar_chart(fig.add_subplot(gs2[0,0]), "Min Pedestrian Clearance", "min_clear", "m", ".1f", configs, results, higher_better=True)
bar_chart(fig.add_subplot(gs2[0,1]), "Mean Pedestrian Clearance", "mean_clear", "m", ".1f", configs, results, higher_better=True)
bar_chart(fig.add_subplot(gs2[1,0]), "Min Time-to-Collision", "min_ttc", "s", ".1f", configs, results, higher_better=True)
bar_chart(fig.add_subplot(gs2[1,1]), "Mean Time-to-Collision", "mean_ttc", "s", ".1f", configs, results, higher_better=True)
bar_chart(fig.add_subplot(gs2[2,0]), "Stop Count", "stops", "Count", "d", configs, results, higher_better=False)
bar_chart(fig.add_subplot(gs2[2,1]), "Time Stopped", "pct_stop", "%", ".1f", configs, results, higher_better=False)

fig.suptitle("7-Way Comparative Study \u2014 Safety Metrics", fontsize=14, fontweight="bold", y=0.99)
plt.savefig(os.path.join(out_dir, "full_report_page2_safety.png"), dpi=150, bbox_inches="tight")
plt.close()
print("Page 2 saved")

# ── Page 3: Per-config velocity profiles ──
fig, axes = plt.subplots(len(configs), 1, figsize=(14, 2.2*len(configs)), sharex=True)
for i, c in enumerate(configs):
    ax = axes[i]; d = results[c]["data"]; t0 = d[0][0]
    ts = [t-t0 for t,_ in d]; vs = [v for _,v in d]; m = results[c]["metrics"]
    ax.plot(ts, vs, color=CONFIG_COLORS[c], linewidth=1.0)
    ax.fill_between(ts, vs, alpha=0.12, color=CONFIG_COLORS[c])
    ax.axhline(y=1.39, color="black", linestyle="--", alpha=0.3)
    ax.set_ylim(-0.1, 2.0); ax.set_xlim(0, 60); ax.set_ylabel("m/s", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title(f'{CONFIG_LABELS[c]} | avg={m["avg_vel"]:.2f} std={m["std"]:.3f} jerk={m["avg_jerk"]:.2f} energy={m["energy"]:.1f} clearance={m["min_clear"]:.1f}m TTC={m["min_ttc"]:.1f}s',
                 fontsize=8, fontweight="bold")
axes[-1].set_xlabel("Time (s)")
fig.suptitle("Per-Configuration Velocity Profiles with Key Metrics", fontsize=13, fontweight="bold")
plt.tight_layout(rect=[0,0,1,0.97])
plt.savefig(os.path.join(out_dir, "full_report_page3_profiles.png"), dpi=150, bbox_inches="tight")
plt.close()
print("Page 3 saved")
print("All done!")
