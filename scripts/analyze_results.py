#!/usr/bin/env python3
"""
Crowd-Matching Evaluation Analyzer — Processes rosbags and generates comparison plots/PDF.

Usage (inside container):
    python3 analyze_results.py [--bag-dir /workspace/rosbags] [--output-dir /workspace/rosbags/analysis]
"""

import argparse
import os
import sys
import sqlite3
import struct
from collections import defaultdict

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    from matplotlib.backends.backend_pdf import PdfPages
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("WARNING: matplotlib not available — text output only")

# ── Constants ──────────────────────────────────────────────────────────────────

CONFIG_ORDER = ['stock', 'campus_tuned', 'campus_tuned_classical_sfm',
                'campus_tuned_crowd_only', 'campus_tuned_following_only',
                'campus_tuned_classical_following', 'campus_tuned_sfm',
                'campus_tuned_sfm_pid']

CONFIG_LABELS = {
    'stock': 'A: Stock',
    'campus_tuned': 'B: Campus-Tuned',
    'campus_tuned_classical_sfm': 'C: Classical',
    'campus_tuned_sfm': 'D: Crowd-Match+Prox',
    'campus_tuned_crowd_only': 'E: Crowd-Match Only',
    'campus_tuned_following_only': 'F: Following Only',
    'campus_tuned_classical_following': 'G: Classical+Follow',
    'campus_tuned_sfm_pid': 'H: Crowd-Match+PID',
}

CONFIG_COLORS = {
    'stock': '#e74c3c',
    'campus_tuned': '#f39c12',
    'campus_tuned_classical_sfm': '#9b59b6',
    'campus_tuned_sfm': '#27ae60',
    'campus_tuned_crowd_only': '#1a5276',
    'campus_tuned_following_only': '#e67e22',
    'campus_tuned_classical_following': '#e91e90',
    'campus_tuned_sfm_pid': '#00bcd4',
}

SPEED_LIMIT = 1.39  # m/s (5 km/h)


# ── Data Reading ───────────────────────────────────────────────────────────────

def read_rosbag_sqlite(bag_path):
    """Read a ROS2 rosbag (SQLite format) and extract velocity data."""
    db_path = None
    if os.path.isdir(bag_path):
        for f in os.listdir(bag_path):
            if f.endswith('.db3'):
                db_path = os.path.join(bag_path, f)
                break
    elif bag_path.endswith('.db3'):
        db_path = bag_path

    if db_path is None or not os.path.exists(db_path):
        print(f"  WARNING: No .db3 file found in {bag_path}")
        return None

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: {'id': row[0], 'type': row[2]} for row in cursor.fetchall()}

    data = {'velocity': [], 'ped_count': []}

    vel_topic = '/vehicle/status/velocity_status'
    if vel_topic in topics:
        topic_id = topics[vel_topic]['id']
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,))
        for timestamp, blob in cursor.fetchall():
            t_sec = timestamp / 1e9
            blob_bytes = bytes(blob)
            try:
                offset = 4
                offset += 8  # stamp
                str_len = struct.unpack_from('<I', blob_bytes, offset)[0]
                offset += 4 + str_len
                offset = (offset + 3) & ~3
                vel = struct.unpack_from('<f', blob_bytes, offset)[0]
                data['velocity'].append((t_sec, abs(vel)))
            except Exception:
                pass

    obj_topic = '/perception/object_recognition/objects'
    if obj_topic in topics:
        topic_id = topics[obj_topic]['id']
        cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 500",
            (topic_id,))
        for timestamp, blob in cursor.fetchall():
            t_sec = timestamp / 1e9
            blob_bytes = bytes(blob)
            try:
                offset = 4
                offset += 8
                str_len = struct.unpack_from('<I', blob_bytes, offset)[0]
                offset += 4 + str_len
                offset = (offset + 3) & ~3
                count = struct.unpack_from('<I', blob_bytes, offset)[0]
                if 0 <= count <= 50:
                    data['ped_count'].append((t_sec, count))
            except Exception:
                pass

    conn.close()
    return data


def extract_clearance_ttc(bag_path):
    """Extract pedestrian clearance and TTC from rosbag using rclpy deserialization."""
    try:
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Odometry
        from autoware_perception_msgs.msg import PredictedObjects
    except ImportError:
        return {}

    db_path = None
    if os.path.isdir(bag_path):
        for f in os.listdir(bag_path):
            if f.endswith('.db3'):
                db_path = os.path.join(bag_path, f)
                break
    if not db_path:
        return {}

    import math
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: {'id': row[0], 'type': row[2]} for row in cursor.fetchall()}

    # Read ego poses
    ego_data = {}  # timestamp -> (x, y, vx)
    kin_topic = '/localization/kinematic_state'
    if kin_topic in topics:
        tid = topics[kin_topic]['id']
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
        for ts, blob in cursor.fetchall():
            try:
                msg = deserialize_message(bytes(blob), Odometry)
                t = ts / 1e9
                ego_data[t] = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.twist.twist.linear.x
                )
            except:
                pass

    # Read pedestrian detections and compute clearance + TTC
    clearances = []
    ttcs = []
    obj_topic = '/perception/object_recognition/objects'
    if obj_topic in topics:
        tid = topics[obj_topic]['id']
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,))
        for ts, blob in cursor.fetchall():
            try:
                msg = deserialize_message(bytes(blob), PredictedObjects)
                t = ts / 1e9
                # Find closest ego pose in time
                ego_t = min(ego_data.keys(), key=lambda et: abs(et - t)) if ego_data else None
                if ego_t is None or abs(ego_t - t) > 0.5:
                    continue
                ex, ey, ev = ego_data[ego_t]

                for obj in msg.objects:
                    px = obj.kinematics.initial_pose_with_covariance.pose.position.x
                    py = obj.kinematics.initial_pose_with_covariance.pose.position.y
                    pvx = obj.kinematics.initial_twist_with_covariance.twist.linear.x
                    pvy = obj.kinematics.initial_twist_with_covariance.twist.linear.y

                    dist = math.hypot(px - ex, py - ey)
                    clearances.append(dist)

                    # TTC: closing speed along the line between ego and ped
                    if dist > 0.1:
                        dx = px - ex
                        dy = py - ey
                        nx = dx / dist
                        ny = dy / dist
                        # Ego velocity projected toward ped (assume ego moves in +x)
                        ego_toward = ev * nx  # simplified: ego forward ≈ +x
                        ped_toward = -(pvx * nx + pvy * ny)  # ped velocity toward ego
                        closing_speed = ego_toward + ped_toward
                        if closing_speed > 0.05:
                            ttc = dist / closing_speed
                            if ttc < 60.0:  # cap at 60s
                                ttcs.append(ttc)
            except:
                pass

    conn.close()

    result = {}
    if clearances:
        result['min_clearance'] = min(clearances)
        result['mean_clearance'] = sum(clearances) / len(clearances)
    if ttcs:
        result['min_ttc'] = min(ttcs)
        result['mean_ttc'] = sum(ttcs) / len(ttcs)
    return result


def trim_to_first_movement(data, threshold=0.05, max_duration=60.0):
    """Trim data to start from first sustained movement (removes loading time)."""
    if not data or not data['velocity']:
        return data
    vels = data['velocity']
    start_idx = 0
    consecutive = 0
    for i, (t, v) in enumerate(vels):
        if v > threshold:
            consecutive += 1
            if consecutive >= 5:
                start_idx = i - 4
                break
        else:
            consecutive = 0
    if start_idx > 0:
        t_start = vels[start_idx][0]
        data['velocity'] = vels[start_idx:]
        data['ped_count'] = [(t, c) for t, c in data.get('ped_count', []) if t >= t_start]

    # Cap at max_duration from first movement
    if data['velocity']:
        t0 = data['velocity'][0][0]
        data['velocity'] = [(t, v) for t, v in data['velocity'] if t - t0 <= max_duration]
        data['ped_count'] = [(t, c) for t, c in data.get('ped_count', []) if t - t0 <= max_duration]

    return data


def read_sfm_csv(path='/tmp/cm_target.csv'):
    """Read crowd-matching target CSV log."""
    try:
        with open(path, 'r') as f:
            lines = f.readlines()[1:]
        times, targets, raws = [], [], []
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) >= 3:
                times.append(float(parts[0]))
                targets.append(float(parts[1]))
                raws.append(float(parts[2]))
        return times, targets, raws
    except FileNotFoundError:
        return None, None, None


def align_sfm_to_velocity(sfm_times, vel_data):
    """Align crowd-matching CSV timestamps to velocity data's time origin.
    Both use ROS clock — find the offset by matching the time ranges."""
    if not sfm_times or not vel_data:
        return sfm_times
    vel_t0 = vel_data[0][0]  # absolute ROS time of first velocity sample
    sfm_t0 = sfm_times[0]
    # Both are absolute ROS times — use the velocity t0 as the common origin
    return [t - vel_t0 for t in sfm_times]


# ── Metrics ────────────────────────────────────────────────────────────────────

def compute_metrics(data):
    """Compute evaluation metrics from extracted data."""
    if not data or not data['velocity']:
        return None
    velocities = [v for _, v in data['velocity']]
    times = [t for t, _ in data['velocity']]
    if not velocities:
        return None

    duration = times[-1] - times[0]
    avg_vel = sum(velocities) / len(velocities)
    max_vel = max(velocities)
    std_vel = (sum((v - avg_vel) ** 2 for v in velocities) / len(velocities)) ** 0.5

    stop_threshold = 0.05
    stops = 0
    was_moving = False
    for v in velocities:
        if v > stop_threshold:
            was_moving = True
        elif was_moving and v <= stop_threshold:
            stops += 1
            was_moving = False

    dt_sample = duration / max(len(times) - 1, 1)
    time_stopped = sum(dt_sample for v in velocities if v <= stop_threshold)
    pct_stopped = (time_stopped / duration * 100) if duration > 0 else 0

    # Acceleration and jerk
    accels = []
    jerks = []
    if len(velocities) > 2 and dt_sample > 0:
        accels = [(velocities[i+1] - velocities[i]) / dt_sample for i in range(len(velocities) - 1)]
        jerks = [(accels[i+1] - accels[i]) / dt_sample for i in range(len(accels) - 1)]
    avg_jerk = (sum(abs(j) for j in jerks) / len(jerks)) if jerks else 0
    max_jerk = max(abs(j) for j in jerks) if jerks else 0

    # Energy proxy: integral of |acceleration| over time
    energy_proxy = sum(abs(a) * dt_sample for a in accels) if accels else 0

    # Deceleration profile area: integral of negative velocity changes
    decel_area = 0.0
    if len(velocities) > 1:
        for i in range(len(velocities) - 1):
            dv = velocities[i+1] - velocities[i]
            if dv < 0:
                decel_area += abs(dv) * dt_sample

    distance = sum(v * dt_sample for v in velocities) if len(velocities) > 1 else 0

    ped_counts = [c for _, c in data.get('ped_count', [])]
    avg_peds = (sum(ped_counts) / len(ped_counts)) if ped_counts else 0

    # Clearance and TTC from extended data (if available)
    min_clearance = data.get('min_clearance', float('inf'))
    mean_clearance = data.get('mean_clearance', 0.0)
    min_ttc = data.get('min_ttc', float('inf'))
    mean_ttc = data.get('mean_ttc', 0.0)

    return {
        'duration': duration, 'avg_velocity': avg_vel, 'max_velocity': max_vel,
        'std_velocity': std_vel, 'stop_count': stops, 'time_stopped': time_stopped,
        'pct_stopped': pct_stopped, 'avg_jerk': avg_jerk, 'max_jerk': max_jerk,
        'energy_proxy': energy_proxy, 'decel_area': decel_area,
        'distance': distance, 'avg_ped_count': avg_peds, 'n_samples': len(velocities),
        'min_clearance': min_clearance if min_clearance != float('inf') else 0.0,
        'mean_clearance': mean_clearance,
        'min_ttc': min_ttc if min_ttc != float('inf') else 0.0,
        'mean_ttc': mean_ttc,
    }


# ── Plotting Helpers ───────────────────────────────────────────────────────────

def _style_ax(ax, title, ylabel, xlabel='Time (s)', ylim=(-0.1, 2.0), grid=True):
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_xlabel(xlabel, fontsize=10)
    if ylim:
        ax.set_ylim(ylim)
    if grid:
        ax.grid(True, alpha=0.3)


def _add_speed_limit(ax):
    ax.axhline(y=SPEED_LIMIT, color='black', linestyle='--', alpha=0.3, label='5 km/h limit')


def _bar_labels(ax, bars, vals, fmt='.2f'):
    for bar, val in zip(bars, vals):
        y_offset = max(vals) * 0.05 if max(vals) > 0 else 0.01
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + y_offset,
                f'{val:{fmt}}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    # Ensure y-axis has headroom for labels
    if vals:
        ax.set_ylim(top=max(vals) * 1.25)


def _get_vel_time_series(all_data, config_name):
    """Get time-zeroed velocity series for a config."""
    if config_name not in all_data:
        return [], []
    for run_data in all_data[config_name]:
        if not run_data['velocity']:
            continue
        t0 = run_data['velocity'][0][0]
        times = [t - t0 for t, _ in run_data['velocity']]
        vels = [v for _, v in run_data['velocity']]
        return times, vels
    return [], []


def _get_max_time(all_data):
    """Get the maximum time across all configs for consistent x-axis, capped at 60s."""
    max_t = 0
    for config_name in CONFIG_ORDER:
        times, _ = _get_vel_time_series(all_data, config_name)
        if times:
            max_t = max(max_t, times[-1])
    return min(max_t, 60.0)


# ── Main Plot (PNG) ────────────────────────────────────────────────────────────

def plot_comparison(all_data, all_metrics, output_dir):
    if not HAS_MATPLOTLIB:
        return

    configs = [c for c in CONFIG_ORDER if c in all_metrics]
    colors = [CONFIG_COLORS.get(c, 'gray') for c in configs]
    bar_labels = [CONFIG_LABELS.get(c, c).split(': ')[-1] for c in configs]
    max_time = _get_max_time(all_data)

    fig = plt.figure(figsize=(14, 16))
    gs = gridspec.GridSpec(4, 2, hspace=0.45, wspace=0.35, height_ratios=[1.2, 1, 1, 1])

    # Row 1: Vehicle Velocity Comparison
    ax1 = fig.add_subplot(gs[0, :])
    for cn in CONFIG_ORDER:
        times, vels = _get_vel_time_series(all_data, cn)
        if times:
            ax1.plot(times, vels, color=CONFIG_COLORS.get(cn, 'gray'),
                     label=CONFIG_LABELS.get(cn, cn), alpha=0.8, linewidth=1.2)
    _add_speed_limit(ax1)
    _style_ax(ax1, 'Measured Vehicle Velocity', 'Velocity (m/s)')
    ax1.set_xlim(0, max_time)
    ax1.legend(loc='upper right', fontsize=8, ncol=2)

    # Row 2-3: Bar charts
    chart_specs = [
        (gs[1, 0], 'Average Velocity', 'avg_velocity', 'm/s', '.2f'),
        (gs[1, 1], 'Velocity StdDev (lower = smoother)', 'std_velocity', 'm/s', '.3f'),
        (gs[2, 0], 'Average |Jerk| (lower = smoother)', 'avg_jerk', 'm/s\u00b3', '.2f'),
        (gs[2, 1], 'Energy Proxy (lower = efficient)', 'energy_proxy', 'm/s', '.1f'),
        (gs[3, 0], 'Min Clearance (higher = safer)', 'min_clearance', 'm', '.1f'),
        (gs[3, 1], 'Min TTC (higher = safer)', 'min_ttc', 's', '.1f'),
    ]
    for gs_pos, title, key, ylabel, fmt in chart_specs:
        ax = fig.add_subplot(gs_pos)
        vals = [all_metrics[c][key] for c in configs]
        bars = ax.bar(bar_labels, vals, color=colors, alpha=0.85, edgecolor='white', linewidth=0.5)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_title(title, fontsize=11, fontweight='bold')
        if key == 'avg_velocity':
            ax.axhline(y=SPEED_LIMIT, color='black', linestyle='--', alpha=0.3)
        _bar_labels(ax, bars, vals, fmt)
        ax.grid(True, axis='y', alpha=0.2)
        ax.tick_params(axis='x', labelrotation=30, labelsize=8)
        plt.setp(ax.get_xticklabels(), ha='right')

    fig.suptitle('Crowd-Matching Velocity Module — Configuration Comparison',
                 fontsize=15, fontweight='bold', y=0.99)
    plt.savefig(os.path.join(output_dir, 'cm_comparison.png'), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  Plot saved: {output_dir}/cm_comparison.png")


# ── PDF Report ─────────────────────────────────────────────────────────────────

def generate_pdf(all_data, all_metrics, output_dir, subtitle='Straight-line campus test with bi-directional pedestrian traffic'):
    if not HAS_MATPLOTLIB:
        return

    configs = [c for c in CONFIG_ORDER if c in all_metrics]
    colors = [CONFIG_COLORS.get(c, 'gray') for c in configs]
    bar_labels_short = [CONFIG_LABELS.get(c, c).split(': ')[-1] for c in configs]
    max_time = _get_max_time(all_data)

    pdf_path = os.path.join(output_dir, 'cm_evaluation_report.pdf')
    with PdfPages(pdf_path) as pdf:

        # ── Page 1: Title + Summary Table ──
        fig = plt.figure(figsize=(11, 8.5))
        fig.text(0.5, 0.93, 'Crowd-Matching Velocity Module — Evaluation Report',
                 ha='center', fontsize=20, fontweight='bold')
        fig.text(0.5, 0.88, 'nUWAy Campus Shuttle — Shared Pathway Pedestrian Negotiation',
                 ha='center', fontsize=13, color='#555555')
        fig.text(0.5, 0.84, subtitle,
                 ha='center', fontsize=11, color='#888888')

        col_labels = [CONFIG_LABELS.get(c, c) for c in configs]
        row_data = [
            ('Avg Velocity (m/s)',      'avg_velocity',  '.3f', 'max'),
            ('Max Velocity (m/s)',      'max_velocity',  '.3f', 'max'),
            ('Velocity StdDev (m/s)',   'std_velocity',  '.3f', 'min'),
            ('Stop Count',             'stop_count',    'd',   'min'),
            ('Time Stopped (%)',       'pct_stopped',   '.1f', 'min'),
            ('Avg |Jerk| (m/s\u00b3)', 'avg_jerk',     '.3f', 'min'),
            ('Distance (m)',           'distance',      '.1f', 'max'),
            ('Duration (s)',           'duration',      '.1f', None),
        ]

        cell_text = []
        for label, key, fmt, _ in row_data:
            cell_text.append([f'{all_metrics[c].get(key, 0):{fmt}}' for c in configs])

        ax_table = fig.add_axes([0.05, 0.12, 0.90, 0.65])
        ax_table.axis('off')
        table = ax_table.table(
            cellText=cell_text,
            rowLabels=[r[0] for r in row_data],
            colLabels=col_labels,
            cellLoc='center', rowLoc='right', loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.auto_set_column_width(list(range(len(configs))))
        table.scale(1.0, 2.0)

        for j, c in enumerate(configs):
            table[0, j].set_facecolor(CONFIG_COLORS.get(c, 'white'))
            table[0, j].set_text_props(color='white', fontweight='bold', fontsize=11)

        for i, (_, key, _, best) in enumerate(row_data):
            if best is None:
                continue
            vals = [all_metrics[c].get(key, 0) for c in configs]
            best_idx = vals.index(min(vals) if best == 'min' else max(vals))
            table[i + 1, best_idx].set_text_props(fontweight='bold')
            table[i + 1, best_idx].set_facecolor('#f0f8f0')

        pdf.savefig(fig, bbox_inches='tight')
        plt.close()

        # ── Page 2: Velocity Time-Series ──
        fig, ax_vel = plt.subplots(1, 1, figsize=(11, 5))

        for cn in CONFIG_ORDER:
            times, vels = _get_vel_time_series(all_data, cn)
            if times:
                ax_vel.plot(times, vels, color=CONFIG_COLORS.get(cn, 'gray'),
                            label=CONFIG_LABELS.get(cn, cn), alpha=0.8, linewidth=1.2)
        _add_speed_limit(ax_vel)
        _style_ax(ax_vel, 'Measured Vehicle Velocity', 'Velocity (m/s)')
        ax_vel.set_xlim(0, max_time)
        ax_vel.legend(loc='upper right', fontsize=8, ncol=2)

        fig.suptitle('Velocity Profiles', fontsize=14, fontweight='bold', y=0.98)
        plt.tight_layout()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close()

        # ── Page 3: Bar Charts ──
        fig = plt.figure(figsize=(11, 8.5))
        gs2 = gridspec.GridSpec(2, 2, hspace=0.4, wspace=0.35)

        chart_specs = [
            (gs2[0, 0], 'Average Velocity', 'avg_velocity', 'm/s', '.2f'),
            (gs2[0, 1], 'Number of Stops', 'stop_count', 'Count', 'd'),
            (gs2[1, 0], 'Velocity StdDev (lower = smoother)', 'std_velocity', 'm/s', '.3f'),
            (gs2[1, 1], 'Average |Jerk| (lower = smoother)', 'avg_jerk', 'm/s\u00b3', '.2f'),
        ]
        for gs_pos, title, key, ylabel, fmt in chart_specs:
            ax = fig.add_subplot(gs_pos)
            vals = [all_metrics[c][key] for c in configs]
            bars = ax.bar(bar_labels_short, vals, color=colors, alpha=0.85,
                          edgecolor='white', linewidth=0.5)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.set_title(title, fontsize=11, fontweight='bold')
            if key == 'avg_velocity':
                ax.axhline(y=SPEED_LIMIT, color='black', linestyle='--', alpha=0.3)
            _bar_labels(ax, bars, vals, fmt)
            ax.grid(True, axis='y', alpha=0.2)
            ax.tick_params(axis='x', labelrotation=30, labelsize=8)
            plt.setp(ax.get_xticklabels(), ha='right')

        fig.suptitle('Metric Comparison', fontsize=14, fontweight='bold', y=0.98)
        pdf.savefig(fig, bbox_inches='tight')
        plt.close()

        # ── Page 4: Per-Config Velocity Profiles ──
        fig, axes = plt.subplots(len(configs), 1, figsize=(11, 3.2 * len(configs)),
                                  sharex=True)
        if len(configs) == 1:
            axes = [axes]
        for i, cn in enumerate(configs):
            ax = axes[i]
            times, vels = _get_vel_time_series(all_data, cn)
            if times:
                ax.plot(times, vels, color=CONFIG_COLORS.get(cn, 'gray'), linewidth=1.0)
                ax.fill_between(times, vels, alpha=0.12, color=CONFIG_COLORS.get(cn, 'gray'))
            m = all_metrics.get(cn, {})
            ax.set_title(
                "%s  |  avg=%.2f m/s  std=%.3f  stops=%d  jerk=%.2f m/s\u00b3" % (
                    CONFIG_LABELS.get(cn, cn), m.get('avg_velocity', 0),
                    m.get('std_velocity', 0), m.get('stop_count', 0), m.get('avg_jerk', 0)),
                fontsize=10, fontweight='bold')
            _add_speed_limit(ax)
            ax.set_ylim(-0.1, 2.0)
            ax.set_xlim(0, max_time)
            ax.set_ylabel('m/s', fontsize=10)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)', fontsize=10)
        fig.suptitle('Per-Configuration Velocity Profiles', fontsize=14, fontweight='bold')
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        pdf.savefig(fig, bbox_inches='tight')
        plt.close()

    print(f"  PDF report saved: {pdf_path}")


# ── Text Table ─────────────────────────────────────────────────────────────────

def print_table(all_metrics):
    labels = {'stock': 'A: Stock', 'campus_tuned': 'B: Campus-Tuned',
              'campus_tuned_sfm': 'C: Crowd-Match'}
    ordered = [c for c in CONFIG_ORDER if c in all_metrics]
    header = "%-25s" % 'Metric' + ''.join(" %20s" % labels.get(c, c) for c in ordered)
    print(header)
    print("-" * len(header))
    rows = [
        ('Avg Velocity (m/s)', 'avg_velocity', '.3f'),
        ('Max Velocity (m/s)', 'max_velocity', '.3f'),
        ('Velocity StdDev (m/s)', 'std_velocity', '.3f'),
        ('Stop Count', 'stop_count', 'd'),
        ('Time Stopped (%)', 'pct_stopped', '.1f'),
        ('Avg |Jerk| (m/s\u00b3)', 'avg_jerk', '.3f'),
        ('Max |Jerk| (m/s\u00b3)', 'max_jerk', '.3f'),
        ('Energy Proxy (m/s)', 'energy_proxy', '.2f'),
        ('Decel Area (m\u00b2/s)', 'decel_area', '.3f'),
        ('Min Clearance (m)', 'min_clearance', '.2f'),
        ('Mean Clearance (m)', 'mean_clearance', '.2f'),
        ('Min TTC (s)', 'min_ttc', '.2f'),
        ('Mean TTC (s)', 'mean_ttc', '.2f'),
        ('Distance (m)', 'distance', '.1f'),
        ('Duration (s)', 'duration', '.1f'),
        ('Avg Ped Count', 'avg_ped_count', '.1f'),
    ]
    for label, key, fmt in rows:
        row = "%-25s" % label
        for c in ordered:
            row += " %20s" % (("%s" % (("%" + fmt) % all_metrics[c].get(key, 0))))
        print(row)


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Analyze crowd-matching evaluation rosbags')
    parser.add_argument('--bag-dir', default='/workspace/rosbags')
    parser.add_argument('--output-dir', default=None)
    parser.add_argument('--duration', type=float, default=60.0, help='Max analysis duration in seconds')
    parser.add_argument('--subtitle', default='Straight-line campus test with bi-directional pedestrian traffic',
                        help='Report subtitle describing the test scenario')
    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = os.path.join(args.bag_dir, 'analysis')
    os.makedirs(args.output_dir, exist_ok=True)

    print("Scanning for rosbags...")
    all_data = defaultdict(list)

    if not os.path.exists(args.bag_dir):
        print(f"ERROR: Bag directory not found: {args.bag_dir}")
        sys.exit(1)

    for entry in sorted(os.listdir(args.bag_dir)):
        entry_path = os.path.join(args.bag_dir, entry)
        if not os.path.isdir(entry_path) or entry == 'analysis':
            continue
        config = None
        for c in ['campus_tuned_classical_following', 'campus_tuned_classical_sfm',
                  'campus_tuned_crowd_only', 'campus_tuned_following_only',
                  'campus_tuned_sfm_pid', 'campus_tuned_sfm', 'campus_tuned', 'stock']:
            if entry.startswith(c):
                config = c
                break
        if config is None:
            continue
        print(f"  Reading {entry} (config: {config})...")
        data = read_rosbag_sqlite(entry_path)
        if data and data['velocity']:
            data = trim_to_first_movement(data, max_duration=args.duration)
            # Extract clearance and TTC metrics
            extended = extract_clearance_ttc(entry_path)
            data.update(extended)
            all_data[config].append(data)
            extra = ""
            if 'min_clearance' in extended:
                extra += f", clearance={extended['min_clearance']:.1f}m"
            if 'min_ttc' in extended:
                extra += f", TTC={extended['min_ttc']:.1f}s"
            print(f"    {len(data['velocity'])} velocity samples (after trimming){extra}")
        else:
            print(f"    WARNING: No velocity data extracted")

    if not all_data:
        print("\nNo data found.")
        sys.exit(1)

    print("\n" + "=" * 70)
    print("  EVALUATION RESULTS")
    print("=" * 70 + "\n")

    all_metrics = {}
    for config_name, runs in all_data.items():
        m = compute_metrics(runs[-1])
        if m:
            all_metrics[config_name] = m

    print_table(all_metrics)

    print("\nGenerating plots...")
    plot_comparison(all_data, all_metrics, args.output_dir)
    print("Generating PDF report...")
    generate_pdf(all_data, all_metrics, args.output_dir, subtitle=args.subtitle)

    csv_path = os.path.join(args.output_dir, 'velocity_data.csv')
    with open(csv_path, 'w') as f:
        f.write('config,time_sec,velocity_mps\n')
        for config, runs in all_data.items():
            for run_data in runs:
                t0 = run_data['velocity'][0][0] if run_data['velocity'] else 0
                for t, v in run_data['velocity']:
                    f.write(f'{config},{t - t0:.3f},{v:.4f}\n')
    print(f"  CSV exported: {csv_path}")
    print("\nDone!")


if __name__ == '__main__':
    main()
