#!/usr/bin/env python3
"""
Analyze density sweep results — runs analysis on each density bag
and produces a summary comparison table + degradation curve plot.
"""
import os
import sys
import json
import subprocess
import csv

def main():
    sweep_dir = sys.argv[1] if len(sys.argv) > 1 else "/workspace/rosbags/density_sweep_20260329"
    output_dir = os.path.join(sweep_dir, "analysis")
    os.makedirs(output_dir, exist_ok=True)

    # Find density bags
    bags = sorted([
        d for d in os.listdir(sweep_dir)
        if os.path.isdir(os.path.join(sweep_dir, d)) and d.startswith("density_")
    ])

    if not bags:
        print(f"No density_* bags found in {sweep_dir}")
        sys.exit(1)

    print(f"Found {len(bags)} density bags: {bags}")

    # For each bag, create a temp dir with the bag renamed to campus_tuned_sfm_*
    # so analyze_results.py can parse it
    script_dir = os.path.dirname(os.path.abspath(__file__))
    analyze_script = os.path.join(script_dir, "analyze_results.py")

    results = []

    for bag in bags:
        # Extract density from name (density_5_*, density_10_*, etc.)
        parts = bag.split("_")
        density = int(parts[1])

        bag_path = os.path.join(sweep_dir, bag)

        # Create a temp analysis dir with a symlink named campus_tuned_sfm_density
        tmp_dir = os.path.join(sweep_dir, f"_tmp_analysis_{density}")
        os.makedirs(tmp_dir, exist_ok=True)
        link_name = os.path.join(tmp_dir, f"campus_tuned_sfm_{density}")
        if os.path.exists(link_name):
            os.remove(link_name)
        os.symlink(bag_path, link_name)

        # Run analysis
        tmp_output = os.path.join(tmp_dir, "analysis")
        print(f"\n{'='*50}")
        print(f"  Analyzing density={density} pedestrians")
        print(f"{'='*50}")

        result = subprocess.run(
            ["python3", analyze_script, "--bag-dir", tmp_dir, "--output-dir", tmp_output],
            capture_output=True, text=True
        )
        print(result.stdout)
        if result.stderr:
            print(result.stderr)

        # Parse metrics from stdout (analyze_results.py prints a table)
        metrics = {'density': density}
        for line in result.stdout.split('\n'):
            line = line.strip()
            if not line:
                continue
            # Parse lines like "Avg Velocity (m/s)                       0.901"
            metric_map = {
                'Avg Velocity': 'avg_velocity',
                'Velocity StdDev': 'velocity_std',
                'Stop Count': 'stop_count',
                'Time Stopped': 'time_stopped',
                'Avg |Jerk|': 'avg_jerk',
                'Max |Jerk|': 'max_jerk',
                'Min Clearance': 'min_clearance',
                'Min TTC': 'min_ttc',
                'Distance': 'distance',
                'Avg Ped Count': 'avg_ped_count',
            }
            for label, key in metric_map.items():
                if line.startswith(label):
                    try:
                        val = line.split()[-1]
                        metrics[key] = float(val)
                    except (ValueError, IndexError):
                        pass
        if len(metrics) > 1:
            results.append(metrics)

        # Cleanup temp dir
        os.remove(link_name)
        if os.path.exists(tmp_output):
            import shutil
            shutil.rmtree(tmp_output)
        os.rmdir(tmp_dir)

    # Write combined results
    if results:
        # Sort by density
        results.sort(key=lambda r: r['density'])

        print(f"\n{'='*60}")
        print(f"  DENSITY SWEEP SUMMARY")
        print(f"{'='*60}")
        print(f"{'Peds':>6} {'Avg Vel':>10} {'StdDev':>10} {'Avg Jerk':>10} {'Stops':>8} {'Min TTC':>10} {'Min Clear':>10}")
        print("-" * 70)

        for r in results:
            def fmt(key, decimals=3):
                v = r.get(key)
                if v is None: return 'N/A'
                return f"{v:.{decimals}f}"
            print(f"{int(r['density']):>6} "
                  f"{fmt('avg_velocity'):>10} "
                  f"{fmt('velocity_std'):>10} "
                  f"{fmt('avg_jerk'):>10} "
                  f"{fmt('stop_count', 0):>8} "
                  f"{fmt('min_ttc'):>10} "
                  f"{fmt('min_clearance'):>10}")

        # Save combined CSV
        combined_csv = os.path.join(output_dir, "density_sweep_results.csv")
        with open(combined_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=results[0].keys())
            writer.writeheader()
            writer.writerows(results)
        print(f"\nSaved: {combined_csv}")

    # Try to create a degradation curve plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np

        densities = [r['density'] for r in results]
        avg_vels = [float(r.get('avg_velocity', 0)) for r in results]
        std_devs = [float(r.get('velocity_std', 0)) for r in results]
        avg_jerks = [float(r.get('avg_jerk', 0)) for r in results]

        fig, axes = plt.subplots(1, 3, figsize=(14, 4))

        axes[0].plot(densities, avg_vels, 'o-', color='#2196F3', linewidth=2, markersize=8)
        axes[0].set_xlabel('Pedestrian Count')
        axes[0].set_ylabel('Average Velocity (m/s)')
        axes[0].set_title('Velocity vs Density')
        axes[0].axhline(y=1.39, color='gray', linestyle='--', alpha=0.5, label='Speed limit (1.39 m/s)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(densities, std_devs, 's-', color='#FF9800', linewidth=2, markersize=8)
        axes[1].set_xlabel('Pedestrian Count')
        axes[1].set_ylabel('Velocity Std Dev (m/s)')
        axes[1].set_title('Smoothness vs Density')
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(densities, avg_jerks, '^-', color='#F44336', linewidth=2, markersize=8)
        axes[2].set_xlabel('Pedestrian Count')
        axes[2].set_ylabel('Average |Jerk| (m/s³)')
        axes[2].set_title('Comfort vs Density')
        axes[2].axhline(y=2.6, color='red', linestyle='--', alpha=0.5, label='Comfort threshold (2.6 m/s³)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plot_path = os.path.join(output_dir, "density_sweep_degradation.png")
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot: {plot_path}")

    except ImportError:
        print("matplotlib not available — skipping plot")

if __name__ == "__main__":
    main()
