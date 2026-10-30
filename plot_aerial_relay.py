#!/usr/bin/env python3
"""
Visualization script for M03 - UAV/HAPS Aerial Network and Relay Model
Reads aerial-relay-traces.csv and generates comprehensive trajectory
and relay selection handover plots.
"""

import sys
import os
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def plot_aerial_traces(csv_file="aerial-relay-traces.csv", output_file="aerial_relay_analysis.png"):
    if not os.path.exists(csv_file):
        print(f"Error: {csv_file} not found. Please run the simulation first.")
        return

    data = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'time_s': float(row['time_s']),
                'node_id': int(row['node_id']),
                'role': row['role'],
                'x_m': float(row['x_m']),
                'y_m': float(row['y_m']),
                'z_m': float(row['z_m']),
                'speed_mps': float(row['speed_mps']),
                'selected_relay': int(row['selected_relay']),
                'available': int(row['available'])
            })

    if not data:
        print("Error: CSV file contains no data.")
        return

    times = sorted(list(set(d['time_s'] for d in data)))
    node_ids = sorted(list(set(d['node_id'] for d in data)))

    # Track active relay over time
    active_relay_by_time = {}
    for d in data:
        active_relay_by_time[d['time_s']] = d['selected_relay']

    # Organize trajectory points by node_id
    trajectories = {nid: {'x': [], 'y': [], 'z': [], 'time': [], 'role': ''} for nid in node_ids}
    for d in data:
        nid = d['node_id']
        trajectories[nid]['x'].append(d['x_m'])
        trajectories[nid]['y'].append(d['y_m'])
        trajectories[nid]['z'].append(d['z_m'])
        trajectories[nid]['time'].append(d['time_s'])
        trajectories[nid]['role'] = d['role']

    # Create figure with 3 subplots: 3-D mobility, top-down coverage, and handovers.
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:purple', 'tab:red']
    fig = plt.figure(figsize=(13, 16))
    ax0 = fig.add_subplot(3, 1, 1, projection='3d')
    ax1 = fig.add_subplot(3, 1, 2)
    ax2 = fig.add_subplot(3, 1, 3)

    # --- Plot 1: 3-D Mobility ---
    ax0.set_title("3-D UAV / HAPS Mobility", fontsize=13, fontweight='bold')
    for idx, nid in enumerate(node_ids):
        tr = trajectories[nid]
        if tr['role'] == 'HAPS':
            ax0.scatter(tr['x'][0], tr['y'][0], tr['z'][0],
                        color='purple', marker='s', s=100,
                        label=f"HAPS (Node {nid}, z=20km)")
        else:
            c = colors[idx % len(colors)]
            ax0.plot(tr['x'], tr['y'], tr['z'], marker='o', markersize=2,
                     label=f"UAV {nid}", color=c)
            ax0.scatter(tr['x'][0], tr['y'][0], tr['z'][0], color=c, marker='o')
            ax0.scatter(tr['x'][-1], tr['y'][-1], tr['z'][-1], color=c, marker='x')

    ax0.scatter([0], [0], [0], color='black', marker='^', s=90,
                label='Ground Source (Node 0)')
    ax0.scatter([3000], [0], [0], color='darkred', marker='v', s=90,
                label='Ground Dest (Node 29)')
    ax0.set_xlabel("X (m)")
    ax0.set_ylabel("Y (m)")
    ax0.set_zlabel("Altitude Z (m)")
    ax0.legend(loc='upper left', fontsize=8)
    ax0.view_init(elev=25, azim=-60)

    # --- Plot 2: 2-D Top-Down Mobility (X vs Y) ---
    ax1.set_title("UAV / Aerial Node Trajectories & Range Boundaries", fontsize=13, fontweight='bold')
    
    # Ground Source & Destination
    ax1.scatter([0], [0], color='black', marker='^', s=120, label='Ground Source (Node 0)', zorder=5)
    ax1.scatter([3000], [0], color='darkred', marker='v', s=120, label='Ground Dest (Node 29)', zorder=5)

    # Coverage circle around source & dest (approximate 2000m range)
    circle_src = plt.Circle((0, 0), 2000, color='gray', fill=False, linestyle='--', alpha=0.5, label='2000m Range Limits')
    circle_dst = plt.Circle((3000, 0), 2000, color='gray', fill=False, linestyle='--', alpha=0.5)
    ax1.add_patch(circle_src)
    ax1.add_patch(circle_dst)

    for idx, nid in enumerate(node_ids):
        tr = trajectories[nid]
        if tr['role'] == 'HAPS':
            ax1.scatter(tr['x'][0], tr['y'][0], color='purple', marker='s', s=100, label=f"HAPS (Node {nid}, z=20km)")
        else:
            c = colors[idx % len(colors)]
            ax1.plot(tr['x'], tr['y'], marker='o', markersize=3, label=f"UAV {nid} Trajectory", color=c)
            # Annotate start and end
            ax1.annotate(f"UAV {nid} Start", (tr['x'][0], tr['y'][0]), textcoords="offset points", xytext=(0, 6), ha='center', fontsize=8)
            ax1.annotate(f"UAV {nid} End", (tr['x'][-1], tr['y'][-1]), textcoords="offset points", xytext=(0, -12), ha='center', fontsize=8)

    ax1.set_xlabel("X Position (meters)", fontsize=11)
    ax1.set_ylabel("Y Position (meters)", fontsize=11)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', fontsize=9)

    # --- Plot 3: Selected Relay Handover Timeline ---
    timeline_t = sorted(active_relay_by_time.keys())
    timeline_relay = [active_relay_by_time[t] for t in timeline_t]

    ax2.set_title("Relay Selection & Dynamic Handover Over Time", fontsize=13, fontweight='bold')
    ax2.step(timeline_t, timeline_relay, where='post', color='darkblue', linewidth=2.5, label='Active Relay ID')
    ax2.scatter(timeline_t, timeline_relay, color='red', s=25, zorder=4)

    # Annotate handovers
    last_r = timeline_relay[0]
    for t, r in zip(timeline_t, timeline_relay):
        if r != last_r:
            ax2.annotate(f"Handover to Node {r}\nat {t:.0f}s", xy=(t, r), xytext=(t, r + 0.5),
                         arrowprops=dict(facecolor='black', shrink=0.08, width=1, headwidth=5),
                         fontsize=9, fontweight='bold')
            last_r = r

    ax2.set_xlabel("Simulation Time (seconds)", fontsize=11)
    ax2.set_ylabel("Selected Node ID", fontsize=11)
    ax2.set_yticks(node_ids)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=10)

    plt.tight_layout()
    plt.savefig(output_file, dpi=180)
    print(f"Analysis plot saved successfully to: {output_file}")

if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "aerial-relay-traces.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "aerial_relay_analysis.png"
    plot_aerial_traces(csv_path, out_path)
