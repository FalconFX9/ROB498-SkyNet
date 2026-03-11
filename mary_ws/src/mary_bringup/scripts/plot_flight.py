#!/usr/bin/env python3
"""
Plot 3D vision_pose path from a flight log CSV.

Usage:
    python3 plot_flight.py                          # auto-picks most recent flight
    python3 plot_flight.py ~/mary_logs/flight_*/    # specific flight directory
    python3 plot_flight.py path/to/vision_pose.csv  # specific CSV file
"""

import sys
import os
import csv
import glob

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def find_latest_csv():
    log_base = os.path.expanduser('~/mary_logs')
    flights = sorted(glob.glob(os.path.join(log_base, 'flight_*')), reverse=True)
    for f in flights:
        csv_path = os.path.join(f, 'vision_pose.csv')
        if os.path.exists(csv_path):
            return csv_path
    return None


def resolve_path(arg):
    """Accept a CSV file or a flight directory."""
    if os.path.isfile(arg):
        return arg
    csv_candidate = os.path.join(arg, 'vision_pose.csv')
    if os.path.isfile(csv_candidate):
        return csv_candidate
    return None


def load_csv(path):
    ts, xs, ys, zs = [], [], [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts.append(float(row['timestamp_s']))
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            zs.append(float(row['z']))
    return np.array(ts), np.array(xs), np.array(ys), np.array(zs)


def plot(csv_path):
    ts, xs, ys, zs = load_csv(csv_path)

    if len(xs) == 0:
        print('No data in CSV.')
        return

    duration = ts[-1] - ts[0]
    t_norm = (ts - ts[0]) / max(duration, 1e-9)
    flight_dir = os.path.basename(os.path.dirname(csv_path))

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(xs, ys, zs, c=t_norm, cmap='plasma', s=3, zorder=3)
    ax.plot(xs, ys, zs, alpha=0.25, linewidth=0.8, color='gray')

    ax.scatter([xs[0]], [ys[0]], [zs[0]],
               color='green', s=80, zorder=5, label='Start')
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]],
               color='red', s=80, zorder=5, label='End')

    ax.set_xlabel('X — East (m)')
    ax.set_ylabel('Y — North (m)')
    ax.set_zlabel('Z — Up (m)')
    ax.set_title(
        f'Flight Path  |  {flight_dir}\n'
        f'{len(xs)} samples  ·  {duration:.1f} s'
    )
    ax.legend()

    cbar = plt.colorbar(sc, ax=ax, pad=0.1, shrink=0.6)
    cbar.set_label('Time (normalized)')

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    if len(sys.argv) > 1:
        csv_path = resolve_path(sys.argv[1])
        if csv_path is None:
            print(f'Cannot find vision_pose.csv in: {sys.argv[1]}')
            sys.exit(1)
    else:
        csv_path = find_latest_csv()
        if csv_path is None:
            print('No flight logs found in ~/mary_logs/  — run a flight first.')
            sys.exit(1)
        print(f'Auto-selected: {csv_path}')

    plot(csv_path)
