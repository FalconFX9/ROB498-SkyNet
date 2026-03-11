#!/usr/bin/env python3
"""
Plot 3D vision_pose path from a flight log.

Usage:
    python3 plot_flight.py                          # auto-picks most recent flight
    python3 plot_flight.py logs/flight_*/           # specific flight directory
    python3 plot_flight.py path/to/vision_pose.npy  # specific npy file
"""

import sys
import os
import glob

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# logs/ lives at the repo root — 4 levels up from this script
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../'))
_LOG_BASE = os.path.join(_REPO_ROOT, 'logs')


def find_latest_npy():
    flights = sorted(glob.glob(os.path.join(_LOG_BASE, 'flight_*')), reverse=True)
    for f in flights:
        npy_path = os.path.join(f, 'vision_pose.npy')
        if os.path.exists(npy_path):
            return npy_path
    return None


def resolve_path(arg):
    if os.path.isfile(arg):
        return arg
    candidate = os.path.join(arg, 'vision_pose.npy')
    if os.path.isfile(candidate):
        return candidate
    return None


def plot(npy_path):
    # Shape: (N, 8) — timestamp_s, x, y, z, qx, qy, qz, qw
    data = np.load(npy_path)

    if data.shape[0] == 0:
        print('No data in file.')
        return

    ts, xs, ys, zs = data[:, 0], data[:, 1], data[:, 2], data[:, 3]
    duration = ts[-1] - ts[0]
    t_norm = (ts - ts[0]) / max(duration, 1e-9)
    flight_dir = os.path.basename(os.path.dirname(npy_path))

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(xs, ys, zs, c=t_norm, cmap='plasma', s=6, zorder=3)
    ax.plot(xs, ys, zs, alpha=0.25, linewidth=0.8, color='gray')

    ax.scatter([xs[0]], [ys[0]], [zs[0]], color='green', s=80, zorder=5, label='Start')
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], color='red', s=80, zorder=5, label='End')

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

    out_path = os.path.join(os.path.dirname(npy_path), 'flight_path.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {out_path}')


if __name__ == '__main__':
    if len(sys.argv) > 1:
        npy_path = resolve_path(sys.argv[1])
        if npy_path is None:
            print(f'Cannot find vision_pose.npy in: {sys.argv[1]}')
            sys.exit(1)
    else:
        npy_path = find_latest_npy()
        if npy_path is None:
            print(f'No flight logs found in {_LOG_BASE}/ — run a flight first.')
            sys.exit(1)
        print(f'Auto-selected: {npy_path}')

    plot(npy_path)
