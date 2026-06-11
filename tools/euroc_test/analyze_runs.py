#!/usr/bin/env python3
"""Analyze a single cuVSLAM state file against EuRoC ground truth.

Loads a state file (pose + velocity + bias) and ground truth,
aligns trajectory via Umeyama, and produces comparison plots.

Usage:
  python3 analyze_runs.py --state /tmp/v203_state.txt \
      --gt /mnt/wdssd/Euroc/V2_03_difficult/mav0/state_groundtruth_estimate0/data.csv
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path


def load_state_file(path):
    """Load state file: timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz"""
    data = []
    with open(path) as f:
        for line in f:
            if line.startswith('#') or line.strip() == '':
                continue
            vals = list(map(float, line.split()))
            data.append(vals)
    data = np.array(data)
    return {
        'ts': data[:, 0],
        'pos': data[:, 1:4],
        'quat': data[:, 4:8],  # qx qy qz qw
        'vel': data[:, 8:11],
        'gyro_bias': data[:, 11:14],
        'acc_bias': data[:, 14:17],
    }


def load_euroc_gt(path):
    """Load EuRoC ground truth CSV.
    Format: timestamp_ns, px py pz, qw qx qy qz, vx vy vz, bw_x bw_y bw_z, ba_x ba_y ba_z
    """
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    return {
        'ts': data[:, 0] * 1e-9,  # ns to seconds
        'pos': data[:, 1:4],
        'quat_wxyz': data[:, 4:8],  # qw qx qy qz (EuRoC order)
        'vel': data[:, 8:11],
        'gyro_bias': data[:, 11:14],
        'acc_bias': data[:, 14:17],
    }


def umeyama_alignment(model, data, with_scale=True):
    """Compute Sim(3) or SE(3) alignment: data = s*R*model + t. Returns s, R, t."""
    mu_m = model.mean(axis=0)
    mu_d = data.mean(axis=0)
    model_c = model - mu_m
    data_c = data - mu_d
    H = model_c.T @ data_c
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    if with_scale:
        var_m = np.sum(model_c ** 2) / len(model)
        s = np.trace(np.diag(S) @ D) / var_m
    else:
        s = 1.0
    t = mu_d - s * R @ mu_m
    return s, R, t


def interp_gt(gt, query_ts):
    """Interpolate GT at query timestamps."""
    result = {}
    for key in ['pos', 'vel', 'gyro_bias', 'acc_bias']:
        cols = []
        for c in range(gt[key].shape[1]):
            cols.append(np.interp(query_ts, gt['ts'], gt[key][:, c]))
        result[key] = np.stack(cols, axis=1)
    return result


def align_trajectory(run_pos, gt_pos):
    """Align run to GT using SE(3) (no scale), return aligned positions, ATE, and R."""
    s, R, t = umeyama_alignment(run_pos, gt_pos, with_scale=False)
    aligned = (R @ run_pos.T).T + t
    errors = np.linalg.norm(aligned - gt_pos, axis=1)
    return aligned, errors, R


def main():
    parser = argparse.ArgumentParser(description='Analyze a single state file against GT')
    parser.add_argument('--state', required=True, help='Path to state file')
    parser.add_argument('--gt', required=True, help='Path to EuRoC state_groundtruth_estimate0/data.csv')
    parser.add_argument('--output', default='analysis.png', help='Output plot file')
    args = parser.parse_args()

    gt = load_euroc_gt(args.gt)
    run_raw = load_state_file(args.state)

    # Filter out pre-IMU-init frames (where gyro_bias and acc_bias are all zero)
    bias_norm = np.linalg.norm(run_raw['gyro_bias'], axis=1) + np.linalg.norm(run_raw['acc_bias'], axis=1)
    mask = bias_norm > 1e-10
    if mask.sum() == 0:
        print("Error: no IMU-initialized frames found")
        return
    run = {k: v[mask] if isinstance(v, np.ndarray) and v.shape[0] == len(mask) else v
           for k, v in run_raw.items()}
    print(f"Loaded {mask.sum()}/{len(mask)} frames after IMU init")
    print(f"GT has {len(gt['ts'])} samples")

    # Interpolate GT at run timestamps
    gt_interp = interp_gt(gt, run['ts'])

    # Align trajectory (SE(3), no scale)
    aligned, ate, R_align = align_trajectory(run['pos'], gt_interp['pos'])
    t_rel = run['ts'] - run['ts'][0]

    fig, axes = plt.subplots(4, 2, figsize=(18, 20))
    fig.suptitle(f'State Analysis (ATE mean={np.mean(ate):.4f} m)', fontsize=14)

    # Position error over time
    axes[0, 0].plot(t_rel, ate, 'b-', alpha=0.8, label=f'ATE (mean={np.mean(ate):.3f})')
    axes[0, 0].set_title('Position Error (ATE) over time')
    axes[0, 0].set_ylabel('Error (m)')
    axes[0, 0].legend(fontsize=8)
    axes[0, 0].grid(True)

    # ATE histogram
    axes[0, 1].hist(ate, bins=50, color='steelblue', alpha=0.7, edgecolor='black')
    axes[0, 1].axvline(np.mean(ate), color='r', linestyle='--', label=f'mean={np.mean(ate):.3f}')
    axes[0, 1].axvline(np.percentile(ate, 99), color='orange', linestyle='--', label=f'99%={np.percentile(ate, 99):.3f}')
    axes[0, 1].set_title('ATE Distribution')
    axes[0, 1].set_xlabel('Error (m)')
    axes[0, 1].set_ylabel('Count')
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True)

    # Velocity x,y,z vs GT
    # Both state file velocity and GT velocity are in their respective world frames.
    # Apply the same Umeyama alignment rotation to map estimate world → GT world.
    gt_t_rel = gt['ts'] - gt['ts'][0]
    vel_aligned = (R_align @ run['vel'].T).T
    colors = ['r', 'g', 'b']
    for c, (name, col) in enumerate(zip(['x', 'y', 'z'], colors)):
        axes[1, 0].plot(t_rel, vel_aligned[:, c], col, alpha=0.6, label=f'Est {name}')
        axes[1, 0].plot(t_rel, gt_interp['vel'][:, c], col, linewidth=2, linestyle='--', alpha=0.4, label=f'GT {name}')
    axes[1, 0].set_title('Velocity (x,y,z)')
    axes[1, 0].set_ylabel('v (m/s)')
    axes[1, 0].legend(fontsize=7)
    axes[1, 0].grid(True)

    # Velocity error x,y,z
    vel_err_xyz = vel_aligned - gt_interp['vel']
    vel_err = np.linalg.norm(vel_err_xyz, axis=1)
    for c, (name, col) in enumerate(zip(['x', 'y', 'z'], colors)):
        axes[1, 1].plot(t_rel, vel_err_xyz[:, c], col, alpha=0.7, label=f'{name} (mean={np.mean(np.abs(vel_err_xyz[:, c])):.3f})')
    axes[1, 1].set_title(f'Velocity error x,y,z (|err| mean={np.mean(vel_err):.3f} m/s)')
    axes[1, 1].set_ylabel('v_err (m/s)')
    axes[1, 1].legend(fontsize=7)
    axes[1, 1].grid(True)

    # Gyro bias components
    for c, (name, ls) in enumerate(zip(['x', 'y', 'z'], ['-', '--', ':'])):
        axes[2, 0].plot(t_rel, run['gyro_bias'][:, c], 'b', alpha=0.6, linestyle=ls, label=f'Est {name}')
        axes[2, 0].plot(gt_t_rel, gt['gyro_bias'][:, c], 'k', linewidth=2, linestyle=ls, label=f'GT {name}')
    axes[2, 0].set_title('Gyro bias (IMU body frame)')
    axes[2, 0].set_ylabel('rad/s')
    axes[2, 0].legend(fontsize=7)
    axes[2, 0].grid(True)

    # Acc bias components
    for c, (name, ls) in enumerate(zip(['x', 'y', 'z'], ['-', '--', ':'])):
        axes[2, 1].plot(t_rel, run['acc_bias'][:, c], 'b', alpha=0.6, linestyle=ls, label=f'Est {name}')
        axes[2, 1].plot(gt_t_rel, gt['acc_bias'][:, c], 'k', linewidth=2, linestyle=ls, label=f'GT {name}')
    axes[2, 1].set_title('Acc bias (IMU body frame)')
    axes[2, 1].set_ylabel('m/s²')
    axes[2, 1].legend(fontsize=7)
    axes[2, 1].grid(True)

    # XY trajectory
    axes[3, 0].plot(aligned[:, 0], aligned[:, 1], 'b-', alpha=0.7, label='Estimate')
    axes[3, 0].plot(gt_interp['pos'][:, 0], gt_interp['pos'][:, 1], 'k-', linewidth=2, label='GT')
    axes[3, 0].set_title('Trajectory XY')
    axes[3, 0].set_xlabel('X (m)')
    axes[3, 0].set_ylabel('Y (m)')
    axes[3, 0].set_aspect('equal')
    axes[3, 0].legend(fontsize=8)
    axes[3, 0].grid(True)

    # XZ trajectory
    axes[3, 1].plot(aligned[:, 0], aligned[:, 2], 'b-', alpha=0.7, label='Estimate')
    axes[3, 1].plot(gt_interp['pos'][:, 0], gt_interp['pos'][:, 2], 'k-', linewidth=2, label='GT')
    axes[3, 1].set_title('Trajectory XZ')
    axes[3, 1].set_xlabel('X (m)')
    axes[3, 1].set_ylabel('Z (m)')
    axes[3, 1].set_aspect('equal')
    axes[3, 1].legend(fontsize=8)
    axes[3, 1].grid(True)

    for ax_row in axes[:-1]:  # skip trajectory row — x-axis is spatial, not time
        for ax in ax_row:
            ax.set_xlabel('Time (s)')

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f"\nPlot saved to: {args.output}")
    print(f"\nATE: mean={np.mean(ate):.4f}  max={np.max(ate):.4f}  99%={np.percentile(ate, 99):.4f} m")
    print(f"Vel error: mean={np.mean(vel_err):.4f} m/s")


if __name__ == '__main__':
    main()
