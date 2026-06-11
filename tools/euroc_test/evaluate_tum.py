#!/usr/bin/env python3
"""Evaluate TUM-format trajectory against ground truth using the repo's chitoku_metrics."""

import argparse
import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "chitoku_metrics"))
from eval_ate_rpe import calculate_ate, calculate_rpe


def load_tum(path):
    """Load TUM file → dict of {timestamp: (tx,ty,tz,qx,qy,qz,qw)}."""
    poses = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            poses[ts] = np.array([float(x) for x in parts[1:8]])
    return poses


def tum_to_matrix(p):
    """Convert [tx,ty,tz,qx,qy,qz,qw] to 4x4 transformation matrix."""
    tx, ty, tz, qx, qy, qz, qw = p
    T = np.eye(4)
    T[:3, 3] = [tx, ty, tz]
    # Rotation from quaternion
    T[0, 0] = 1 - 2*(qy*qy + qz*qz)
    T[0, 1] = 2*(qx*qy - qz*qw)
    T[0, 2] = 2*(qx*qz + qy*qw)
    T[1, 0] = 2*(qx*qy + qz*qw)
    T[1, 1] = 1 - 2*(qx*qx + qz*qz)
    T[1, 2] = 2*(qy*qz - qx*qw)
    T[2, 0] = 2*(qx*qz - qy*qw)
    T[2, 1] = 2*(qy*qz + qx*qw)
    T[2, 2] = 1 - 2*(qx*qx + qy*qy)
    return T


def associate(gt, est, max_diff=0.01):
    """Associate GT and estimated poses by nearest timestamp."""
    gt_ts = sorted(gt.keys())
    matched_gt, matched_est = [], []
    for ts_est, p_est in sorted(est.items()):
        # find closest GT timestamp
        idx = np.searchsorted(gt_ts, ts_est)
        candidates = []
        if idx < len(gt_ts):
            candidates.append(gt_ts[idx])
        if idx > 0:
            candidates.append(gt_ts[idx - 1])
        if not candidates:
            continue
        closest = min(candidates, key=lambda t: abs(t - ts_est))
        if abs(closest - ts_est) <= max_diff:
            matched_gt.append(tum_to_matrix(gt[closest]))
            matched_est.append(tum_to_matrix(p_est))
    return matched_gt, matched_est


def align_umeyama(gt_t, est_t):
    """Umeyama alignment (rotation + translation, no scale)."""
    n = gt_t.shape[0]
    mu_gt = gt_t.mean(0)
    mu_est = est_t.mean(0)
    gt_c = gt_t - mu_gt
    est_c = est_t - mu_est
    W = gt_c.T @ est_c / n
    U, s, Vt = np.linalg.svd(W)
    d = np.linalg.det(U @ Vt)
    S = np.diag([1, 1, d])
    R = U @ S @ Vt
    t = mu_gt - R @ mu_est
    return R, t


def main():
    parser = argparse.ArgumentParser(description="Evaluate TUM trajectory vs ground truth")
    parser.add_argument("gt_tum",  help="Ground truth TUM file")
    parser.add_argument("est_tum", help="Estimated trajectory TUM file")
    parser.add_argument("--max_diff", type=float, default=0.01,
                        help="Max timestamp difference for association (s)")
    parser.add_argument("--rpe_delta", type=int, default=10,
                        help="Frame delta for RPE")
    args = parser.parse_args()

    gt  = load_tum(args.gt_tum)
    est = load_tum(args.est_tum)

    if not est:
        print("ERROR: estimated trajectory is empty")
        sys.exit(1)

    gt_poses, est_poses = associate(gt, est, args.max_diff)
    if len(gt_poses) < 2:
        print(f"ERROR: only {len(gt_poses)} matched poses (too few)")
        sys.exit(1)

    # Align estimated to GT (Umeyama)
    gt_t  = np.array([p[:3, 3] for p in gt_poses])
    est_t = np.array([p[:3, 3] for p in est_poses])
    R_align, t_align = align_umeyama(gt_t, est_t)
    aligned_poses = []
    for p in est_poses:
        p_new = p.copy()
        p_new[:3, 3] = R_align @ p[:3, 3] + t_align
        p_new[:3, :3] = R_align @ p[:3, :3]
        aligned_poses.append(p_new)

    ate, errors, _, _ = calculate_ate(gt_poses, aligned_poses)
    rms_rpe_rot, _, rms_rpe_trans, trans_errors = calculate_rpe(
        gt_poses, aligned_poses, args.rpe_delta)

    print(f"Matched : {len(gt_poses)} / {len(est)} poses")
    print(f"ATE     : mean={ate:.4f}  max={errors.max():.4f}  "
          f"99%={np.percentile(errors, 99):.4f}  m")
    print(f"RPE(t.) : rmse={rms_rpe_trans:.4f} m  (delta={args.rpe_delta})")
    print(f"RPE(r.) : rmse={np.degrees(rms_rpe_rot):.4f} deg")


if __name__ == "__main__":
    main()
