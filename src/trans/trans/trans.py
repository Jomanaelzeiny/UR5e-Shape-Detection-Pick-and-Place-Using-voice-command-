#!/usr/bin/env python3
"""
calibrate_camera_to_base.py
Run once to compute camera → base_link transform from point correspondences.
Saves R and t to a YAML file used by the ROS node.
"""

import numpy as np
import yaml

# ── Fill these in from your data collection ──────────────────────────────────
# Camera frame points  (from /detection/target_coords)
camera_points = np.array([
    [-0.037, 0.13, 0.679],   # point 1
    [0.158, 0.1300, 0.7390],   # point 2
    [-0.231, 0.1250, 0.761],   # point 3
    [0.0610, 0.124, 0.849],   # point 4
    # add more for better accuracy
])

# Corresponding robot base_link points  (from TCP pose at same object position)
base_points = np.array([
    [-0.105,  0.0990, 0.951],  # point 1
    [-0.180,  0.304, 0.949],  # point 2
    [-0.167,  -0.1070, 0.956],  # point 3
    [-0.289,  0.182, 0.9510],  # point 4
])

# ── SVD-based rigid body transform ───────────────────────────────────────────
def compute_transform(src: np.ndarray, dst: np.ndarray):
    """
    Compute R, t such that dst ≈ R @ src + t
    src = camera points,  dst = base_link points
    """
    assert src.shape == dst.shape and src.shape[0] >= 3

    # Centroids
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)

    # Center the point clouds
    A = src - src_c
    B = dst - dst_c

    # SVD of cross-covariance matrix
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    # Fix reflection case (det should be +1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = dst_c - R @ src_c
    return R, t

R, t = compute_transform(camera_points, base_points)

# ── Compute reprojection error ────────────────────────────────────────────────
errors = []
for p_cam, p_base in zip(camera_points, base_points):
    p_est = R @ p_cam + t
    err = np.linalg.norm(p_est - p_base)
    errors.append(err)
    print(f"  estimated: {p_est}  actual: {p_base}  error: {err*1000:.2f} mm")

print(f"\nMean error: {np.mean(errors)*1000:.2f} mm")
print(f"Max  error: {np.max(errors)*1000:.2f} mm")

# ── Save to YAML ──────────────────────────────────────────────────────────────
output = {
    "R": R.tolist(),
    "t": t.tolist()
}
with open("camera_to_base.yaml", "w") as f:
    yaml.dump(output, f)

print("\nSaved → camera_to_base.yaml")
print("R =\n", R)
print("t =", t)