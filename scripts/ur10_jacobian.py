#!/usr/bin/env python3
"""Numerically validate a classic UR10 mock pose using standard DH values."""
import argparse
import numpy as np


# (a, alpha, d), metres/radians. Standard nominal UR10 (CB-series) geometry.
UR10_DH = (
    (0.0, np.pi / 2, 0.1273),
    (-0.6120, 0.0, 0.0),
    (-0.5723, 0.0, 0.0),
    (0.0, np.pi / 2, 0.163941),
    (0.0, -np.pi / 2, 0.1157),
    (0.0, 0.0, 0.0922),
)


def dh_transform(a, alpha, d, theta):
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def geometric_jacobian(joints):
    q = np.asarray(joints, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError("exactly six finite joint positions are required")
    transform = np.eye(4)
    origins, axes = [], []
    for params, angle in zip(UR10_DH, q):
        origins.append(transform[:3, 3].copy())
        axes.append(transform[:3, 2].copy())
        transform = transform @ dh_transform(*params, angle)
    endpoint = transform[:3, 3]
    jacobian = np.zeros((6, 6))
    for index, (origin, axis) in enumerate(zip(origins, axes)):
        jacobian[:3, index] = np.cross(axis, endpoint - origin)
        jacobian[3:, index] = axis
    return jacobian


def singularity_metrics(joints):
    singular_values = np.linalg.svd(geometric_jacobian(joints), compute_uv=False)
    smallest = singular_values[-1]
    condition = np.inf if smallest < 1e-12 else singular_values[0] / smallest
    return singular_values, float(condition)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("joints", nargs=6, type=float)
    args = parser.parse_args()
    values, condition = singularity_metrics(args.joints)
    print("singular_values:", " ".join(f"{value:.9f}" for value in values))
    print(f"condition_number: {condition:.9f}")


if __name__ == "__main__":
    main()
