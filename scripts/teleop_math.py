#!/usr/bin/env python3
"""Pure math and safety-gating helpers for Quest -> UR teleoperation.

Quaternion values use ROS' (x, y, z, w) convention throughout.  This module
has no ROS dependency so coordinate, finite-value and timeout behavior can be
unit-tested on the host.
"""
from dataclasses import dataclass
import math
import numpy as np


EPS = 1e-9


def finite_vector(values, size, name="vector"):
    value = np.asarray(values, dtype=float)
    if value.shape != (size,) or not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain {size} finite values")
    return value


def normalize_quaternion(quaternion):
    q = finite_vector(quaternion, 4, "quaternion")
    norm = np.linalg.norm(q)
    if norm < EPS:
        raise ValueError("quaternion norm is zero")
    return q / norm


def quaternion_conjugate(quaternion):
    q = normalize_quaternion(quaternion)
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quaternion_multiply(left, right):
    x1, y1, z1, w1 = normalize_quaternion(left)
    x2, y2, z2, w2 = normalize_quaternion(right)
    return normalize_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quaternion_to_matrix(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(rotation):
    r = validate_rotation_matrix(rotation)
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(r[2, 1] - r[1, 2]) / s,
             (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s,
             0.25 * s]
    else:
        index = int(np.argmax(np.diag(r)))
        if index == 0:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            q = [0.25 * s, (r[0, 1] + r[1, 0]) / s,
                 (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s]
        elif index == 1:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            q = [(r[0, 1] + r[1, 0]) / s, 0.25 * s,
                 (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            q = [(r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s,
                 0.25 * s, (r[1, 0] - r[0, 1]) / s]
    return normalize_quaternion(q)


def validate_rotation_matrix(rotation):
    r = np.asarray(rotation, dtype=float)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(r), 1.0, atol=1e-6):
        raise ValueError("rotation matrix must be orthonormal with determinant +1")
    return r


def average_quaternions(quaternions):
    if not quaternions:
        raise ValueError("at least one quaternion is required")
    reference = normalize_quaternion(quaternions[0])
    aligned = []
    for value in quaternions:
        q = normalize_quaternion(value)
        aligned.append(-q if np.dot(q, reference) < 0 else q)
    return normalize_quaternion(np.mean(aligned, axis=0))


def anchor_calibration(vr_anchor_quaternion, robot_anchor_quaternion):
    """Rotation that maps controller axes at anchor time into robot TCP axes.

    The operator must hold the controller aligned with the intended TCP axes
    when enabling pose streaming. This makes the calibration explicit instead
    of assuming Quest world axes already equal base_link axes.
    """
    robot = quaternion_to_matrix(robot_anchor_quaternion)
    vr = quaternion_to_matrix(vr_anchor_quaternion)
    return validate_rotation_matrix(robot @ vr.T)


def map_anchored_pose(vr_position, vr_quaternion, vr_anchor_position,
                      vr_anchor_quaternion, robot_anchor_position,
                      robot_anchor_quaternion, vr_to_robot_rotation,
                      position_scale=1.0):
    p = finite_vector(vr_position, 3, "VR position")
    p0 = finite_vector(vr_anchor_position, 3, "VR anchor position")
    robot_p0 = finite_vector(robot_anchor_position, 3, "robot anchor position")
    q = normalize_quaternion(vr_quaternion)
    q0 = normalize_quaternion(vr_anchor_quaternion)
    robot_q0 = normalize_quaternion(robot_anchor_quaternion)
    rotation = validate_rotation_matrix(vr_to_robot_rotation)
    scale = float(position_scale)
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("position_scale must be finite and positive")

    target_position = robot_p0 + scale * (rotation @ (p - p0))
    relative_vr = quaternion_to_matrix(q) @ quaternion_to_matrix(q0).T
    relative_robot = rotation @ relative_vr @ rotation.T
    target_rotation = relative_robot @ quaternion_to_matrix(robot_q0)
    return target_position, matrix_to_quaternion(target_rotation)


def clamp_norm(vector, maximum):
    value = np.asarray(vector, dtype=float)
    if not np.all(np.isfinite(value)):
        raise ValueError("cannot clamp a non-finite vector")
    maximum = float(maximum)
    if not math.isfinite(maximum) or maximum < 0:
        raise ValueError("maximum must be finite and non-negative")
    norm = np.linalg.norm(value)
    return value * min(1.0, maximum / norm) if norm > EPS else value


def rotation_vector(target_quaternion, current_quaternion):
    error = quaternion_multiply(target_quaternion,
                                quaternion_conjugate(current_quaternion))
    if error[3] < 0:
        error = -error
    vector_norm = np.linalg.norm(error[:3])
    if vector_norm < EPS:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, error[3]))
    return angle * error[:3] / vector_norm


def pose_error_to_twist(target_position, target_quaternion, current_position,
                        current_quaternion, linear_gain=1.0, angular_gain=0.8,
                        max_linear=0.025, max_angular=0.10,
                        angular_enabled=True):
    target_p = finite_vector(target_position, 3, "target position")
    current_p = finite_vector(current_position, 3, "current position")
    linear = clamp_norm(float(linear_gain) * (target_p - current_p), max_linear)
    if angular_enabled:
        angular = clamp_norm(
            float(angular_gain) * rotation_vector(target_quaternion, current_quaternion),
            max_angular,
        )
    else:
        # Still validate both quaternions so corrupt inputs never pass silently.
        normalize_quaternion(target_quaternion)
        normalize_quaternion(current_quaternion)
        angular = np.zeros(3)
    return linear, angular


@dataclass(frozen=True)
class GateInputs:
    now: float
    goal_received: float | None
    inputs_received: float | None
    deadman: bool
    timeout: float


def gate_reason(state):
    if state.goal_received is None:
        return "no target pose"
    if state.inputs_received is None:
        return "no controller input"
    if state.now - state.goal_received >= state.timeout:
        return "target pose stale"
    if state.now - state.inputs_received >= state.timeout:
        return "controller input stale"
    if not state.deadman:
        return "deadman released"
    return None
