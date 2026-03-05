from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, value))


def so3_log(R: np.ndarray) -> np.ndarray:
    """Returns the axis-angle rotation vector from a 3x3 rotation matrix."""
    trace = float(np.trace(R))
    cos_theta = clamp((trace - 1.0) * 0.5)
    theta = math.acos(cos_theta)

    if theta < 1e-12:
        return np.zeros(3)

    if abs(theta - math.pi) < 1e-6:
        diag = np.diag(R)
        axis = np.sqrt(np.maximum((diag + 1.0) * 0.5, 0.0))
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        return axis * theta

    w_hat = (R - R.T) / (2.0 * math.sin(theta))
    return np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]]) * theta


def so3_exp(w: Iterable[float]) -> np.ndarray:
    """Returns rotation matrix from axis-angle vector."""
    w = np.asarray(w, dtype=float).reshape(3)
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)

    k = w / theta
    kx, ky, kz = k
    K = np.array(
        [
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0],
        ],
        dtype=float,
    )
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def rotation_to_rpy_deg(R: np.ndarray) -> list[float]:
    """
    Converts R to roll-pitch-yaw in degrees using ZYX convention:
    R = Rz(yaw) * Ry(pitch) * Rx(roll).
    """
    sy = -float(R[2, 0])
    sy = clamp(sy)
    pitch = math.asin(sy)

    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        # Gimbal lock fallback: pin yaw to 0 and recover roll from first row.
        yaw = 0.0
        roll = math.atan2(-float(R[0, 1]), float(R[1, 1]))

    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
