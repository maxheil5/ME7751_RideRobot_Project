from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

from se3 import rotation_to_rpy_deg

THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[3]
KINEMATICS_DIR = REPO_ROOT / "3 - Software (Core Code)" / "src" / "kinematics"

if str(KINEMATICS_DIR) not in sys.path:
    sys.path.insert(0, str(KINEMATICS_DIR))

from dh_parameters import DHM, joint_max_deg, joint_min_deg  # noqa: E402

INV_JOINT = np.array([-1, 1, 1, -1, 1, -1], dtype=float)


def mdh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array(
        [
            [c, -s, 0.0, a],
            [s * ca, c * ca, -sa, -sa * d],
            [s * sa, c * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _to_pose(T: np.ndarray) -> dict:
    R = T[:3, :3]
    p = T[:3, 3]
    return {
        "positionMm": [float(p[0]), float(p[1]), float(p[2])],
        "rpyDeg": [float(v) for v in rotation_to_rpy_deg(R)],
    }


def get_joint_limits() -> dict:
    return {
        "minDeg": [float(v) for v in joint_min_deg],
        "maxDeg": [float(v) for v in joint_max_deg],
    }


def fk_kinematics(q_deg: list[float] | np.ndarray) -> dict:
    q_deg_arr = np.asarray(q_deg, dtype=float).reshape(6)
    q_rad = np.deg2rad(q_deg_arr)

    alpha = DHM[:, 0]
    a = DHM[:, 1]
    d = DHM[:, 2]
    theta_offset = DHM[:, 3]

    theta = theta_offset + INV_JOINT * q_rad

    T = np.eye(4)
    link_frames: list[np.ndarray] = []
    for i in range(6):
        T = T @ mdh_transform(float(alpha[i]), float(a[i]), float(d[i]), float(theta[i]))
        link_frames.append(T.copy())

    warnings: list[str] = []
    min_lim = np.array(joint_min_deg, dtype=float)
    max_lim = np.array(joint_max_deg, dtype=float)
    if np.any(q_deg_arr < min_lim) or np.any(q_deg_arr > max_lim):
        warnings.append("One or more joints are outside configured limits.")

    return {
        "T": T.tolist(),
        "pose": _to_pose(T),
        "linkFrames": [frame.tolist() for frame in link_frames],
        "warnings": warnings,
    }


def fk_batch(q_deg_list: list[list[float]]) -> dict:
    Ts = []
    poses = []
    link_frames_list = []
    warnings: list[str] = []

    for q_deg in q_deg_list:
        result = fk_kinematics(q_deg)
        Ts.append(result["T"])
        poses.append(result["pose"])
        link_frames_list.append(result["linkFrames"])
        if result["warnings"]:
            warnings.extend(result["warnings"])

    return {
        "TList": Ts,
        "poseList": poses,
        "linkFramesList": link_frames_list,
        "warnings": warnings,
    }
