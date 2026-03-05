from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from fk import get_joint_limits


@dataclass
class IkOptions:
    eps_wrist: float = 1e-8
    tol_reach: float = 1e-10
    tol_unique: float = 1e-6


def wrap_to_pi(value: np.ndarray | float) -> np.ndarray | float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def map_theta_to_q(th4: float, th5: float, th6: float) -> tuple[float, float, float]:
    # Mapping from theta-space used in DH equations to robot joint-space q.
    q4 = float(wrap_to_pi(-th4))
    q5 = float(wrap_to_pi(th5))
    q6 = float(wrap_to_pi(math.pi - th6))
    return q4, q5, q6


def enforce_limits(q: np.ndarray, jmin: np.ndarray, jmax: np.ndarray) -> tuple[bool, np.ndarray]:
    q_adj = q.copy()
    for i in range(6):
        qi = q_adj[i]
        candidates = np.array([qi, qi + 2 * math.pi, qi - 2 * math.pi, qi + 4 * math.pi, qi - 4 * math.pi], dtype=float)
        in_range = candidates[(candidates >= jmin[i] - 1e-12) & (candidates <= jmax[i] + 1e-12)]
        if in_range.size == 0:
            return False, q_adj
        nearest_idx = int(np.argmin(np.abs(in_range - qi)))
        q_adj[i] = in_range[nearest_idx]
    return True, q_adj


def is_duplicate(q: np.ndarray, q_list: list[np.ndarray], tol: float) -> bool:
    for existing in q_list:
        if np.all(np.abs(wrap_to_pi(q - existing)) < tol):
            return True
    return False


def angle_distance_sq(q: np.ndarray, seed: np.ndarray) -> float:
    return float(np.sum(np.square(wrap_to_pi(q - seed))))


def _normalize_T(T: list[list[float]] | np.ndarray) -> np.ndarray:
    T_arr = np.asarray(T, dtype=float)
    if T_arr.shape != (4, 4):
        raise ValueError("T must be 4x4.")
    return T_arr


def ik_solve(T: list[list[float]] | np.ndarray, seed_deg: list[float] | None = None, opts: IkOptions | None = None) -> dict[str, Any]:
    opts = opts or IkOptions()
    T06 = _normalize_T(T)
    R06 = T06[:3, :3]
    p06 = T06[:3, 3]

    d6 = 290.0
    A = 1300.0
    d1 = 1045.0
    L = math.hypot(1025.0, 55.0)
    phi = math.atan2(55.0, 1025.0)

    limits = get_joint_limits()
    jmin = np.deg2rad(np.array(limits["minDeg"], dtype=float))
    jmax = np.deg2rad(np.array(limits["maxDeg"], dtype=float))

    z6 = R06[:, 2]
    p_wc = p06 - d6 * z6
    xw, yw, zw = [float(v) for v in p_wc]

    rho = math.hypot(xw, yw)

    q1_a = math.atan2(-yw, xw)
    B_a = rho

    q1_b = float(wrap_to_pi(q1_a + math.pi))
    B_b = -rho

    shoulders = [(q1_a, B_a, 1), (q1_b, B_b, 2)]

    Q_all: list[np.ndarray] = []
    meta_all: list[dict[str, Any]] = []
    warnings: list[str] = []

    for q1, B, s_idx in shoulders:
        r = B - 500.0
        z = d1 - zw

        D = (r * r + z * z - A * A - L * L) / (2.0 * A * L)

        if abs(D) > 1.0 + opts.tol_reach:
            continue

        D = max(-1.0, min(1.0, D))

        gamma_plus = math.atan2(+math.sqrt(max(0.0, 1.0 - D * D)), D)
        gamma_minus = math.atan2(-math.sqrt(max(0.0, 1.0 - D * D)), D)
        gammas = [(gamma_plus, 1), (gamma_minus, 2)]

        for gamma, e_idx in gammas:
            q2 = math.atan2(z, r) - math.atan2(L * math.sin(gamma), A + L * math.cos(gamma))
            q3 = gamma - phi

            q1n = float(wrap_to_pi(q1))
            q2n = float(wrap_to_pi(q2))
            q3n = float(wrap_to_pi(q3))

            c1, s1 = math.cos(q1n), math.sin(q1n)
            c23, s23 = math.cos(q2n + q3n), math.sin(q2n + q3n)

            R03 = np.array(
                [
                    [s23 * c1, c23 * c1, s1],
                    [-s23 * s1, -c23 * s1, c1],
                    [c23, -s23, 0.0],
                ],
                dtype=float,
            )

            R36 = R03.T @ R06
            c5 = float(R36[1, 2])
            s5 = float(math.hypot(R36[1, 0], R36[1, 1]))

            if s5 > opts.eps_wrist:
                th5a = math.atan2(+s5, c5)
                th4a = math.atan2(float(R36[2, 2]), -float(R36[0, 2]))
                th6a = math.atan2(-float(R36[1, 1]), float(R36[1, 0]))

                q4_1, q5_1, q6_1 = map_theta_to_q(th4a, th5a, th6a)
                q_w1 = np.array([q1n, q2n, q3n, q4_1, q5_1, q6_1], dtype=float)

                th5b = -th5a
                th4b = float(wrap_to_pi(th4a + math.pi))
                th6b = float(wrap_to_pi(th6a + math.pi))

                q4_2, q5_2, q6_2 = map_theta_to_q(th4b, th5b, th6b)
                q_w2 = np.array([q1n, q2n, q3n, q4_2, q5_2, q6_2], dtype=float)

                candidates = [
                    (q_w1, [s_idx, e_idx, 1], False, "ok"),
                    (q_w2, [s_idx, e_idx, 2], False, "ok"),
                ]
            else:
                if c5 >= 0.0:
                    th5 = 0.0
                    psi = math.atan2(-float(R36[0, 1]), float(R36[0, 0]))
                    th4 = 0.0
                    th6 = psi
                else:
                    th5 = math.pi
                    psi = math.atan2(-float(R36[0, 1]), -float(R36[0, 0]))
                    th4 = 0.0
                    th6 = -psi

                q4, q5, q6 = map_theta_to_q(th4, th5, th6)
                q_sing = np.array([q1n, q2n, q3n, q4, q5, q6], dtype=float)
                candidates = [(q_sing, [s_idx, e_idx, 0], True, "wrist_singularity_resolution")]

            for q_candidate, branch, singular, reason in candidates:
                ok, q_adj = enforce_limits(q_candidate, jmin, jmax)
                if not ok:
                    continue

                if is_duplicate(q_adj, Q_all, opts.tol_unique):
                    continue

                Q_all.append(q_adj)
                meta_all.append(
                    {
                        "branch": [int(branch[0]), int(branch[1]), int(branch[2])],
                        "singular": bool(singular),
                        "reason": reason,
                    }
                )

    if not Q_all:
        warnings.append("IK returned no valid branch for this target pose.")
        return {"solutionsDeg": [], "meta": [], "warnings": warnings}

    if seed_deg is not None:
        seed = np.deg2rad(np.asarray(seed_deg, dtype=float).reshape(6))
        order = sorted(range(len(Q_all)), key=lambda idx: angle_distance_sq(Q_all[idx], seed))
        Q_all = [Q_all[idx] for idx in order]
        meta_all = [meta_all[idx] for idx in order]

    solutions_deg = [np.rad2deg(q).tolist() for q in Q_all]
    return {
        "solutionsDeg": solutions_deg,
        "meta": meta_all,
        "warnings": warnings,
    }
