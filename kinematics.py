"""Planar RRR forward and inverse kinematics for weld-target poses."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin, sqrt

from scene import build_default_scene


ANGLE_TOLERANCE = 1e-12
POSITION_TOLERANCE = 1e-9
L1, L2, L3 = build_default_scene().link_lengths


@dataclass(frozen=True)
class IKSolution:
    """Single inverse-kinematics branch with a residual summary."""

    q: tuple[float, float, float]
    elbow_branch: str
    residual: dict[str, float]


def normalize_angle(angle_rad: float) -> float:
    """Wrap an angle to the interval (-pi, pi]."""
    wrapped = (angle_rad + pi) % (2.0 * pi) - pi
    if wrapped <= -pi:
        return pi
    return wrapped


def fk(q: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return the end-effector pose (x, y, phi) for joint angles in radians."""
    q1, q2, q3 = q
    q12 = q1 + q2
    phi = normalize_angle(q12 + q3)
    x = L1 * cos(q1) + L2 * cos(q12) + L3 * cos(phi)
    y = L1 * sin(q1) + L2 * sin(q12) + L3 * sin(phi)
    return (x, y, phi)


def pose_error(
    pose_a: tuple[float, float, float],
    pose_b: tuple[float, float, float],
) -> dict[str, float]:
    """Return Cartesian and angular error from `pose_a` to `pose_b`."""
    return {
        "dx": pose_b[0] - pose_a[0],
        "dy": pose_b[1] - pose_a[1],
        "dphi": normalize_angle(pose_b[2] - pose_a[2]),
    }


def _branch_solution(
    x: float,
    y: float,
    phi: float,
    cos_q2: float,
    sin_q2: float,
    elbow_branch: str,
) -> IKSolution:
    """Solve one 2R wrist-center branch and attach the FK residual."""
    xw = x - L3 * cos(phi)
    yw = y - L3 * sin(phi)
    q2 = atan2(sin_q2, cos_q2)
    q1 = atan2(yw, xw) - atan2(L2 * sin_q2, L1 + L2 * cos_q2)
    q3 = phi - q1 - q2
    q = tuple(normalize_angle(angle) for angle in (q1, q2, q3))
    residual = pose_error(fk(q), (x, y, normalize_angle(phi)))
    return IKSolution(q=q, elbow_branch=elbow_branch, residual=residual)


def _is_residual_small(residual: dict[str, float]) -> bool:
    """Return whether an FK residual is within numerical tolerance."""
    return (
        abs(residual["dx"]) <= POSITION_TOLERANCE
        and abs(residual["dy"]) <= POSITION_TOLERANCE
        and abs(residual["dphi"]) <= ANGLE_TOLERANCE
    )


def ik_pose(x: float, y: float, phi: float) -> list[tuple[float, float, float]]:
    """Return all real IK solutions for a planar RRR end-effector pose."""
    xw = x - L3 * cos(phi)
    yw = y - L3 * sin(phi)
    wrist_radius_sq = xw * xw + yw * yw
    cos_q2_raw = (wrist_radius_sq - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)

    if cos_q2_raw < -1.0 - POSITION_TOLERANCE or cos_q2_raw > 1.0 + POSITION_TOLERANCE:
        return []

    cos_q2 = min(1.0, max(-1.0, cos_q2_raw))
    sin_q2_abs = sqrt(max(0.0, 1.0 - cos_q2 * cos_q2))
    candidates: list[IKSolution] = []

    branch_data = [("elbow_up", sin_q2_abs)]
    if sin_q2_abs > POSITION_TOLERANCE:
        branch_data.append(("elbow_down", -sin_q2_abs))

    for elbow_branch, sin_q2 in branch_data:
        solution = _branch_solution(x, y, phi, cos_q2, sin_q2, elbow_branch)
        if _is_residual_small(solution.residual):
            candidates.append(solution)

    return [solution.q for solution in candidates]
