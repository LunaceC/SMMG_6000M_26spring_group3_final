"""Task 3 trajectory planning from the frozen Task 2 path artifact.

This module consumes ``artifacts/task2_cartesian_paths.json`` and produces a
continuous, time-parameterized trajectory in both joint space and Cartesian
space.  The implementation follows the project brief by using cubic
polynomials between consecutive Task 2 waypoints.

The most important design choice is that Task 3 does not re-plan the path.
It preserves the Task 1 visit sequence and the Task 2 collision-free path.  It
only assigns timing and interpolates the existing waypoints.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from math import ceil, hypot
from pathlib import Path
from typing import Any, Sequence

from kinematics import fk, normalize_angle


REPO_ROOT = Path(__file__).resolve().parent
TASK2_ARTIFACT_PATH = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"
TASK3_ARTIFACT_PATH = REPO_ROOT / "artifacts" / "task3_trajectories.json"

# Timing parameters.  They are intentionally conservative and easy to explain.
JOINT_NOMINAL_SPEED_RAD_S = 0.50
CARTESIAN_NOMINAL_SPEED_MM_S = 25.0
ORIENTATION_NOMINAL_SPEED_RAD_S = 0.50
MIN_SUBSEGMENT_DURATION_S = 0.25
SAMPLE_TIME_STEP_S = 0.10


@dataclass(frozen=True)
class CubicSample:
    """Position, velocity, and acceleration of one scalar cubic trajectory."""

    position: float
    velocity: float
    acceleration: float


def _read_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _as_triplet_list(values: Sequence[Any]) -> list[tuple[float, float, float]]:
    """Convert JSON-loaded waypoint data into typed 3-tuples.

    Task 2 stores joint waypoints as lists ``[q1, q2, q3]`` and Cartesian
    waypoints as dictionaries ``{"x": x, "y": y, "phi": phi}``.  This helper
    accepts both formats so the rest of the trajectory planner can operate on
    plain 3-tuples.
    """
    triplets: list[tuple[float, float, float]] = []
    for item in values:
        if isinstance(item, dict):
            triplets.append((float(item["x"]), float(item["y"]), float(item["phi"])))
        else:
            triplets.append((float(item[0]), float(item[1]), float(item[2])))
    return triplets


def _wrapped_delta(start: float, goal: float) -> float:
    """Return the shortest angular displacement from ``start`` to ``goal``."""
    return normalize_angle(goal - start)


def _joint_delta(q0: tuple[float, float, float], q1: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return shortest wrapped displacement for all three revolute joints."""
    return tuple(_wrapped_delta(q0[index], q1[index]) for index in range(3))  # type: ignore[return-value]


def _cartesian_delta(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return Cartesian displacement; orientation uses shortest wrapped angle."""
    return (p1[0] - p0[0], p1[1] - p0[1], _wrapped_delta(p0[2], p1[2]))


def _subsegment_duration(
    q0: tuple[float, float, float],
    q1: tuple[float, float, float],
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
) -> float:
    """Allocate a duration that is feasible for joint and Cartesian motion.

    The same time stamps are shared by the joint-space and Cartesian-space
    cubic trajectories, so each subsegment duration is selected as the maximum
    of three simple timing estimates: joint motion time, Cartesian translation
    time, and tool-orientation time.
    """
    dq = _joint_delta(q0, q1)
    dp = _cartesian_delta(p0, p1)
    joint_time = max(abs(value) for value in dq) / JOINT_NOMINAL_SPEED_RAD_S
    xy_time = hypot(dp[0], dp[1]) / CARTESIAN_NOMINAL_SPEED_MM_S
    phi_time = abs(dp[2]) / ORIENTATION_NOMINAL_SPEED_RAD_S
    return max(joint_time, xy_time, phi_time, MIN_SUBSEGMENT_DURATION_S)


def _waypoint_times(
    joint_waypoints: Sequence[tuple[float, float, float]],
    cartesian_waypoints: Sequence[tuple[float, float, float]],
) -> list[float]:
    """Return cumulative waypoint times for one Task 2 segment."""
    if len(joint_waypoints) != len(cartesian_waypoints):
        raise ValueError("joint_waypoints and cartesian_waypoints must have the same length")
    if len(joint_waypoints) < 2:
        raise ValueError("At least two waypoints are required for trajectory planning")

    times = [0.0]
    for index in range(len(joint_waypoints) - 1):
        dt = _subsegment_duration(
            joint_waypoints[index],
            joint_waypoints[index + 1],
            cartesian_waypoints[index],
            cartesian_waypoints[index + 1],
        )
        times.append(times[-1] + dt)
    return times


def _cubic_sample(start: float, delta: float, duration: float, local_t: float) -> CubicSample:
    """Evaluate cubic interpolation with zero endpoint velocities.

    The scalar trajectory is

        s(t) = 3(t/T)^2 - 2(t/T)^3
        x(t) = x0 + Δx s(t)

    with xdot(0) = xdot(T) = 0.  This is the standard cubic polynomial
    generated by boundary conditions x(0)=x0, x(T)=x1, xdot(0)=xdot(T)=0.
    """
    if duration <= 0.0:
        return CubicSample(position=start + delta, velocity=0.0, acceleration=0.0)

    clamped_t = min(max(local_t, 0.0), duration)
    tau = clamped_t / duration
    blend = 3.0 * tau * tau - 2.0 * tau * tau * tau
    blend_dot = (6.0 * tau - 6.0 * tau * tau) / duration
    blend_ddot = (6.0 - 12.0 * tau) / (duration * duration)
    return CubicSample(
        position=start + delta * blend,
        velocity=delta * blend_dot,
        acceleration=delta * blend_ddot,
    )


def _time_samples(total_time: float) -> list[float]:
    """Return deterministic sample times including 0 and the exact final time."""
    if total_time <= 0.0:
        return [0.0]
    sample_count = max(1, int(ceil(total_time / SAMPLE_TIME_STEP_S)))
    values = [min(index * SAMPLE_TIME_STEP_S, total_time) for index in range(sample_count + 1)]
    values[-1] = total_time
    return values


def _active_subsegment_index(times: Sequence[float], time_value: float, previous_index: int) -> int:
    """Return the waypoint interval containing ``time_value``."""
    index = previous_index
    while index < len(times) - 2 and time_value > times[index + 1] + 1e-12:
        index += 1
    return index


def plan_segment_trajectory(segment_record: dict[str, Any]) -> dict[str, Any]:
    """Generate Task 3 trajectory data for one Task 2 segment record."""
    joint_waypoints = _as_triplet_list(segment_record["chosen_joint_waypoints"])
    cartesian_waypoints = _as_triplet_list(segment_record["cartesian_waypoints"])
    times = _waypoint_times(joint_waypoints, cartesian_waypoints)
    total_time = times[-1]
    samples = _time_samples(total_time)

    joint_positions: list[tuple[float, float, float]] = []
    joint_velocities: list[tuple[float, float, float]] = []
    joint_accelerations: list[tuple[float, float, float]] = []
    cartesian_positions: list[tuple[float, float, float]] = []
    cartesian_velocities: list[tuple[float, float, float]] = []
    cartesian_accelerations: list[tuple[float, float, float]] = []
    fk_cartesian_positions: list[tuple[float, float, float]] = []

    interval_index = 0
    for t_value in samples:
        interval_index = _active_subsegment_index(times, t_value, interval_index)
        start_time = times[interval_index]
        end_time = times[interval_index + 1]
        duration = end_time - start_time
        local_t = t_value - start_time

        q0 = joint_waypoints[interval_index]
        q1 = joint_waypoints[interval_index + 1]
        p0 = cartesian_waypoints[interval_index]
        p1 = cartesian_waypoints[interval_index + 1]
        dq = _joint_delta(q0, q1)
        dp = _cartesian_delta(p0, p1)

        q_samples = tuple(_cubic_sample(q0[j], dq[j], duration, local_t) for j in range(3))
        p_samples = tuple(_cubic_sample(p0[j], dp[j], duration, local_t) for j in range(3))

        q_position = tuple(sample.position for sample in q_samples)
        q_velocity = tuple(sample.velocity for sample in q_samples)
        q_acceleration = tuple(sample.acceleration for sample in q_samples)
        p_position = tuple(sample.position for sample in p_samples)
        p_velocity = tuple(sample.velocity for sample in p_samples)
        p_acceleration = tuple(sample.acceleration for sample in p_samples)

        joint_positions.append(q_position)  # type: ignore[arg-type]
        joint_velocities.append(q_velocity)  # type: ignore[arg-type]
        joint_accelerations.append(q_acceleration)  # type: ignore[arg-type]
        cartesian_positions.append(p_position)  # type: ignore[arg-type]
        cartesian_velocities.append(p_velocity)  # type: ignore[arg-type]
        cartesian_accelerations.append(p_acceleration)  # type: ignore[arg-type]
        fk_cartesian_positions.append(fk(q_position))  # type: ignore[arg-type]

    return {
        "segment_index": segment_record["segment_index"],
        "source_label": segment_record["source_label"],
        "target_label": segment_record["target_label"],
        "source_state_id": segment_record.get("source_state_id"),
        "target_state_id": segment_record.get("target_state_id"),
        "segment_type": segment_record.get("segment_type"),
        "time_parameterization": "piecewise_cubic_zero_endpoint_velocity",
        "waypoint_times": times,
        "duration": total_time,
        "sample_count": len(samples),
        "time": samples,
        "joint_space": {
            "positions": joint_positions,
            "velocities": joint_velocities,
            "accelerations": joint_accelerations,
            "waypoints": joint_waypoints,
        },
        "cartesian_space": {
            "positions": cartesian_positions,
            "velocities": cartesian_velocities,
            "accelerations": cartesian_accelerations,
            "waypoints": cartesian_waypoints,
            "fk_from_joint_positions": fk_cartesian_positions,
        },
    }


def build_task3_trajectories(
    task2_path: str | Path = TASK2_ARTIFACT_PATH,
    output_path: str | Path = TASK3_ARTIFACT_PATH,
) -> dict[str, Any]:
    """Build the Task 3 artifact from the Task 2 path artifact."""
    task2_payload = _read_json(task2_path)
    segment_records = list(task2_payload["ordered_segment_records"])
    trajectories = [plan_segment_trajectory(record) for record in segment_records]

    output_payload: dict[str, Any] = {
        "source_task2_artifact": str(Path(task2_path)),
        "task1_cycle_node_order": task2_payload["task1_cycle_node_order"],
        "task1_weld_order": task2_payload["task1_weld_order"],
        "segment_count": task2_payload["segment_count"],
        "trajectory_segment_count": len(trajectories),
        "time_law": {
            "method": "piecewise cubic polynomial",
            "boundary_conditions": "zero velocity at every Task 2 waypoint",
            "joint_nominal_speed_rad_s": JOINT_NOMINAL_SPEED_RAD_S,
            "cartesian_nominal_speed_mm_s": CARTESIAN_NOMINAL_SPEED_MM_S,
            "orientation_nominal_speed_rad_s": ORIENTATION_NOMINAL_SPEED_RAD_S,
            "min_subsegment_duration_s": MIN_SUBSEGMENT_DURATION_S,
            "sample_time_step_s": SAMPLE_TIME_STEP_S,
        },
        "total_duration": sum(float(item["duration"]) for item in trajectories),
        "trajectories": trajectories,
    }
    _write_json(output_path, output_payload)
    return output_payload


if __name__ == "__main__":
    build_task3_trajectories()
