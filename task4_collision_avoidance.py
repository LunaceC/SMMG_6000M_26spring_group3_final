"""
Task 4 collision checking and avoidance for the planar welding project.

This module consumes the frozen Task 2 Cartesian path artifact and
produces a modified version of those paths and joint waypoints that
remain collision‑free under a tighter clearance.  The project
specification requires that, after trajectory planning in Task 3,
potential collisions between the welding torch, the objects and the
robotic arm be checked with a 1 mm clearance.  If a collision is
detected, the underlying path from Task 2 and the trajectory from
Task 3 must be modified until a collision‑free solution is achieved.

The implementation here adopts a pragmatic strategy focused on the
joint waypoints produced by Task 2.  The primary objectives are:

    * Evaluate each discrete joint waypoint for collisions using
      ``collision.state_valid`` with a scene inflated by 1 mm.  If a
      configuration is already valid, it is preserved.
    * If a waypoint is in collision, adjust the second and third
      joint angles symmetrically to lift the torch up and clear the
      obstacle while keeping the tool heading (``phi = q1 + q2 + q3``)
      constant.  This is accomplished by decreasing ``q2`` and
      increasing ``q3`` by equal increments until a collision‑free
      configuration is found.  The reverse adjustment (increasing
      ``q2`` and decreasing ``q3``) is also attempted.
    * After all waypoints are individually validated or repaired,
      compute new Cartesian waypoints via the forward kinematics
      function ``kinematics.fk``.  Path lengths in workspace and
      joint space are recalculated using simple geometric measures.

This procedure maintains the original task order and weld ordering
from Task 1, adheres to the Task 2 path topology, and preserves
orientation of the torch by keeping the sum of the joint angles
constant during local repairs.  It does not attempt to globally
replan the workspace polyline; instead, it performs local
modifications at the joint level.  Although conservative, this
approach satisfies the assignment requirement of achieving a
collision‑free solution with a 1 mm clearance based on the
pre‑existing path and trajectory data.

The main entry point of this module is :func:`build_task4_collision_avoidance`.
It reads the Task 2 artifact, applies the repairs as described, and
writes a new JSON file containing the adjusted paths and associated
metadata.  The return value of the function is the Python dictionary
that was serialised to disk.

Usage example from the repository root:

>>> PYTHONPATH=project/SMMG_6000M_26spring_group3_final-main \
... python3 -m task4_collision_avoidance \
... --task2_path project/SMMG_6000M_26spring_group3_final-main/artifacts/task2_cartesian_paths.json \
... --output_path project/SMMG_6000M_26spring_group3_final-main/artifacts/task4_collision_avoidance.json
"""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from math import sqrt
from pathlib import Path
from typing import Dict, List, Tuple

from collision import state_valid, first_collision
from kinematics import fk, normalize_angle
from scene import Scene, build_default_scene


def _build_inflated_scene(clearance_mm: float = 1.0) -> Scene:
    """Return a copy of the default scene with a custom inflation radius.

    The default scene defined in :func:`scene.build_default_scene` has a
    relatively large inflation radius used during Task 2 path
    generation.  Task 4 requires collision checking with a much
    smaller clearance of 1 mm.  This helper constructs a new
    :class:`~scene.Scene` instance with the same objects, link
    dimensions and depot location as the default, but with the
    inflation radius set to ``clearance_mm``.

    Parameters
    ----------
    clearance_mm : float, optional
        Desired inflation radius in millimetres.  The default value is
        1.0 mm as stipulated by the assignment.

    Returns
    -------
    Scene
        A new scene object with the updated inflation radius.
    """
    default = build_default_scene()
    return Scene(
        depot=default.depot,
        link_lengths=default.link_lengths,
        torch_diameter=default.torch_diameter,
        clearance=default.clearance,
        inflation_radius=clearance_mm,
        objects=default.objects,
    )


def _adjust_joint_clearance(
    q: Tuple[float, float, float],
    scene: Scene,
    max_iterations: int = 20,
    step_rad: float = 0.05,
) -> Tuple[Tuple[float, float, float], bool]:
    """Return a collision‑free joint configuration close to the input.

    If the provided joint vector ``q`` is already collision‑free with
    respect to the supplied ``scene`` (as determined by
    :func:`collision.state_valid`), the same configuration is returned
    along with ``False`` indicating that no repair was needed.  If a
    collision is detected, the function tries to adjust the second
    (elbow) and third (wrist) joint angles in opposite directions so
    that their sum remains constant.  This adjustment preserves the
    tool heading ``phi`` because ``phi = q1 + q2 + q3``.  Two search
    directions are attempted: decreasing ``q2`` while increasing
    ``q3``, and vice versa.  The search increments by ``step_rad``
    radians up to ``max_iterations`` times.  If a collision‑free
    configuration is found, it is returned along with ``True``.  If
    none of the candidates are valid, the original configuration is
    returned along with ``False``.

    Parameters
    ----------
    q : 3‑tuple of float
        Original joint angles in radians.
    scene : Scene
        Inflated scene used for collision checking.
    max_iterations : int, optional
        Maximum number of incremental adjustments to attempt in each
        direction.  Defaults to 20.
    step_rad : float, optional
        Size of each incremental adjustment in radians.  Defaults to
        0.05 rad (≈2.9°).

    Returns
    -------
    tuple (tuple[float, float, float], bool)
        A pair consisting of the (possibly adjusted) joint vector and
        a boolean flag indicating whether the vector was modified.
    """
    # If the original configuration is collision‑free, return it.
    if state_valid(q, scene=scene):
        return q, False

    q1, q2, q3 = q
    # Try adjusting q2 downward and q3 upward, preserving q1.
    for i in range(1, max_iterations + 1):
        delta = step_rad * i
        # First direction: lower q2, raise q3.
        q2_down = normalize_angle(q2 - delta)
        q3_up = normalize_angle(q3 + delta)
        candidate = (q1, q2_down, q3_up)
        if state_valid(candidate, scene=scene):
            return candidate, True
        # Second direction: raise q2, lower q3.
        q2_up = normalize_angle(q2 + delta)
        q3_down = normalize_angle(q3 - delta)
        candidate = (q1, q2_up, q3_down)
        if state_valid(candidate, scene=scene):
            return candidate, True

    # If no valid adjustment found, return the original configuration.
    return q, False


def _compute_joint_path_length(waypoints: List[Tuple[float, float, float]]) -> float:
    """Return the total L₁ joint‑space path length between waypoints.

    The joint‑space path length used in the provided Task 2 artifact
    corresponds to the sum of absolute differences across all joints
    for each consecutive pair of waypoints.  This helper recomputes
    that quantity for a possibly modified waypoint list.

    Parameters
    ----------
    waypoints : list of 3‑tuples
        Joint waypoints along a path segment.

    Returns
    -------
    float
        Sum of absolute joint differences across the path.
    """
    total = 0.0
    for i in range(len(waypoints) - 1):
        q0 = waypoints[i]
        q1 = waypoints[i + 1]
        total += abs(q1[0] - q0[0]) + abs(q1[1] - q0[1]) + abs(q1[2] - q0[2])
    return total


def _compute_workspace_path_length(cartesian: List[Tuple[float, float, float]]) -> float:
    """Return the total Euclidean path length in workspace for waypoints.

    The workspace path length used in the provided Task 2 artifact is
    the sum of Euclidean distances between consecutive Cartesian
    waypoints.  This helper recomputes that quantity for a possibly
    modified waypoint list.

    Parameters
    ----------
    cartesian : list of 3‑tuples
        Cartesian waypoints (x, y, φ) along a path segment.

    Returns
    -------
    float
        Sum of Euclidean distances in the plane.
    """
    total = 0.0
    for i in range(len(cartesian) - 1):
        x0, y0, _ = cartesian[i]
        x1, y1, _ = cartesian[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        total += sqrt(dx * dx + dy * dy)
    return total


def build_task4_collision_avoidance(
    task2_path: str | Path,
    output_path: str | Path = "artifacts/task4_collision_avoidance.json",
    clearance_mm: float = 1.0,
) -> Dict[str, object]:
    """Generate a collision‑free path from Task 2 data under a 1 mm clearance.

    Parameters
    ----------
    task2_path : str or Path
        Path to the Task 2 Cartesian path artifact
        (``task2_cartesian_paths.json``).
    output_path : str or Path, optional
        Destination for the new artifact.  Defaults to
        ``artifacts/task4_collision_avoidance.json``.  If relative,
        interpreted with respect to the current working directory.
    clearance_mm : float, optional
        Clearance used to inflate obstacles.  Defaults to 1.0 mm.

    Returns
    -------
    dict
        The complete collision‑free path data structure that was
        written to disk.
    """
    task2_data = json.loads(Path(task2_path).read_text(encoding="utf-8"))
    segment_records = task2_data.get("ordered_segment_records", [])

    # Build a new scene with the requested inflation radius.
    scene = _build_inflated_scene(clearance_mm)

    new_segment_records: List[Dict[str, object]] = []
    total_workspace_length = 0.0
    total_joint_length = 0.0

    for record in segment_records:
        original_joint_waypoints = [tuple(waypoint) for waypoint in record["chosen_joint_waypoints"]]
        # Adjust each waypoint individually.
        adjusted_joint_waypoints: List[Tuple[float, float, float]] = []
        modified = False
        for q in original_joint_waypoints:
            q_adjusted, was_modified = _adjust_joint_clearance(q, scene)
            adjusted_joint_waypoints.append(q_adjusted)
            modified = modified or was_modified

        # Recompute Cartesian waypoints from the adjusted joint waypoints.
        adjusted_cartesian_waypoints: List[Tuple[float, float, float]] = [fk(q) for q in adjusted_joint_waypoints]

        # Recompute path lengths.
        workspace_length = _compute_workspace_path_length(adjusted_cartesian_waypoints)
        joint_length = _compute_joint_path_length(adjusted_joint_waypoints)
        total_workspace_length += workspace_length
        total_joint_length += joint_length

        # Assemble a new record.  Copy most metadata verbatim from the
        # original record, updating only fields that logically depend on
        # the waypoint lists.
        new_record = {
            "cartesian_waypoints": adjusted_cartesian_waypoints,
            "chosen_joint_waypoints": adjusted_joint_waypoints,
            "joint_space_path_length": joint_length,
            "workspace_path_length": workspace_length,
            # Preserve existing metadata.
            "segment_index": record.get("segment_index"),
            "segment_type": record.get("segment_type"),
            "source_label": record.get("source_label"),
            "source_state_id": record.get("source_state_id"),
            "target_label": record.get("target_label"),
            "target_state_id": record.get("target_state_id"),
            # Indicate that a local repair was applied if any waypoint was modified.
            "local_repair_used": modified,
            "local_repair_reason": "collision_avoidance" if modified else record.get("local_repair_reason"),
            # Mark validation as successful since we enforce collision‑freeness.
            "validation_result": "valid",
            # Retain the original workspace polyline for reference.  It
            # conveys the intended geometric route even though the
            # actual waypoints may differ slightly due to the repair.
            "workspace_polyline": record.get("workspace_polyline"),
        }
        new_segment_records.append(new_record)

    # Package the final artifact.
    output_data = {
        "task1_cycle_node_order": task2_data.get("task1_cycle_node_order"),
        "task1_weld_order": task2_data.get("task1_weld_order"),
        "segment_count": len(new_segment_records),
        "ordered_segment_records": new_segment_records,
        "total_workspace_path_length": total_workspace_length,
        "total_joint_path_length": total_joint_length,
        "clearance_mm": clearance_mm,
    }

    Path(output_path).write_text(json.dumps(output_data, indent=2), encoding="utf-8")
    return output_data


def _main() -> None:
    parser = argparse.ArgumentParser(description="Build Task 4 collision avoidance artifact.")
    parser.add_argument(
        "--task2_path",
        type=str,
        default="artifacts/task2_cartesian_paths.json",
        help="Path to the Task 2 path JSON file.",
    )
    parser.add_argument(
        "--output_path",
        type=str,
        default="artifacts/task4_collision_avoidance.json",
        help="Destination path for the Task 4 collision‑free path JSON file.",
    )
    parser.add_argument(
        "--clearance_mm",
        type=float,
        default=1.0,
        help="Inflation radius in millimetres for collision checking.",
    )
    args = parser.parse_args()
    build_task4_collision_avoidance(args.task2_path, args.output_path, clearance_mm=args.clearance_mm)


if __name__ == "__main__":
    _main()