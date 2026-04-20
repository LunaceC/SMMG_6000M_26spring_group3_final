"""Collision checking for Link 3 and torch against inflated scene objects."""

from __future__ import annotations

from math import ceil, cos, radians, sin

import numpy as np

from kinematics import fk, normalize_angle
from scene import CircleObject, HexagonObject, Scene, build_default_scene


DEFAULT_SCENE = build_default_scene()
L1, L2, L3 = DEFAULT_SCENE.link_lengths
CLEARANCE_TOLERANCE = 1e-9


def _link3_segment(q: tuple[float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Return wrist and tool-tip points for a joint configuration."""
    x, y, phi = fk(q)
    tip = np.array([x, y], dtype=float)
    wrist = tip - L3 * np.array([cos(phi), sin(phi)], dtype=float)
    return wrist, tip


def _point_to_segment_distance(point: np.ndarray, seg_start: np.ndarray, seg_end: np.ndarray) -> float:
    """Return the Euclidean distance from a point to a segment."""
    segment = seg_end - seg_start
    segment_length_sq = float(np.dot(segment, segment))
    if segment_length_sq <= 0.0:
        return float(np.linalg.norm(point - seg_start))

    projection = float(np.dot(point - seg_start, segment) / segment_length_sq)
    projection = min(1.0, max(0.0, projection))
    closest = seg_start + projection * segment
    return float(np.linalg.norm(point - closest))


def _segments_intersect(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> bool:
    """Return whether two closed planar segments intersect."""
    def cross(u: np.ndarray, v: np.ndarray) -> float:
        return float(u[0] * v[1] - u[1] * v[0])

    def orient(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
        return cross(q - p, r - p)

    def on_segment(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
        return (
            min(p[0], r[0]) - CLEARANCE_TOLERANCE <= q[0] <= max(p[0], r[0]) + CLEARANCE_TOLERANCE
            and min(p[1], r[1]) - CLEARANCE_TOLERANCE <= q[1] <= max(p[1], r[1]) + CLEARANCE_TOLERANCE
        )

    o1 = orient(a0, a1, b0)
    o2 = orient(a0, a1, b1)
    o3 = orient(b0, b1, a0)
    o4 = orient(b0, b1, a1)

    if o1 * o2 < 0.0 and o3 * o4 < 0.0:
        return True

    if abs(o1) <= CLEARANCE_TOLERANCE and on_segment(a0, b0, a1):
        return True
    if abs(o2) <= CLEARANCE_TOLERANCE and on_segment(a0, b1, a1):
        return True
    if abs(o3) <= CLEARANCE_TOLERANCE and on_segment(b0, a0, b1):
        return True
    if abs(o4) <= CLEARANCE_TOLERANCE and on_segment(b0, a1, b1):
        return True

    return False


def _segment_to_segment_distance(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
    """Return the minimum Euclidean distance between two planar segments."""
    if _segments_intersect(a0, a1, b0, b1):
        return 0.0

    return min(
        _point_to_segment_distance(a0, b0, b1),
        _point_to_segment_distance(a1, b0, b1),
        _point_to_segment_distance(b0, a0, a1),
        _point_to_segment_distance(b1, a0, a1),
    )


def _hexagon_normals() -> tuple[np.ndarray, ...]:
    """Return the fixed outward unit normals for the regular hexagons."""
    return tuple(
        np.array([cos(radians(30.0 + 60.0 * index)), sin(radians(30.0 + 60.0 * index))], dtype=float)
        for index in range(6)
    )


HEXAGON_NORMALS = _hexagon_normals()


def _clip_segment_to_inflated_hexagon(
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    scene_object: HexagonObject,
    inflated_inradius: float,
) -> tuple[bool, float, float]:
    """Clip a segment against an inflated hexagon and return the inside interval."""
    center = np.array(scene_object.center, dtype=float)
    direction = seg_end - seg_start
    t_enter = 0.0
    t_exit = 1.0

    for normal in HEXAGON_NORMALS:
        offset_start = float(np.dot(normal, seg_start - center))
        offset_direction = float(np.dot(normal, direction))
        slack = inflated_inradius - offset_start

        if abs(offset_direction) <= CLEARANCE_TOLERANCE:
            if slack < -CLEARANCE_TOLERANCE:
                return (False, 0.0, 0.0)
            continue

        boundary_t = slack / offset_direction
        if offset_direction > 0.0:
            t_exit = min(t_exit, boundary_t)
        else:
            t_enter = max(t_enter, boundary_t)

        if t_enter > t_exit + CLEARANCE_TOLERANCE:
            return (False, 0.0, 0.0)

    return (True, max(0.0, t_enter), min(1.0, t_exit))


def _inflated_hexagon_vertices(scene_object: HexagonObject, inflation_radius: float) -> tuple[np.ndarray, ...]:
    """Return vertices for an inflated regular hexagon in deterministic order."""
    cx, cy = scene_object.center
    inflated_circumradius = (scene_object.inradius + inflation_radius) / cos(radians(30.0))
    return tuple(
        np.array(
            [
                cx + inflated_circumradius * cos(radians(60.0 * vertex_index)),
                cy + inflated_circumradius * sin(radians(60.0 * vertex_index)),
            ],
            dtype=float,
        )
        for vertex_index in range(6)
    )


def _point_signed_distance_to_inflated_hexagon(
    point: np.ndarray,
    scene_object: HexagonObject,
    inflated_inradius: float,
) -> float:
    """Return signed distance from a point to the inflated hexagon boundary."""
    center = np.array(scene_object.center, dtype=float)
    offsets = [float(np.dot(normal, point - center) - inflated_inradius) for normal in HEXAGON_NORMALS]
    max_offset = max(offsets)
    if max_offset <= 0.0:
        return max_offset

    vertices = _inflated_hexagon_vertices(scene_object, inflated_inradius - scene_object.inradius)
    edge_distances = [
        _point_to_segment_distance(point, vertices[index], vertices[(index + 1) % 6])
        for index in range(6)
    ]
    return min(edge_distances)


def _circle_segment_clearance(
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    scene_object: CircleObject,
    inflation_radius: float,
) -> float:
    """Return signed clearance from Link 3 to an inflated circle."""
    center = np.array(scene_object.center, dtype=float)
    inflated_radius = scene_object.radius + inflation_radius
    return _point_to_segment_distance(center, seg_start, seg_end) - inflated_radius


def _hexagon_segment_clearance(
    seg_start: np.ndarray,
    seg_end: np.ndarray,
    scene_object: HexagonObject,
    inflation_radius: float,
) -> float:
    """Return signed clearance from Link 3 to an inflated regular hexagon."""
    inflated_inradius = scene_object.inradius + inflation_radius
    intersects, t_enter, t_exit = _clip_segment_to_inflated_hexagon(
        seg_start,
        seg_end,
        scene_object,
        inflated_inradius,
    )

    if intersects:
        if t_exit - t_enter <= CLEARANCE_TOLERANCE:
            return 0.0
        midpoint = seg_start + 0.5 * (t_enter + t_exit) * (seg_end - seg_start)
        return _point_signed_distance_to_inflated_hexagon(midpoint, scene_object, inflated_inradius)

    vertices = _inflated_hexagon_vertices(scene_object, inflation_radius)
    return min(
        _segment_to_segment_distance(seg_start, seg_end, vertices[index], vertices[(index + 1) % 6])
        for index in range(6)
    )


def state_clearance(
    q: tuple[float, float, float],
    scene: Scene | None = None,
) -> float:
    """Return the minimum signed Link-3 clearance to all inflated objects."""
    active_scene = DEFAULT_SCENE if scene is None else scene
    seg_start, seg_end = _link3_segment(q)
    clearances: list[float] = []

    for scene_object in active_scene.objects:
        if isinstance(scene_object, CircleObject):
            clearances.append(
                _circle_segment_clearance(seg_start, seg_end, scene_object, active_scene.inflation_radius)
            )
        else:
            clearances.append(
                _hexagon_segment_clearance(seg_start, seg_end, scene_object, active_scene.inflation_radius)
            )

    return min(clearances)


def state_valid(
    q: tuple[float, float, float],
    scene: Scene | None = None,
) -> bool:
    """Return whether Link 3 is collision-free with inflated clearance."""
    return state_clearance(q, scene=scene) >= -CLEARANCE_TOLERANCE


def _joint_delta(q_start: tuple[float, float, float], q_goal: tuple[float, float, float]) -> np.ndarray:
    """Return the shortest wrapped joint delta from start to goal."""
    return np.array(
        [normalize_angle(q_goal[index] - q_start[index]) for index in range(3)],
        dtype=float,
    )


def _tool_travel_upper_bound(joint_delta: np.ndarray) -> float:
    """Return a conservative upper bound on tool-point travel in joint interpolation."""
    return (
        (L1 + L2 + L3) * abs(float(joint_delta[0]))
        + (L2 + L3) * abs(float(joint_delta[1]))
        + L3 * abs(float(joint_delta[2]))
    )


def first_collision(
    q_start: tuple[float, float, float],
    q_goal: tuple[float, float, float],
    step_mm: float = 0.5,
    scene: Scene | None = None,
) -> dict[str, object] | None:
    """Return the first sampled collision along a joint-space interpolation, if any."""
    if step_mm <= 0.0:
        raise ValueError("step_mm must be positive.")

    start = np.array(q_start, dtype=float)
    delta = _joint_delta(q_start, q_goal)
    sample_count = max(1, int(ceil(_tool_travel_upper_bound(delta) / step_mm)))

    for sample_index in range(sample_count + 1):
        t = sample_index / sample_count
        q = tuple(float(value) for value in (start + t * delta))
        clearance = state_clearance(q, scene=scene)
        if clearance < -CLEARANCE_TOLERANCE:
            return {
                "sample_index": sample_index,
                "sample_count": sample_count,
                "t": t,
                "q": q,
                "clearance": clearance,
            }

    return None


def check_motion(
    q_start: tuple[float, float, float],
    q_goal: tuple[float, float, float],
    step_mm: float = 0.5,
    scene: Scene | None = None,
) -> bool:
    """Return whether a sampled joint-space motion stays collision-free."""
    return first_collision(q_start, q_goal, step_mm=step_mm, scene=scene) is None
