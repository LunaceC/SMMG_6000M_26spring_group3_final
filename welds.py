"""Deterministic weld-point generation for the planar RRR scene."""

from __future__ import annotations

from math import cos, radians, sin

import numpy as np

from scene import CircleObject, HexagonObject, Scene, WeldPoint


def _normal_from_angle(alpha_deg: float) -> np.ndarray:
    """Return the unit outward normal for a boundary angle in degrees."""
    alpha_rad = radians(alpha_deg)
    return np.array([cos(alpha_rad), sin(alpha_rad)], dtype=float)


def _boundary_radius(scene_object: CircleObject | HexagonObject) -> float:
    """Return the circle radius or hexagon inradius used for weld placement."""
    if isinstance(scene_object, CircleObject):
        return scene_object.radius
    return scene_object.inradius


def _weld_point_id(scene_object: CircleObject | HexagonObject, index: int) -> str:
    """Return the stable weld identifier for an object-local index."""
    return f"{scene_object.id}_W{index}"


def _tool_heading_deg(alpha_deg: float) -> float:
    """Return the Link-3 tip direction for a weld boundary normal angle."""
    return (alpha_deg + 180.0) % 360.0


def generate_weld_points(scene: Scene) -> list[WeldPoint]:
    """Generate weld points in deterministic object order then angle order."""
    weld_points: list[WeldPoint] = []
    for scene_object in scene.objects:
        center = np.array(scene_object.center, dtype=float)
        offset_distance = _boundary_radius(scene_object) + scene.inflation_radius
        for weld_index, alpha_deg in enumerate(scene_object.weld_angles_deg):
            normal = _normal_from_angle(alpha_deg)
            position = center + offset_distance * normal
            weld_points.append(
                WeldPoint(
                    id=_weld_point_id(scene_object, weld_index),
                    object_id=scene_object.id,
                    class_id=scene_object.class_id,
                    alpha_deg=alpha_deg,
                    position=(float(position[0]), float(position[1])),
                    normal=(float(normal[0]), float(normal[1])),
                    tool_heading_deg=_tool_heading_deg(alpha_deg),
                )
            )
    return weld_points
