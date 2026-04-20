"""Static scene geometry for the planar RRR welding project."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin


DEGREES_30 = pi / 6.0
HEXAGON_WELD_ANGLES_DEG = (30.0, 90.0, 150.0, 210.0, 270.0, 330.0)
CIRCLE_WELD_ANGLES_DEG = (0.0, 90.0, 180.0, 270.0)


@dataclass(frozen=True)
class CircleObject:
    """Circular workpiece with fixed weld-point angles."""

    id: str
    class_id: str
    center: tuple[float, float]
    radius: float

    @property
    def weld_angles_deg(self) -> tuple[float, ...]:
        """Return the authoritative weld normal angles in degrees."""
        return CIRCLE_WELD_ANGLES_DEG


@dataclass(frozen=True)
class HexagonObject:
    """Regular hexagonal workpiece parameterized by inradius."""

    id: str
    class_id: str
    center: tuple[float, float]
    inradius: float

    @property
    def weld_angles_deg(self) -> tuple[float, ...]:
        """Return the authoritative weld normal angles in degrees."""
        return HEXAGON_WELD_ANGLES_DEG

    @property
    def circumradius(self) -> float:
        """Return the vertex radius implied by the inradius."""
        return self.inradius / cos(DEGREES_30)

    def vertices(self) -> tuple[tuple[float, float], ...]:
        """Return vertices in deterministic counter-clockwise order."""
        cx, cy = self.center
        radius = self.circumradius
        return tuple(
            (
                cx + radius * cos(pi / 3.0 * vertex_index),
                cy + radius * sin(pi / 3.0 * vertex_index),
            )
            for vertex_index in range(6)
        )


@dataclass(frozen=True)
class WeldPoint:
    """Weld-point geometry derived from a scene object."""

    id: str
    object_id: str
    class_id: str
    alpha_deg: float
    position: tuple[float, float]
    normal: tuple[float, float]
    tool_heading_deg: float


@dataclass(frozen=True)
class Scene:
    """Complete static scene description used by geometry modules."""

    depot: tuple[float, float]
    link_lengths: tuple[float, float, float]
    torch_diameter: float
    clearance: float
    inflation_radius: float
    objects: tuple[CircleObject | HexagonObject, ...]


def build_default_scene() -> Scene:
    """Return the authoritative default project scene."""
    return Scene(
        depot=(300.0, 0.0),
        link_lengths=(170.0, 160.0, 10.0),
        torch_diameter=10.0,
        clearance=1.0,
        inflation_radius=6.0,
        objects=(
            CircleObject(id="C1", class_id="class1", center=(60.0, 140.0), radius=25.0),
            HexagonObject(id="P1", class_id="class2", center=(150.0, 140.0), inradius=20.0),
            CircleObject(id="C2", class_id="class3", center=(240.0, 140.0), radius=35.0),
            CircleObject(id="C3", class_id="class4", center=(150.0, 40.0), radius=30.0),
            HexagonObject(id="P2", class_id="class5", center=(250.0, 40.0), inradius=20.0),
        ),
    )
