"""Task 2 Cartesian workspace planning on the frozen Hamilton Task 1 sequence."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
import json
from math import atan2, ceil, cos, pi, radians, sin
from pathlib import Path

from collision import check_motion, state_valid
from kinematics import ik_pose, normalize_angle
from scene import CircleObject, HexagonObject, Scene, WeldPoint, build_default_scene
from welds import generate_weld_points


TASK1_SOLUTION_PATH = Path("artifacts/task1_hamiltonian_solution.json")
TASK2_CARTESIAN_WORKSPACE_ARTIFACT_PATH = Path("artifacts/task2_cartesian_workspace_paths.json")
TASK2_CARTESIAN_LIFTED_ARTIFACT_PATH = Path("artifacts/task2_cartesian_lifted_paths.json")
TASK2_CARTESIAN_ARTIFACT_PATH = Path("artifacts/task2_cartesian_paths.json")
TASK2_CARTESIAN_REPAIR_REPORT_PATH = Path("artifacts/task2_cartesian_repair_report.json")
CIRCLE_POLYGON_SIDES = 48
TORCH_TIP_INFLATION_RADIUS_MM = 6.0
LIFTABILITY_BUFFER_EXTRA_MM = 4.0
SAME_OBJECT_CONTOUR_REPAIR_EXTRAS_MM = (8.0, 12.0, 16.0)
VISIBILITY_SAMPLE_STEP_MM = 2.0
CARTESIAN_WAYPOINT_SPACING_MM = 5.0
CONTOUR_INTERPOLATOR_STEP_MM = 2.0
CONTOUR_INTERPOLATOR_MIN_STEP_MM = 0.25
CONTOUR_INTERPOLATOR_MAX_SUBDIVISIONS = 5
CONTOUR_JOINT_JUMP_L1_THRESHOLD = 1.2
ENDPOINT_BLEND_DISTANCE_MM = 15.0
APPROACH_RETREAT_DISTANCE_MM = 12.0
ANCHOR_OFFSET_MM = 1.0
LOCAL_REPAIR_MAX_FORBIDDEN_EDGES = 8
LOCAL_REPAIR_HUBS = (
    (20.0, -60.0),
    (80.0, -60.0),
    (150.0, -60.0),
    (220.0, -60.0),
    (300.0, -60.0),
    (20.0, 200.0),
    (150.0, 200.0),
    (300.0, 200.0),
)
LOCAL_REPAIR_GRID_STEP_MM = 20.0
LOCAL_REPAIR_GRID_BOUNDS = (-20.0, 340.0, -80.0, 220.0)
LIFT_REPAIR_CONFIGS = (
    (CARTESIAN_WAYPOINT_SPACING_MM, ENDPOINT_BLEND_DISTANCE_MM, "default", "tangent"),
    (CARTESIAN_WAYPOINT_SPACING_MM, 10.0, "blend10", "tangent"),
    (3.0, 10.0, "spacing3_blend10", "tangent"),
    (3.0, 5.0, "spacing3_blend5", "tangent"),
    (3.0, 0.0, "spacing3_blend0", "tangent"),
    (CARTESIAN_WAYPOINT_SPACING_MM, 0.0, "weld_interp", "weld_interp"),
    (3.0, 0.0, "spacing3_weld_interp", "weld_interp"),
)


@dataclass(frozen=True)
class WorkspaceObstacle:
    """One inflated polygonal workspace obstacle."""

    obstacle_id: str
    source_kind: str
    vertices: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class GraphNode:
    """One deterministic workspace graph node."""

    node_id: str
    point: tuple[float, float]
    obstacle_id: str | None = None
    vertex_index: int | None = None


@dataclass(frozen=True)
class SegmentCandidate:
    """One workspace segment candidate after IK lifting."""

    workspace_polyline: tuple[tuple[float, float], ...]
    workspace_path_length: float
    cartesian_waypoints: tuple[tuple[float, float, float], ...]
    chosen_joint_waypoints: tuple[tuple[float, float, float], ...]
    source_state_id: str
    target_state_id: str
    validation_result: bool
    local_repair_used: bool
    local_repair_reason: str | None
    joint_space_path_length: float


def _read_json(path: str | Path) -> dict[str, object]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_task1_hamiltonian_solution(path: str | Path = TASK1_SOLUTION_PATH) -> dict[str, object]:
    """Load the active Task 1 Hamiltonian artifact."""
    return _read_json(path)


def build_task2_visit_sequence(task1_solution: dict[str, object]) -> list[str]:
    """Return the exact depot-anchored visit sequence from Task 1."""
    cycle = list(task1_solution["cycle_node_order"])
    if cycle[0] != "DEPOT" or cycle[-1] != "DEPOT":
        raise ValueError("Task 1 cycle must start and end at DEPOT.")
    return cycle


def _weld_lookup(scene: Scene) -> dict[str, WeldPoint]:
    return {weld.id: weld for weld in generate_weld_points(scene)}


def _distance(point_a: tuple[float, float], point_b: tuple[float, float]) -> float:
    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    return (dx * dx + dy * dy) ** 0.5


def _polyline_length(polyline: tuple[tuple[float, float], ...]) -> float:
    return sum(_distance(polyline[index], polyline[index + 1]) for index in range(len(polyline) - 1))


def _wrap_angle_distance(angle_a: float, angle_b: float) -> float:
    return abs(normalize_angle(angle_b - angle_a))


def _wrap_angle_lerp(angle_a: float, angle_b: float, alpha: float) -> float:
    return normalize_angle(angle_a + alpha * normalize_angle(angle_b - angle_a))


def _segment_point_samples(
    point_a: tuple[float, float],
    point_b: tuple[float, float],
    *,
    step_mm: float = VISIBILITY_SAMPLE_STEP_MM,
) -> list[tuple[float, float]]:
    length = _distance(point_a, point_b)
    if length <= 0.0:
        return [point_a]
    sample_count = max(1, int(length / step_mm))
    return [
        (
            point_a[0] + (point_b[0] - point_a[0]) * (sample_index / sample_count),
            point_a[1] + (point_b[1] - point_a[1]) * (sample_index / sample_count),
        )
        for sample_index in range(sample_count + 1)
    ]


def _point_to_segment_distance(
    point: tuple[float, float],
    seg_a: tuple[float, float],
    seg_b: tuple[float, float],
) -> float:
    ax, ay = seg_a
    bx, by = seg_b
    px, py = point
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 0.0:
        return _distance(point, seg_a)
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = max(0.0, min(1.0, t))
    closest = (ax + t * dx, ay + t * dy)
    return _distance(point, closest)


def _point_on_polygon_boundary(
    point: tuple[float, float],
    vertices: tuple[tuple[float, float], ...],
    *,
    tolerance: float = 1e-6,
) -> bool:
    for index in range(len(vertices)):
        if _point_to_segment_distance(point, vertices[index], vertices[(index + 1) % len(vertices)]) <= tolerance:
            return True
    return False


def _point_in_polygon_strict(
    point: tuple[float, float],
    vertices: tuple[tuple[float, float], ...],
    *,
    tolerance: float = 1e-6,
) -> bool:
    if _point_on_polygon_boundary(point, vertices, tolerance=tolerance):
        return False
    x_pos, y_pos = point
    inside = False
    for index in range(len(vertices)):
        x0, y0 = vertices[index]
        x1, y1 = vertices[(index + 1) % len(vertices)]
        if ((y0 > y_pos) != (y1 > y_pos)) and (
            x_pos < (x1 - x0) * (y_pos - y0) / ((y1 - y0) + 1e-12) + x0
        ):
            inside = not inside
    return inside


def build_inflated_workspace_obstacles(
    scene: Scene | None = None,
    *,
    circle_polygon_sides: int = CIRCLE_POLYGON_SIDES,
    inflation_radius_mm: float = TORCH_TIP_INFLATION_RADIUS_MM,
) -> list[WorkspaceObstacle]:
    """Build deterministic inflated polygonal workspace obstacles for torch-tip planning."""
    active_scene = build_default_scene() if scene is None else scene
    obstacles: list[WorkspaceObstacle] = []
    for scene_object in active_scene.objects:
        if isinstance(scene_object, CircleObject):
            radius = scene_object.radius + inflation_radius_mm
            vertices = tuple(
                (
                    scene_object.center[0] + radius * cos(2.0 * pi * index / circle_polygon_sides),
                    scene_object.center[1] + radius * sin(2.0 * pi * index / circle_polygon_sides),
                )
                for index in range(circle_polygon_sides)
            )
            obstacles.append(
                WorkspaceObstacle(
                    obstacle_id=scene_object.id,
                    source_kind="circle_polygonized",
                    vertices=vertices,
                )
            )
            continue

        inflated_inradius = scene_object.inradius + inflation_radius_mm
        inflated_circumradius = inflated_inradius / cos(radians(30.0))
        vertices = tuple(
            (
                scene_object.center[0] + inflated_circumradius * cos(radians(60.0 * vertex_index)),
                scene_object.center[1] + inflated_circumradius * sin(radians(60.0 * vertex_index)),
            )
            for vertex_index in range(6)
        )
        obstacles.append(
            WorkspaceObstacle(
                obstacle_id=scene_object.id,
                source_kind="hexagon",
                vertices=vertices,
            )
        )
    return obstacles


def _segment_collision_free(
    point_a: tuple[float, float],
    point_b: tuple[float, float],
    obstacles: list[WorkspaceObstacle],
) -> bool:
    for sample_point in _segment_point_samples(point_a, point_b):
        for obstacle in obstacles:
            if _point_in_polygon_strict(sample_point, obstacle.vertices):
                return False
    return True


def build_visibility_roadmap(
    obstacles: list[WorkspaceObstacle],
) -> tuple[dict[str, GraphNode], dict[str, dict[str, float]]]:
    """Build one deterministic visibility roadmap over obstacle vertices only."""
    nodes: dict[str, GraphNode] = {}
    adjacency: dict[str, dict[str, float]] = {}

    for obstacle in obstacles:
        for vertex_index, vertex in enumerate(obstacle.vertices):
            node_id = f"{obstacle.obstacle_id}_V{vertex_index}"
            nodes[node_id] = GraphNode(
                node_id=node_id,
                point=vertex,
                obstacle_id=obstacle.obstacle_id,
                vertex_index=vertex_index,
            )
            adjacency[node_id] = {}

    def add_edge(node_a: str, node_b: str) -> None:
        if node_b in adjacency[node_a]:
            return
        length = _distance(nodes[node_a].point, nodes[node_b].point)
        adjacency[node_a][node_b] = length
        adjacency[node_b][node_a] = length

    for obstacle in obstacles:
        vertex_count = len(obstacle.vertices)
        for vertex_index in range(vertex_count):
            node_a = f"{obstacle.obstacle_id}_V{vertex_index}"
            node_b = f"{obstacle.obstacle_id}_V{(vertex_index + 1) % vertex_count}"
            add_edge(node_a, node_b)

    node_ids = sorted(nodes.keys())
    for left_index, node_a in enumerate(node_ids):
        for node_b in node_ids[left_index + 1:]:
            if nodes[node_a].obstacle_id == nodes[node_b].obstacle_id:
                vertex_count = len(next(obstacle.vertices for obstacle in obstacles if obstacle.obstacle_id == nodes[node_a].obstacle_id))
                delta = abs(int(nodes[node_a].vertex_index) - int(nodes[node_b].vertex_index))
                if delta == 1 or delta == vertex_count - 1:
                    continue
            if not _segment_collision_free(nodes[node_a].point, nodes[node_b].point, obstacles):
                continue
            add_edge(node_a, node_b)

    return nodes, adjacency


def _visit_point_and_obstacle(
    label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
) -> tuple[tuple[float, float], str | None]:
    if label == "DEPOT":
        return scene.depot, None
    weld = weld_lookup[label]
    return weld.position, weld.object_id


def _host_boundary_vertex_links(
    point: tuple[float, float],
    obstacle: WorkspaceObstacle,
) -> list[tuple[str, tuple[float, float]]]:
    links: list[tuple[str, tuple[float, float]]] = []
    vertex_count = len(obstacle.vertices)
    for vertex_index in range(vertex_count):
        edge_a = obstacle.vertices[vertex_index]
        edge_b = obstacle.vertices[(vertex_index + 1) % vertex_count]
        if _point_to_segment_distance(point, edge_a, edge_b) > 1e-4:
            continue
        links.append((f"{obstacle.obstacle_id}_V{vertex_index}", edge_a))
        links.append((f"{obstacle.obstacle_id}_V{(vertex_index + 1) % vertex_count}", edge_b))
    deduped: dict[str, tuple[float, float]] = {}
    for node_id, vertex in links:
        deduped[node_id] = vertex
    return sorted(deduped.items())


def _dijkstra_shortest_path(
    adjacency: dict[str, dict[str, float]],
    points: dict[str, tuple[float, float]],
    start_id: str,
    goal_id: str,
    *,
    forbidden_edges: frozenset[tuple[str, str]] = frozenset(),
) -> list[str] | None:
    frontier: list[tuple[float, float, int, str]] = []
    queue_counter = count()
    heappush(frontier, (0.0, 0.0, next(queue_counter), start_id))
    best_cost = {start_id: 0.0}
    parent: dict[str, str | None] = {start_id: None}

    while frontier:
        _, current_g, _, current_id = heappop(frontier)
        if current_g > best_cost.get(current_id, float("inf")):
            continue
        if current_id == goal_id:
            path = [goal_id]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            path.reverse()
            return path
        for neighbor_id, edge_cost in adjacency[current_id].items():
            edge_key = tuple(sorted((current_id, neighbor_id)))
            if edge_key in forbidden_edges:
                continue
            tentative_cost = current_g + edge_cost
            if tentative_cost >= best_cost.get(neighbor_id, float("inf")):
                continue
            best_cost[neighbor_id] = tentative_cost
            parent[neighbor_id] = current_id
            heuristic = _distance(points[neighbor_id], points[goal_id])
            heappush(frontier, (tentative_cost + heuristic, tentative_cost, next(queue_counter), neighbor_id))
    return None


def _compress_polyline(polyline: list[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    compressed: list[tuple[float, float]] = []
    for point in polyline:
        if compressed and _distance(compressed[-1], point) <= 1e-9:
            continue
        compressed.append(point)
    return tuple(compressed)


def _compress_collinear_polyline(
    polyline: tuple[tuple[float, float], ...],
    *,
    tolerance: float = 1e-6,
) -> tuple[tuple[float, float], ...]:
    if len(polyline) <= 2:
        return polyline
    compressed = [polyline[0]]
    for index in range(1, len(polyline) - 1):
        a_point = compressed[-1]
        b_point = polyline[index]
        c_point = polyline[index + 1]
        cross_value = (b_point[0] - a_point[0]) * (c_point[1] - b_point[1]) - (b_point[1] - a_point[1]) * (c_point[0] - b_point[0])
        if abs(cross_value) <= tolerance:
            continue
        compressed.append(b_point)
    compressed.append(polyline[-1])
    return tuple(compressed)


def _obstacle_center_and_radius(obstacle: WorkspaceObstacle) -> tuple[tuple[float, float], float]:
    x_coords = [vertex[0] for vertex in obstacle.vertices]
    y_coords = [vertex[1] for vertex in obstacle.vertices]
    center = (sum(x_coords) / len(x_coords), sum(y_coords) / len(y_coords))
    radius = max(_distance(center, vertex) for vertex in obstacle.vertices)
    return center, radius


def _candidate_local_repair_hubs(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    obstacles: list[WorkspaceObstacle],
) -> tuple[tuple[float, float], ...]:
    source_point, source_obstacle_id = _visit_point_and_obstacle(source_label, scene=scene, weld_lookup=weld_lookup)
    target_point, target_obstacle_id = _visit_point_and_obstacle(target_label, scene=scene, weld_lookup=weld_lookup)

    hub_points: list[tuple[float, float]] = []
    obstacle_lookup = {obstacle.obstacle_id: obstacle for obstacle in obstacles}
    if source_obstacle_id is not None and source_obstacle_id == target_obstacle_id:
        obstacle = obstacle_lookup[source_obstacle_id]
        center, radius = _obstacle_center_and_radius(obstacle)
        source_angle = atan2(source_point[1] - center[1], source_point[0] - center[0])
        target_angle = atan2(target_point[1] - center[1], target_point[0] - center[0])
        midpoint_angle = normalize_angle(source_angle + 0.5 * normalize_angle(target_angle - source_angle))
        for offset in (radius + 24.0, radius + 40.0, radius + 90.0):
            for delta in (0.0, pi / 8.0, -pi / 8.0, pi / 4.0, -pi / 4.0):
                angle = midpoint_angle + delta
                hub_points.append((center[0] + offset * cos(angle), center[1] + offset * sin(angle)))
    else:
        hub_points.extend(LOCAL_REPAIR_HUBS)
    if not (source_obstacle_id is not None and source_obstacle_id == target_obstacle_id):
        for obstacle_id in sorted({source_obstacle_id, target_obstacle_id} - {None}):
            obstacle = obstacle_lookup[obstacle_id]
            center, radius = _obstacle_center_and_radius(obstacle)
            for offset in (radius + 24.0, radius + 40.0, radius + 90.0):
                for angle_index in range(16):
                    angle = 2.0 * pi * angle_index / 16.0
                    hub_points.append((center[0] + offset * cos(angle), center[1] + offset * sin(angle)))

    deduped: list[tuple[float, float]] = []
    for point in hub_points:
        rounded = (round(point[0], 6), round(point[1], 6))
        if rounded in deduped:
            continue
        deduped.append(rounded)
    return tuple(deduped)


def _full_ring_hubs_for_obstacle(
    obstacle: WorkspaceObstacle,
) -> tuple[tuple[float, float], ...]:
    center, radius = _obstacle_center_and_radius(obstacle)
    hub_points: list[tuple[float, float]] = []
    for offset in (radius + 24.0, radius + 40.0, radius + 90.0):
        for angle_index in range(16):
            angle = 2.0 * pi * angle_index / 16.0
            hub_points.append((center[0] + offset * cos(angle), center[1] + offset * sin(angle)))
    return tuple(hub_points)


def _scene_object_lookup(scene: Scene) -> dict[str, CircleObject | HexagonObject]:
    return {scene_object.id: scene_object for scene_object in scene.objects}


def classify_task2_segment_type(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
) -> str:
    if source_label == "DEPOT" or target_label == "DEPOT":
        return "inter_object"
    source_weld = weld_lookup[source_label]
    target_weld = weld_lookup[target_label]
    return "same_object_neighbor" if source_weld.object_id == target_weld.object_id else "inter_object"


def _circle_arc_polyline(
    source_point: tuple[float, float],
    target_point: tuple[float, float],
    *,
    center: tuple[float, float],
    radius: float,
    spacing_mm: float = CARTESIAN_WAYPOINT_SPACING_MM,
) -> tuple[tuple[float, float], ...]:
    start_angle = atan2(source_point[1] - center[1], source_point[0] - center[0])
    end_angle = atan2(target_point[1] - center[1], target_point[0] - center[0])
    delta = normalize_angle(end_angle - start_angle)
    arc_length = abs(delta) * radius
    sample_count = max(1, int(arc_length / spacing_mm))
    polyline = [source_point]
    for sample_index in range(1, sample_count):
        angle = start_angle + delta * (sample_index / sample_count)
        polyline.append((center[0] + radius * cos(angle), center[1] + radius * sin(angle)))
    polyline.append(target_point)
    return _compress_polyline(polyline)


def _polygon_boundary_direction_polyline(
    point_a: tuple[float, float],
    point_b: tuple[float, float],
    obstacle: WorkspaceObstacle,
    *,
    direction: int,
) -> tuple[tuple[float, float], ...] | None:
    links_a = _host_boundary_vertex_links(point_a, obstacle)
    links_b = _host_boundary_vertex_links(point_b, obstacle)
    if not links_a or not links_b:
        return None
    vertex_count = len(obstacle.vertices)
    best: tuple[float, tuple[tuple[float, float], ...]] | None = None
    for node_a, vertex_a in links_a:
        idx_a = int(node_a.split("_V")[1])
        for node_b, vertex_b in links_b:
            idx_b = int(node_b.split("_V")[1])
            path = [point_a]
            if _distance(point_a, vertex_a) > 1e-9:
                path.append(vertex_a)
            current = idx_a
            while current != idx_b:
                current = (current + direction) % vertex_count
                vertex = obstacle.vertices[current]
                if _distance(path[-1], vertex) > 1e-9:
                    path.append(vertex)
            if _distance(path[-1], point_b) > 1e-9:
                path.append(point_b)
            polyline = _compress_collinear_polyline(tuple(path))
            cost = _polyline_length(polyline)
            if best is None or cost < best[0]:
                best = (cost, polyline)
    return None if best is None else best[1]


def contour_following_polyline_for_segment(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    obstacles: list[WorkspaceObstacle],
) -> tuple[tuple[tuple[float, float], ...], str]:
    source_point, source_obstacle_id = _visit_point_and_obstacle(source_label, scene=scene, weld_lookup=weld_lookup)
    target_point, target_obstacle_id = _visit_point_and_obstacle(target_label, scene=scene, weld_lookup=weld_lookup)
    if source_obstacle_id is None or source_obstacle_id != target_obstacle_id:
        raise ValueError("Contour following requires a same-object segment.")
    object_lookup = _scene_object_lookup(scene)
    obstacle_lookup = {obstacle.obstacle_id: obstacle for obstacle in obstacles}
    scene_object = object_lookup[source_obstacle_id]
    if isinstance(scene_object, CircleObject):
        polyline = _circle_arc_polyline(
            source_point,
            target_point,
            center=scene_object.center,
            radius=scene_object.radius + TORCH_TIP_INFLATION_RADIUS_MM,
        )
        return polyline, "contour_circle_arc"

    obstacle = obstacle_lookup[source_obstacle_id]
    clockwise = _polygon_boundary_direction_polyline(source_point, target_point, obstacle, direction=1)
    counter_clockwise = _polygon_boundary_direction_polyline(source_point, target_point, obstacle, direction=-1)
    candidates = [poly for poly in (clockwise, counter_clockwise) if poly is not None]
    if not candidates:
        raise ValueError(f"No contour-following polygon path for {source_label} -> {target_label}.")
    polyline = min(candidates, key=_polyline_length)
    return polyline, "contour_polygon_boundary"


def _try_direct_hub_repairs(
    hub_points: tuple[tuple[float, float], ...],
    source_label: str,
    target_label: str,
    *,
    weld_lookup: dict[str, WeldPoint],
    obstacles: list[WorkspaceObstacle],
    source_visit_for_ids: str,
    target_visit_for_ids: str,
    source_point: tuple[float, float],
    target_point: tuple[float, float],
    hub_name_prefix: str,
) -> tuple[tuple[tuple[float, float], ...], tuple[tuple[float, float, float], ...], list[SegmentCandidate], bool, str | None] | None:
    forced_hub_candidates: list[tuple[float, tuple[tuple[float, float], ...], str]] = []
    for hub_index, hub_point in enumerate(hub_points):
        if not _segment_collision_free(source_point, hub_point, obstacles):
            continue
        if not _segment_collision_free(hub_point, target_point, obstacles):
            continue
        polyline = _compress_collinear_polyline((source_point, hub_point, target_point))
        forced_hub_candidates.append((_polyline_length(polyline), polyline, f"{hub_name_prefix}_{hub_index}"))
    forced_hub_candidates.sort(key=lambda item: item[0])
    for _, hub_polyline, hub_reason in forced_hub_candidates:
        for spacing_mm, blend_distance_mm, config_name, heading_mode in (
            (CARTESIAN_WAYPOINT_SPACING_MM, ENDPOINT_BLEND_DISTANCE_MM, "default", "tangent"),
            (CARTESIAN_WAYPOINT_SPACING_MM, 10.0, "blend10", "tangent"),
            (CARTESIAN_WAYPOINT_SPACING_MM, 0.0, "weld_interp", "weld_interp"),
        ):
            hub_waypoints = polyline_to_cartesian_waypoints(
                hub_polyline,
                source_label=source_label,
                target_label=target_label,
                weld_lookup=weld_lookup,
                spacing_mm=spacing_mm,
                blend_distance_mm=blend_distance_mm,
                heading_mode=heading_mode,
            )
            hub_realizations = _lift_cartesian_segment(
                source_label,
                target_label,
                hub_waypoints,
                source_state_id_label=source_visit_for_ids,
                target_state_id_label=target_visit_for_ids,
            )
            if not hub_realizations:
                continue
            return (
                hub_polyline,
                hub_waypoints,
                [
                    SegmentCandidate(
                        workspace_polyline=realization.workspace_polyline,
                        workspace_path_length=realization.workspace_path_length,
                        cartesian_waypoints=realization.cartesian_waypoints,
                        chosen_joint_waypoints=realization.chosen_joint_waypoints,
                        source_state_id=realization.source_state_id,
                        target_state_id=realization.target_state_id,
                        validation_result=realization.validation_result,
                        local_repair_used=True,
                        local_repair_reason=(
                            hub_reason
                            if config_name == "default"
                            else f"{hub_reason}_{config_name}"
                        ),
                        joint_space_path_length=realization.joint_space_path_length,
                    )
                    for realization in hub_realizations
                ],
                True,
                hub_reason,
            )
    return None


def _try_forced_hub_repairs(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    obstacles: list[WorkspaceObstacle],
    source_visit_for_ids: str,
    target_visit_for_ids: str,
    source_point: tuple[float, float],
    target_point: tuple[float, float],
) -> tuple[tuple[tuple[float, float], ...], tuple[tuple[float, float, float], ...], list[SegmentCandidate], bool, str | None] | None:
    forced_hubs = _candidate_local_repair_hubs(
        source_label,
        target_label,
        scene=scene,
        weld_lookup=weld_lookup,
        obstacles=obstacles,
    )
    return _try_direct_hub_repairs(
        forced_hubs,
        source_label,
        target_label,
        weld_lookup=weld_lookup,
        obstacles=obstacles,
        source_visit_for_ids=source_visit_for_ids,
        target_visit_for_ids=target_visit_for_ids,
        source_point=source_point,
        target_point=target_point,
        hub_name_prefix="forced_hub",
    )


def shortest_workspace_polyline_for_segment(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    obstacles: list[WorkspaceObstacle],
    roadmap_nodes: dict[str, GraphNode],
    roadmap_adjacency: dict[str, dict[str, float]],
    forbidden_edges: frozenset[tuple[str, str]] = frozenset(),
    extra_points: dict[str, tuple[float, float]] | None = None,
) -> tuple[tuple[tuple[float, float], ...], list[str]] | None:
    """Return one shortest workspace polyline for one fixed Task 1 segment."""
    source_point, source_obstacle_id = _visit_point_and_obstacle(source_label, scene=scene, weld_lookup=weld_lookup)
    target_point, target_obstacle_id = _visit_point_and_obstacle(target_label, scene=scene, weld_lookup=weld_lookup)

    points = {node_id: node.point for node_id, node in roadmap_nodes.items()}
    adjacency = {node_id: dict(neighbors) for node_id, neighbors in roadmap_adjacency.items()}
    if extra_points is not None:
        for node_id, point in extra_points.items():
            points[node_id] = point
            adjacency[node_id] = {}
    points["START"] = source_point
    points["GOAL"] = target_point
    adjacency["START"] = {}
    adjacency["GOAL"] = {}

    def add_edge(node_a: str, node_b: str) -> None:
        if node_b in adjacency[node_a]:
            return
        length = _distance(points[node_a], points[node_b])
        adjacency[node_a][node_b] = length
        adjacency[node_b][node_a] = length

    if _segment_collision_free(source_point, target_point, obstacles):
        add_edge("START", "GOAL")

    obstacle_lookup = {obstacle.obstacle_id: obstacle for obstacle in obstacles}
    if source_obstacle_id is not None:
        for node_id, _ in _host_boundary_vertex_links(source_point, obstacle_lookup[source_obstacle_id]):
            add_edge("START", node_id)
    if target_obstacle_id is not None:
        for node_id, _ in _host_boundary_vertex_links(target_point, obstacle_lookup[target_obstacle_id]):
            add_edge("GOAL", node_id)

    for node_id, point in roadmap_nodes.items():
        if _segment_collision_free(source_point, point.point, obstacles):
            add_edge("START", node_id)
        if _segment_collision_free(target_point, point.point, obstacles):
            add_edge("GOAL", node_id)
    if extra_points is not None:
        extra_ids = sorted(extra_points.keys())
        for extra_id in extra_ids:
            extra_point = extra_points[extra_id]
            if _segment_collision_free(source_point, extra_point, obstacles):
                add_edge("START", extra_id)
            if _segment_collision_free(target_point, extra_point, obstacles):
                add_edge("GOAL", extra_id)
            for node_id, point in roadmap_nodes.items():
                if _segment_collision_free(extra_point, point.point, obstacles):
                    add_edge(extra_id, node_id)
        for left_index, node_a in enumerate(extra_ids):
            for node_b in extra_ids[left_index + 1:]:
                if _segment_collision_free(extra_points[node_a], extra_points[node_b], obstacles):
                    add_edge(node_a, node_b)

    path_ids = _dijkstra_shortest_path(adjacency, points, "START", "GOAL", forbidden_edges=forbidden_edges)
    if path_ids is None:
        return None

    polyline = [points[node_id] for node_id in path_ids]
    return _compress_polyline(polyline), path_ids


def shortest_workspace_polyline_between_points(
    source_point: tuple[float, float],
    target_point: tuple[float, float],
    *,
    obstacles: list[WorkspaceObstacle],
    roadmap_nodes: dict[str, GraphNode],
    roadmap_adjacency: dict[str, dict[str, float]],
    forbidden_edges: frozenset[tuple[str, str]] = frozenset(),
    extra_points: dict[str, tuple[float, float]] | None = None,
) -> tuple[tuple[tuple[float, float], ...], list[str]] | None:
    points = {node_id: node.point for node_id, node in roadmap_nodes.items()}
    adjacency = {node_id: dict(neighbors) for node_id, neighbors in roadmap_adjacency.items()}
    if extra_points is not None:
        for node_id, point in extra_points.items():
            points[node_id] = point
            adjacency[node_id] = {}
    points["START"] = source_point
    points["GOAL"] = target_point
    adjacency["START"] = {}
    adjacency["GOAL"] = {}

    def add_edge(node_a: str, node_b: str) -> None:
        if node_b in adjacency[node_a]:
            return
        length = _distance(points[node_a], points[node_b])
        adjacency[node_a][node_b] = length
        adjacency[node_b][node_a] = length

    if _segment_collision_free(source_point, target_point, obstacles):
        add_edge("START", "GOAL")
    for node_id, node in roadmap_nodes.items():
        if _segment_collision_free(source_point, node.point, obstacles):
            add_edge("START", node_id)
        if _segment_collision_free(target_point, node.point, obstacles):
            add_edge("GOAL", node_id)
    if extra_points is not None:
        extra_ids = sorted(extra_points.keys())
        for extra_id in extra_ids:
            extra_point = extra_points[extra_id]
            if _segment_collision_free(source_point, extra_point, obstacles):
                add_edge("START", extra_id)
            if _segment_collision_free(target_point, extra_point, obstacles):
                add_edge("GOAL", extra_id)
            for node_id, node in roadmap_nodes.items():
                if _segment_collision_free(extra_point, node.point, obstacles):
                    add_edge(extra_id, node_id)
        for left_index, node_a in enumerate(extra_ids):
            for node_b in extra_ids[left_index + 1:]:
                if _segment_collision_free(extra_points[node_a], extra_points[node_b], obstacles):
                    add_edge(node_a, node_b)

    path_ids = _dijkstra_shortest_path(adjacency, points, "START", "GOAL", forbidden_edges=forbidden_edges)
    if path_ids is None:
        return None
    polyline = [points[node_id] for node_id in path_ids]
    return _compress_polyline(polyline), path_ids


def _path_tangent(polyline: tuple[tuple[float, float], ...], index: int) -> float:
    if len(polyline) == 1:
        return 0.0
    if index == 0:
        neighbor = polyline[1]
        point = polyline[0]
    elif index == len(polyline) - 1:
        neighbor = polyline[-1]
        point = polyline[-2]
    else:
        point = polyline[index - 1]
        neighbor = polyline[index + 1]
    return atan2(neighbor[1] - point[1], neighbor[0] - point[0])


def polyline_to_cartesian_waypoints(
    polyline: tuple[tuple[float, float], ...],
    *,
    source_label: str,
    target_label: str,
    weld_lookup: dict[str, WeldPoint],
    spacing_mm: float = CARTESIAN_WAYPOINT_SPACING_MM,
    blend_distance_mm: float = ENDPOINT_BLEND_DISTANCE_MM,
    heading_mode: str = "tangent",
) -> tuple[tuple[float, float, float], ...]:
    """Discretize one workspace polyline into Cartesian waypoints with deterministic headings."""
    cumulative_points: list[tuple[tuple[float, float], float]] = [(polyline[0], 0.0)]
    running_length = 0.0
    for point_a, point_b in zip(polyline[:-1], polyline[1:], strict=True):
        edge_length = _distance(point_a, point_b)
        if edge_length <= 0.0:
            continue
        sample_count = max(1, int(edge_length / spacing_mm))
        for sample_index in range(1, sample_count + 1):
            t_value = sample_index / sample_count
            point = (
                point_a[0] + (point_b[0] - point_a[0]) * t_value,
                point_a[1] + (point_b[1] - point_a[1]) * t_value,
            )
            cumulative_points.append((point, running_length + edge_length * t_value))
        running_length += edge_length

    total_length = running_length
    source_heading = None if source_label == "DEPOT" else radians(weld_lookup[source_label].tool_heading_deg)
    target_heading = None if target_label == "DEPOT" else radians(weld_lookup[target_label].tool_heading_deg)

    positions = [point for point, _ in cumulative_points]
    waypoint_records: list[tuple[float, float, float]] = []
    for index, (point, arc_length) in enumerate(cumulative_points):
        tangent_heading = _path_tangent(tuple(positions), index)
        if heading_mode == "weld_interp" and source_heading is not None and target_heading is not None and total_length > 1e-9:
            heading = _wrap_angle_lerp(source_heading, target_heading, arc_length / total_length)
        else:
            heading = tangent_heading
            if source_heading is not None and arc_length < blend_distance_mm:
                alpha = arc_length / max(blend_distance_mm, 1e-9)
                heading = _wrap_angle_lerp(source_heading, heading, alpha)
            if target_heading is not None and total_length - arc_length < blend_distance_mm:
                alpha = (total_length - arc_length) / max(blend_distance_mm, 1e-9)
                heading = _wrap_angle_lerp(target_heading, heading, alpha)
        if index == 0 and source_heading is not None:
            heading = source_heading
        if index == len(cumulative_points) - 1 and target_heading is not None:
            heading = target_heading
        waypoint_records.append((point[0], point[1], normalize_angle(heading)))
    return tuple(waypoint_records)


def _densify_polyline(
    polyline: tuple[tuple[float, float], ...],
    *,
    max_step_mm: float,
) -> tuple[tuple[float, float], ...]:
    if len(polyline) <= 1:
        return polyline
    densified: list[tuple[float, float]] = [polyline[0]]
    for point_a, point_b in zip(polyline[:-1], polyline[1:], strict=True):
        edge_length = _distance(point_a, point_b)
        if edge_length <= 1e-9:
            continue
        sample_count = max(1, int(ceil(edge_length / max_step_mm)))
        for sample_index in range(1, sample_count + 1):
            t_value = sample_index / sample_count
            densified.append(
                (
                    point_a[0] + (point_b[0] - point_a[0]) * t_value,
                    point_a[1] + (point_b[1] - point_a[1]) * t_value,
                )
            )
    return _compress_polyline(densified)


def _same_object_heading_sign(
    positions: tuple[tuple[float, float], ...],
    *,
    source_heading: float,
) -> float:
    tangent_heading = _path_tangent(positions, 0)
    plus_heading = normalize_angle(tangent_heading + pi / 2.0)
    minus_heading = normalize_angle(tangent_heading - pi / 2.0)
    plus_error = _wrap_angle_distance(source_heading, plus_heading)
    minus_error = _wrap_angle_distance(source_heading, minus_heading)
    return 1.0 if plus_error <= minus_error else -1.0


def _same_object_contour_cartesian_waypoints(
    polyline: tuple[tuple[float, float], ...],
    *,
    source_label: str,
    target_label: str,
    weld_lookup: dict[str, WeldPoint],
    spacing_mm: float = CONTOUR_INTERPOLATOR_STEP_MM,
) -> tuple[tuple[float, float, float], ...]:
    positions = _densify_polyline(polyline, max_step_mm=spacing_mm)
    source_heading = radians(weld_lookup[source_label].tool_heading_deg)
    target_heading = radians(weld_lookup[target_label].tool_heading_deg)
    heading_sign = _same_object_heading_sign(positions, source_heading=source_heading)
    waypoint_records: list[tuple[float, float, float]] = []
    for index, point in enumerate(positions):
        tangent_heading = _path_tangent(positions, index)
        heading = normalize_angle(tangent_heading + heading_sign * pi / 2.0)
        if index == 0:
            heading = source_heading
        elif index == len(positions) - 1:
            heading = target_heading
        waypoint_records.append((point[0], point[1], heading))
    return tuple(waypoint_records)


def _wrapped_joint_l1(q_start: tuple[float, float, float], q_goal: tuple[float, float, float]) -> float:
    return sum(abs(normalize_angle(q_goal[index] - q_start[index])) for index in range(3))


def _state_id_for_visit(
    visit_label: str,
    branch_index: int,
    *,
    source_label: str,
    target_label: str,
    is_source: bool,
) -> str:
    if visit_label != "DEPOT":
        return f"{visit_label}_B{branch_index}"
    return f"{'DEPOT_START' if is_source else 'DEPOT_END'}_B{branch_index}_{source_label}_TO_{target_label}"


def _lift_cartesian_segment(
    source_label: str,
    target_label: str,
    cartesian_waypoints: tuple[tuple[float, float, float], ...],
    *,
    source_state_id_label: str,
    target_state_id_label: str,
) -> list[SegmentCandidate]:
    """Lift one Cartesian waypoint chain to feasible joint realizations with continuity."""
    waypoint_candidates: list[list[tuple[str, tuple[float, float, float]]]] = []
    for waypoint_index, (x_pos, y_pos, phi) in enumerate(cartesian_waypoints):
        ik_solutions = [
            q
            for q in ik_pose(x_pos, y_pos, phi)
            if state_valid(q)
        ]
        candidates_for_waypoint: list[tuple[str, tuple[float, float, float]]] = []
        for branch_index, q in enumerate(ik_solutions):
            if waypoint_index == 0:
                state_id = _state_id_for_visit(
                    source_label,
                    branch_index,
                    source_label=source_state_id_label,
                    target_label=target_state_id_label,
                    is_source=True,
                )
            elif waypoint_index == len(cartesian_waypoints) - 1:
                state_id = _state_id_for_visit(
                    target_label,
                    branch_index,
                    source_label=source_state_id_label,
                    target_label=target_state_id_label,
                    is_source=False,
                )
            else:
                state_id = f"WP{waypoint_index}_B{branch_index}"
            candidates_for_waypoint.append((state_id, q))
        if not candidates_for_waypoint:
            return []
        waypoint_candidates.append(candidates_for_waypoint)

    feasible_segments: list[SegmentCandidate] = []
    for start_state_id, start_q in waypoint_candidates[0]:
        dp_costs: dict[int, float] = {candidate_index: 0.0 for candidate_index, (_, q) in enumerate(waypoint_candidates[0]) if q == start_q}
        predecessors: list[dict[int, int]] = [{}]
        for waypoint_index in range(len(waypoint_candidates) - 1):
            next_costs: dict[int, float] = {}
            next_predecessors: dict[int, int] = {}
            for source_index, source_cost in dp_costs.items():
                source_q = waypoint_candidates[waypoint_index][source_index][1]
                for target_index, (_, target_q) in enumerate(waypoint_candidates[waypoint_index + 1]):
                    if not check_motion(source_q, target_q, step_mm=0.5):
                        continue
                    new_cost = source_cost + _wrapped_joint_l1(source_q, target_q)
                    if new_cost >= next_costs.get(target_index, float("inf")):
                        continue
                    next_costs[target_index] = new_cost
                    next_predecessors[target_index] = source_index
            if not next_costs:
                dp_costs = {}
                break
            dp_costs = next_costs
            predecessors.append(next_predecessors)
        if not dp_costs:
            continue

        final_index = min(dp_costs, key=dp_costs.get)
        target_state_id, _ = waypoint_candidates[-1][final_index]
        index_chain = [final_index]
        current_index = final_index
        for waypoint_index in range(len(waypoint_candidates) - 1, 0, -1):
            current_index = predecessors[waypoint_index][current_index]
            index_chain.append(current_index)
        index_chain.reverse()
        chosen_joint_waypoints = tuple(
            waypoint_candidates[waypoint_index][index_chain[waypoint_index]][1]
            for waypoint_index in range(len(waypoint_candidates))
        )
        feasible_segments.append(
            SegmentCandidate(
                workspace_polyline=tuple((x_pos, y_pos) for x_pos, y_pos, _ in cartesian_waypoints),
                workspace_path_length=_polyline_length(tuple((x_pos, y_pos) for x_pos, y_pos, _ in cartesian_waypoints)),
                cartesian_waypoints=cartesian_waypoints,
                chosen_joint_waypoints=chosen_joint_waypoints,
                source_state_id=start_state_id,
                target_state_id=target_state_id,
                validation_result=True,
                local_repair_used=False,
                local_repair_reason=None,
                joint_space_path_length=dp_costs[final_index],
            )
        )
    return feasible_segments


def _lift_cartesian_segment_greedy(
    source_label: str,
    target_label: str,
    cartesian_waypoints: tuple[tuple[float, float, float], ...],
    *,
    source_state_id_label: str,
    target_state_id_label: str,
) -> list[SegmentCandidate]:
    waypoint_candidates: list[list[tuple[int, tuple[float, float, float]]]] = []
    for x_pos, y_pos, phi in cartesian_waypoints:
        candidates = [(branch_index, q) for branch_index, q in enumerate(ik_pose(x_pos, y_pos, phi)) if state_valid(q)]
        if not candidates:
            return []
        waypoint_candidates.append(candidates)

    feasible: list[SegmentCandidate] = []
    for start_branch_index, start_q in waypoint_candidates[0]:
        chosen = [start_q]
        current_q = start_q
        current_branch = start_branch_index
        success = True
        for waypoint_index in range(1, len(waypoint_candidates)):
            candidates = sorted(
                waypoint_candidates[waypoint_index],
                key=lambda item: (
                    abs(item[0] - current_branch),
                    _wrapped_joint_l1(current_q, item[1]),
                ),
            )
            accepted = None
            for branch_index, q in candidates:
                if not check_motion(current_q, q, step_mm=0.5):
                    continue
                accepted = (branch_index, q)
                break
            if accepted is None:
                success = False
                break
            current_branch, current_q = accepted
            chosen.append(current_q)
        if not success:
            continue
        feasible.append(
            SegmentCandidate(
                workspace_polyline=tuple((x_pos, y_pos) for x_pos, y_pos, _ in cartesian_waypoints),
                workspace_path_length=_polyline_length(tuple((x_pos, y_pos) for x_pos, y_pos, _ in cartesian_waypoints)),
                cartesian_waypoints=cartesian_waypoints,
                chosen_joint_waypoints=tuple(chosen),
                source_state_id=_state_id_for_visit(
                    source_label,
                    start_branch_index,
                    source_label=source_state_id_label,
                    target_label=target_state_id_label,
                    is_source=True,
                ),
                target_state_id=_state_id_for_visit(
                    target_label,
                    current_branch,
                    source_label=source_state_id_label,
                    target_label=target_state_id_label,
                    is_source=False,
                ),
                validation_result=True,
                local_repair_used=False,
                local_repair_reason=None,
                joint_space_path_length=sum(
                    _wrapped_joint_l1(chosen[index], chosen[index + 1]) for index in range(len(chosen) - 1)
                ),
            )
        )
    return feasible


def _try_same_object_contour_realization(
    source_label: str,
    target_label: str,
    cartesian_waypoints: list[tuple[float, float, float]],
    *,
    start_branch_index: int,
    start_q: tuple[float, float, float],
    source_state_id_label: str,
    target_state_id_label: str,
) -> SegmentCandidate | None:
    working_waypoints = list(cartesian_waypoints)
    chosen_joint_waypoints = [start_q]
    current_q = start_q
    current_branch = start_branch_index
    waypoint_index = 1
    interval_subdivisions: dict[int, int] = {}

    while waypoint_index < len(working_waypoints):
        next_pose = working_waypoints[waypoint_index]
        candidates = sorted(
            [(branch_index, q) for branch_index, q in enumerate(ik_pose(*next_pose)) if state_valid(q)],
            key=lambda item: (
                abs(item[0] - current_branch),
                _wrapped_joint_l1(current_q, item[1]),
            ),
        )
        accepted = None
        for branch_index, q in candidates:
            jump_l1 = _wrapped_joint_l1(current_q, q)
            if jump_l1 > CONTOUR_JOINT_JUMP_L1_THRESHOLD:
                continue
            if not check_motion(current_q, q, step_mm=0.5):
                continue
            accepted = (branch_index, q)
            break
        if accepted is not None:
            current_branch, current_q = accepted
            chosen_joint_waypoints.append(current_q)
            waypoint_index += 1
            continue

        interval_key = waypoint_index - 1
        left_pose = working_waypoints[waypoint_index - 1]
        right_pose = working_waypoints[waypoint_index]
        interval_length = _distance((left_pose[0], left_pose[1]), (right_pose[0], right_pose[1]))
        subdivision_count = interval_subdivisions.get(interval_key, 0)
        if interval_length <= CONTOUR_INTERPOLATOR_MIN_STEP_MM or subdivision_count >= CONTOUR_INTERPOLATOR_MAX_SUBDIVISIONS:
            return None
        midpoint_pose = (
            0.5 * (left_pose[0] + right_pose[0]),
            0.5 * (left_pose[1] + right_pose[1]),
            _wrap_angle_lerp(left_pose[2], right_pose[2], 0.5),
        )
        working_waypoints.insert(waypoint_index, midpoint_pose)
        interval_subdivisions[interval_key] = subdivision_count + 1

    return SegmentCandidate(
        workspace_polyline=tuple((x_pos, y_pos) for x_pos, y_pos, _ in working_waypoints),
        workspace_path_length=_polyline_length(tuple((x_pos, y_pos) for x_pos, y_pos, _ in working_waypoints)),
        cartesian_waypoints=tuple(working_waypoints),
        chosen_joint_waypoints=tuple(chosen_joint_waypoints),
        source_state_id=_state_id_for_visit(
            source_label,
            start_branch_index,
            source_label=source_state_id_label,
            target_label=target_state_id_label,
            is_source=True,
        ),
        target_state_id=_state_id_for_visit(
            target_label,
            current_branch,
            source_label=source_state_id_label,
            target_label=target_state_id_label,
            is_source=False,
        ),
        validation_result=True,
        local_repair_used=False,
        local_repair_reason=None,
        joint_space_path_length=sum(
            _wrapped_joint_l1(chosen_joint_waypoints[index], chosen_joint_waypoints[index + 1])
            for index in range(len(chosen_joint_waypoints) - 1)
        ),
    )


def _lift_same_object_contour_segment(
    source_label: str,
    target_label: str,
    polyline: tuple[tuple[float, float], ...],
    *,
    weld_lookup: dict[str, WeldPoint],
    source_state_id_label: str,
    target_state_id_label: str,
) -> tuple[tuple[tuple[float, float, float], ...], list[SegmentCandidate], str | None]:
    cartesian_waypoints = list(
        _same_object_contour_cartesian_waypoints(
            polyline,
            source_label=source_label,
            target_label=target_label,
            weld_lookup=weld_lookup,
        )
    )
    if not cartesian_waypoints:
        return (), [], None
    feasible: list[SegmentCandidate] = []
    for start_branch_index, start_q in [
        (branch_index, q)
        for branch_index, q in enumerate(ik_pose(*cartesian_waypoints[0]))
        if state_valid(q)
    ]:
        realization = _try_same_object_contour_realization(
            source_label,
            target_label,
            cartesian_waypoints,
            start_branch_index=start_branch_index,
            start_q=start_q,
            source_state_id_label=source_state_id_label,
            target_state_id_label=target_state_id_label,
        )
        if realization is not None:
            feasible.append(realization)
    return tuple(cartesian_waypoints), feasible, None


def _lift_polyline_with_local_heading_repairs(
    source_label: str,
    target_label: str,
    polyline: tuple[tuple[float, float], ...],
    *,
    weld_lookup: dict[str, WeldPoint],
    source_state_id_label: str,
    target_state_id_label: str,
    segment_type: str = "inter_object",
) -> tuple[tuple[tuple[float, float, float], ...], list[SegmentCandidate], str | None]:
    """Try deterministic local waypoint/heading repairs on one fixed workspace polyline."""
    last_waypoints: tuple[tuple[float, float, float], ...] | None = None
    if segment_type == "same_object_neighbor":
        configs = (
            (CARTESIAN_WAYPOINT_SPACING_MM, 0.0, "weld_interp", "weld_interp"),
            (3.0, 0.0, "spacing3_weld_interp", "weld_interp"),
            (CARTESIAN_WAYPOINT_SPACING_MM, ENDPOINT_BLEND_DISTANCE_MM, "default", "tangent"),
            (CARTESIAN_WAYPOINT_SPACING_MM, 10.0, "blend10", "tangent"),
            (3.0, 10.0, "spacing3_blend10", "tangent"),
            (3.0, 5.0, "spacing3_blend5", "tangent"),
            (3.0, 0.0, "spacing3_blend0", "tangent"),
        )
    else:
        configs = LIFT_REPAIR_CONFIGS
    for spacing_mm, blend_distance_mm, repair_name, heading_mode in configs:
        cartesian_waypoints = polyline_to_cartesian_waypoints(
            polyline,
            source_label=source_label,
            target_label=target_label,
            weld_lookup=weld_lookup,
            spacing_mm=spacing_mm,
            blend_distance_mm=blend_distance_mm,
            heading_mode=heading_mode,
        )
        last_waypoints = cartesian_waypoints
        if segment_type == "same_object_neighbor":
            realizations = _lift_cartesian_segment_greedy(
                source_label,
                target_label,
                cartesian_waypoints,
                source_state_id_label=source_state_id_label,
                target_state_id_label=target_state_id_label,
            )
            if not realizations:
                realizations = _lift_cartesian_segment(
                    source_label,
                    target_label,
                    cartesian_waypoints,
                    source_state_id_label=source_state_id_label,
                    target_state_id_label=target_state_id_label,
                )
        else:
            realizations = _lift_cartesian_segment(
                source_label,
                target_label,
                cartesian_waypoints,
                source_state_id_label=source_state_id_label,
                target_state_id_label=target_state_id_label,
            )
        if realizations:
            repair_reason = None if repair_name == "default" else repair_name
            return cartesian_waypoints, realizations, repair_reason
    if last_waypoints is None:
        last_waypoints = ()
    return last_waypoints, [], None


def _segment_path_edges(path_ids: list[str]) -> list[tuple[str, str]]:
    return [tuple(sorted((path_ids[index], path_ids[index + 1]))) for index in range(len(path_ids) - 1)]


def _grid_repair_polyline(
    source_point: tuple[float, float],
    target_point: tuple[float, float],
    obstacles: list[WorkspaceObstacle],
) -> tuple[tuple[float, float], ...] | None:
    """Return one local grid-graph repair polyline between two points, if any."""
    xmin, xmax, ymin, ymax = LOCAL_REPAIR_GRID_BOUNDS
    step = LOCAL_REPAIR_GRID_STEP_MM
    x_values = []
    x_pos = xmin
    while x_pos <= xmax + 1e-9:
        x_values.append(round(x_pos, 6))
        x_pos += step
    y_values = []
    y_pos = ymin
    while y_pos <= ymax + 1e-9:
        y_values.append(round(y_pos, 6))
        y_pos += step

    node_points: dict[str, tuple[float, float]] = {
        "START": source_point,
        "GOAL": target_point,
    }
    grid_ids: list[str] = []
    for x_index, x_coord in enumerate(x_values):
        for y_index, y_coord in enumerate(y_values):
            point = (x_coord, y_coord)
            if any(_point_in_polygon_strict(point, obstacle.vertices) for obstacle in obstacles):
                continue
            node_id = f"G_{x_index}_{y_index}"
            node_points[node_id] = point
            grid_ids.append(node_id)

    adjacency: dict[str, dict[str, float]] = {node_id: {} for node_id in node_points}

    def add_edge(node_a: str, node_b: str) -> None:
        if node_b in adjacency[node_a]:
            return
        length = _distance(node_points[node_a], node_points[node_b])
        adjacency[node_a][node_b] = length
        adjacency[node_b][node_a] = length

    grid_index_lookup = {
        node_id: tuple(int(part) for part in node_id.split("_")[1:])
        for node_id in grid_ids
    }
    reverse_lookup = {indices: node_id for node_id, indices in grid_index_lookup.items()}
    for node_id, (x_index, y_index) in grid_index_lookup.items():
        for delta_x in (-1, 0, 1):
            for delta_y in (-1, 0, 1):
                if delta_x == 0 and delta_y == 0:
                    continue
                neighbor_id = reverse_lookup.get((x_index + delta_x, y_index + delta_y))
                if neighbor_id is None:
                    continue
                if not _segment_collision_free(node_points[node_id], node_points[neighbor_id], obstacles):
                    continue
                add_edge(node_id, neighbor_id)

    for grid_id in grid_ids:
        point = node_points[grid_id]
        if _distance(source_point, point) <= step * 1.6 and _segment_collision_free(source_point, point, obstacles):
            add_edge("START", grid_id)
        if _distance(target_point, point) <= step * 1.6 and _segment_collision_free(target_point, point, obstacles):
            add_edge("GOAL", grid_id)
    if _segment_collision_free(source_point, target_point, obstacles):
        add_edge("START", "GOAL")

    path_ids = _dijkstra_shortest_path(adjacency, node_points, "START", "GOAL")
    if path_ids is None:
        return None
    return _compress_collinear_polyline(tuple(node_points[node_id] for node_id in path_ids))


def _weld_heading_rad(label: str, weld_lookup: dict[str, WeldPoint]) -> float | None:
    if label == "DEPOT":
        return None
    return radians(weld_lookup[label].tool_heading_deg)


def _approach_retreat_point(
    label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    distance_mm: float = APPROACH_RETREAT_DISTANCE_MM,
) -> tuple[float, float]:
    point, _ = _visit_point_and_obstacle(label, scene=scene, weld_lookup=weld_lookup)
    heading = _weld_heading_rad(label, weld_lookup)
    if heading is None:
        return point
    return (
        point[0] - distance_mm * cos(heading),
        point[1] - distance_mm * sin(heading),
    )


def _same_object_offset_point(
    label: str,
    *,
    weld_lookup: dict[str, WeldPoint],
    extra_mm: float,
) -> tuple[float, float]:
    weld = weld_lookup[label]
    return (
        weld.position[0] + extra_mm * weld.normal[0],
        weld.position[1] + extra_mm * weld.normal[1],
    )


def _same_object_offset_contour_repair(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    source_visit_for_ids: str,
    target_visit_for_ids: str,
    extra_mm: float,
) -> tuple[tuple[tuple[float, float], ...], tuple[tuple[float, float, float], ...], list[SegmentCandidate], bool, str | None] | None:
    source_weld = weld_lookup[source_label]
    target_weld = weld_lookup[target_label]
    if source_weld.object_id != target_weld.object_id:
        return None

    source_point = source_weld.position
    target_point = target_weld.position
    offset_source = _same_object_offset_point(source_label, weld_lookup=weld_lookup, extra_mm=extra_mm)
    offset_target = _same_object_offset_point(target_label, weld_lookup=weld_lookup, extra_mm=extra_mm)

    buffered_obstacles = build_inflated_workspace_obstacles(
        scene,
        circle_polygon_sides=CIRCLE_POLYGON_SIDES,
        inflation_radius_mm=TORCH_TIP_INFLATION_RADIUS_MM + extra_mm,
    )
    object_lookup = _scene_object_lookup(scene)
    obstacle_lookup = {obstacle.obstacle_id: obstacle for obstacle in buffered_obstacles}
    scene_object = object_lookup[source_weld.object_id]
    if isinstance(scene_object, CircleObject):
        contour_polyline = _circle_arc_polyline(
            offset_source,
            offset_target,
            center=scene_object.center,
            radius=scene_object.radius + TORCH_TIP_INFLATION_RADIUS_MM + extra_mm,
        )
    else:
        obstacle = obstacle_lookup[source_weld.object_id]
        clockwise = _polygon_boundary_direction_polyline(offset_source, offset_target, obstacle, direction=1)
        counter_clockwise = _polygon_boundary_direction_polyline(offset_source, offset_target, obstacle, direction=-1)
        candidates = [poly for poly in (clockwise, counter_clockwise) if poly is not None]
        if not candidates:
            return None
        contour_polyline = min(candidates, key=_polyline_length)

    full_polyline = _compress_collinear_polyline(
        (source_point, offset_source, *contour_polyline[1:-1], offset_target, target_point)
    )
    cartesian_waypoints, realizations, repair_reason = _try_lift_on_polyline(
        source_label,
        target_label,
        full_polyline,
        weld_lookup=weld_lookup,
        source_visit_for_ids=source_visit_for_ids,
        target_visit_for_ids=target_visit_for_ids,
        segment_type="same_object_neighbor",
        repair_reason=f"same_object_offset_contour_{extra_mm:.1f}mm",
        local_repair_used=True,
    )
    if not realizations:
        return None
    return full_polyline, cartesian_waypoints, realizations, True, repair_reason


def _try_lift_on_polyline(
    source_label: str,
    target_label: str,
    polyline: tuple[tuple[float, float], ...],
    *,
    weld_lookup: dict[str, WeldPoint],
    source_visit_for_ids: str,
    target_visit_for_ids: str,
    segment_type: str = "inter_object",
    repair_reason: str | None = None,
    local_repair_used: bool = False,
) -> tuple[tuple[tuple[float, float, float], ...], list[SegmentCandidate], str | None]:
    cartesian_waypoints, realizations, lift_repair_reason = _lift_polyline_with_local_heading_repairs(
        source_label,
        target_label,
        polyline,
        weld_lookup=weld_lookup,
        source_state_id_label=source_visit_for_ids,
        target_state_id_label=target_visit_for_ids,
        segment_type=segment_type,
    )
    if not realizations:
        return cartesian_waypoints, [], None
    effective_reason = repair_reason
    if lift_repair_reason is not None:
        effective_reason = lift_repair_reason if repair_reason is None else f"{repair_reason}_{lift_repair_reason}"
    updated = [
        SegmentCandidate(
            workspace_polyline=realization.workspace_polyline,
            workspace_path_length=realization.workspace_path_length,
            cartesian_waypoints=realization.cartesian_waypoints,
            chosen_joint_waypoints=realization.chosen_joint_waypoints,
            source_state_id=realization.source_state_id,
            target_state_id=realization.target_state_id,
            validation_result=realization.validation_result,
            local_repair_used=local_repair_used or realization.local_repair_used,
            local_repair_reason=effective_reason,
            joint_space_path_length=realization.joint_space_path_length,
        )
        for realization in realizations
    ]
    return cartesian_waypoints, updated, effective_reason


def _apply_local_repair(
    source_label: str,
    target_label: str,
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    obstacles: list[WorkspaceObstacle],
    roadmap_nodes: dict[str, GraphNode],
    roadmap_adjacency: dict[str, dict[str, float]],
    source_visit_for_ids: str,
    target_visit_for_ids: str,
) -> tuple[tuple[tuple[float, float], ...], tuple[tuple[float, float, float], ...], list[SegmentCandidate], bool, str | None]:
    source_point, _ = _visit_point_and_obstacle(source_label, scene=scene, weld_lookup=weld_lookup)
    target_point, _ = _visit_point_and_obstacle(target_label, scene=scene, weld_lookup=weld_lookup)
    _, source_obstacle_id = _visit_point_and_obstacle(source_label, scene=scene, weld_lookup=weld_lookup)
    _, target_obstacle_id = _visit_point_and_obstacle(target_label, scene=scene, weld_lookup=weld_lookup)
    primary = shortest_workspace_polyline_for_segment(
        source_label,
        target_label,
        scene=scene,
        weld_lookup=weld_lookup,
        obstacles=obstacles,
        roadmap_nodes=roadmap_nodes,
        roadmap_adjacency=roadmap_adjacency,
    )
    if primary is None:
        raise ValueError(f"No workspace path found for segment {source_label} -> {target_label}.")

    primary_polyline, primary_path_ids = primary
    cartesian_waypoints, realizations, lift_repair_reason = _lift_polyline_with_local_heading_repairs(
        source_label,
        target_label,
        primary_polyline,
        weld_lookup=weld_lookup,
        source_state_id_label=source_visit_for_ids,
        target_state_id_label=target_visit_for_ids,
    )
    if realizations:
        return primary_polyline, cartesian_waypoints, realizations, lift_repair_reason is not None, lift_repair_reason

    same_obstacle_segment = source_obstacle_id is not None and source_obstacle_id == target_obstacle_id
    if same_obstacle_segment:
        forced_hub_result = _try_forced_hub_repairs(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
            obstacles=obstacles,
            source_visit_for_ids=source_visit_for_ids,
            target_visit_for_ids=target_visit_for_ids,
            source_point=source_point,
            target_point=target_point,
        )
        if forced_hub_result is not None:
            return forced_hub_result
        obstacle_lookup = {obstacle.obstacle_id: obstacle for obstacle in obstacles}
        ring_hub_result = _try_direct_hub_repairs(
            _full_ring_hubs_for_obstacle(obstacle_lookup[source_obstacle_id]),
            source_label,
            target_label,
            weld_lookup=weld_lookup,
            obstacles=obstacles,
            source_visit_for_ids=source_visit_for_ids,
            target_visit_for_ids=target_visit_for_ids,
            source_point=source_point,
            target_point=target_point,
            hub_name_prefix="ring_hub",
        )
        if ring_hub_result is not None:
            return ring_hub_result

    for edge_index, edge in enumerate(_segment_path_edges(primary_path_ids)[:LOCAL_REPAIR_MAX_FORBIDDEN_EDGES]):
        alternative = shortest_workspace_polyline_for_segment(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
            obstacles=obstacles,
            roadmap_nodes=roadmap_nodes,
            roadmap_adjacency=roadmap_adjacency,
            forbidden_edges=frozenset({edge}),
        )
        if alternative is None:
            continue
        alternative_polyline, _ = alternative
        alternative_waypoints, alternative_realizations, lift_repair_reason = _lift_polyline_with_local_heading_repairs(
            source_label,
            target_label,
            alternative_polyline,
            weld_lookup=weld_lookup,
            source_state_id_label=source_visit_for_ids,
            target_state_id_label=target_visit_for_ids,
        )
        if alternative_realizations:
            return (
                alternative_polyline,
                alternative_waypoints,
                [
                    SegmentCandidate(
                        workspace_polyline=realization.workspace_polyline,
                        workspace_path_length=realization.workspace_path_length,
                        cartesian_waypoints=realization.cartesian_waypoints,
                        chosen_joint_waypoints=realization.chosen_joint_waypoints,
                        source_state_id=realization.source_state_id,
                        target_state_id=realization.target_state_id,
                        validation_result=realization.validation_result,
                        local_repair_used=True,
                        local_repair_reason=(
                            f"forbid_edge_{edge_index}"
                            if lift_repair_reason is None
                            else f"forbid_edge_{edge_index}_{lift_repair_reason}"
                        ),
                        joint_space_path_length=realization.joint_space_path_length,
                    )
                    for realization in alternative_realizations
                ],
                True,
                f"forbid_edge_{edge_index}",
            )

    repair_hub_points = {f"REPAIR_HUB_{index}": point for index, point in enumerate(LOCAL_REPAIR_HUBS)}
    hub_repair = shortest_workspace_polyline_for_segment(
        source_label,
        target_label,
        scene=scene,
        weld_lookup=weld_lookup,
        obstacles=obstacles,
        roadmap_nodes=roadmap_nodes,
        roadmap_adjacency=roadmap_adjacency,
        extra_points=repair_hub_points,
    )
    if hub_repair is not None:
        hub_polyline, _ = hub_repair
        hub_waypoints, hub_realizations, lift_repair_reason = _lift_polyline_with_local_heading_repairs(
            source_label,
            target_label,
            hub_polyline,
            weld_lookup=weld_lookup,
            source_state_id_label=source_visit_for_ids,
            target_state_id_label=target_visit_for_ids,
        )
        if hub_realizations:
            return (
                hub_polyline,
                hub_waypoints,
                [
                    SegmentCandidate(
                        workspace_polyline=realization.workspace_polyline,
                        workspace_path_length=realization.workspace_path_length,
                        cartesian_waypoints=realization.cartesian_waypoints,
                        chosen_joint_waypoints=realization.chosen_joint_waypoints,
                        source_state_id=realization.source_state_id,
                        target_state_id=realization.target_state_id,
                        validation_result=realization.validation_result,
                        local_repair_used=True,
                        local_repair_reason=(
                            "repair_hub_graph"
                            if lift_repair_reason is None
                            else f"repair_hub_graph_{lift_repair_reason}"
                        ),
                        joint_space_path_length=realization.joint_space_path_length,
                    )
                    for realization in hub_realizations
                ],
                True,
                "repair_hub_graph",
            )

    if not same_obstacle_segment:
        forced_hub_result = _try_forced_hub_repairs(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
            obstacles=obstacles,
            source_visit_for_ids=source_visit_for_ids,
            target_visit_for_ids=target_visit_for_ids,
            source_point=source_point,
            target_point=target_point,
        )
        if forced_hub_result is not None:
            return forced_hub_result

    grid_repair_polyline = _grid_repair_polyline(source_point, target_point, obstacles)
    if grid_repair_polyline is not None:
        grid_waypoints, grid_realizations, lift_repair_reason = _lift_polyline_with_local_heading_repairs(
            source_label,
            target_label,
            grid_repair_polyline,
            weld_lookup=weld_lookup,
            source_state_id_label=source_visit_for_ids,
            target_state_id_label=target_visit_for_ids,
        )
        if grid_realizations:
            return (
                grid_repair_polyline,
                grid_waypoints,
                [
                    SegmentCandidate(
                        workspace_polyline=realization.workspace_polyline,
                        workspace_path_length=realization.workspace_path_length,
                        cartesian_waypoints=realization.cartesian_waypoints,
                        chosen_joint_waypoints=realization.chosen_joint_waypoints,
                        source_state_id=realization.source_state_id,
                        target_state_id=realization.target_state_id,
                        validation_result=realization.validation_result,
                        local_repair_used=True,
                        local_repair_reason=(
                            "repair_grid_graph"
                            if lift_repair_reason is None
                            else f"repair_grid_graph_{lift_repair_reason}"
                        ),
                        joint_space_path_length=realization.joint_space_path_length,
                    )
                    for realization in grid_realizations
                ],
                True,
                "repair_grid_graph",
            )

    raise ValueError(f"Workspace segment {source_label} -> {target_label} was not robot-realizable, and local repair failed.")


def _stage2_repair_segment(
    segment_record: dict[str, object],
    *,
    scene: Scene,
    weld_lookup: dict[str, WeldPoint],
    base_obstacles: list[WorkspaceObstacle],
    base_roadmap_nodes: dict[str, GraphNode],
    base_roadmap_adjacency: dict[str, dict[str, float]],
) -> tuple[dict[str, object], dict[str, object]]:
    segment_index = int(segment_record["segment_index"])
    source_label = str(segment_record["source_label"])
    target_label = str(segment_record["target_label"])
    polyline = tuple((float(point[0]), float(point[1])) for point in segment_record["workspace_polyline"])
    source_point, source_obstacle_id = _visit_point_and_obstacle(source_label, scene=scene, weld_lookup=weld_lookup)
    target_point, target_obstacle_id = _visit_point_and_obstacle(target_label, scene=scene, weld_lookup=weld_lookup)
    segment_type = str(segment_record.get("segment_type", "inter_object"))
    same_object = segment_type == "same_object_neighbor"

    import time

    segment_started = time.perf_counter()
    repair_report = {
        "segment_index": segment_index,
        "segment_type": segment_type,
        "source_label": source_label,
        "target_label": target_label,
        "contour_interpolator_succeeded": False,
        "plain_lift_succeeded": False,
        "repair_level_succeeded": None,
        "same_object_repair_used": False,
        "extra_liftability_buffer_used": False,
        "micro_segments_used": False,
        "local_segment_fallback_used": False,
        "fallback_used": False,
        "validation_succeeded": False,
        "unresolved": False,
        "runtime_s": None,
    }

    if same_object:
        cartesian_waypoints, realizations, _ = _lift_same_object_contour_segment(
            source_label,
            target_label,
            polyline,
            weld_lookup=weld_lookup,
            source_state_id_label=source_label,
            target_state_id_label=target_label,
        )
        if realizations:
            repair_report["contour_interpolator_succeeded"] = True
            repair_report["repair_level_succeeded"] = "same_object_contour_interpolator"
            repair_report["validation_succeeded"] = True
            repair_report["runtime_s"] = time.perf_counter() - segment_started
            return (
                {
                    "segment_index": segment_index,
                    "source_label": source_label,
                    "target_label": target_label,
                    "segment_type": segment_type,
                    "workspace_polyline": [list(point) for point in polyline],
                    "workspace_path_length": float(segment_record["workspace_path_length"]),
                    "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in cartesian_waypoints],
                    "ik_succeeded": True,
                    "validation_succeeded": True,
                    "runtime_s": repair_report["runtime_s"],
                    "repair_level_used": "same_object_contour_interpolator",
                    "realizations": [
                        {
                            "source_state_id": r.source_state_id,
                            "target_state_id": r.target_state_id,
                            "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                            "joint_space_path_length": r.joint_space_path_length,
                            "validation_result": r.validation_result,
                            "local_repair_used": r.local_repair_used,
                            "local_repair_reason": r.local_repair_reason,
                        }
                        for r in realizations
                    ],
                },
                repair_report,
            )
        repair_report["fallback_used"] = True
    else:
        cartesian_waypoints, realizations, repair_reason = _try_lift_on_polyline(
            source_label,
            target_label,
            polyline,
            weld_lookup=weld_lookup,
            source_visit_for_ids=source_label,
            target_visit_for_ids=target_label,
            segment_type=segment_type,
        )
        if realizations:
            repair_report["plain_lift_succeeded"] = True
            repair_report["repair_level_succeeded"] = "plain_lift"
            repair_report["validation_succeeded"] = True
            repair_report["runtime_s"] = time.perf_counter() - segment_started
            return (
                {
                    "segment_index": segment_index,
                    "source_label": source_label,
                    "target_label": target_label,
                    "segment_type": segment_type,
                    "workspace_polyline": [list(point) for point in polyline],
                    "workspace_path_length": float(segment_record["workspace_path_length"]),
                    "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in cartesian_waypoints],
                    "ik_succeeded": True,
                    "validation_succeeded": True,
                    "runtime_s": repair_report["runtime_s"],
                    "repair_level_used": "plain_lift",
                    "realizations": [
                        {
                            "source_state_id": r.source_state_id,
                            "target_state_id": r.target_state_id,
                            "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                            "joint_space_path_length": r.joint_space_path_length,
                            "validation_result": r.validation_result,
                            "local_repair_used": r.local_repair_used,
                            "local_repair_reason": r.local_repair_reason,
                        }
                        for r in realizations
                    ],
                },
                repair_report,
            )

    if same_object:
        for extra_mm in SAME_OBJECT_CONTOUR_REPAIR_EXTRAS_MM:
            same_object_offset_result = _same_object_offset_contour_repair(
                source_label,
                target_label,
                scene=scene,
                weld_lookup=weld_lookup,
                source_visit_for_ids=source_label,
                target_visit_for_ids=target_label,
                extra_mm=extra_mm,
            )
            if same_object_offset_result is None:
                continue
            repaired_polyline, repaired_waypoints, repaired_realizations, _, repair_level = same_object_offset_result
            repair_report["repair_level_succeeded"] = repair_level or "same_object_local_repair"
            repair_report["same_object_repair_used"] = True
            repair_report["fallback_used"] = True
            repair_report["validation_succeeded"] = True
            repair_report["runtime_s"] = time.perf_counter() - segment_started
            return (
                {
                    "segment_index": segment_index,
                    "source_label": source_label,
                    "target_label": target_label,
                    "segment_type": segment_type,
                    "workspace_polyline": [list(point) for point in repaired_polyline],
                    "workspace_path_length": _polyline_length(repaired_polyline),
                    "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in repaired_waypoints],
                    "ik_succeeded": True,
                    "validation_succeeded": True,
                    "runtime_s": repair_report["runtime_s"],
                    "repair_level_used": repair_level,
                    "realizations": [
                        {
                            "source_state_id": r.source_state_id,
                            "target_state_id": r.target_state_id,
                            "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                            "joint_space_path_length": r.joint_space_path_length,
                            "validation_result": r.validation_result,
                            "local_repair_used": True,
                            "local_repair_reason": r.local_repair_reason,
                        }
                        for r in repaired_realizations
                    ],
                },
                repair_report,
            )

        same_object_result = _try_forced_hub_repairs(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
            obstacles=base_obstacles,
            source_visit_for_ids=source_label,
            target_visit_for_ids=target_label,
            source_point=source_point,
            target_point=target_point,
        )
        if same_object_result is None:
            obstacle_lookup = {obstacle.obstacle_id: obstacle for obstacle in base_obstacles}
            same_object_result = _try_direct_hub_repairs(
                _full_ring_hubs_for_obstacle(obstacle_lookup[source_obstacle_id]),
                source_label,
                target_label,
                weld_lookup=weld_lookup,
                obstacles=base_obstacles,
                source_visit_for_ids=source_label,
                target_visit_for_ids=target_label,
                source_point=source_point,
                target_point=target_point,
                hub_name_prefix="ring_hub",
            )
        if same_object_result is not None:
            repaired_polyline, repaired_waypoints, repaired_realizations, _, repair_level = same_object_result
            repair_report["repair_level_succeeded"] = repair_level or "same_object_local_repair"
            repair_report["same_object_repair_used"] = True
            repair_report["fallback_used"] = True
            repair_report["validation_succeeded"] = True
            repair_report["runtime_s"] = time.perf_counter() - segment_started
            return (
                {
                    "segment_index": segment_index,
                    "source_label": source_label,
                    "target_label": target_label,
                    "segment_type": segment_record.get("segment_type"),
                    "workspace_polyline": [list(point) for point in repaired_polyline],
                    "workspace_path_length": _polyline_length(repaired_polyline),
                    "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in repaired_waypoints],
                    "ik_succeeded": True,
                    "validation_succeeded": True,
                    "runtime_s": repair_report["runtime_s"],
                    "repair_level_used": repair_level,
                    "realizations": [
                        {
                            "source_state_id": r.source_state_id,
                            "target_state_id": r.target_state_id,
                            "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                            "joint_space_path_length": r.joint_space_path_length,
                            "validation_result": r.validation_result,
                            "local_repair_used": True,
                            "local_repair_reason": r.local_repair_reason,
                        }
                        for r in repaired_realizations
                    ],
                },
                repair_report,
            )

    buffered_obstacles = build_inflated_workspace_obstacles(
        scene,
        circle_polygon_sides=CIRCLE_POLYGON_SIDES,
        inflation_radius_mm=TORCH_TIP_INFLATION_RADIUS_MM + LIFTABILITY_BUFFER_EXTRA_MM,
    )
    buffered_roadmap_nodes, buffered_roadmap_adjacency = build_visibility_roadmap(buffered_obstacles)
    buffered_result = shortest_workspace_polyline_for_segment(
        source_label,
        target_label,
        scene=scene,
        weld_lookup=weld_lookup,
        obstacles=buffered_obstacles,
        roadmap_nodes=buffered_roadmap_nodes,
        roadmap_adjacency=buffered_roadmap_adjacency,
    )
    if buffered_result is not None:
        buffered_polyline, _ = buffered_result
        buffered_waypoints, buffered_realizations, _ = _try_lift_on_polyline(
            source_label,
            target_label,
            buffered_polyline,
            weld_lookup=weld_lookup,
            source_visit_for_ids=source_label,
            target_visit_for_ids=target_label,
            segment_type=str(segment_record.get("segment_type", "inter_object")),
            repair_reason=f"liftability_buffer_{LIFTABILITY_BUFFER_EXTRA_MM:.1f}mm",
            local_repair_used=True,
        )
        if buffered_realizations:
            repair_report["repair_level_succeeded"] = "liftability_buffer_replan"
            repair_report["extra_liftability_buffer_used"] = True
            repair_report["fallback_used"] = True
            repair_report["validation_succeeded"] = True
            repair_report["runtime_s"] = time.perf_counter() - segment_started
            return (
                {
                    "segment_index": segment_index,
                    "source_label": source_label,
                    "target_label": target_label,
                    "segment_type": segment_record.get("segment_type"),
                    "workspace_polyline": [list(point) for point in buffered_polyline],
                    "workspace_path_length": _polyline_length(buffered_polyline),
                    "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in buffered_waypoints],
                    "ik_succeeded": True,
                    "validation_succeeded": True,
                    "runtime_s": repair_report["runtime_s"],
                    "repair_level_used": "liftability_buffer_replan",
                    "realizations": [
                        {
                            "source_state_id": r.source_state_id,
                            "target_state_id": r.target_state_id,
                            "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                            "joint_space_path_length": r.joint_space_path_length,
                            "validation_result": r.validation_result,
                            "local_repair_used": True,
                            "local_repair_reason": r.local_repair_reason,
                        }
                        for r in buffered_realizations
                    ],
                },
                repair_report,
            )

    source_retreat = _approach_retreat_point(source_label, scene=scene, weld_lookup=weld_lookup)
    target_retreat = _approach_retreat_point(target_label, scene=scene, weld_lookup=weld_lookup)
    if _segment_collision_free(source_point, source_retreat, base_obstacles) and _segment_collision_free(target_retreat, target_point, base_obstacles):
        micro_transit = shortest_workspace_polyline_between_points(
            source_retreat,
            target_retreat,
            obstacles=base_obstacles,
            roadmap_nodes=base_roadmap_nodes,
            roadmap_adjacency=base_roadmap_adjacency,
        )
        if micro_transit is not None:
            transit_polyline, _ = micro_transit
            micro_polyline = _compress_collinear_polyline((source_point, source_retreat, *transit_polyline[1:-1], target_retreat, target_point))
            micro_waypoints, micro_realizations, _ = _try_lift_on_polyline(
                source_label,
                target_label,
                micro_polyline,
                weld_lookup=weld_lookup,
                source_visit_for_ids=source_label,
                target_visit_for_ids=target_label,
                segment_type=str(segment_record.get("segment_type", "inter_object")),
                repair_reason="micro_segments",
                local_repair_used=True,
            )
            if micro_realizations:
                repair_report["repair_level_succeeded"] = "endpoint_micro_segments"
                repair_report["micro_segments_used"] = True
                repair_report["fallback_used"] = True
                repair_report["validation_succeeded"] = True
                repair_report["runtime_s"] = time.perf_counter() - segment_started
                return (
                    {
                        "segment_index": segment_index,
                        "source_label": source_label,
                        "target_label": target_label,
                        "segment_type": segment_record.get("segment_type"),
                        "workspace_polyline": [list(point) for point in micro_polyline],
                        "workspace_path_length": _polyline_length(micro_polyline),
                        "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in micro_waypoints],
                        "ik_succeeded": True,
                        "validation_succeeded": True,
                        "runtime_s": repair_report["runtime_s"],
                        "repair_level_used": "endpoint_micro_segments",
                        "realizations": [
                            {
                                "source_state_id": r.source_state_id,
                                "target_state_id": r.target_state_id,
                                "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                                "joint_space_path_length": r.joint_space_path_length,
                                "validation_result": r.validation_result,
                                "local_repair_used": True,
                                "local_repair_reason": r.local_repair_reason,
                            }
                            for r in micro_realizations
                        ],
                    },
                    repair_report,
                )

    try:
        fallback_polyline, fallback_waypoints, fallback_realizations, _, fallback_reason = _apply_local_repair(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
            obstacles=base_obstacles,
            roadmap_nodes=base_roadmap_nodes,
            roadmap_adjacency=base_roadmap_adjacency,
            source_visit_for_ids=source_label,
            target_visit_for_ids=target_label,
        )
        repair_report["repair_level_succeeded"] = "local_segment_only_fallback"
        repair_report["local_segment_fallback_used"] = True
        repair_report["fallback_used"] = True
        repair_report["validation_succeeded"] = True
        repair_report["runtime_s"] = time.perf_counter() - segment_started
        return (
            {
                "segment_index": segment_index,
                "source_label": source_label,
                "target_label": target_label,
                "segment_type": segment_record.get("segment_type"),
                "workspace_polyline": [list(point) for point in fallback_polyline],
                "workspace_path_length": _polyline_length(fallback_polyline),
                "cartesian_waypoints": [{"x": w[0], "y": w[1], "phi": w[2]} for w in fallback_waypoints],
                "ik_succeeded": True,
                "validation_succeeded": True,
                "runtime_s": repair_report["runtime_s"],
                "repair_level_used": fallback_reason or "local_segment_only_fallback",
                "realizations": [
                    {
                        "source_state_id": r.source_state_id,
                        "target_state_id": r.target_state_id,
                        "chosen_joint_waypoints": [list(q) for q in r.chosen_joint_waypoints],
                        "joint_space_path_length": r.joint_space_path_length,
                        "validation_result": r.validation_result,
                        "local_repair_used": True,
                        "local_repair_reason": r.local_repair_reason,
                    }
                    for r in fallback_realizations
                ],
            },
            repair_report,
        )
    except ValueError:
        pass

    repair_report["unresolved"] = True
    repair_report["runtime_s"] = time.perf_counter() - segment_started
    return (
        {
            "segment_index": segment_index,
            "source_label": source_label,
            "target_label": target_label,
            "segment_type": segment_record.get("segment_type"),
            "workspace_polyline": segment_record["workspace_polyline"],
            "workspace_path_length": segment_record["workspace_path_length"],
            "cartesian_waypoints": [],
            "ik_succeeded": False,
            "validation_succeeded": False,
            "runtime_s": repair_report["runtime_s"],
            "repair_level_used": None,
            "realizations": [],
        },
        repair_report,
    )


def build_task2_cartesian_paths(
    task1_solution_path: str | Path = TASK1_SOLUTION_PATH,
) -> dict[str, object]:
    """Build the active Cartesian Task 2 path package from the frozen Hamilton order."""
    scene = build_default_scene()
    task1_solution = load_task1_hamiltonian_solution(task1_solution_path)
    visit_sequence = build_task2_visit_sequence(task1_solution)
    weld_order = list(task1_solution["weld_order"])
    weld_lookup = _weld_lookup(scene)
    obstacles = build_inflated_workspace_obstacles(scene)
    roadmap_nodes, roadmap_adjacency = build_visibility_roadmap(obstacles)

    segment_option_records: list[list[SegmentCandidate]] = []
    segment_metadata: list[dict[str, object]] = []
    repaired_indices: list[int] = []

    for segment_index, (source_label, target_label) in enumerate(zip(visit_sequence[:-1], visit_sequence[1:], strict=True)):
        print(
            "task2 cartesian segment planning: "
            f"index={segment_index} {source_label}->{target_label}",
            flush=True,
        )
        polyline, cartesian_waypoints, realizations, repaired, repair_reason = _apply_local_repair(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
            obstacles=obstacles,
            roadmap_nodes=roadmap_nodes,
            roadmap_adjacency=roadmap_adjacency,
            source_visit_for_ids=source_label,
            target_visit_for_ids=target_label,
        )
        if repaired:
            repaired_indices.append(segment_index)
        segment_option_records.append(realizations)
        segment_metadata.append(
            {
                "segment_index": segment_index,
                "source_label": source_label,
                "target_label": target_label,
                "workspace_polyline": [list(point) for point in polyline],
                "workspace_path_length": _polyline_length(polyline),
                "cartesian_waypoints": [
                    {"x": waypoint[0], "y": waypoint[1], "phi": waypoint[2]}
                    for waypoint in cartesian_waypoints
                ],
                "local_repair_used": repaired,
                "local_repair_reason": repair_reason,
            }
        )

    dp_costs: list[dict[str, float]] = [{realization.source_state_id: 0.0 for realization in segment_option_records[0]}]
    predecessors: list[dict[str, tuple[str, SegmentCandidate]]] = [{}]

    for segment_index, realizations in enumerate(segment_option_records):
        if segment_index == 0:
            next_costs: dict[str, float] = {}
            next_predecessors: dict[str, tuple[str, SegmentCandidate]] = {}
            for realization in realizations:
                new_cost = realization.joint_space_path_length
                if new_cost >= next_costs.get(realization.target_state_id, float("inf")):
                    continue
                next_costs[realization.target_state_id] = new_cost
                next_predecessors[realization.target_state_id] = (realization.source_state_id, realization)
            dp_costs.append(next_costs)
            predecessors.append(next_predecessors)
            continue

        current_costs = dp_costs[segment_index]
        next_costs: dict[str, float] = {}
        next_predecessors: dict[str, tuple[str, SegmentCandidate]] = {}
        for realization in realizations:
            source_cost = current_costs.get(realization.source_state_id)
            if source_cost is None:
                continue
            new_cost = source_cost + realization.joint_space_path_length
            if new_cost >= next_costs.get(realization.target_state_id, float("inf")):
                continue
            next_costs[realization.target_state_id] = new_cost
            next_predecessors[realization.target_state_id] = (realization.source_state_id, realization)
        if not next_costs:
            raise ValueError(f"No globally consistent IK-lifted Task 2 chain exists at segment {segment_index}.")
        dp_costs.append(next_costs)
        predecessors.append(next_predecessors)

    final_costs = dp_costs[-1]
    best_end_state_id = min(final_costs, key=final_costs.get)
    chosen_segments_reversed: list[SegmentCandidate] = []
    current_state_id = best_end_state_id
    for segment_index in range(len(segment_option_records), 0, -1):
        predecessor_state_id, realization = predecessors[segment_index][current_state_id]
        chosen_segments_reversed.append(realization)
        current_state_id = predecessor_state_id
    chosen_segments = list(reversed(chosen_segments_reversed))

    ordered_segment_records: list[dict[str, object]] = []
    total_workspace_path_length = 0.0
    total_joint_path_length = 0.0
    for metadata, realization in zip(segment_metadata, chosen_segments, strict=True):
        total_workspace_path_length += metadata["workspace_path_length"]
        total_joint_path_length += realization.joint_space_path_length
        ordered_segment_records.append(
            {
                "segment_index": metadata["segment_index"],
                "source_label": metadata["source_label"],
                "target_label": metadata["target_label"],
                "source_state_id": realization.source_state_id,
                "target_state_id": realization.target_state_id,
                "workspace_polyline": metadata["workspace_polyline"],
                "workspace_path_length": metadata["workspace_path_length"],
                "cartesian_waypoints": metadata["cartesian_waypoints"],
                "chosen_joint_waypoints": [list(q) for q in realization.chosen_joint_waypoints],
                "validation_result": realization.validation_result,
                "local_repair_used": metadata["local_repair_used"],
                "local_repair_reason": metadata["local_repair_reason"],
                "joint_space_path_length": realization.joint_space_path_length,
            }
        )

    return {
        "task1_weld_order": weld_order,
        "task1_cycle_node_order": visit_sequence,
        "obstacle_model": {
            "inflation_radius_mm": TORCH_TIP_INFLATION_RADIUS_MM,
            "justification": "torch_radius 5mm + clearance 1mm from current scene assumptions",
            "circle_polygon_sides": CIRCLE_POLYGON_SIDES,
        },
        "ordered_segment_records": ordered_segment_records,
        "segment_count": len(ordered_segment_records),
        "collision_free_segment_count": sum(1 for record in ordered_segment_records if record["validation_result"]),
        "locally_repaired_segment_indices": repaired_indices,
        "total_workspace_path_length": total_workspace_path_length,
        "total_joint_path_length": total_joint_path_length,
        "all_segments_valid": all(record["validation_result"] for record in ordered_segment_records),
    }


def _save_json(payload: dict[str, object], path: str | Path) -> Path:
    artifact_path = Path(path)
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return artifact_path


def build_task2_cartesian_workspace_paths(
    task1_solution_path: str | Path = TASK1_SOLUTION_PATH,
    *,
    time_budget_s: float = 120.0,
) -> dict[str, object]:
    scene = build_default_scene()
    task1_solution = load_task1_hamiltonian_solution(task1_solution_path)
    visit_sequence = build_task2_visit_sequence(task1_solution)
    weld_order = list(task1_solution["weld_order"])
    weld_lookup = _weld_lookup(scene)
    obstacles = build_inflated_workspace_obstacles(scene)
    roadmap_nodes, roadmap_adjacency = build_visibility_roadmap(obstacles)

    import time

    build_started = time.perf_counter()
    segment_records: list[dict[str, object]] = []
    failure: dict[str, object] | None = None
    for segment_index, (source_label, target_label) in enumerate(zip(visit_sequence[:-1], visit_sequence[1:], strict=True)):
        segment_started = time.perf_counter()
        elapsed_before = segment_started - build_started
        if elapsed_before > time_budget_s:
            failure = {
                "reason": "time_budget_exceeded",
                "segment_index": segment_index,
                "source_label": source_label,
                "target_label": target_label,
            }
            break
        segment_type = classify_task2_segment_type(
            source_label,
            target_label,
            scene=scene,
            weld_lookup=weld_lookup,
        )
        if segment_type == "same_object_neighbor":
            polyline, planning_method = contour_following_polyline_for_segment(
                source_label,
                target_label,
                scene=scene,
                weld_lookup=weld_lookup,
                obstacles=obstacles,
            )
            result = (polyline, [])
        else:
            visibility_result = shortest_workspace_polyline_for_segment(
                source_label,
                target_label,
                scene=scene,
                weld_lookup=weld_lookup,
                obstacles=obstacles,
                roadmap_nodes=roadmap_nodes,
                roadmap_adjacency=roadmap_adjacency,
            )
            if visibility_result is None:
                result = None
                planning_method = "visibility_graph"
            else:
                result = visibility_result
                planning_method = "visibility_graph"
        runtime_s = time.perf_counter() - segment_started
        if result is None:
            print(
                f"stage1 workspace segment index={segment_index} type={segment_type} {source_label}->{target_label} "
                f"path_length=NONE runtime_s={runtime_s:.3f}",
                flush=True,
            )
            failure = {
                "reason": "no_workspace_path",
                "segment_index": segment_index,
                "source_label": source_label,
                "target_label": target_label,
                "runtime_s": runtime_s,
            }
            break
        polyline, path_ids = result
        path_length = _polyline_length(polyline)
        print(
            f"stage1 workspace segment index={segment_index} type={segment_type} {source_label}->{target_label} "
            f"path_length={path_length:.6f} runtime_s={runtime_s:.3f}",
            flush=True,
        )
        segment_records.append(
            {
                "segment_index": segment_index,
                "segment_type": segment_type,
                "planning_method": planning_method,
                "source_label": source_label,
                "target_label": target_label,
                "workspace_polyline": [list(point) for point in polyline],
                "workspace_path_length": path_length,
                "graph_path_ids": path_ids,
                "runtime_s": runtime_s,
            }
        )

    total_runtime_s = time.perf_counter() - build_started
    completed = failure is None and len(segment_records) == len(visit_sequence) - 1
    return {
        "task1_weld_order": weld_order,
        "task1_cycle_node_order": visit_sequence,
        "obstacle_model": {
            "inflation_radius_mm": TORCH_TIP_INFLATION_RADIUS_MM,
            "justification": "torch_radius 5mm + clearance 1mm from current scene assumptions",
            "circle_polygon_sides": CIRCLE_POLYGON_SIDES,
        },
        "segment_records": segment_records,
        "segment_count_completed": len(segment_records),
        "segment_count_expected": len(visit_sequence) - 1,
        "stage_status": "SUCCESS" if completed else "FAILED",
        "failure": failure,
        "total_workspace_path_length": sum(record["workspace_path_length"] for record in segment_records),
        "runtime_s": total_runtime_s,
    }


def _build_task2_cartesian_lifted_payload(
    *,
    workspace_paths_path: str | Path,
    segment_records: list[dict[str, object]],
    segment_count_expected: int,
    repair_records: list[dict[str, object]],
    unresolved_segments: list[dict[str, object]],
    runtime_s: float,
    resume_start_index: int,
) -> dict[str, object]:
    completed = not unresolved_segments and len(segment_records) == segment_count_expected
    return {
        "workspace_paths_artifact": str(Path(workspace_paths_path)),
        "segment_records": segment_records,
        "segment_count_completed": len(segment_records),
        "segment_count_expected": segment_count_expected,
        "stage_status": "SUCCESS" if completed else "FAILED",
        "failure": None if not unresolved_segments else unresolved_segments[0],
        "unresolved_segments": unresolved_segments,
        "repair_records": repair_records,
        "runtime_s": runtime_s,
        "resume_start_index": resume_start_index,
    }


def _build_task2_cartesian_repair_report_payload(
    lifted_payload: dict[str, object],
) -> dict[str, object]:
    repair_records = [dict(record) for record in lifted_payload.get("repair_records", [])]
    segment_type_by_index: dict[int, str] = {}
    workspace_paths_artifact = lifted_payload.get("workspace_paths_artifact")
    if workspace_paths_artifact is not None and Path(str(workspace_paths_artifact)).exists():
        workspace_payload = _read_json(workspace_paths_artifact)
        segment_type_by_index = {
            int(record["segment_index"]): str(record.get("segment_type", "inter_object"))
            for record in workspace_payload.get("segment_records", [])
        }
    for record in repair_records:
        segment_index = int(record["segment_index"])
        if record.get("segment_type") is None and segment_index in segment_type_by_index:
            record["segment_type"] = segment_type_by_index[segment_index]
        if "fallback_used" not in record:
            record["fallback_used"] = record.get("repair_level_succeeded") not in (None, "plain_lift", "same_object_contour_interpolator")
        if "contour_interpolator_succeeded" not in record:
            record["contour_interpolator_succeeded"] = False
    unresolved_segments = list(lifted_payload.get("unresolved_segments", []))
    return {
        "segment_repairs": repair_records,
        "same_object_segment_count": sum(1 for record in repair_records if record.get("segment_type") == "same_object_neighbor"),
        "inter_object_segment_count": sum(1 for record in repair_records if record.get("segment_type") == "inter_object"),
        "contour_interpolator_success_count": sum(1 for record in repair_records if record.get("contour_interpolator_succeeded")),
        "fallback_used_segment_indices": [record["segment_index"] for record in repair_records if record.get("fallback_used")],
        "repaired_segment_indices": [record["segment_index"] for record in repair_records if record["repair_level_succeeded"] not in (None, "plain_lift", "same_object_contour_interpolator")],
        "unresolved_segment_indices": [record["segment_index"] for record in repair_records if record["unresolved"]],
        "unresolved_segments": unresolved_segments,
        "repair_levels_used": sorted({record["repair_level_succeeded"] for record in repair_records if record["repair_level_succeeded"] is not None}),
    }


def build_task2_cartesian_lifted_paths(
    workspace_paths_path: str | Path = TASK2_CARTESIAN_WORKSPACE_ARTIFACT_PATH,
    *,
    time_budget_s: float = 300.0,
    lifted_artifact_path: str | Path = TASK2_CARTESIAN_LIFTED_ARTIFACT_PATH,
    repair_report_path: str | Path = TASK2_CARTESIAN_REPAIR_REPORT_PATH,
) -> dict[str, object]:
    scene = build_default_scene()
    weld_lookup = _weld_lookup(scene)
    workspace_payload = _read_json(workspace_paths_path)
    segment_records_in = list(workspace_payload["segment_records"])
    base_obstacles = build_inflated_workspace_obstacles(scene)
    base_roadmap_nodes, base_roadmap_adjacency = build_visibility_roadmap(base_obstacles)

    import time

    existing_segment_records: list[dict[str, object]] = []
    existing_repair_records: list[dict[str, object]] = []
    if Path(lifted_artifact_path).exists():
        existing_payload = _read_json(lifted_artifact_path)
        if str(existing_payload.get("workspace_paths_artifact")) == str(Path(workspace_paths_path)):
            existing_segment_records = list(existing_payload.get("segment_records", []))
            existing_repair_records = list(existing_payload.get("repair_records", []))
    resume_start_index = min(len(existing_segment_records), len(segment_records_in))
    print(f"stage2 resume start index={resume_start_index}", flush=True)

    lift_started = time.perf_counter()
    lifted_segment_records: list[dict[str, object]] = existing_segment_records[:resume_start_index]
    repair_records: list[dict[str, object]] = existing_repair_records[:resume_start_index]
    unresolved_segments: list[dict[str, object]] = []
    for record in segment_records_in[resume_start_index:]:
        segment_index = int(record["segment_index"])
        source_label = str(record["source_label"])
        target_label = str(record["target_label"])
        segment_type = str(record.get("segment_type", "inter_object"))
        segment_started = time.perf_counter()
        if segment_started - lift_started > time_budget_s:
            unresolved_segments.append(
                {
                    "reason": "time_budget_exceeded",
                    "segment_index": segment_index,
                    "segment_type": segment_type,
                    "source_label": source_label,
                    "target_label": target_label,
                }
            )
            break
        print(
            f"stage2 processing segment index={segment_index} type={segment_type} {source_label}->{target_label}",
            flush=True,
        )
        lifted_record, repair_report = _stage2_repair_segment(
            record,
            scene=scene,
            weld_lookup=weld_lookup,
            base_obstacles=base_obstacles,
            base_roadmap_nodes=base_roadmap_nodes,
            base_roadmap_adjacency=base_roadmap_adjacency,
        )
        repair_records.append(repair_report)
        lifted_segment_records.append(lifted_record)
        if segment_type == "same_object_neighbor":
            print(
                f"stage2 lift segment index={segment_index} type={segment_type} {source_label}->{target_label} "
                f"contour_success={repair_report['contour_interpolator_succeeded']} "
                f"fallback_used={repair_report['fallback_used']} "
                f"validation_succeeded={repair_report['validation_succeeded']} "
                f"runtime_s={float(repair_report['runtime_s']):.3f}",
                flush=True,
            )
        else:
            print(
                f"stage2 lift segment index={segment_index} type={segment_type} {source_label}->{target_label} "
                f"repair_level={repair_report['repair_level_succeeded']} "
                f"validation_succeeded={repair_report['validation_succeeded']} "
                f"runtime_s={float(repair_report['runtime_s']):.3f}",
                flush=True,
            )
        partial_payload = _build_task2_cartesian_lifted_payload(
            workspace_paths_path=workspace_paths_path,
            segment_records=lifted_segment_records,
            segment_count_expected=len(segment_records_in),
            repair_records=repair_records,
            unresolved_segments=unresolved_segments,
            runtime_s=time.perf_counter() - lift_started,
            resume_start_index=resume_start_index,
        )
        _save_json(partial_payload, lifted_artifact_path)
        _save_json(_build_task2_cartesian_repair_report_payload(partial_payload), repair_report_path)
        if repair_report["unresolved"]:
            unresolved_segments.append(
                {
                    "reason": "ik_lift_failed",
                    "segment_index": segment_index,
                    "segment_type": segment_type,
                    "source_label": source_label,
                    "target_label": target_label,
                }
            )
            partial_payload = _build_task2_cartesian_lifted_payload(
                workspace_paths_path=workspace_paths_path,
                segment_records=lifted_segment_records,
                segment_count_expected=len(segment_records_in),
                repair_records=repair_records,
                unresolved_segments=unresolved_segments,
                runtime_s=time.perf_counter() - lift_started,
                resume_start_index=resume_start_index,
            )
            _save_json(partial_payload, lifted_artifact_path)
            _save_json(_build_task2_cartesian_repair_report_payload(partial_payload), repair_report_path)
            break

    total_runtime_s = time.perf_counter() - lift_started
    payload = _build_task2_cartesian_lifted_payload(
        workspace_paths_path=workspace_paths_path,
        segment_records=lifted_segment_records,
        segment_count_expected=len(segment_records_in),
        repair_records=repair_records,
        unresolved_segments=unresolved_segments,
        runtime_s=total_runtime_s,
        resume_start_index=resume_start_index,
    )
    _save_json(payload, lifted_artifact_path)
    _save_json(_build_task2_cartesian_repair_report_payload(payload), repair_report_path)
    return payload


def build_task2_cartesian_repair_report(
    lifted_paths_path: str | Path = TASK2_CARTESIAN_LIFTED_ARTIFACT_PATH,
) -> dict[str, object]:
    lifted_payload = _read_json(lifted_paths_path)
    return _build_task2_cartesian_repair_report_payload(lifted_payload)


def assemble_task2_cartesian_paths(
    workspace_paths_path: str | Path = TASK2_CARTESIAN_WORKSPACE_ARTIFACT_PATH,
    lifted_paths_path: str | Path = TASK2_CARTESIAN_LIFTED_ARTIFACT_PATH,
) -> dict[str, object]:
    workspace_payload = _read_json(workspace_paths_path)
    lifted_payload = _read_json(lifted_paths_path)
    if workspace_payload["stage_status"] != "SUCCESS":
        raise ValueError("Stage 1 workspace artifact is not complete.")
    if lifted_payload["stage_status"] != "SUCCESS":
        raise ValueError("Stage 2 lifted artifact is not complete.")

    workspace_records = list(workspace_payload["segment_records"])
    lifted_records = list(lifted_payload["segment_records"])
    if len(workspace_records) != len(lifted_records):
        raise ValueError("Workspace/lifted segment counts do not match.")

    dp_costs: list[dict[str, float]] = [{}]
    predecessors: list[dict[str, tuple[str, dict[str, object]]]] = [{}]
    for segment_index, lifted_record in enumerate(lifted_records):
        realizations = list(lifted_record["realizations"])
        if segment_index == 0:
            next_costs: dict[str, float] = {}
            next_predecessors: dict[str, tuple[str, dict[str, object]]] = {}
            for realization in realizations:
                cost = float(realization["joint_space_path_length"])
                target_state_id = str(realization["target_state_id"])
                if cost >= next_costs.get(target_state_id, float("inf")):
                    continue
                next_costs[target_state_id] = cost
                next_predecessors[target_state_id] = (str(realization["source_state_id"]), realization)
            if not next_costs:
                raise ValueError("No feasible lifted realizations for segment 0.")
            dp_costs.append(next_costs)
            predecessors.append(next_predecessors)
            continue

        current_costs = dp_costs[segment_index]
        next_costs = {}
        next_predecessors = {}
        for realization in realizations:
            source_state_id = str(realization["source_state_id"])
            source_cost = current_costs.get(source_state_id)
            if source_cost is None:
                continue
            new_cost = source_cost + float(realization["joint_space_path_length"])
            target_state_id = str(realization["target_state_id"])
            if new_cost >= next_costs.get(target_state_id, float("inf")):
                continue
            next_costs[target_state_id] = new_cost
            next_predecessors[target_state_id] = (source_state_id, realization)
        if not next_costs:
            raise ValueError(f"No globally consistent lifted chain exists at segment {segment_index}.")
        dp_costs.append(next_costs)
        predecessors.append(next_predecessors)

    final_state_id = min(dp_costs[-1], key=dp_costs[-1].get)
    chosen_realizations_reversed: list[dict[str, object]] = []
    current_state_id = final_state_id
    for segment_index in range(len(lifted_records), 0, -1):
        predecessor_state_id, realization = predecessors[segment_index][current_state_id]
        chosen_realizations_reversed.append(realization)
        current_state_id = predecessor_state_id
    chosen_realizations = list(reversed(chosen_realizations_reversed))

    ordered_segment_records: list[dict[str, object]] = []
    total_workspace_path_length = 0.0
    total_joint_path_length = 0.0
    for workspace_record, lifted_record, realization in zip(workspace_records, lifted_records, chosen_realizations, strict=True):
        total_workspace_path_length += float(workspace_record["workspace_path_length"])
        total_joint_path_length += float(realization["joint_space_path_length"])
        ordered_segment_records.append(
            {
                "segment_index": int(workspace_record["segment_index"]),
                "segment_type": workspace_record.get("segment_type"),
                "source_label": workspace_record["source_label"],
                "target_label": workspace_record["target_label"],
                "source_state_id": realization["source_state_id"],
                "target_state_id": realization["target_state_id"],
                "workspace_polyline": workspace_record["workspace_polyline"],
                "workspace_path_length": workspace_record["workspace_path_length"],
                "cartesian_waypoints": lifted_record["cartesian_waypoints"],
                "chosen_joint_waypoints": realization["chosen_joint_waypoints"],
                "validation_result": bool(realization["validation_result"]),
                "local_repair_used": False,
                "local_repair_reason": None,
                "joint_space_path_length": realization["joint_space_path_length"],
            }
        )

    return {
        "task1_weld_order": workspace_payload["task1_weld_order"],
        "task1_cycle_node_order": workspace_payload["task1_cycle_node_order"],
        "obstacle_model": workspace_payload["obstacle_model"],
        "ordered_segment_records": ordered_segment_records,
        "segment_count": len(ordered_segment_records),
        "collision_free_segment_count": sum(1 for record in ordered_segment_records if record["validation_result"]),
        "locally_repaired_segment_indices": [],
        "total_workspace_path_length": total_workspace_path_length,
        "total_joint_path_length": total_joint_path_length,
        "all_segments_valid": all(record["validation_result"] for record in ordered_segment_records),
    }


def save_task2_cartesian_paths(
    payload: dict[str, object],
    path: str | Path = TASK2_CARTESIAN_ARTIFACT_PATH,
) -> Path:
    return _save_json(payload, path)
