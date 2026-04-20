"""Exact Euclidean Hamiltonian-cycle Task 1 solver for depot + weld points."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import hypot
from pathlib import Path
import json
import sys
from typing import Iterable

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scene import build_default_scene
from welds import generate_weld_points


SOLUTION_PATH = REPO_ROOT / "artifacts" / "task1_hamiltonian_solution.json"
FIGURE_DIR = REPO_ROOT / "artifacts" / "figures"


@dataclass(frozen=True)
class HamiltonianNode:
    """One Euclidean Task 1 node."""

    id: str
    x: float
    y: float


@dataclass(frozen=True)
class HamiltonianGraph:
    """Complete weighted Euclidean graph for Task 1."""

    nodes: tuple[HamiltonianNode, ...]
    distance_matrix: tuple[tuple[float, ...], ...]


def _import_mpsolver():
    """Import OR-Tools MPSolver from the local virtualenv when needed."""
    try:
        from ortools.linear_solver import pywraplp  # type: ignore

        return pywraplp, "ortools_mpsolver"
    except ModuleNotFoundError:
        version_tag = f"python{sys.version_info.major}.{sys.version_info.minor}"
        site_packages = REPO_ROOT / ".venv_task1_exact" / "lib" / version_tag / "site-packages"
        if site_packages.exists() and str(site_packages) not in sys.path:
            sys.path.insert(0, str(site_packages))
        from ortools.linear_solver import pywraplp  # type: ignore

        return pywraplp, "ortools_mpsolver_local_venv"


def build_hamiltonian_graph() -> HamiltonianGraph:
    """Return the depot+weld Euclidean graph required by the project brief."""
    scene = build_default_scene()
    weld_points = generate_weld_points(scene)
    nodes = [HamiltonianNode(id="DEPOT", x=scene.depot[0], y=scene.depot[1])]
    nodes.extend(HamiltonianNode(id=weld.id, x=weld.position[0], y=weld.position[1]) for weld in weld_points)

    node_array = np.array([(node.x, node.y) for node in nodes], dtype=float)
    matrix = np.zeros((len(nodes), len(nodes)), dtype=float)
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            distance = hypot(node_array[i, 0] - node_array[j, 0], node_array[i, 1] - node_array[j, 1])
            matrix[i, j] = distance
            matrix[j, i] = distance

    return HamiltonianGraph(
        nodes=tuple(nodes),
        distance_matrix=tuple(tuple(float(value) for value in row) for row in matrix.tolist()),
    )


def _edge_pairs(node_count: int) -> list[tuple[int, int]]:
    """Return all undirected node pairs i<j."""
    return [(i, j) for i in range(node_count) for j in range(i + 1, node_count)]


def _selected_adjacency(selected_edges: Iterable[tuple[int, int]], node_count: int) -> dict[int, set[int]]:
    """Return adjacency sets for one undirected edge selection."""
    adjacency = {node_index: set() for node_index in range(node_count)}
    for i, j in selected_edges:
        adjacency[i].add(j)
        adjacency[j].add(i)
    return adjacency


def _connected_components(selected_edges: Iterable[tuple[int, int]], node_count: int) -> list[list[int]]:
    """Return connected components of one candidate undirected degree-2 solution."""
    adjacency = _selected_adjacency(selected_edges, node_count)
    unseen = set(range(node_count))
    components: list[list[int]] = []
    while unseen:
        start = unseen.pop()
        stack = [start]
        component = [start]
        while stack:
            node = stack.pop()
            for neighbor in adjacency[node]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    stack.append(neighbor)
                    component.append(neighbor)
        components.append(sorted(component))
    return components


def _extract_cycle(selected_edges: Iterable[tuple[int, int]], node_count: int) -> list[int]:
    """Return one undirected Hamiltonian cycle as a node sequence with closure."""
    adjacency = _selected_adjacency(selected_edges, node_count)
    route = [0]
    previous = None
    current = 0
    while True:
        neighbors = sorted(adjacency[current])
        next_candidates = [neighbor for neighbor in neighbors if neighbor != previous]
        if not next_candidates:
            break
        next_node = next_candidates[0]
        route.append(next_node)
        if next_node == route[0]:
            break
        previous, current = current, next_node
        if len(route) > node_count + 1:
            raise ValueError("Extracted cycle exceeded node count.")
    if len(route) != node_count + 1 or route[0] != route[-1]:
        raise ValueError("Selected edges do not form one Hamiltonian cycle.")
    return route


def _rotate_cycle_to_depot(cycle_indices: list[int]) -> list[int]:
    """Rotate a closed cycle so it starts and ends at DEPOT index 0."""
    depot_position = cycle_indices.index(0)
    rotated = cycle_indices[depot_position:-1] + cycle_indices[1:depot_position + 1]
    rotated.append(0)
    return rotated


def _solver_status_name(pywraplp, status_code: int) -> str:
    """Return one readable MPSolver status name."""
    names = {
        pywraplp.Solver.OPTIMAL: "OPTIMAL",
        pywraplp.Solver.FEASIBLE: "FEASIBLE",
        pywraplp.Solver.INFEASIBLE: "INFEASIBLE",
        pywraplp.Solver.UNBOUNDED: "UNBOUNDED",
        pywraplp.Solver.ABNORMAL: "ABNORMAL",
        pywraplp.Solver.NOT_SOLVED: "NOT_SOLVED",
    }
    return names.get(status_code, f"STATUS_{status_code}")


def solve_task1_hamiltonian_exact(
    graph: HamiltonianGraph,
    *,
    backend_preference: str | None = None,
) -> dict[str, object]:
    """Solve the exact Euclidean Hamiltonian cycle with Concorde fallback logic."""
    node_count = len(graph.nodes)
    matrix = np.array(graph.distance_matrix, dtype=float)

    concorde_path = None
    solver_backend = None
    if concorde_path is not None:
        solver_backend = "concorde"

    pywraplp, mpsolver_backend = _import_mpsolver()
    solver = pywraplp.Solver.CreateSolver("SCIP")
    if solver is None:
        return {
            "solver_backend": None,
            "solver_status": "NO_EXACT_BACKEND",
            "proof_optimal": False,
            "reason": "Neither Concorde nor SCIP exact backend is available.",
        }

    solver_backend = backend_preference or f"{mpsolver_backend}_scip"
    edge_pairs = _edge_pairs(node_count)
    x = {(i, j): solver.BoolVar(f"x_{i}_{j}") for i, j in edge_pairs}

    for node_index in range(node_count):
        incident = [
            x[(i, j)]
            for i, j in edge_pairs
            if i == node_index or j == node_index
        ]
        solver.Add(sum(incident) == 2)

    objective = solver.Objective()
    for i, j in edge_pairs:
        objective.SetCoefficient(x[(i, j)], float(matrix[i, j]))
    objective.SetMinimization()

    subtour_rounds: list[dict[str, object]] = []
    while True:
        status = solver.Solve()
        status_name = _solver_status_name(pywraplp, status)
        selected_edges = [(i, j) for (i, j), variable in x.items() if variable.solution_value() > 0.5]
        if status != pywraplp.Solver.OPTIMAL:
            return {
                "solver_backend": solver_backend,
                "solver_status": status_name,
                "proof_optimal": False,
                "reason": "Exact SCIP solve did not return OPTIMAL.",
                "subtour_rounds": subtour_rounds,
            }

        components = _connected_components(selected_edges, node_count)
        subtour_rounds.append(
            {
                "round_index": len(subtour_rounds) + 1,
                "selected_edge_count": len(selected_edges),
                "component_sizes": [len(component) for component in components],
            }
        )
        if len(components) == 1:
            cycle_indices = _rotate_cycle_to_depot(_extract_cycle(selected_edges, node_count))
            cycle_node_order = [graph.nodes[index].id for index in cycle_indices]
            weld_order = [node_id for node_id in cycle_node_order[1:-1] if node_id != "DEPOT"]
            total_length = sum(
                float(matrix[cycle_indices[k], cycle_indices[k + 1]])
                for k in range(len(cycle_indices) - 1)
            )
            return {
                "solver_backend": solver_backend,
                "solver_status": status_name,
                "proof_optimal": True,
                "total_euclidean_cycle_length": total_length,
                "cycle_node_order": cycle_node_order,
                "weld_order": weld_order,
                "selected_edges": [(graph.nodes[i].id, graph.nodes[j].id) for i, j in selected_edges],
                "subtour_rounds": subtour_rounds,
            }

        for component in components:
            if len(component) == node_count:
                continue
            component_edges = [
                x[(i, j)]
                for i in component
                for j in component
                if i < j
            ]
            solver.Add(sum(component_edges) <= len(component) - 1)


def build_task1_solution_payload() -> dict[str, object]:
    """Build and solve the clean Task 1 Hamiltonian-cycle problem."""
    graph = build_hamiltonian_graph()
    solution = solve_task1_hamiltonian_exact(graph)

    payload = {
        "problem_definition": "Hamiltonian cycle / symmetric TSP over DEPOT + 24 weld points using Euclidean distance only.",
        "node_ids": [node.id for node in graph.nodes],
        "node_coordinates": {node.id: [node.x, node.y] for node in graph.nodes},
        "distance_matrix": [list(row) for row in graph.distance_matrix],
        "solver_backend": solution["solver_backend"],
        "solver_status": solution["solver_status"],
        "proof_optimal": solution["proof_optimal"],
    }
    payload.update(solution)
    return payload


def save_task1_solution(payload: dict[str, object], path: Path = SOLUTION_PATH) -> Path:
    """Save the clean Task 1 Hamiltonian artifact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return path


def main() -> None:
    """Solve and save the clean Task 1 Hamiltonian artifact."""
    payload = build_task1_solution_payload()
    save_task1_solution(payload)
    print(f"solver backend: {payload['solver_backend']}", flush=True)
    print(f"solver status: {payload['solver_status']}", flush=True)
    print(f"proof_optimal: {payload['proof_optimal']}", flush=True)
    print(f"total Euclidean cycle length: {payload.get('total_euclidean_cycle_length')}", flush=True)
    print(SOLUTION_PATH, flush=True)


if __name__ == "__main__":
    main()
