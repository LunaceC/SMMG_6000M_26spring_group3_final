"""Plot the clean Euclidean Task 1 node set, cycle, and distance matrix."""

from __future__ import annotations

from pathlib import Path
import json
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SOLUTION_PATH = REPO_ROOT / "artifacts" / "task1_hamiltonian_solution.json"
NODES_FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_nodes.png"
CYCLE_FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_cycle.png"
MATRIX_FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task1_distance_matrix.png"


def _read_json(path: Path) -> dict[str, object]:
    """Read one JSON artifact."""
    return json.loads(path.read_text(encoding="utf-8"))


def _load_matplotlib():
    """Import matplotlib with a non-interactive backend."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _node_array(payload: dict[str, object]) -> tuple[list[str], np.ndarray]:
    """Return node IDs and coordinates in artifact order."""
    node_ids = list(payload["node_ids"])
    coordinates = np.array([payload["node_coordinates"][node_id] for node_id in node_ids], dtype=float)
    return node_ids, coordinates


def plot_task1_hamiltonian_graph() -> list[Path]:
    """Save all Task 1 Hamiltonian figures."""
    payload = _read_json(SOLUTION_PATH)
    plt = _load_matplotlib()
    node_ids, coordinates = _node_array(payload)
    depot_index = node_ids.index("DEPOT")

    NODES_FIGURE_PATH.parent.mkdir(parents=True, exist_ok=True)

    fig_nodes, ax_nodes = plt.subplots(figsize=(8.8, 6.0))
    ax_nodes.scatter(coordinates[:, 0], coordinates[:, 1], s=28.0, color="0.3", zorder=3)
    ax_nodes.scatter(
        [coordinates[depot_index, 0]],
        [coordinates[depot_index, 1]],
        s=70.0,
        marker="s",
        color="black",
        zorder=5,
        label="DEPOT",
    )
    for node_id, (x_pos, y_pos) in zip(node_ids, coordinates, strict=True):
        ax_nodes.text(x_pos + 2.0, y_pos + 2.0, node_id, fontsize=7)
    ax_nodes.set_title("Task 1 Hamiltonian Nodes")
    ax_nodes.set_xlabel("x [mm]")
    ax_nodes.set_ylabel("y [mm]")
    ax_nodes.set_aspect("equal", adjustable="box")
    ax_nodes.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    ax_nodes.legend(loc="upper left", fontsize=8)
    fig_nodes.tight_layout()
    fig_nodes.savefig(NODES_FIGURE_PATH, dpi=180)
    plt.close(fig_nodes)

    fig_cycle, ax_cycle = plt.subplots(figsize=(8.8, 6.0))
    ax_cycle.scatter(coordinates[:, 0], coordinates[:, 1], s=24.0, color="0.45", zorder=3)
    ax_cycle.scatter(
        [coordinates[depot_index, 0]],
        [coordinates[depot_index, 1]],
        s=80.0,
        marker="s",
        color="black",
        zorder=6,
    )
    cycle_ids = payload.get("cycle_node_order", [])
    if cycle_ids:
        cycle_points = np.array([payload["node_coordinates"][node_id] for node_id in cycle_ids], dtype=float)
        ax_cycle.plot(cycle_points[:, 0], cycle_points[:, 1], color="tab:blue", linewidth=1.8, zorder=2)
        for visit_index, weld_id in enumerate(payload.get("weld_order", []), start=1):
            x_pos, y_pos = payload["node_coordinates"][weld_id]
            ax_cycle.text(x_pos + 2.0, y_pos + 2.0, str(visit_index), fontsize=8, color="black")
    for node_id, (x_pos, y_pos) in zip(node_ids, coordinates, strict=True):
        if node_id == "DEPOT":
            ax_cycle.text(x_pos + 3.0, y_pos + 3.0, "DEPOT", fontsize=9, fontweight="bold")
    ax_cycle.set_title("Task 1 Hamiltonian Cycle")
    ax_cycle.set_xlabel("x [mm]")
    ax_cycle.set_ylabel("y [mm]")
    ax_cycle.set_aspect("equal", adjustable="box")
    ax_cycle.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    fig_cycle.tight_layout()
    fig_cycle.savefig(CYCLE_FIGURE_PATH, dpi=180)
    plt.close(fig_cycle)

    fig_matrix, ax_matrix = plt.subplots(figsize=(8.2, 7.2))
    matrix = np.array(payload["distance_matrix"], dtype=float)
    image = ax_matrix.imshow(matrix, cmap="viridis", origin="upper")
    ax_matrix.set_title("Task 1 Euclidean Distance Matrix")
    ax_matrix.set_xticks(range(len(node_ids)))
    ax_matrix.set_yticks(range(len(node_ids)))
    ax_matrix.set_xticklabels(node_ids, rotation=90, fontsize=6)
    ax_matrix.set_yticklabels(node_ids, fontsize=6)
    fig_matrix.colorbar(image, ax=ax_matrix, fraction=0.046, pad=0.04, label="distance [mm]")
    fig_matrix.tight_layout()
    fig_matrix.savefig(MATRIX_FIGURE_PATH, dpi=180)
    plt.close(fig_matrix)

    print(f"saved figure: {NODES_FIGURE_PATH}", flush=True)
    print(f"saved figure: {CYCLE_FIGURE_PATH}", flush=True)
    print(f"saved figure: {MATRIX_FIGURE_PATH}", flush=True)
    return [NODES_FIGURE_PATH, CYCLE_FIGURE_PATH, MATRIX_FIGURE_PATH]


def main() -> None:
    """Plot all Task 1 Hamiltonian figures."""
    plot_task1_hamiltonian_graph()


if __name__ == "__main__":
    main()
