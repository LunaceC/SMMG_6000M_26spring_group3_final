"""Report the clean Euclidean Hamiltonian-cycle Task 1 result."""

from __future__ import annotations

from pathlib import Path
import json
import sys

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


def main() -> None:
    """Print the clean Task 1 Hamiltonian summary."""
    payload = _read_json(SOLUTION_PATH)
    print(f"solver backend: {payload.get('solver_backend')}")
    print(f"solver status: {payload.get('solver_status')}")
    print(f"proof_optimal: {payload.get('proof_optimal')}")
    print(f"total Euclidean cycle length: {payload.get('total_euclidean_cycle_length')}")
    print(f"weld order: {payload.get('weld_order')}")
    print(
        "figure paths: "
        f"{[str(path) for path in (NODES_FIGURE_PATH, CYCLE_FIGURE_PATH, MATRIX_FIGURE_PATH) if path.exists()]}"
    )


if __name__ == "__main__":
    main()
