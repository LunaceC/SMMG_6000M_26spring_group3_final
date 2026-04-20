"""Print the active Task 1 paths for the collaborator package."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

ACTIVE_SOLVER_SCRIPT = REPO_ROOT / "scripts" / "solve_task1_hamiltonian.py"
ACTIVE_SOLUTION_ARTIFACT = REPO_ROOT / "artifacts" / "task1_hamiltonian_solution.json"
ACTIVE_FIGURE_PATHS = [
    REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_nodes.png",
    REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_cycle.png",
    REPO_ROOT / "artifacts" / "figures" / "task1_distance_matrix.png",
]
TASK1_DOC = REPO_ROOT / "docs" / "TASK1.md"


def main() -> None:
    """Print the active Task 1 pipeline paths."""
    print(f"active Task 1 solver script path: {ACTIVE_SOLVER_SCRIPT}", flush=True)
    print(f"active Task 1 solution artifact path: {ACTIVE_SOLUTION_ARTIFACT}", flush=True)
    print(f"active Task 1 figure paths: {[str(path) for path in ACTIVE_FIGURE_PATHS]}", flush=True)
    print(f"Task 1 package doc path: {TASK1_DOC}", flush=True)


if __name__ == "__main__":
    main()
