"""Solve the clean Euclidean Hamiltonian-cycle Task 1 problem."""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from task1_hamiltonian import build_task1_solution_payload, save_task1_solution


def main() -> None:
    """Solve and save the clean Task 1 Hamiltonian artifact."""
    payload = build_task1_solution_payload()
    path = save_task1_solution(payload)
    print(f"solver backend: {payload['solver_backend']}", flush=True)
    print(f"solver status: {payload['solver_status']}", flush=True)
    print(f"proof_optimal: {payload['proof_optimal']}", flush=True)
    print(f"total Euclidean cycle length: {payload.get('total_euclidean_cycle_length')}", flush=True)
    print(path, flush=True)


if __name__ == "__main__":
    main()
