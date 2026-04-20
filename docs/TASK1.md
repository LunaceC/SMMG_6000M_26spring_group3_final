# Task 1

## Role

Task 1 is the weld-sequence solver for the collaborator package. It computes a Hamiltonian cycle / symmetric TSP over the depot and all `24` weld points using Euclidean workspace distance as the active objective.

Task 1 does not perform collision reasoning. Its output is the visit order that Task 2 must follow.

## Active modules and scripts

Code:
- `task1_hamiltonian.py`
- `scene.py`
- `welds.py`

Scripts:
- `scripts/solve_task1_hamiltonian.py`
- `scripts/plot_task1_hamiltonian_graph.py`
- `scripts/report_task1_hamiltonian.py`
- `scripts/check_task1_active_pipeline.py`

Tests:
- `tests/test_task1_hamiltonian_smoke.py`
- `tests/test_task1_active_pipeline_smoke.py`

## Artifacts and figures

Authoritative Task 1 artifact:
- `artifacts/task1_hamiltonian_solution.json`

Task 1 figures:
- `artifacts/figures/task1_hamiltonian_nodes.png`
- `artifacts/figures/task1_hamiltonian_cycle.png`
- `artifacts/figures/task1_distance_matrix.png`

## How to run Task 1

After activating the project virtual environment:

```bash
python scripts/solve_task1_hamiltonian.py
python scripts/plot_task1_hamiltonian_graph.py
python scripts/report_task1_hamiltonian.py
```

`scripts/solve_task1_hamiltonian.py` requires `ortools` from `requirements.txt`.

## What Task 1 outputs into Task 2

Task 2 consumes `artifacts/task1_hamiltonian_solution.json`, especially:
- `cycle_node_order`
- `weld_order`
- `node_coordinates`
- `total_euclidean_cycle_length`

The committed frozen weld order currently begins:
- `P2_W4 -> P2_W3 -> P2_W2 -> C3_W0 -> ...`
