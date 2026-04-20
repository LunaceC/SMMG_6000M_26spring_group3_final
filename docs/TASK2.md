# Task 2

## Role

Task 2 is the active collision-aware stage in this collaborator package. It consumes the frozen Task 1 weld order and generates collision-free paths between consecutive visits.

## Active Task 2 method

The kept Task 2 pipeline is:
- plan torch-tip motion in Cartesian workspace first
- lift each segment to robot joint states with IK
- choose a globally consistent lifted chain across the full visit order
- validate the lifted motion with the active Link-3 / torch collision checker in `collision.py`

## Active modules and scripts

Code:
- `task2_cartesian_paths.py`
- `collision.py`
- `kinematics.py`
- `scene.py`
- `welds.py`

Scripts:
- `scripts/build_task2_cartesian_paths.py`
- `scripts/plot_task2_cartesian_paths.py`
- `scripts/report_task2_cartesian_paths.py`
- `scripts/check_task2_active_pipeline.py`

Tests:
- `tests/test_task2_cartesian_paths_smoke.py`
- `tests/test_task2_active_pipeline_smoke.py`

## Artifacts and figures

Authoritative Task 2 artifact:
- `artifacts/task2_cartesian_paths.json`

Task 2 figure:
- `artifacts/figures/task2_cartesian_paths.png`

Current frozen artifact summary:
- `segment_count = 25`
- `collision_free_segment_count = 25`
- `all_segments_valid = true`
- obstacle model uses `inflation_radius_mm = 6.0`
- obstacle model uses `circle_polygon_sides = 48`

## How to run Task 2

Task 2 expects `artifacts/task1_hamiltonian_solution.json` to already exist.

After activating the project virtual environment:

```bash
python scripts/build_task2_cartesian_paths.py
python scripts/plot_task2_cartesian_paths.py
python scripts/report_task2_cartesian_paths.py
```

If a fresh full rebuild is slow on your machine, continue from the committed `artifacts/task2_cartesian_paths.json`. That frozen artifact remains the authoritative Task 2 result in this branch.

## What Task 2 outputs into Task 3

Task 3 should consume `artifacts/task2_cartesian_paths.json`.

The main payload pieces exported by Task 2 are:
- `task1_cycle_node_order`
- `task1_weld_order`
- `obstacle_model`
- `ordered_segment_records`
- `segment_count`
- `collision_free_segment_count`
- `all_segments_valid`

Each ordered segment record includes:
- source and target visit labels
- chosen source and target robot state ids
- workspace polyline and Cartesian waypoints
- chosen joint waypoints
- per-segment validity flags
- workspace and joint-space path-length values
