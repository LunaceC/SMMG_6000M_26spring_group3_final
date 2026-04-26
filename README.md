# Task1 / Task2 Collaborator Package

## What this repo is

This branch is the collaborator-facing handoff package for the planar welding project at the end of Task 2.

It keeps only:
- environment and robot model definitions
- Task 1 weld-sequence determination
- Task 2 collision-free path generation from the Task 1 sequence
- the upstream interface note that defines what Task 3 should consume from Task 2

Task 4 implementation are intentionally excluded from this branch.

## Quick start

Use a virtual environment so the dependencies install into an isolated interpreter instead of an externally managed system Python.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r requirements.txt
```

The virtual environment is the recommended path for collaborators because it avoids system-package restrictions and makes `ortools`, `numpy`, and `matplotlib` install cleanly.

## Project structure

- Root modules: the active scene, weld, kinematics, collision, Task 1, and Task 2 implementations.
- `scripts/`: entry points for solving, plotting, and reporting Task 1 and Task 2 results.
- `docs/`: the collaborator-facing problem summary, Task 1 note, Task 2 note, and Task 3 upstream interface contract.
- `artifacts/`: the frozen Task 1 and Task 2 JSON outputs plus their figures.
- `tests/`: the kept Task1/Task2 smoke and sanity tests for the collaborator package.

## Problem / robot summary

The project models a planar RRR welding robot in a fixed 2D scene with a depot, five obstacles, and a deterministic weld registry. Task 1 computes a depot-anchored weld visit order using Euclidean distance, and Task 2 turns that order into collision-checked Cartesian and lifted joint paths.

See `docs/PROBLEM_AND_MODEL.md` for the robot base convention, link lengths, tool and clearance assumptions, object naming, weld naming, and locked geometry assumptions.

## Task 1

Task 1 solves the weld visiting order over the depot plus all weld points.

```bash
python scripts/solve_task1_hamiltonian.py
python scripts/plot_task1_hamiltonian_graph.py
python scripts/report_task1_hamiltonian.py
```

Task 1 outputs:
- `artifacts/task1_hamiltonian_solution.json`
- `artifacts/figures/task1_hamiltonian_nodes.png`
- `artifacts/figures/task1_hamiltonian_cycle.png`
- `artifacts/figures/task1_distance_matrix.png`

See `docs/TASK1.md` for the Task 1 solver role, kept scripts, artifacts, and the exact fields passed downstream into Task 2.

## Task 2

Task 2 generates collision-free paths from the frozen Task 1 weld order.

```bash
python scripts/build_task2_cartesian_paths.py
python scripts/plot_task2_cartesian_paths.py
python scripts/report_task2_cartesian_paths.py
```

Task 2 outputs:
- `artifacts/task2_cartesian_paths.json`
- `artifacts/figures/task2_cartesian_paths.png`

The committed `artifacts/task2_cartesian_paths.json` is the authoritative frozen Task 2 artifact for this branch. If a fresh full Task 2 rebuild is slow in your environment, collaborators can still inspect, validate, and continue downstream work from that committed artifact.

See `docs/TASK2.md` for the active Task 2 method, kept scripts, artifacts, and the exact payload exported to Task 3.

## Task 3

Task 3 generates time-parameterized trajectories from the frozen Task 2 path artifact. It uses piecewise cubic polynomials in both joint space and Cartesian space.

```bash
python scripts/build_task3_trajectories.py
python scripts/plot_task3_trajectories.py
python scripts/report_task3_trajectories.py
```

Task 3 outputs:

- `artifacts/task3_trajectories.json`
- `artifacts/figures/task3_cartesian_xy_trajectory.png`
- `artifacts/figures/task3_joint_trajectory_segment0.png`
- `artifacts/figures/task3_cartesian_trajectory_segment0.png`

See `docs/TASK3.md` for the Task 3 interpolation method, scripts, artifacts, and output schema.

## Validation

Run the kept collaborator-package test subset with:

```bash
python -m unittest tests.test_scene_welds tests.test_kinematics tests.test_collision tests.test_task1_hamiltonian_smoke tests.test_task1_active_pipeline_smoke tests.test_task2_cartesian_paths_smoke tests.test_task2_active_pipeline_smoke
```

Freshly rerunnable after setup:
- compile sanity for the active Task1/Task2 modules and scripts
- Task 1 solve, plot, and report
- Task 2 report and plot
- the kept Task1/Task2 unittest subset

Artifact-backed in this branch:
- `artifacts/task1_hamiltonian_solution.json` is the frozen Task 1 result already committed in the repo
- `artifacts/task2_cartesian_paths.json` is the frozen authoritative Task 2 result and can be used directly if a fresh Task 2 rebuild is slow

If you skip the virtual environment and run against a system Python without `ortools`, `python scripts/solve_task1_hamiltonian.py` may fail with `ModuleNotFoundError`.

## Upstream interface for Task 3

Task 3 should consume `artifacts/task2_cartesian_paths.json`.

See `docs/UPSTREAM_INTERFACE_FOR_TASK3.md` for the exact fields and conventions that Task 3 must preserve.

## Out of scope in this branch

- Task 3 implementation
- Task 4 implementation
