# Task 3

## Role

Task 3 consumes the authoritative Task 2 path artifact, `artifacts/task2_cartesian_paths.json`, and generates a time-parameterized trajectory in both joint space and Cartesian space.

The implementation preserves the Task 1 weld order and the Task 2 collision-free path. It does not change the path. It only adds timing and smooth interpolation between the existing waypoints.

## Active Task 3 method

The active method is piecewise cubic polynomial interpolation.

For each consecutive pair of Task 2 waypoints, the planner builds a cubic polynomial with zero endpoint velocities:

```text
q(t) = q0 + 3(q1 - q0)(t/T)^2 - 2(q1 - q0)(t/T)^3
```

The same formulation is used for Cartesian coordinates. For angular coordinates, the shortest wrapped angle difference is used.

Each subsegment duration is selected as the maximum of:

- joint-space timing estimate,
- Cartesian translation timing estimate,
- end-effector orientation timing estimate,
- a minimum duration guard.

This gives one shared time grid for joint-space and Cartesian-space trajectories.

## Active modules and scripts

Code:

- `task3_trajectory_planning.py`
- `kinematics.py`

Scripts:

- `scripts/build_task3_trajectories.py`
- `scripts/plot_task3_trajectories.py`
- `scripts/report_task3_trajectories.py`

Tests:

- `tests/test_task3_trajectory_planning_smoke.py`

## Artifacts and figures

Authoritative Task 3 artifact:

- `artifacts/task3_trajectories.json`

Task 3 figures:

- `artifacts/figures/task3_cartesian_xy_trajectory.png`
- `artifacts/figures/task3_joint_trajectory_segment0.png`
- `artifacts/figures/task3_cartesian_trajectory_segment0.png`

## How to run Task 3

Task 3 expects `artifacts/task2_cartesian_paths.json` to already exist.

```bash
python scripts/build_task3_trajectories.py
python scripts/plot_task3_trajectories.py
python scripts/report_task3_trajectories.py
```

## Output format

Each trajectory segment stores:

- `segment_index`
- `source_label`
- `target_label`
- `waypoint_times`
- `duration`
- `sample_count`
- `time`
- `joint_space.positions`
- `joint_space.velocities`
- `joint_space.accelerations`
- `cartesian_space.positions`
- `cartesian_space.velocities`
- `cartesian_space.accelerations`
- `cartesian_space.fk_from_joint_positions`

The `cartesian_space.positions` sequence is the cubic Cartesian trajectory generated directly from Task 2 Cartesian waypoints. The `cartesian_space.fk_from_joint_positions` sequence is the end-effector pose obtained by applying forward kinematics to the joint-space trajectory samples.
