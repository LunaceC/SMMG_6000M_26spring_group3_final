"""Report summary statistics for the Task 3 trajectory artifact."""

from __future__ import annotations

import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FINAL_PATH = REPO_ROOT / "artifacts" / "task3_trajectories.json"
FIGURE_XY_PATH = REPO_ROOT / "artifacts" / "figures" / "task3_cartesian_xy_trajectory.png"
FIGURE_JOINT_PATH = REPO_ROOT / "artifacts" / "figures" / "task3_joint_trajectory_segment0.png"


def main() -> None:
    if not FINAL_PATH.exists():
        print("Task 3 trajectory artifact: MISSING")
        print("Run: python scripts/build_task3_trajectories.py")
        return

    payload = json.loads(FINAL_PATH.read_text(encoding="utf-8"))
    trajectories = list(payload["trajectories"])
    durations = [float(item["duration"]) for item in trajectories]
    sample_counts = [int(item["sample_count"]) for item in trajectories]

    print("Task 3 active formulation: piecewise cubic trajectory planning")
    print(f"trajectory segment count: {payload['trajectory_segment_count']}")
    print(f"total duration: {payload['total_duration']:.6f} s")
    print(f"minimum segment duration: {min(durations):.6f} s")
    print(f"maximum segment duration: {max(durations):.6f} s")
    print(f"total trajectory samples: {sum(sample_counts)}")
    print(f"time law: {payload['time_law']['method']}")
    print(f"boundary conditions: {payload['time_law']['boundary_conditions']}")
    print(f"Cartesian XY figure exists: {FIGURE_XY_PATH.exists()}")
    print(f"joint segment-0 figure exists: {FIGURE_JOINT_PATH.exists()}")


if __name__ == "__main__":
    main()
