"""Build the Task 3 trajectory artifact from the frozen Task 2 path artifact."""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from task3_trajectory_planning import build_task3_trajectories


FINAL_PATH = REPO_ROOT / "artifacts" / "task3_trajectories.json"
TASK2_PATH = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"


def main() -> None:
    payload = build_task3_trajectories(task2_path=TASK2_PATH, output_path=FINAL_PATH)
    print("Task 3 trajectory planning completed")
    print(f"output artifact: {FINAL_PATH}")
    print(f"trajectory segment count: {payload['trajectory_segment_count']}")
    print(f"total duration: {payload['total_duration']:.6f} s")


if __name__ == "__main__":
    main()
