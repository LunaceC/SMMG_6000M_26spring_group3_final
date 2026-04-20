"""Build the active Cartesian Task 2 package from the frozen Task 1 output."""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from task2_cartesian_paths import (
    TASK2_CARTESIAN_ARTIFACT_PATH,
    build_task2_cartesian_paths,
    save_task2_cartesian_paths,
)


def main() -> None:
    payload = build_task2_cartesian_paths()
    artifact_path = save_task2_cartesian_paths(payload, REPO_ROOT / TASK2_CARTESIAN_ARTIFACT_PATH)
    print(f"task2 cartesian paths saved: {artifact_path}", flush=True)
    print(f"segment count: {payload['segment_count']}", flush=True)
    print(f"collision-free segment count: {payload['collision_free_segment_count']}", flush=True)
    print(f"all segments valid: {payload['all_segments_valid']}", flush=True)
    print(f"locally repaired segment indices: {payload['locally_repaired_segment_indices']}", flush=True)
    print(f"total workspace path length: {payload['total_workspace_path_length']}", flush=True)
    print(f"total joint path length: {payload['total_joint_path_length']}", flush=True)


if __name__ == "__main__":
    main()
