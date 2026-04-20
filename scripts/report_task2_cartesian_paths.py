"""Report the active frozen Cartesian Task 2 package."""

from __future__ import annotations

import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


FINAL_PATH = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"
FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task2_cartesian_paths.png"


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> None:
    if not FINAL_PATH.exists():
        print("Task 2 active artifact: MISSING")
        return

    payload = _read_json(FINAL_PATH)
    ordered_segment_records = list(payload["ordered_segment_records"])
    same_object_count = sum(1 for record in ordered_segment_records if record.get("segment_type") == "same_object_neighbor")
    inter_object_count = sum(1 for record in ordered_segment_records if record.get("segment_type") == "inter_object")
    repaired_indices = [record["segment_index"] for record in ordered_segment_records if record.get("local_repair_used")]

    print("Task 2 active formulation: Cartesian workspace-first planning")
    print(f"segment count: {payload['segment_count']}")
    print(f"collision-free segment count: {payload['collision_free_segment_count']}")
    print(f"same-object segment count: {same_object_count}")
    print(f"inter-object segment count: {inter_object_count}")
    print(f"total workspace path length: {payload['total_workspace_path_length']}")
    print(f"total joint path length: {payload['total_joint_path_length']}")
    print(f"locally repaired segment indices: {repaired_indices}")
    print(f"final figure exists: {FIGURE_PATH.exists()}")


if __name__ == "__main__":
    main()
