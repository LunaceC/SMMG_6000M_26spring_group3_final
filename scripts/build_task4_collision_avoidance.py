"""
Driver script for Task 4 collision checking and avoidance.

This script reads the frozen Task 2 artifact (``task2_cartesian_paths.json``)
and invokes the collision avoidance routine defined in
``task4_collision_avoidance.py`` to generate a new JSON file with
collision‑free joint waypoints at a 1 mm clearance.  The resulting
artifact is written to ``artifacts/task4_collision_avoidance.json``.

Run this script from the repository root as follows:

    PYTHONPATH=project/SMMG_6000M_26spring_group3_final-main \
        python3 project/SMMG_6000M_26spring_group3_final-main/scripts/build_task4_collision_avoidance.py

"""

from __future__ import annotations

from pathlib import Path
import sys

# Ensure that the repository modules are on the import path.  We resolve
# the location of this script relative to the repository root and
# insert the appropriate directory into sys.path.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from task4_collision_avoidance import build_task4_collision_avoidance


def main() -> None:
    """Entry point for generating the Task 4 collision‑free path artifact."""
    task2_path = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"
    output_path = REPO_ROOT / "artifacts" / "task4_collision_avoidance.json"
    build_task4_collision_avoidance(task2_path=str(task2_path), output_path=str(output_path), clearance_mm=1.0)
    print(f"Task 4 collision avoidance artifact written to {output_path}")


if __name__ == "__main__":
    main()