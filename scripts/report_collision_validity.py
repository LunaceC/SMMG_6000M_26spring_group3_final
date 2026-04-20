"""Report Link-3 collision validity for every IK branch of every weld."""

from __future__ import annotations

from math import radians
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from collision import state_valid
from kinematics import ik_pose
from scene import build_default_scene
from welds import generate_weld_points


def main() -> None:
    """Print valid IK branch counts per weld and grouped totals."""
    scene = build_default_scene()
    welds = generate_weld_points(scene)

    print(f"{'weld_id':<8} {'ik_branches':>11} {'valid_branches':>14}")

    grouped: dict[int, list[str]] = {0: [], 1: [], 2: []}

    for weld in welds:
        solutions = ik_pose(weld.position[0], weld.position[1], radians(weld.tool_heading_deg))
        valid_count = sum(1 for solution in solutions if state_valid(solution, scene=scene))
        grouped.setdefault(valid_count, []).append(weld.id)
        print(f"{weld.id:<8} {len(solutions):11d} {valid_count:14d}")

    print(f"total welds: {len(welds)}")
    print(f"welds with 0 valid branches: {len(grouped.get(0, []))}")
    print(f"welds with 1 valid branch: {len(grouped.get(1, []))}")
    print(f"welds with 2 valid branches: {len(grouped.get(2, []))}")

    for valid_count in (0, 1, 2):
        weld_ids = grouped.get(valid_count, [])
        label = f"valid branch group {valid_count}"
        members = ", ".join(weld_ids) if weld_ids else "-"
        print(f"{label}: {members}")


if __name__ == "__main__":
    main()
