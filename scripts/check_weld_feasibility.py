"""Check planar RRR pose IK feasibility for every generated weld target."""

from __future__ import annotations

from math import radians
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kinematics import ik_pose
from scene import build_default_scene
from welds import generate_weld_points


def main() -> None:
    """Print a concise weld-by-weld IK feasibility table."""
    scene = build_default_scene()
    welds = generate_weld_points(scene)

    print(f"{'weld_id':<8} {'x':>9} {'y':>9} {'phi_deg':>9} {'num_solutions':>14}")

    feasible_count = 0
    for weld in welds:
        x, y = weld.position
        phi_deg = weld.tool_heading_deg
        solutions = ik_pose(x, y, radians(phi_deg))
        if solutions:
            feasible_count += 1
        print(f"{weld.id:<8} {x:9.3f} {y:9.3f} {phi_deg:9.3f} {len(solutions):14d}")

    total_welds = len(welds)
    infeasible_count = total_welds - feasible_count
    print(f"total welds: {total_welds}")
    print(f"feasible welds: {feasible_count}")
    print(f"infeasible welds: {infeasible_count}")


if __name__ == "__main__":
    main()
