"""Plot Task 3 trajectory figures."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[1]
FINAL_PATH = REPO_ROOT / "artifacts" / "task3_trajectories.json"
FIGURE_DIR = REPO_ROOT / "artifacts" / "figures"


def _load_payload() -> dict[str, object]:
    if not FINAL_PATH.exists():
        raise FileNotFoundError(
            f"{FINAL_PATH} does not exist. Run scripts/build_task3_trajectories.py first."
        )
    return json.loads(FINAL_PATH.read_text(encoding="utf-8"))


def _plot_full_cartesian_xy(payload: dict[str, object]) -> None:
    trajectories = payload["trajectories"]
    plt.figure(figsize=(7, 5))
    for segment in trajectories:
        cartesian_positions = segment["cartesian_space"]["positions"]
        x_values = [point[0] for point in cartesian_positions]
        y_values = [point[1] for point in cartesian_positions]
        plt.plot(x_values, y_values, linewidth=1.0)
    plt.xlabel("x (mm)")
    plt.ylabel("y (mm)")
    plt.title("Task 3 Cartesian-space trajectory following Task 2 path")
    plt.axis("equal")
    plt.grid(True, linewidth=0.3)
    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "task3_cartesian_xy_trajectory.png", dpi=200)
    plt.close()


def _plot_segment0_joint_positions(payload: dict[str, object]) -> None:
    segment = payload["trajectories"][0]
    time_values = segment["time"]
    joint_positions = segment["joint_space"]["positions"]
    plt.figure(figsize=(8, 4.5))
    for joint_index, label in enumerate(("q1", "q2", "q3")):
        plt.plot(time_values, [q[joint_index] for q in joint_positions], label=label)
    plt.xlabel("time (s)")
    plt.ylabel("joint angle (rad)")
    plt.title("Task 3 joint-space cubic trajectory, segment 0")
    plt.legend()
    plt.grid(True, linewidth=0.3)
    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "task3_joint_trajectory_segment0.png", dpi=200)
    plt.close()


def _plot_segment0_cartesian_positions(payload: dict[str, object]) -> None:
    segment = payload["trajectories"][0]
    time_values = segment["time"]
    cartesian_positions = segment["cartesian_space"]["positions"]
    plt.figure(figsize=(8, 4.5))
    for coordinate_index, label in enumerate(("x", "y", "phi")):
        plt.plot(time_values, [pose[coordinate_index] for pose in cartesian_positions], label=label)
    plt.xlabel("time (s)")
    plt.ylabel("position / orientation")
    plt.title("Task 3 Cartesian-space cubic trajectory, segment 0")
    plt.legend()
    plt.grid(True, linewidth=0.3)
    plt.tight_layout()
    plt.savefig(FIGURE_DIR / "task3_cartesian_trajectory_segment0.png", dpi=200)
    plt.close()


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    payload = _load_payload()
    _plot_full_cartesian_xy(payload)
    _plot_segment0_joint_positions(payload)
    _plot_segment0_cartesian_positions(payload)
    print(f"Generated Task 3 figures in {FIGURE_DIR}")


if __name__ == "__main__":
    main()
