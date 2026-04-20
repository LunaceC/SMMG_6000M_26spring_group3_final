"""Plot the active Cartesian Task 2 path package."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scene import CircleObject, HexagonObject, build_default_scene
from task2_cartesian_paths import build_inflated_workspace_obstacles
from welds import generate_weld_points


ARTIFACT_PATH = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"
FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task2_cartesian_paths.png"


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_matplotlib():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _plot_scene_objects(ax, scene_objects) -> None:
    plt = _load_matplotlib()
    for scene_object in scene_objects:
        if isinstance(scene_object, CircleObject):
            ax.add_patch(plt.Circle(scene_object.center, scene_object.radius, fill=False, linewidth=1.8, color="tab:blue"))
            continue
        vertices = np.array(scene_object.vertices(), dtype=float)
        ax.add_patch(plt.Polygon(vertices, closed=True, fill=False, linewidth=1.8, color="tab:orange"))


def main() -> None:
    payload = _read_json(ARTIFACT_PATH)
    scene = build_default_scene()
    weld_points = generate_weld_points(scene)
    weld_lookup = {weld.id: weld for weld in weld_points}
    inflated_obstacles = build_inflated_workspace_obstacles(scene)
    plt = _load_matplotlib()
    figure, ax = plt.subplots(figsize=(10.0, 6.8))

    _plot_scene_objects(ax, scene.objects)
    for obstacle in inflated_obstacles:
        vertices = np.array(obstacle.vertices, dtype=float)
        ax.plot(
            np.append(vertices[:, 0], vertices[0, 0]),
            np.append(vertices[:, 1], vertices[0, 1]),
            linestyle="--",
            linewidth=1.0,
            color="0.55",
            alpha=0.85,
            zorder=1,
        )

    weld_positions = np.array([weld.position for weld in weld_points], dtype=float)
    ax.scatter(weld_positions[:, 0], weld_positions[:, 1], s=20.0, color="tab:red", zorder=4)
    ax.scatter([scene.depot[0]], [scene.depot[1]], s=60.0, marker="s", color="black", zorder=5)
    ax.text(scene.depot[0] + 4.0, scene.depot[1] + 4.0, "DEPOT", fontsize=9, fontweight="bold")

    for visit_number, weld_id in enumerate(payload["task1_weld_order"], start=1):
        x_pos, y_pos = weld_lookup[weld_id].position
        ax.text(x_pos + 2.0, y_pos + 2.0, str(visit_number), fontsize=8, color="black")

    nominal_label_used = False
    repaired_label_used = False
    for segment in payload["ordered_segment_records"]:
        polyline = np.array(segment["workspace_polyline"], dtype=float)
        if len(polyline) < 2:
            continue
        repaired = bool(segment["local_repair_used"])
        ax.plot(
            polyline[:, 0],
            polyline[:, 1],
            color="tab:orange" if repaired else "0.35",
            linewidth=2.2 if repaired else 1.8,
            alpha=0.95,
            zorder=3,
            label=(
                "locally repaired segment" if repaired and not repaired_label_used else
                "workspace shortest-path segment" if (not repaired and not nominal_label_used) else
                None
            ),
        )
        nominal_label_used = nominal_label_used or not repaired
        repaired_label_used = repaired_label_used or repaired

    ax.set_title("Task 2 Cartesian Workspace Paths")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0.0, 320.0)
    ax.set_ylim(-20.0, 200.0)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper left", fontsize=8)

    FIGURE_PATH.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(FIGURE_PATH, dpi=180)
    plt.close(figure)
    print(f"phase G: figure saved {FIGURE_PATH}", flush=True)


if __name__ == "__main__":
    main()
