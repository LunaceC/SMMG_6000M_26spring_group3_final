"""Render the default welding scene and its weld-point registry."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scene import CircleObject, HexagonObject, build_default_scene
from welds import generate_weld_points


def _load_matplotlib():
    """Import matplotlib lazily and configure a non-interactive backend."""
    try:
        import matplotlib
    except ModuleNotFoundError as exc:
        raise SystemExit("matplotlib is required to render the scene plot.") from exc

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _plot_objects(ax: plt.Axes, scene_objects: tuple[CircleObject | HexagonObject, ...]) -> None:
    """Draw all scene objects with simple shape-specific styling."""
    for scene_object in scene_objects:
        if isinstance(scene_object, CircleObject):
            patch = plt.Circle(
                scene_object.center,
                scene_object.radius,
                fill=False,
                linewidth=2.0,
                color="tab:blue",
            )
            ax.add_patch(patch)
            continue

        vertices = np.array(scene_object.vertices(), dtype=float)
        patch = plt.Polygon(vertices, closed=True, fill=False, linewidth=2.0, color="tab:orange")
        ax.add_patch(patch)


def _plot_welds(ax: plt.Axes, weld_points) -> None:
    """Draw weld points, normal arrows, and labels."""
    positions = np.array([weld.position for weld in weld_points], dtype=float)
    normals = np.array([weld.normal for weld in weld_points], dtype=float)

    ax.scatter(positions[:, 0], positions[:, 1], s=28.0, color="tab:red", zorder=3)
    ax.quiver(
        positions[:, 0],
        positions[:, 1],
        normals[:, 0],
        normals[:, 1],
        angles="xy",
        scale_units="xy",
        scale=0.08,
        width=0.004,
        color="tab:green",
        zorder=2,
    )
    for weld in weld_points:
        ax.text(
            weld.position[0] + 2.0,
            weld.position[1] + 2.0,
            weld.id,
            fontsize=8,
            ha="left",
            va="bottom",
        )


def plot_scene(output_path: Path, dpi: int) -> Path:
    """Create the default scene plot and write it to disk."""
    plt = _load_matplotlib()
    scene = build_default_scene()
    weld_points = generate_weld_points(scene)
    figure, ax = plt.subplots(figsize=(8.0, 5.2))
    _plot_objects(ax, scene.objects)
    _plot_welds(ax, weld_points)

    ax.scatter([scene.depot[0]], [scene.depot[1]], s=48.0, color="black", marker="s", zorder=4)
    ax.text(scene.depot[0] + 3.0, scene.depot[1] + 3.0, "Depot", fontsize=9, ha="left", va="bottom")

    ax.set_title("Planar RRR Welding Scene")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0.0, 320.0)
    ax.set_ylim(-10.0, 200.0)
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=dpi)
    plt.close(figure)
    return output_path


def main() -> None:
    """Parse arguments and render the scene plot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "artifacts" / "scene_welds.png",
        help="Destination image path.",
    )
    parser.add_argument("--dpi", type=int, default=160, help="Output image resolution.")
    args = parser.parse_args()

    written_path = plot_scene(args.output, args.dpi)
    print(written_path)


if __name__ == "__main__":
    main()
