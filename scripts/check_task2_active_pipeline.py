"""Print the active Task 2 pipeline paths."""

from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TASK2_DOC = REPO_ROOT / "docs" / "TASK2.md"


def main() -> None:
    print(f"active Task 2 builder: {REPO_ROOT / 'scripts' / 'build_task2_cartesian_paths.py'}")
    print(f"active Task 2 report: {REPO_ROOT / 'scripts' / 'report_task2_cartesian_paths.py'}")
    print(f"active Task 2 plot: {REPO_ROOT / 'scripts' / 'plot_task2_cartesian_paths.py'}")
    print(f"active Task 2 artifact: {REPO_ROOT / 'artifacts' / 'task2_cartesian_paths.json'}")
    print(f"active Task 2 figure: {REPO_ROOT / 'artifacts' / 'figures' / 'task2_cartesian_paths.png'}")
    print(f"Task 2 package doc path: {TASK2_DOC}")


if __name__ == "__main__":
    main()
