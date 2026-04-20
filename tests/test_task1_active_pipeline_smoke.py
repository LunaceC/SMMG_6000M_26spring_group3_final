"""Smoke tests for the active Hamilton Task 1 pipeline manifest and artifacts."""

from __future__ import annotations

from pathlib import Path
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]

SOLUTION_PATH = REPO_ROOT / "artifacts" / "task1_hamiltonian_solution.json"
FIGURE_PATHS = [
    REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_nodes.png",
    REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_cycle.png",
    REPO_ROOT / "artifacts" / "figures" / "task1_distance_matrix.png",
]
DOC_PATH = REPO_ROOT / "docs" / "TASK1.md"


class Task1ActivePipelineSmokeTest(unittest.TestCase):
    """Verify the active Hamilton Task 1 pipeline files exist."""

    def test_hamilton_solution_exists(self) -> None:
        self.assertTrue(SOLUTION_PATH.exists())

    def test_hamilton_figures_exist(self) -> None:
        for figure_path in FIGURE_PATHS:
            self.assertTrue(figure_path.exists())

    def test_task1_doc_exists(self) -> None:
        self.assertTrue(DOC_PATH.exists())


if __name__ == "__main__":
    unittest.main()
