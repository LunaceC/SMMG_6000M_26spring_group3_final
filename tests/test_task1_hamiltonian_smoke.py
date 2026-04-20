"""Smoke tests for the clean Euclidean Hamiltonian-cycle Task 1 artifacts."""

from __future__ import annotations

from pathlib import Path
import json
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SOLUTION_PATH = REPO_ROOT / "artifacts" / "task1_hamiltonian_solution.json"
NODES_FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_nodes.png"
CYCLE_FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task1_hamiltonian_cycle.png"
MATRIX_FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task1_distance_matrix.png"


def _read_json(path: Path) -> dict[str, object]:
    """Read one JSON artifact."""
    return json.loads(path.read_text(encoding="utf-8"))


class Task1HamiltonianSmokeTest(unittest.TestCase):
    """Verify the new clean Task 1 Hamiltonian artifacts are created and consistent."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.solution = _read_json(SOLUTION_PATH)

    def test_solution_artifact_exists(self) -> None:
        self.assertTrue(SOLUTION_PATH.exists())

    def test_figure_artifacts_exist(self) -> None:
        self.assertTrue(NODES_FIGURE_PATH.exists())
        self.assertTrue(CYCLE_FIGURE_PATH.exists())
        self.assertTrue(MATRIX_FIGURE_PATH.exists())

    def test_non_optimal_fields_if_needed(self) -> None:
        if self.solution.get("proof_optimal"):
            self.skipTest("Exact Hamiltonian solution was proved optimal.")
        self.assertIn("solver_backend", self.solution)
        self.assertIn("solver_status", self.solution)

    def test_optimal_cycle_structure_if_proved(self) -> None:
        if not self.solution.get("proof_optimal"):
            self.skipTest("Exact Hamiltonian solution was not proved optimal.")
        weld_order = self.solution["weld_order"]
        self.assertEqual(len(weld_order), 24)
        self.assertEqual(len(set(weld_order)), 24)
        cycle = self.solution["cycle_node_order"]
        self.assertEqual(cycle[0], "DEPOT")
        self.assertEqual(cycle[-1], "DEPOT")

    def test_distance_matrix_shape(self) -> None:
        matrix = self.solution["distance_matrix"]
        self.assertEqual(len(matrix), 25)
        self.assertTrue(all(len(row) == 25 for row in matrix))


if __name__ == "__main__":
    unittest.main()
