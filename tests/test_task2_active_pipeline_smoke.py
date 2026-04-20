from __future__ import annotations

from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]


class Task2ActivePipelineSmokeTest(unittest.TestCase):
    def test_task2_final_artifact_exists(self) -> None:
        self.assertTrue((REPO_ROOT / "artifacts" / "task2_cartesian_paths.json").exists())

    def test_task2_final_figure_exists(self) -> None:
        self.assertTrue((REPO_ROOT / "artifacts" / "figures" / "task2_cartesian_paths.png").exists())

    def test_task2_doc_exists(self) -> None:
        self.assertTrue((REPO_ROOT / "docs" / "TASK2.md").exists())


if __name__ == "__main__":
    unittest.main()
