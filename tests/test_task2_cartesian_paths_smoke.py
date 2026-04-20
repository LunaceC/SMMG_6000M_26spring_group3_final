"""Smoke tests for the active frozen Cartesian Task 2 artifact."""

from __future__ import annotations

import json
from pathlib import Path
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]

FINAL_PATH = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"
FIGURE_PATH = REPO_ROOT / "artifacts" / "figures" / "task2_cartesian_paths.png"


def _read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


class Task2CartesianPathsSmokeTest(unittest.TestCase):
    def test_final_artifact_exists(self) -> None:
        self.assertTrue(FINAL_PATH.exists())

    def test_final_figure_exists(self) -> None:
        self.assertTrue(FIGURE_PATH.exists())

    def test_final_artifact_shape(self) -> None:
        payload = _read_json(FINAL_PATH)
        self.assertEqual(payload["segment_count"], 25)
        self.assertTrue(all(segment["validation_result"] for segment in payload["ordered_segment_records"]))
        self.assertEqual(payload["ordered_segment_records"][0]["source_label"], "DEPOT")
        self.assertEqual(payload["ordered_segment_records"][-1]["target_label"], "DEPOT")
        segment = payload["ordered_segment_records"][0]
        for key in (
            "workspace_polyline",
            "workspace_path_length",
            "cartesian_waypoints",
            "chosen_joint_waypoints",
            "validation_result",
        ):
            self.assertIn(key, segment)


if __name__ == "__main__":
    unittest.main()
