"""Smoke tests for the Task 3 trajectory artifact and planner."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from kinematics import normalize_angle
from task3_trajectory_planning import build_task3_trajectories


FINAL_PATH = REPO_ROOT / "artifacts" / "task3_trajectories.json"
TASK2_PATH = REPO_ROOT / "artifacts" / "task2_cartesian_paths.json"


class Task3TrajectoryPlanningSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        build_task3_trajectories(task2_path=TASK2_PATH, output_path=FINAL_PATH)

    def test_final_artifact_exists(self) -> None:
        self.assertTrue(FINAL_PATH.exists())

    def test_artifact_shape(self) -> None:
        payload = json.loads(FINAL_PATH.read_text(encoding="utf-8"))
        self.assertEqual(payload["trajectory_segment_count"], 25)
        self.assertEqual(payload["segment_count"], 25)
        self.assertGreater(payload["total_duration"], 0.0)
        self.assertEqual(payload["trajectories"][0]["source_label"], "DEPOT")
        self.assertEqual(payload["trajectories"][-1]["target_label"], "DEPOT")

    def test_sample_lists_have_matching_lengths(self) -> None:
        payload = json.loads(FINAL_PATH.read_text(encoding="utf-8"))
        for trajectory in payload["trajectories"]:
            sample_count = trajectory["sample_count"]
            self.assertEqual(len(trajectory["time"]), sample_count)
            self.assertEqual(len(trajectory["joint_space"]["positions"]), sample_count)
            self.assertEqual(len(trajectory["joint_space"]["velocities"]), sample_count)
            self.assertEqual(len(trajectory["joint_space"]["accelerations"]), sample_count)
            self.assertEqual(len(trajectory["cartesian_space"]["positions"]), sample_count)
            self.assertEqual(len(trajectory["cartesian_space"]["velocities"]), sample_count)
            self.assertEqual(len(trajectory["cartesian_space"]["accelerations"]), sample_count)

    def test_endpoint_waypoints_are_preserved(self) -> None:
        payload = json.loads(FINAL_PATH.read_text(encoding="utf-8"))
        for trajectory in payload["trajectories"]:
            joint_waypoints = trajectory["joint_space"]["waypoints"]
            joint_positions = trajectory["joint_space"]["positions"]
            cart_waypoints = trajectory["cartesian_space"]["waypoints"]
            cart_positions = trajectory["cartesian_space"]["positions"]
            for a, b in zip(joint_positions[0], joint_waypoints[0]):
                self.assertAlmostEqual(normalize_angle(a - b), 0.0, places=9)
            for a, b in zip(joint_positions[-1], joint_waypoints[-1]):
                self.assertAlmostEqual(normalize_angle(a - b), 0.0, places=9)
            self.assertAlmostEqual(cart_positions[0][0], cart_waypoints[0][0], places=9)
            self.assertAlmostEqual(cart_positions[0][1], cart_waypoints[0][1], places=9)
            self.assertAlmostEqual(normalize_angle(cart_positions[0][2] - cart_waypoints[0][2]), 0.0, places=9)
            self.assertAlmostEqual(cart_positions[-1][0], cart_waypoints[-1][0], places=9)
            self.assertAlmostEqual(cart_positions[-1][1], cart_waypoints[-1][1], places=9)
            self.assertAlmostEqual(normalize_angle(cart_positions[-1][2] - cart_waypoints[-1][2]), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
