"""Unit tests for planar RRR forward and inverse kinematics."""

from __future__ import annotations

import math
import unittest

import numpy as np

from kinematics import fk, ik_pose, normalize_angle, pose_error
from scene import build_default_scene
from welds import generate_weld_points


class KinematicsTest(unittest.TestCase):
    """Validate FK and IK behavior against fixed geometry and weld targets."""

    def test_fk_zero_pose(self) -> None:
        pose = fk((0.0, 0.0, 0.0))
        self.assertTrue(np.allclose(pose, (340.0, 0.0, 0.0), atol=1e-12))

    def test_ik_fk_roundtrip_on_handpicked_pose(self) -> None:
        target_q = (0.4, 0.6, -0.3)
        target_pose = fk(target_q)
        solutions = ik_pose(*target_pose)

        self.assertGreaterEqual(len(solutions), 1)
        self.assertTrue(
            any(np.allclose(fk(solution), target_pose, atol=1e-9) for solution in solutions)
        )

    def test_ik_returns_two_branches_when_expected(self) -> None:
        target_pose = fk((0.35, 0.8, -0.45))
        solutions = ik_pose(*target_pose)

        self.assertEqual(len(solutions), 2)
        self.assertFalse(np.allclose(solutions[0], solutions[1], atol=1e-9))
        for solution in solutions:
            self.assertTrue(np.allclose(fk(solution), target_pose, atol=1e-9))

    def test_all_generated_weld_targets_are_checked(self) -> None:
        scene = build_default_scene()
        welds = generate_weld_points(scene)
        checked_ids: list[str] = []
        feasible_ids: list[str] = []

        for weld in welds:
            checked_ids.append(weld.id)
            target_pose = (
                weld.position[0],
                weld.position[1],
                math.radians(weld.tool_heading_deg),
            )
            solutions = ik_pose(*target_pose)
            if solutions:
                feasible_ids.append(weld.id)
            for solution in solutions:
                actual_pose = fk(solution)
                error = pose_error(actual_pose, target_pose)
                with self.subTest(weld_id=weld.id, solution=solution):
                    self.assertLessEqual(abs(error["dx"]), 1e-9)
                    self.assertLessEqual(abs(error["dy"]), 1e-9)
                    self.assertLessEqual(abs(normalize_angle(error["dphi"])), 1e-12)

        self.assertEqual(checked_ids, [weld.id for weld in welds])
        self.assertEqual(len(checked_ids), 24)
        self.assertEqual(len(feasible_ids), 24)


if __name__ == "__main__":
    unittest.main()
