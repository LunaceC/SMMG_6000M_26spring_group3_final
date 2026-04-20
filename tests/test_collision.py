"""Unit tests for Link-3 collision and motion validity checks."""

from __future__ import annotations

import math
import unittest

from collision import check_motion, first_collision, state_clearance, state_valid
from kinematics import ik_pose
from scene import build_default_scene
from welds import generate_weld_points


class CollisionTest(unittest.TestCase):
    """Validate Link-3 clearance against inflated scene objects."""

    def setUp(self) -> None:
        self.scene = build_default_scene()
        self.welds = generate_weld_points(self.scene)

    def test_every_weld_solution_is_state_valid(self) -> None:
        for weld in self.welds:
            target_pose = (
                weld.position[0],
                weld.position[1],
                math.radians(weld.tool_heading_deg),
            )
            solutions = ik_pose(*target_pose)
            self.assertEqual(len(solutions), 2)
            for solution in solutions:
                with self.subTest(weld_id=weld.id, solution=solution):
                    self.assertTrue(state_valid(solution, scene=self.scene))
                    self.assertGreaterEqual(state_clearance(solution, scene=self.scene), -1e-9)

    def test_penetrating_pose_is_invalid(self) -> None:
        colliding_pose = (80.0, 140.0, 0.0)
        solution = ik_pose(*colliding_pose)[0]

        self.assertFalse(state_valid(solution, scene=self.scene))
        self.assertLess(state_clearance(solution, scene=self.scene), 0.0)

    def test_weld_pose_tangent_clearance_is_zero(self) -> None:
        weld = self.welds[0]
        q = ik_pose(weld.position[0], weld.position[1], math.radians(weld.tool_heading_deg))[0]
        clearance = state_clearance(q, scene=self.scene)

        self.assertTrue(state_valid(q, scene=self.scene))
        self.assertAlmostEqual(clearance, 0.0, places=9)

    def test_check_motion_rejects_crossing_motion(self) -> None:
        start_weld = self.welds[0]
        goal_weld = self.welds[1]
        q_start = ik_pose(
            start_weld.position[0],
            start_weld.position[1],
            math.radians(start_weld.tool_heading_deg),
        )[0]
        q_goal = ik_pose(
            goal_weld.position[0],
            goal_weld.position[1],
            math.radians(goal_weld.tool_heading_deg),
        )[0]

        self.assertTrue(state_valid(q_start, scene=self.scene))
        self.assertTrue(state_valid(q_goal, scene=self.scene))
        self.assertFalse(check_motion(q_start, q_goal, scene=self.scene))
        self.assertIsNotNone(first_collision(q_start, q_goal, scene=self.scene))

    def test_check_motion_accepts_static_valid_state(self) -> None:
        weld = self.welds[-1]
        q = ik_pose(weld.position[0], weld.position[1], math.radians(weld.tool_heading_deg))[1]

        self.assertTrue(check_motion(q, q, scene=self.scene))


if __name__ == "__main__":
    unittest.main()
