"""Unit tests for scene construction and weld-point generation."""

from __future__ import annotations

import math
import unittest
from collections import Counter

import numpy as np

from scene import CircleObject, HexagonObject, build_default_scene
from welds import generate_weld_points


class SceneWeldsTest(unittest.TestCase):
    """Validate the locked scene geometry and weld registry behavior."""

    def setUp(self) -> None:
        self.scene = build_default_scene()
        self.welds = generate_weld_points(self.scene)
        self.object_lookup = {scene_object.id: scene_object for scene_object in self.scene.objects}

    def test_total_weld_count_is_24(self) -> None:
        self.assertEqual(len(self.welds), 24)

    def test_per_object_counts_match_shape_conventions(self) -> None:
        counts = Counter(weld.object_id for weld in self.welds)
        self.assertEqual(
            counts,
            Counter(
                {
                    "C1": 4,
                    "P1": 6,
                    "C2": 4,
                    "C3": 4,
                    "P2": 6,
                }
            ),
        )

    def test_normals_have_unit_norm(self) -> None:
        for weld in self.welds:
            with self.subTest(weld_id=weld.id):
                normal_norm = np.linalg.norm(np.array(weld.normal, dtype=float))
                self.assertTrue(math.isclose(normal_norm, 1.0, rel_tol=0.0, abs_tol=1e-12))

    def test_welds_lie_at_expected_radial_offset(self) -> None:
        for weld in self.welds:
            scene_object = self.object_lookup[weld.object_id]
            center = np.array(scene_object.center, dtype=float)
            position = np.array(weld.position, dtype=float)
            expected_offset = self.scene.inflation_radius + (
                scene_object.radius if isinstance(scene_object, CircleObject) else scene_object.inradius
            )
            actual_offset = np.linalg.norm(position - center)
            with self.subTest(weld_id=weld.id):
                self.assertTrue(math.isclose(actual_offset, expected_offset, rel_tol=0.0, abs_tol=1e-9))

    def test_ordering_is_stable_and_ids_are_unique(self) -> None:
        expected_ids = [
            "C1_W0",
            "C1_W1",
            "C1_W2",
            "C1_W3",
            "P1_W0",
            "P1_W1",
            "P1_W2",
            "P1_W3",
            "P1_W4",
            "P1_W5",
            "C2_W0",
            "C2_W1",
            "C2_W2",
            "C2_W3",
            "C3_W0",
            "C3_W1",
            "C3_W2",
            "C3_W3",
            "P2_W0",
            "P2_W1",
            "P2_W2",
            "P2_W3",
            "P2_W4",
            "P2_W5",
        ]
        actual_ids = [weld.id for weld in self.welds]
        self.assertEqual(actual_ids, expected_ids)
        self.assertEqual(len(actual_ids), len(set(actual_ids)))

    def test_heading_matches_inward_link_direction(self) -> None:
        for weld in self.welds:
            expected_heading = (weld.alpha_deg + 180.0) % 360.0
            with self.subTest(weld_id=weld.id):
                self.assertTrue(
                    math.isclose(weld.tool_heading_deg, expected_heading, rel_tol=0.0, abs_tol=1e-12)
                )

    def test_wrist_center_lies_outside_object_along_outward_normal(self) -> None:
        link3_length = self.scene.link_lengths[2]
        for weld in self.welds:
            tip = np.array(weld.position, dtype=float)
            normal = np.array(weld.normal, dtype=float)
            phi = math.radians(weld.tool_heading_deg)
            wrist = tip - link3_length * np.array([math.cos(phi), math.sin(phi)], dtype=float)
            with self.subTest(weld_id=weld.id):
                self.assertGreater(float(np.dot(wrist - tip, normal)), 0.0)

    def test_hexagon_vertices_are_deterministic(self) -> None:
        hexagon = next(scene_object for scene_object in self.scene.objects if isinstance(scene_object, HexagonObject))
        expected_first_vertex = np.array([173.0940107676, 140.0])
        actual_first_vertex = np.array(hexagon.vertices()[0], dtype=float)
        self.assertTrue(np.allclose(actual_first_vertex, expected_first_vertex, atol=1e-9))


if __name__ == "__main__":
    unittest.main()
