#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path

import numpy as np


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from slam_practice.geometry import (
    AIRSIM_OPTICAL_TO_BODY,
    ROS_OPTICAL_TO_BODY,
    compose_pose,
    ned_to_enu_vector,
    normalize_quaternion,
)


class GeometryTest(unittest.TestCase):
    def test_converts_ned_vector_to_enu(self):
        np.testing.assert_allclose(ned_to_enu_vector((1, 2, 3)), (2, 1, -3))

    def test_normalizes_xyzw_quaternion(self):
        np.testing.assert_allclose(normalize_quaternion((0, 0, 0, 2)), (0, 0, 0, 1))

    def test_composes_identity_parent_with_camera_translation(self):
        position, quaternion = compose_pose(
            (1, 2, 3), (0, 0, 0, 1), (0, -0.1, 0), (0, 0, 0, 1)
        )
        np.testing.assert_allclose(position, (1, 1.9, 3))
        np.testing.assert_allclose(quaternion, (0, 0, 0, 1))

    def test_optical_axes_are_right_down_forward_in_body(self):
        expected = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
        np.testing.assert_allclose(AIRSIM_OPTICAL_TO_BODY, expected)

    def test_ros_flu_optical_axes_are_right_down_forward(self):
        expected = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        np.testing.assert_allclose(ROS_OPTICAL_TO_BODY, expected)


if __name__ == "__main__":
    unittest.main()
