#!/usr/bin/env python3
import unittest

import numpy as np

from slam_practice.control import compute_velocity_command


class ComputeVelocityCommandTest(unittest.TestCase):
    def test_zero_error_and_feedforward_produces_zero(self):
        result = compute_velocity_command([1, 2, 3], [1, 2, 3], [0, 0, 0], 1.0, 1.5, 0.8)
        np.testing.assert_allclose(result, [0, 0, 0])

    def test_combines_proportional_feedback_and_feedforward(self):
        result = compute_velocity_command([0, 0, 0], [1, -2, 0.5], [0.2, 0.1, -0.1], 0.5, 5.0, 2.0)
        np.testing.assert_allclose(result, [0.7, -0.9, 0.15])

    def test_saturates_horizontal_norm_without_changing_direction(self):
        result = compute_velocity_command([0, 0, 0], [3, 4, 0], [0, 0, 0], 1.0, 1.5, 0.8)
        np.testing.assert_allclose(result, [0.9, 1.2, 0.0])

    def test_saturates_vertical_speed_independently(self):
        positive = compute_velocity_command([0, 0, 0], [0, 0, 3], [0, 0, 0], 1.0, 1.5, 0.8)
        negative = compute_velocity_command([0, 0, 0], [0, 0, -3], [0, 0, 0], 1.0, 1.5, 0.8)
        self.assertAlmostEqual(positive[2], 0.8)
        self.assertAlmostEqual(negative[2], -0.8)

    def test_rejects_nonfinite_values_and_invalid_limits(self):
        bad_cases = (
            ([np.nan, 0, 0], [0, 0, 0], [0, 0, 0], 1.0, 1.5, 0.8),
            ([0, 0, 0], [0, 0, 0], [np.inf, 0, 0], 1.0, 1.5, 0.8),
            ([0, 0, 0], [0, 0, 0], [0, 0, 0], -1.0, 1.5, 0.8),
            ([0, 0, 0], [0, 0, 0], [0, 0, 0], 1.0, 0.0, 0.8),
        )
        for arguments in bad_cases:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                compute_velocity_command(*arguments)


if __name__ == "__main__":
    unittest.main()
