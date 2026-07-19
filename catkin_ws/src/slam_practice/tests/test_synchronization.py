#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from slam_practice.synchronization import synchronized_stamp, validate_stereo_delta


class SynchronizationTest(unittest.TestCase):
    def test_accepts_pair_within_three_milliseconds(self):
        self.assertAlmostEqual(validate_stereo_delta(10.0, 10.0029), 0.0029, places=7)
        self.assertAlmostEqual(synchronized_stamp(10.0, 10.002), 10.001)

    def test_rejects_pair_outside_three_milliseconds(self):
        with self.assertRaisesRegex(ValueError, "3.000 ms"):
            validate_stereo_delta(10.0, 10.0031)

    def test_rejects_non_finite_stamp(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            validate_stereo_delta(float("nan"), 1.0)


if __name__ == "__main__":
    unittest.main()
