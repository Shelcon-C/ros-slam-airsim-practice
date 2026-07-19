#!/usr/bin/env python3
import sys
import unittest
from pathlib import Path


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from slam_practice.calibration import CameraCalibration, orbslam_yaml_text


class CalibrationTest(unittest.TestCase):
    def test_serializes_orbslam3_camera_fields(self):
        calibration = CameraCalibration(
            width=752,
            height=480,
            fx=458.654,
            fy=457.296,
            cx=367.215,
            cy=248.375,
            distortion=(-0.28340811, 0.07395907, 0.00019359, 0.0000176187),
            fps=20.0,
        )

        text = orbslam_yaml_text(calibration)

        self.assertIn('Camera.type: "PinHole"', text)
        self.assertIn("Camera1.fx: 458.654000000", text)
        self.assertIn("Camera1.k1: -0.283408110", text)
        self.assertIn("Camera.width: 752", text)
        self.assertIn("Camera.fps: 20.000000000", text)
        self.assertIn("ORBextractor.nFeatures: 1000", text)
        self.assertIn("Viewer.KeyFrameSize: 0.05", text)

    def test_rejects_non_positive_focal_length(self):
        with self.assertRaisesRegex(ValueError, "focal"):
            CameraCalibration(640, 480, 0.0, 320.0, 320.0, 240.0, (0, 0, 0, 0), 20)


if __name__ == "__main__":
    unittest.main()
