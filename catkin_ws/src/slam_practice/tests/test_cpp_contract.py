#!/usr/bin/env python3
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[4]
SOURCE = REPOSITORY / "catkin_ws" / "src" / "orbslam3_ros" / "src" / "mono_node.cpp"


class OrbSlamWrapperContractTest(unittest.TestCase):
    def test_mono_wrapper_publishes_required_outputs(self):
        self.assertTrue(SOURCE.is_file(), f"missing ORB-SLAM3 wrapper: {SOURCE}")
        text = SOURCE.read_text(encoding="utf-8")
        for token in (
            "TrackMonocular",
            "Tcw.inverse()",
            "Tracking::OK",
            '"/orbslam3/pose"',
            '"/orbslam3/odometry"',
            '"/orbslam3/path"',
            '"/orbslam3/tracked_points"',
            "SaveKeyFrameTrajectoryTUM",
        ):
            self.assertIn(token, text)

    def test_cmake_links_orbslam3_bundled_dependencies(self):
        path = Path(__file__).resolve().parents[2] / "orbslam3_ros" / "CMakeLists.txt"
        text = path.read_text(encoding="utf-8")
        for token in ("libDBoW2.so", "libg2o.so", "Boost", "OpenSSL::Crypto"):
            self.assertIn(token, text)


if __name__ == "__main__":
    unittest.main()
