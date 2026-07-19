#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from slam_practice.trajectory import convert_euroc_groundtruth, format_tum_pose


class TrajectoryTest(unittest.TestCase):
    def test_formats_tum_pose_with_xyzw_quaternion(self):
        line = format_tum_pose(1.25, (1, 2, 3), (0.1, 0.2, 0.3, 0.9))

        fields = line.split()
        self.assertEqual(len(fields), 8)
        self.assertEqual(fields[0], "1.250000000")
        self.assertEqual(fields[-4:], ["0.100000000", "0.200000000", "0.300000000", "0.900000000"])

    def test_converts_euroc_quaternion_order_to_tum(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "data.csv"
            destination = Path(temporary) / "groundtruth.tum"
            source.write_text(
                "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,vx,vy,vz\n"
                "1000000000,1,2,3,0.9,0.1,0.2,0.3,0,0,0\n",
                encoding="utf-8",
            )

            count = convert_euroc_groundtruth(source, destination)

            self.assertEqual(count, 1)
            self.assertEqual(
                destination.read_text(encoding="utf-8").strip().split(),
                ["1.000000000", "1.000000000", "2.000000000", "3.000000000",
                 "0.100000000", "0.200000000", "0.300000000", "0.900000000"],
            )

    def test_converts_body_groundtruth_to_camera_frame_with_extrinsic(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "data.csv"
            destination = Path(temporary) / "camera_groundtruth.tum"
            source.write_text(
                "1000000000,1,2,3,1,0,0,0\n",
                encoding="utf-8",
            )
            body_t_camera = [
                [1, 0, 0, 0.1],
                [0, 1, 0, -0.2],
                [0, 0, 1, 0.3],
                [0, 0, 0, 1],
            ]

            convert_euroc_groundtruth(source, destination, body_T_sensor=body_t_camera)

            fields = [float(value) for value in destination.read_text(encoding="utf-8").split()]
            self.assertEqual(fields[0], 1.0)
            self.assertEqual(fields[1:4], [1.1, 1.8, 3.3])
            self.assertEqual(fields[4:], [0.0, 0.0, 0.0, 1.0])


if __name__ == "__main__":
    unittest.main()
