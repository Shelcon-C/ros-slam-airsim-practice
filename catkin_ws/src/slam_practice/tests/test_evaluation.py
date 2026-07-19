#!/usr/bin/env python3
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from slam_practice.evaluation import build_evo_commands


class EvaluationTest(unittest.TestCase):
    def test_monocular_commands_align_and_correct_scale(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            commands = build_evo_commands(Path("gt.tum"), Path("estimate.tum"), output, monocular=True)

        self.assertEqual(len(commands), 4)
        self.assertEqual(commands[0][0], "evo_ape")
        self.assertEqual(commands[1][0], "evo_rpe")
        self.assertEqual(commands[2][0], "evo_rpe")
        self.assertIn("--align", commands[0])
        self.assertIn("--correct_scale", commands[0])
        self.assertIn("--correct_scale", commands[1])
        self.assertIn("--correct_scale", commands[2])
        self.assertEqual(commands[3][0], "evo_res")
        self.assertEqual(commands[0][commands[0].index("--pose_relation") + 1], "trans_part")
        self.assertEqual(commands[1][commands[1].index("--pose_relation") + 1], "trans_part")
        self.assertEqual(commands[2][commands[2].index("--pose_relation") + 1], "angle_deg")
        self.assertIn("rpe_translation_results.zip", commands[3][2])
        self.assertIn("rpe_rotation_results.zip", commands[3][3])

    def test_stereo_inertial_commands_do_not_correct_scale(self):
        commands = build_evo_commands(Path("gt.tum"), Path("estimate.tum"), Path("out"), monocular=False)

        self.assertIn("--align", commands[0])
        self.assertNotIn("--correct_scale", commands[0])
        self.assertNotIn("--correct_scale", commands[1])
        self.assertNotIn("--correct_scale", commands[2])


if __name__ == "__main__":
    unittest.main()
