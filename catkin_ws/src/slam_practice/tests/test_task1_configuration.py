#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE_ROOT.parents[2]


class Task1ConfigurationTest(unittest.TestCase):
    def test_orb_config_contains_official_euroc_intrinsics(self):
        path = PACKAGE_ROOT / "config" / "orbslam3" / "euroc_mono.yaml"
        self.assertTrue(path.is_file())
        text = path.read_text(encoding="utf-8")
        for token in (
            "Camera1.fx: 458.654",
            "Camera1.fy: 457.296",
            "Camera1.cx: 367.215",
            "Camera1.cy: 248.375",
            "Camera.width: 752",
            "Camera.height: 480",
            "ORBextractor.nFeatures: 1000",
        ):
            self.assertIn(token, text)

    def test_camera_info_contains_same_intrinsics(self):
        path = PACKAGE_ROOT / "config" / "camera" / "euroc_cam0.yaml"
        self.assertTrue(path.is_file())
        text = path.read_text(encoding="utf-8")
        self.assertIn("image_width: 752", text)
        self.assertIn("image_height: 480", text)
        self.assertIn("458.654", text)
        self.assertIn("457.296", text)

    def test_launch_exposes_replaceable_inputs_and_outputs(self):
        path = PACKAGE_ROOT / "launch" / "task1_euroc_mono.launch"
        self.assertTrue(path.is_file())
        root = ET.parse(path).getroot()
        arguments = {element.attrib["name"] for element in root.findall("arg")}
        self.assertTrue(
            {"dataset_root", "camera_yaml", "orb_settings", "vocabulary_path", "use_viewer",
             "playback_rate", "trajectory_output", "start_offset", "duration"}.issubset(arguments)
        )

    def test_task1_shell_scripts_exist(self):
        for name in (
            "install_noetic_dependencies.sh",
            "fetch_third_party.sh",
            "download_euroc.sh",
            "build_workspace.sh",
        ):
            with self.subTest(name=name):
                path = REPOSITORY / "scripts" / name
                self.assertTrue(path.is_file(), str(path))
                self.assertTrue(path.read_text(encoding="utf-8").startswith("#!/usr/bin/env bash"))

    def test_legacy_noetic_python_uses_compatible_evo_environment(self):
        text = (REPOSITORY / "scripts" / "install_noetic_dependencies.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--system-site-packages", text)
        self.assertIn("evo==1.30.6", text)
        self.assertNotIn("pip install --user", text)


if __name__ == "__main__":
    unittest.main()
