#!/usr/bin/env python3
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE_ROOT.parents[2]


class Task2ConfigurationTest(unittest.TestCase):
    def test_airsim_settings_define_stereo_and_imu(self):
        path = REPOSITORY / "airsim" / "settings_stereo_imu.json"
        settings = json.loads(path.read_text(encoding="utf-8"))
        drone = settings["Vehicles"]["Drone1"]
        left = drone["Cameras"]["left"]
        right = drone["Cameras"]["right"]
        self.assertAlmostEqual(abs(right["Y"] - left["Y"]), 0.20)
        left_scene = next(item for item in left["CaptureSettings"] if item["ImageType"] == 0)
        right_scene = next(item for item in right["CaptureSettings"] if item["ImageType"] == 0)
        self.assertEqual(left_scene, right_scene)
        self.assertEqual((left_scene["Width"], left_scene["Height"]), (640, 480))
        self.assertTrue(drone["Sensors"]["Imu"]["Enabled"])

    def test_vins_config_uses_only_normalized_sensor_topics(self):
        path = PACKAGE_ROOT / "config" / "vins" / "airsim_stereo_imu.yaml"
        text = path.read_text(encoding="utf-8")
        for token in (
            'imu_topic: "/vins_fusion/imu"',
            'image0_topic: "/vins_fusion/cam0/image_raw"',
            'image1_topic: "/vins_fusion/cam1/image_raw"',
            "imu: 1",
            "num_of_cam: 2",
            "body_T_cam0",
            "body_T_cam1",
        ):
            self.assertIn(token, text)
        self.assertNotIn("odom_local_enu", text)

    def test_camera_files_are_pinhole_640_by_480(self):
        for camera in ("airsim_cam0.yaml", "airsim_cam1.yaml"):
            with self.subTest(camera=camera):
                text = (PACKAGE_ROOT / "config" / "vins" / camera).read_text(encoding="utf-8")
                for token in ("model_type: PINHOLE", "image_width: 640", "image_height: 480", "fx: 320.0", "fy: 320.0"):
                    self.assertIn(token, text)

    def test_launch_uses_enu_wrapper_and_vins_node(self):
        wrapper = PACKAGE_ROOT / "launch" / "airsim_noetic_wsl.launch"
        task = PACKAGE_ROOT / "launch" / "task2_airsim_vins.launch"
        ET.parse(wrapper)
        ET.parse(task)
        wrapper_text = wrapper.read_text(encoding="utf-8")
        task_text = task.read_text(encoding="utf-8")
        for token in (
            "world_enu",
            "odom_local_enu",
            "coordinate_system_enu",
            "WSL_HOST_IP",
            "static_transforms.launch",
        ):
            self.assertIn(token, wrapper_text)
        self.assertIn('pkg="vins"', task_text)
        self.assertIn('type="vins_node"', task_text)
        self.assertIn('name="vins_estimator"', task_text)
        self.assertNotIn('name="output_file"', task_text)
        self.assertEqual(task_text.count('name="output_path"'), 2)

    def test_wsl_and_topic_diagnostics_cover_required_interfaces(self):
        wsl_script = (REPOSITORY / "scripts" / "detect_wsl_host.sh").read_text(encoding="utf-8")
        topic_script = (REPOSITORY / "scripts" / "check_task2_topics.sh").read_text(encoding="utf-8")
        build_script = (REPOSITORY / "scripts" / "build_task2_workspace.sh").read_text(encoding="utf-8")
        for token in ("/etc/resolv.conf", "WSL_HOST_IP", "41451"):
            self.assertIn(token, wsl_script)
        for token in (
            "/vins_fusion/cam0/image_raw",
            "/vins_fusion/cam1/image_raw",
            "/vins_fusion/imu",
            "/slam_practice/vins/odometry",
        ):
            self.assertIn(token, topic_script)
        for token in ('${AIRSIM_ROOT}/ros', "VINS-Fusion", "catkin_make", "ORB_SLAM3_ROOT"):
            self.assertIn(token, build_script)


if __name__ == "__main__":
    unittest.main()
