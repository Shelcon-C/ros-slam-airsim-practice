#!/usr/bin/env python3
import ast
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE_ROOT / "scripts"


class ScriptContractsTest(unittest.TestCase):
    def read_script(self, name):
        path = SCRIPTS / name
        self.assertTrue(path.is_file(), f"missing script: {path}")
        text = path.read_text(encoding="utf-8")
        tree = ast.parse(text, filename=str(path))
        functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
        self.assertIn("main", functions, f"{name} must define main()")
        return text

    def test_euroc_publisher_contract(self):
        text = self.read_script("euroc_mono_publisher.py")
        for token in (
            "~dataset_root",
            "/camera/mono/image_raw",
            "/camera/mono/camera_info",
            "playback_rate",
            "wait_for_subscriber",
            "get_num_connections",
        ):
            self.assertIn(token, text)

    def test_trajectory_recorder_contract(self):
        text = self.read_script("trajectory_recorder.py")
        for token in ("~input_topic", "~message_type", "format_tum_pose", "PoseStamped", "Odometry"):
            self.assertIn(token, text)

    def test_command_line_tools_have_main(self):
        for script in ("euroc_groundtruth_to_tum.py", "calibrate_camera.py", "evaluate_trajectory.py"):
            with self.subTest(script=script):
                self.read_script(script)

    def test_groundtruth_converter_accepts_camera_extrinsic(self):
        text = self.read_script("euroc_groundtruth_to_tum.py")
        for token in ("--sensor-yaml", "T_BS", "body_T_sensor"):
            self.assertIn(token, text)

    def test_stereo_imu_relay_contract(self):
        text = self.read_script("stereo_imu_relay.py")
        for token in (
            "ApproximateTimeSynchronizer",
            "slop=0.003",
            "mono8",
            "/vins_fusion/cam0/image_raw",
            "/vins_fusion/cam1/image_raw",
            "/vins_fusion/imu",
            "synchronized_stamp",
        ):
            self.assertIn(token, text)

    def test_vins_output_and_truth_recorders_contract(self):
        adapter = self.read_script("vins_output_adapter.py")
        for token in ("/vins_estimator/odometry", "/slam_practice/vins/odometry", "/slam_practice/vins/path"):
            self.assertIn(token, adapter)
        truth = self.read_script("airsim_gt_recorder.py")
        for token in ("/airsim_node/Drone1/odom_local_enu", "format_tum_pose"):
            self.assertIn(token, truth)

    def test_ego_depth_adapter_uses_vins_pose_and_float_depth(self):
        text = self.read_script("depth_pose_adapter.py")
        for token in (
            "/airsim_node/Drone1/left/DepthPlanner",
            "/slam_practice/vins/odometry",
            "/ego_bridge/depth",
            "/ego_bridge/camera_pose",
            "32FC1",
            "ROS_OPTICAL_TO_BODY_QUATERNION",
        ):
            self.assertIn(token, text)
        self.assertNotIn("odom_local_enu", text)

    def test_ego_controller_is_disabled_by_default_and_watchdog_bounded(self):
        text = self.read_script("ego_position_controller.py")
        for token in (
            "PositionCommand",
            "VelCmd",
            "SetBool",
            "watchdog_timeout",
            "max_xy_speed",
            "max_z_speed",
            "compute_velocity_command",
            "/airsim_node/vel_cmd_world_frame",
        ):
            self.assertIn(token, text)
        self.assertIn("self.enabled = False", text)


if __name__ == "__main__":
    unittest.main()
