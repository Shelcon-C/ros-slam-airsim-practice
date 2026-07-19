#!/usr/bin/env python3
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE_ROOT.parents[2]


class EgoConfigurationTest(unittest.TestCase):
    def test_launch_connects_vins_depth_goal_planner_and_controller(self):
        launch_path = PACKAGE_ROOT / "launch" / "task2_ego_airsim.launch"
        ET.parse(launch_path)
        text = launch_path.read_text(encoding="utf-8")
        for token in (
            "advanced_param.xml",
            "/move_base_simple/goal",
            "/slam_practice/vins/odometry",
            "/ego_bridge/depth",
            "/ego_bridge/camera_pose",
            "/planning/pos_cmd",
            "depth_pose_adapter.py",
            "ego_position_controller.py",
            "max_xy_speed",
            "watchdog_timeout",
        ):
            self.assertIn(token, text)
        self.assertNotIn("simulator.xml", text)

    def test_rviz_and_diagnostics_expose_goal_and_planning_topics(self):
        rviz_text = (PACKAGE_ROOT / "rviz" / "task2_ego.rviz").read_text(encoding="utf-8")
        diagnostic = (REPOSITORY / "scripts" / "check_ego_topics.sh").read_text(encoding="utf-8")
        for token in ("rviz/SetGoal", "/planning/pos_cmd", "/grid_map/occupancy"):
            self.assertIn(token, rviz_text)
        for token in (
            "/ego_bridge/depth",
            "/ego_bridge/camera_pose",
            "/move_base_simple/goal",
            "/planning/pos_cmd",
            "set_enabled",
        ):
            self.assertIn(token, diagnostic)

    def test_optional_build_script_links_upstream_planner(self):
        text = (REPOSITORY / "scripts" / "build_ego_workspace.sh").read_text(encoding="utf-8")
        for token in ("ego-planner", "catkin_make", "ORB_SLAM3_ROOT", "AirSim/ros/devel/setup.bash"):
            self.assertIn(token, text)


if __name__ == "__main__":
    unittest.main()
