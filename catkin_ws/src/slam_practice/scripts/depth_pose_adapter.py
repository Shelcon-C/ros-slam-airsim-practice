#!/usr/bin/env python3
"""把 AirSim DepthPlanner 和 VINS 机体位姿转换为 EGO-Planner 深度输入。"""

import copy
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from slam_practice.geometry import ROS_OPTICAL_TO_BODY_QUATERNION, compose_pose


class DepthPoseAdapter:
    def __init__(self) -> None:
        depth_input = rospy.get_param("~depth_input", "/airsim_node/Drone1/left/DepthPlanner")
        odometry_input = rospy.get_param("~odometry_input", "/slam_practice/vins/odometry")
        depth_output = rospy.get_param("~depth_output", "/ego_bridge/depth")
        pose_output = rospy.get_param("~pose_output", "/ego_bridge/camera_pose")
        self.pose_tolerance = float(rospy.get_param("~pose_tolerance", 0.10))
        self.max_depth = float(rospy.get_param("~max_depth", 20.0))
        self.camera_translation = np.asarray(
            rospy.get_param("~body_to_camera_translation", [0.0, 0.1, 0.0]), dtype=float
        )
        if self.pose_tolerance <= 0 or self.max_depth <= 0:
            raise ValueError("pose tolerance and max depth must be positive")
        if self.camera_translation.shape != (3,) or not np.all(np.isfinite(self.camera_translation)):
            raise ValueError("body-to-camera translation must contain three finite values")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_odometry = None
        self.depth_publisher = rospy.Publisher(depth_output, Image, queue_size=5)
        self.pose_publisher = rospy.Publisher(pose_output, PoseStamped, queue_size=5)
        self.odometry_subscriber = rospy.Subscriber(
            odometry_input, Odometry, self.odometry_callback, queue_size=20
        )
        self.depth_subscriber = rospy.Subscriber(depth_input, Image, self.depth_callback, queue_size=5)

    def odometry_callback(self, message: Odometry) -> None:
        with self.lock:
            self.latest_odometry = copy.deepcopy(message)

    def depth_callback(self, message: Image) -> None:
        with self.lock:
            odometry = copy.deepcopy(self.latest_odometry)
        if odometry is None:
            rospy.logwarn_throttle(2.0, "Waiting for VINS odometry before publishing depth")
            return
        time_difference = abs((message.header.stamp - odometry.header.stamp).to_sec())
        if time_difference > self.pose_tolerance:
            rospy.logwarn_throttle(
                2.0, "Depth/VINS timestamp difference %.3f s exceeds tolerance", time_difference
            )
            return

        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(message, desired_encoding="32FC1"), dtype=np.float32
            ).copy()
        except Exception as error:
            rospy.logerr_throttle(2.0, "Depth conversion failed: %s", error)
            return
        depth[~np.isfinite(depth)] = 0.0
        depth[(depth <= 0.0) | (depth > self.max_depth)] = 0.0
        depth_message = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_message.header = copy.deepcopy(message.header)
        depth_message.header.frame_id = "cam0_optical"

        body_pose = odometry.pose.pose
        camera_position, camera_quaternion = compose_pose(
            (body_pose.position.x, body_pose.position.y, body_pose.position.z),
            (
                body_pose.orientation.x,
                body_pose.orientation.y,
                body_pose.orientation.z,
                body_pose.orientation.w,
            ),
            self.camera_translation,
            ROS_OPTICAL_TO_BODY_QUATERNION,
        )
        camera_pose = PoseStamped()
        camera_pose.header.stamp = message.header.stamp
        camera_pose.header.frame_id = "world"
        camera_pose.pose.position.x, camera_pose.pose.position.y, camera_pose.pose.position.z = camera_position
        (
            camera_pose.pose.orientation.x,
            camera_pose.pose.orientation.y,
            camera_pose.pose.orientation.z,
            camera_pose.pose.orientation.w,
        ) = camera_quaternion

        # EGO-Planner 用 ExactTime 同步深度和相机位姿，因此两条消息共用深度时间戳。
        self.pose_publisher.publish(camera_pose)
        self.depth_publisher.publish(depth_message)


def main() -> None:
    rospy.init_node("depth_pose_adapter")
    DepthPoseAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
