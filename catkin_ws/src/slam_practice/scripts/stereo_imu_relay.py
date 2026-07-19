#!/usr/bin/env python3
"""把 AirSim 双目图像和 IMU 规范化为 VINS-Fusion 输入 Topic。"""

import copy

import rospy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, Imu

from slam_practice.synchronization import synchronized_stamp


class StereoImuRelay:
    def __init__(self) -> None:
        left_input = rospy.get_param("~left_input", "/airsim_node/Drone1/left/Scene")
        right_input = rospy.get_param("~right_input", "/airsim_node/Drone1/right/Scene")
        imu_input = rospy.get_param("~imu_input", "/airsim_node/Drone1/imu/Imu")
        left_output = rospy.get_param("~left_output", "/vins_fusion/cam0/image_raw")
        right_output = rospy.get_param("~right_output", "/vins_fusion/cam1/image_raw")
        imu_output = rospy.get_param("~imu_output", "/vins_fusion/imu")
        self.bridge = CvBridge()
        self.left_publisher = rospy.Publisher(left_output, Image, queue_size=20)
        self.right_publisher = rospy.Publisher(right_output, Image, queue_size=20)
        self.imu_publisher = rospy.Publisher(imu_output, Imu, queue_size=200)
        self.left_subscriber = Subscriber(left_input, Image)
        self.right_subscriber = Subscriber(right_input, Image)
        # VINS-Fusion 上游节点使用 3 ms 同步容差，本桥接层保持同一约束。
        self.stereo_sync = ApproximateTimeSynchronizer(
            [self.left_subscriber, self.right_subscriber], queue_size=30, slop=0.003,
            allow_headerless=False
        )
        self.stereo_sync.registerCallback(self.stereo_callback)
        self.imu_subscriber = rospy.Subscriber(imu_input, Imu, self.imu_callback, queue_size=1000)
        rospy.loginfo("AirSim stereo/IMU relay is ready")

    def stereo_callback(self, left: Image, right: Image) -> None:
        left_time = left.header.stamp.to_sec()
        right_time = right.header.stamp.to_sec()
        try:
            shared_time = synchronized_stamp(left_time, right_time, tolerance=0.003)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, str(error))
            return
        try:
            left_gray = self.bridge.imgmsg_to_cv2(left, desired_encoding="mono8")
            right_gray = self.bridge.imgmsg_to_cv2(right, desired_encoding="mono8")
        except Exception as error:
            rospy.logerr_throttle(2.0, "Stereo image conversion failed: %s", error)
            return

        stamp = rospy.Time.from_sec(shared_time)
        left_message = self.bridge.cv2_to_imgmsg(left_gray, encoding="mono8")
        right_message = self.bridge.cv2_to_imgmsg(right_gray, encoding="mono8")
        left_message.header.stamp = stamp
        right_message.header.stamp = stamp
        left_message.header.frame_id = "cam0_optical"
        right_message.header.frame_id = "cam1_optical"
        self.left_publisher.publish(left_message)
        self.right_publisher.publish(right_message)

    def imu_callback(self, message: Imu) -> None:
        # AirSim Wrapper 已配置 ENU；这里不再次旋转，避免重复坐标变换。
        output = copy.deepcopy(message)
        output.header.frame_id = "body"
        self.imu_publisher.publish(output)


def main() -> None:
    rospy.init_node("stereo_imu_relay")
    StereoImuRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
