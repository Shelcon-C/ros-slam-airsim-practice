#!/usr/bin/env python3
"""把 VINS-Fusion 私有命名空间输出转换为稳定 Topic 与 TF。"""

import copy

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path


class VinsOutputAdapter:
    def __init__(self) -> None:
        input_topic = rospy.get_param("~input_topic", "/vins_estimator/odometry")
        odometry_topic = rospy.get_param("~odometry_topic", "/slam_practice/vins/odometry")
        path_topic = rospy.get_param("~path_topic", "/slam_practice/vins/path")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.max_path_poses = int(rospy.get_param("~max_path_poses", 20000))
        if self.max_path_poses <= 0:
            raise ValueError("~max_path_poses must be positive")
        self.odometry_publisher = rospy.Publisher(odometry_topic, Odometry, queue_size=20)
        self.path_publisher = rospy.Publisher(path_topic, Path, queue_size=2, latch=True)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self.subscriber = rospy.Subscriber(input_topic, Odometry, self.callback, queue_size=100)

    def callback(self, message: Odometry) -> None:
        odometry = copy.deepcopy(message)
        odometry.header.frame_id = self.world_frame
        odometry.child_frame_id = self.body_frame
        self.odometry_publisher.publish(odometry)

        pose = PoseStamped()
        pose.header = odometry.header
        pose.pose = odometry.pose.pose
        self.path.header.stamp = odometry.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_path_poses:
            del self.path.poses[: len(self.path.poses) - self.max_path_poses]
        self.path_publisher.publish(self.path)

        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self.body_frame
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rospy.init_node("vins_output_adapter")
    VinsOutputAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
